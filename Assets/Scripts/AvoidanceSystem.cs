using Drawing;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;

/// <summary>
/// Velocity-based avoidance system for smooth unit movement in large groups.
/// Uses reciprocal velocity obstacles for cooperative avoidance.
/// </summary>
[BurstCompile]
[UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
[UpdateAfter(typeof(StartMovementSystem))]
[UpdateBefore(typeof(NavAgentOverlapResolutionSystem))]
public partial struct AvoidanceSystem : ISystem
{
    private readonly static int2 mapSize = new int2(512, 512) * 4;
    private readonly static int cellSize = 8;

    private SpatialGrid<AvoidanceAgentData> spatialGrid;

    [BurstCompile]
    public void OnCreate(ref SystemState state)
    {
        spatialGrid = new SpatialGrid<AvoidanceAgentData>(mapSize, cellSize, 100000, Allocator.Persistent);
    }

    [BurstCompile]
    public void OnDestroy(ref SystemState state)
    {
        spatialGrid.Dispose();
    }

    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        // Step 1: Reset grid
        var resetJob = new SpatialGrid<AvoidanceAgentData>.ResetPrefixSum()
        {
            activeCells = spatialGrid.activeCells,
            cellCounts = spatialGrid.cellCounts,
        };

        // Step 2: Calculate cell counts
        var calculateCountsJob = new CalculateAvoidanceCellCounts()
        {
            gridProperties = spatialGrid.gridProperties,
            cellCounts = spatialGrid.cellCounts,
        };

        // Step 3: Calculate cell offsets
        var calculateOffsetsJob = new SpatialGrid<AvoidanceAgentData>.CalculateCellOffsets()
        {
            activeCells = spatialGrid.activeCells,
            cellCounts = spatialGrid.cellCounts,
            cellOffsets = spatialGrid.cellOffsets,
        };

        // Step 4: Populate grid with agent data
        var populateGridJob = new PopulateAvoidanceGridData()
        {
            gridProperties = spatialGrid.gridProperties,
            cellOffsets = spatialGrid.cellOffsets,
            spatialData = spatialGrid.cellData,
        };

        // Step 5: Process avoidance
        var processAvoidanceJob = new ProcessAvoidance()
        {
            activeCells = spatialGrid.activeCells.AsDeferredJobArray(),
            gridProperties = spatialGrid.gridProperties,
            cellCounts = spatialGrid.cellCounts,
            cellOffsets = spatialGrid.cellOffsets,
            spatialData = spatialGrid.cellData,
            TimeStep = 0.1f,
            AvoidanceHorizon = 2.0f,
            SeparationWeight = 1.5f,
        };

        // Step 6: Apply avoidance velocities
        var applyAvoidanceJob = new ApplyAvoidanceVelocities()
        {
            spatialData = spatialGrid.cellData,
            DeltaTime = 0.1f,
        };

        state.Dependency = resetJob.Schedule(state.Dependency);
        state.Dependency = calculateCountsJob.ScheduleParallel(state.Dependency);
        state.Dependency = calculateOffsetsJob.Schedule(state.Dependency);
        state.Dependency = populateGridJob.ScheduleParallel(state.Dependency);
        state.Dependency = processAvoidanceJob.Schedule(spatialGrid.activeCells, 1, state.Dependency);
        state.Dependency = applyAvoidanceJob.ScheduleParallel(state.Dependency);

        state.Dependency.Complete();
    }

    public struct AvoidanceAgentData
    {
        public float2 Position;
        public float2 Velocity;
        public float2 DesiredVelocity;
        public float2 Forward;
        //public float Length;
        public float Radius;
        public float MaxSpeed;
        public int Priority;

        public float2 AvoidanceVelocity;
        public int AvoidanceCount;

        public void GetMinMax(out float2 min, out float2 max)
        {
            //var p1 = Position - Forward * Length * 0.5f;
            //var p2 = Position + Forward * Length * 0.5f;
            var p1 = Position;
            var p2 = Position;
            min = math.min(p1 - Radius, p2 - Radius);
            max = math.max(p1 + Radius, p2 + Radius);
        }
    }

    [BurstCompile]
    public partial struct CalculateAvoidanceCellCounts : IJobEntity
    {
        [ReadOnly] public SpatialGridProperties gridProperties;

        [NativeDisableParallelForRestriction]
        [WriteOnly] public NativeArray<int> cellCounts;

        public void Execute(in SimLocalTransform simTransform, ref AgentComponent agent, in AvoidanceVelocity avoidVel)
        {
            // Calculate agent bounds for spatial grid
            float2 pos = simTransform.Value.Position.xz;
            float2 forward = math.normalize(simTransform.Value.Forward().xz);

            float2 agentMin, agentMax;
            agent.GetMinMax(pos, forward, out agentMin, out agentMax);

            // Expand bounds by avoidance horizon
            float lookAheadDist = 2.0f;
            agentMin -= lookAheadDist;
            agentMax += lookAheadDist;

            var minCell = gridProperties.GetCellKeyClamped(agentMin);
            var maxCell = gridProperties.GetCellKeyClamped(agentMax);

            agent.Indicies.Clear();

            for (int y = minCell.y; y <= maxCell.y; y++)
            {
                for (int x = minCell.x; x <= maxCell.x; x++)
                {
                    var cellIndex = gridProperties.GetCellIndex(new int2(x, y));

                    unsafe
                    {
                        System.Threading.Interlocked.Increment(ref UnsafeUtility.ArrayElementAsRef<int>(cellCounts.GetUnsafePtr(), cellIndex));
                        agent.Indicies.Add(cellIndex);
                    }
                }
            }
        }
    }

    [BurstCompile]
    public partial struct PopulateAvoidanceGridData : IJobEntity
    {
        [ReadOnly] public SpatialGridProperties gridProperties;
        [ReadOnly] public NativeArray<int> cellOffsets;

        [NativeDisableParallelForRestriction]
        [WriteOnly] public NativeArray<AvoidanceAgentData> spatialData;

        public void Execute(
            in SimLocalTransform simTransform,
            in AgentComponent agent,
            in AvoidanceVelocity avoidVel,
            in MaxSpeedComponent maxSpeed,
            in DesiredVelocity desiredVel)
        {
            float2 pos = simTransform.Value.Position.xz;
            float2 forward = math.normalize(simTransform.Value.Forward().xz);

            var agentData = new AvoidanceAgentData
            {
                Position = pos,
                Velocity = avoidVel.Value,
                DesiredVelocity = desiredVel.Value,
                Forward = forward,
                //Length = agent.Length,
                Radius = agent.Radius + agent.Length * 0.5f,
                MaxSpeed = maxSpeed.Value,
                Priority = agent.AvoidancePriority,
                AvoidanceVelocity = float2.zero,
                AvoidanceCount = 0,
            };

            // Calculate bounds with avoidance horizon
            float2 agentMin, agentMax;
            agent.GetMinMax(pos, forward, out agentMin, out agentMax);

            float lookAheadDist = 2.0f;
            agentMin -= lookAheadDist;
            agentMax += lookAheadDist;

            var minCell = gridProperties.GetCellKeyClamped(agentMin);
            var maxCell = gridProperties.GetCellKeyClamped(agentMax);

            int i = 0;

            for (int y = minCell.y; y <= maxCell.y; y++)
            {
                for (int x = minCell.x; x <= maxCell.x; x++)
                {
                    var cellIndex = gridProperties.GetCellIndex(new int2(x, y));
                    var cellOffset = cellOffsets[cellIndex];

                    unsafe
                    {
                        var localIndex = agent.Indicies[i++];

                        var index = cellOffset + cellOffset;

                        //var index = System.Threading.Interlocked.Increment(ref UnsafeUtility.ArrayElementAsRef<int>(cellOffsets.GetUnsafePtr(), cellIndex)) - 1;
                        spatialData[index] = agentData;
                    }
                }
            }
        }
    }

    [BurstCompile]
    public struct ProcessAvoidance : IJobParallelForDefer
    {
        public SpatialGridProperties gridProperties;
        public float TimeStep;
        public float AvoidanceHorizon;
        public float SeparationWeight;

        [ReadOnly] public NativeArray<int> cellCounts;
        [ReadOnly] public NativeArray<int> cellOffsets;
        [ReadOnly] public NativeArray<int> activeCells;

        [NativeDisableParallelForRestriction]
        public NativeArray<AvoidanceAgentData> spatialData;

        public void Execute(int index)
        {
            var cellIndex = activeCells[index];
            var data = spatialData.GetSubArray(cellOffsets[cellIndex], cellCounts[cellIndex]);

            for (int i = 0; i < data.Length; i++)
            {
                unsafe
                {
                    ref var agentA = ref UnsafeUtility.ArrayElementAsRef<AvoidanceAgentData>(data.GetUnsafePtr(), i);

                    // Calculate avoidance velocity by sampling preferred velocities
                    float2 bestVelocity = agentA.DesiredVelocity;
                    float bestCost = CalculateVelocityCost(ref agentA, bestVelocity, data.AsReadOnly(), i);

                    // Sample velocities in a circular pattern
                    int samples = 12;
                    float angleStep = math.PI * 2.0f / samples;

                    for (int s = 0; s < samples; s++)
                    {
                        float angle = angleStep * s;
                        float2 dir = new float2(math.cos(angle), math.sin(angle));

                        // Try multiple speeds
                        for (float speedMult = 0.5f; speedMult <= 1.0f; speedMult += 0.25f)
                        {
                            float2 testVel = dir * agentA.MaxSpeed * speedMult;
                            float cost = CalculateVelocityCost(ref agentA, testVel, data.AsReadOnly(), i);

                            if (cost < bestCost)
                            {
                                bestCost = cost;
                                bestVelocity = testVel;
                            }
                        }
                    }

                    agentA.AvoidanceVelocity = bestVelocity;
                    agentA.AvoidanceCount = 1;
                }
            }
        }

        private float CalculateVelocityCost(
            ref AvoidanceAgentData agentA,
            float2 testVelocity,
            NativeArray<AvoidanceAgentData>.ReadOnly data,
            int skipIndex)
        {
            float cost = 0f;

            // Cost for deviating from desired velocity
            float2 desiredDiff = testVelocity - agentA.DesiredVelocity;
            cost += math.lengthsq(desiredDiff) * 0.5f;

            // Check collisions with other agents
            for (int j = 0; j < data.Length; j++)
            {
                if (j == skipIndex) continue;

                var agentB = data[j];

                // Skip if too far away
                float2 relPos = agentB.Position - agentA.Position;
                float distSq = math.lengthsq(relPos);

                float maxDist = AvoidanceHorizon + agentA.Radius + agentB.Radius;
                if (distSq > maxDist * maxDist)
                    continue;

                // Predict future collision using velocity obstacles
                float2 relVel = testVelocity - agentB.Velocity;

                // Time to closest approach
                float timeToCollision = -math.dot(relPos, relVel) / (math.lengthsq(relVel) + 0.001f);
                timeToCollision = math.clamp(timeToCollision, 0f, AvoidanceHorizon);

                // Future positions
                float2 futureA = agentA.Position + testVelocity * timeToCollision;
                float2 futureB = agentB.Position + agentB.Velocity * timeToCollision;

                float2 futureDist = futureA - futureB;
                float futureDistMag = math.length(futureDist);

                float minSeparation = (agentA.Radius + agentB.Radius) * SeparationWeight;

                if (futureDistMag < minSeparation)
                {
                    // Collision detected - add penalty
                    float penetration = minSeparation - futureDistMag;

                    // Higher penalty for earlier collisions
                    float timeFactor = math.max(0.1f, 1.0f - timeToCollision / AvoidanceHorizon);

                    // Priority affects who should avoid more
                    float priorityFactor = ComputeAvoidanceFactor(agentA.Priority, agentB.Priority);

                    cost += penetration * timeFactor * priorityFactor * 50.0f;
                }
            }

            return cost;
        }

        private static float ComputeAvoidanceFactor(int priA, int priB)
        {
            // Lower priority agents avoid more
            float wA = 1f / (1f + priA);
            float wB = 1f / (1f + priB);
            float sum = wA + wB;
            return sum > 0f ? (wA / sum) * 2.0f : 1.0f;
        }
    }

    [BurstCompile]
    public partial struct ApplyAvoidanceVelocities : IJobEntity
    {
        [ReadOnly] public NativeArray<AvoidanceAgentData> spatialData;
        public float DeltaTime;

        public void Execute(
            ref SimLocalTransform simTransform,
            ref AvoidanceVelocity avoidVel,
            ref DesiredVelocity desiredVel,
            ref AgentComponent agent)
        {
            // Find our data in the spatial grid
            float2 computedVelocity = desiredVel.Value;

            // Average all instances (should typically be just one per cell overlap)
            for (int i = 0; i < agent.Indicies.Length; i++)
            {
                var data = spatialData[agent.Indicies[i]];
                if (data.AvoidanceCount > 0)
                {
                    computedVelocity = data.AvoidanceVelocity;
                    break;
                }
            }

            // Smooth the avoidance velocity for stability
            float smoothing = 0.3f;
            avoidVel.Value = math.lerp(avoidVel.Value, computedVelocity, smoothing);

            // Update desired velocity to the smoothed avoidance velocity
            desiredVel.Value = avoidVel.Value;

            // Update position
            float3 velocity3D = new float3(avoidVel.Value.x, 0, avoidVel.Value.y);
            simTransform.Value.Position += velocity3D * DeltaTime;

            // Update rotation to face movement direction
            if (math.lengthsq(avoidVel.Value) > 0.01f)
            {
                float2 forward = math.normalize(avoidVel.Value);
                float angle = math.atan2(forward.x, forward.y);
                simTransform.Value.Rotation = quaternion.RotateY(angle);
            }

            agent.Indicies.Clear();
        }
    }
}