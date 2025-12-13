using Drawing;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;

public struct AgentSpatialData
{
    public float2 Position;
    public float2 Forward;
    public float Length;      // Semi-axis along forward
    public float Radius;        // Semi-axis perpendicular to forward
    public float2 CorectionOffset;
    public int CorrectionCount;
    public float Priority;

    public float PenetrationSum;
    public float Padding;

    public void GetMinMax(out float2 min, out float2 max)
    {
        var p1 = Position - Forward * Length * 0.5f;
        var p2 = Position + Forward * Length * 0.5f;
        min = math.min(p1 - Radius, p2 - Radius);
        max = math.max(p1 + Radius, p2 + Radius);
    }
}

[BurstCompile]
[UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
[UpdateAfter(typeof(EndMovementSystem))]
public partial struct NavAgentOverlapResolutionSystem : ISystem
{
    const int Iterations = 8;    // Number of positional solver iterations
    const float SeparationBias = 1.1f;//1.05f;//1.1f;//1.05f;

    private readonly static int2 mapSize = new int2(512, 512) * 4;
    private readonly static int cellSize = 8;

    private SpatialGrid<AgentSpatialData> spatialGrid;

    [BurstCompile]
    public void OnCreate(ref SystemState state) 
    {
        spatialGrid = new SpatialGrid<AgentSpatialData>(mapSize, cellSize, 100000, Allocator.Persistent);
    }

    [BurstCompile]
    public void OnDestroy(ref SystemState state) { spatialGrid.Dispose(); }

    //[BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        if (Boostrap.Instance.DrawGrid)
        {
            spatialGrid.Draw();
        }

        for (int i = 0; i < Iterations; i++)
        {
            var resetJob = new SpatialGrid<AgentSpatialData>.ResetPrefixSum()
            {
                activeCells = spatialGrid.activeCells,
                cellCounts = spatialGrid.cellCounts,
            };

            var calculateCountsJob = new CalculateHardForceCellCounts()
            {
                gridProperties = spatialGrid.gridProperties,
                cellCounts = spatialGrid.cellCounts,
            };

            var calculateOffsetsJob = new SpatialGrid<AgentSpatialData>.CalculateCellOffsets()
            {
                activeCells = spatialGrid.activeCells,
                cellCounts = spatialGrid.cellCounts,
                cellOffsets = spatialGrid.cellOffsets,
            };

            var PopulateHardForceGridData = new PopulateHardForceGridData()
            {
                gridProperties = spatialGrid.gridProperties,
                cellOffsets = spatialGrid.cellOffsets,
                spatialData = spatialGrid.cellData,
            };


            var ProcessHardForces = new ProcessHardForces()
            {
                activeCells = spatialGrid.activeCells.AsDeferredJobArray(),
                gridProperties = spatialGrid.gridProperties,
                cellCounts = spatialGrid.cellCounts,
                cellOffsets = spatialGrid.cellOffsets,
                spatialData = spatialGrid.cellData,
                SeparationBias = SeparationBias,
            };

            var IJobEntity_WriteHardForces = new IJobEntity_WriteHardForces()
            {
                spatialData = spatialGrid.cellData,
                CrowdingFactorChangeSpeed = 1,
                Iterations = Iterations,
            };

            state.Dependency = resetJob.Schedule(state.Dependency);
            state.Dependency = calculateCountsJob.ScheduleParallel(state.Dependency);
            state.Dependency = calculateOffsetsJob.Schedule(state.Dependency);
            state.Dependency = PopulateHardForceGridData.ScheduleParallel(state.Dependency);
            state.Dependency = ProcessHardForces.Schedule(spatialGrid.activeCells, 1, state.Dependency);
            state.Dependency = IJobEntity_WriteHardForces.ScheduleParallel(state.Dependency);
        }

        state.Dependency.Complete();
    }

    [BurstCompile]
    public partial struct CalculateHardForceCellCounts : IJobEntity
    {
        [ReadOnly]  public SpatialGridProperties gridProperties;

        [NativeDisableParallelForRestriction]
        [WriteOnly] public NativeArray<int> cellCounts;

        public void Execute(in SimLocalTransform simTranslation, ref AgentComponent agent)
        {
            agent.Indicies.Clear();

            //var min = simTranslation.Value.Position.xz - agent.Radius;
            //var max = simTranslation.Value.Position.xz + agent.Radius;
            agent.GetMinMax(simTranslation.Value.Position.xz, math.normalize(simTranslation.Value.Forward().xz), out var agentMinimum, out var agentMaximum);
            var minCell = gridProperties.GetCellKeyClamped(agentMinimum);
            var maxCell = gridProperties.GetCellKeyClamped(agentMaximum);

            for (int y = minCell.y; y <= maxCell.y; y++)
            {
                for (int x = minCell.x; x <= maxCell.x; x++)
                {
                    var cellIndex = gridProperties.GetCellIndex(new int2(x, y));

                    unsafe
                    {
                        var index = System.Threading.Interlocked.Increment(ref UnsafeUtility.ArrayElementAsRef<int>(cellCounts.GetUnsafePtr(), cellIndex)) - 1;
                        
                        if (agent.Indicies.Length + 1 >= agent.Indicies.Capacity)
                        {
                            UnityEngine.Debug.LogError("We hit max capacity, we should not be able to do this...");
                            continue;
                        }
                        agent.Indicies.Add(index);
                    }
                }
            }
        }
    }

    [BurstCompile]
    public partial struct PopulateHardForceGridData : IJobEntity
    {
        [ReadOnly] public SpatialGridProperties gridProperties;

        [ReadOnly] public NativeArray<int> cellOffsets;

        [NativeDisableParallelForRestriction]
        [WriteOnly] public NativeArray<AgentSpatialData> spatialData;

        public void Execute(in SimLocalTransform simTranslation, ref AgentComponent agent)
        {
            var agentSpatialData = new AgentSpatialData
            {
                Position = simTranslation.Value.Position.xz,
                Forward = math.normalize(simTranslation.Value.Forward().xz),
                Length = agent.Length,
                Radius = agent.Radius,
                Priority = 50,
                CorectionOffset = 0,
                CorrectionCount = 0,
            };

            agent.GetMinMax(simTranslation.Value.Position.xz, math.normalize(simTranslation.Value.Forward().xz), out var agentMinimum, out var agentMaximum);
            var minCell = gridProperties.GetCellKeyClamped(agentMinimum);
            var maxCell = gridProperties.GetCellKeyClamped(agentMaximum);

            //var min = simTranslation.Value.Position.xz - agent.Radius;
            //var max = simTranslation.Value.Position.xz + agent.Radius;

            //var minCell = gridProperties.GetCellKeyClamped(min);
            //var maxCell = gridProperties.GetCellKeyClamped(max);

            int i = 0;

            for (int y = minCell.y; y <= maxCell.y; y++)
            {
                for (int x = minCell.x; x <= maxCell.x; x++)
                {
                    var cellIndex = gridProperties.GetCellIndex(new int2(x, y));
                    var cellOffset = cellOffsets[cellIndex];

                    if (i + 1 >= agent.Indicies.Capacity)
                    {
                        UnityEngine.Debug.LogError("We hit max capacity, we should not be able to do this...");
                        continue;
                    }

                    var dataIndex = agent.Indicies[i] + cellOffset;

                    agent.Indicies[i++] = dataIndex;

                    spatialData[dataIndex] = agentSpatialData;
                }
            }
        }
    }

    [BurstCompile]
    public struct ProcessHardForces : IJobParallelForDefer
    {
        public SpatialGridProperties gridProperties;

        public float SeparationBias;

        [ReadOnly] public NativeArray<int> cellCounts;
        [ReadOnly] public NativeArray<int> cellOffsets;
        [ReadOnly] public NativeArray<int> activeCells;

        [NativeDisableParallelForRestriction] public NativeArray<AgentSpatialData> spatialData;

        private static float ComputeMoveFraction(float priA, float priB)
        {
            // 0 = highest priority -> weight smaller
            float wA = 1f / (1f + priA);
            float wB = 1f / (1f + priB);
            float sum = wA + wB;
            return sum > 0f ? wA / sum : 0.5f;
        }

        private static (int2, int2) Intersection(int2 minA, int2 maxA, int2 minB, int2 maxB)
        {
            return (math.max(minA, minB), math.min(maxA, maxB));
        }

        public unsafe void Execute(int index)
        {
            // TODO:! We need to sort by priority. So we apply the positions to the important ones first? Although would this actually make a difference?

            var cellIndex = activeCells[index];

            var data = spatialData.GetSubArray(cellOffsets[cellIndex], cellCounts[cellIndex]);

            for (int i = 0; i < data.Length; i++)
            {
                ref var agentA = ref UnsafeUtility.ArrayElementAsRef<AgentSpatialData>(data.GetUnsafePtr(), i);

                agentA.GetMinMax(out var agentAMinimum, out var agentAMaximum);
                var agentAMin = gridProperties.GetCellKeyClamped(agentAMinimum);
                var agentAMax = gridProperties.GetCellKeyClamped(agentAMaximum);

                for (int j = i + 1; j < data.Length; j++)
                {
                    ref var agentB = ref UnsafeUtility.ArrayElementAsRef<AgentSpatialData>(data.GetUnsafePtr(), j);

                    // TODO: These can be pre-calculated
                    agentB.GetMinMax(out var agentBMinimum, out var agentBMaximum);
                    var agentBMin = gridProperties.GetCellKeyClamped(agentBMinimum);
                    var agentBMax = gridProperties.GetCellKeyClamped(agentBMaximum);

                    bool isBottomIntersection = cellIndex != gridProperties.GetCellIndex(Intersection(agentAMin, agentAMax, agentBMin, agentBMax).Item1);

                    // Remove duplicates.
                    if (isBottomIntersection)
                    {
                        continue;
                    }

                    // TODO: Do broad phase check!
                    //float2 delta = agentA.Position - agentB.Position;
                    //float distSq = math.lengthsq(delta);

                    //float minDist = (agentA.Radius + agentB.Radius) * SeparationBias;
                    //float minDistSq = minDist * minDist;

                    //// Broad phase circle.
                    //if (distSq >= minDistSq)
                    //    continue;

                    Capsule2D A = new Capsule2D
                    {
                        position = agentA.Position,
                        forward = agentA.Forward,
                        halfLength = agentA.Length * 0.5f,
                        radius = agentA.Radius * SeparationBias,
                    };
                    Capsule2D B = new Capsule2D
                    {
                        position = agentB.Position,
                        forward = agentB.Forward,
                        halfLength = agentB.Length * 0.5f,
                        radius = agentB.Radius * SeparationBias,
                    };

                    var collision = CapsuleCollision.CheckCollision(A, B);

                    // TODO: If they are right on top do stable direction.
                    //normal = math.normalize(new float2(0.73f, 0.68f)); // arbitrary stable dir
                    if (collision.colliding)
                    {
                        float2 normal = collision.normal;
                        float penetration = collision.penetrationDepth;

                        float moveFracA = ComputeMoveFraction(agentA.Priority, agentB.Priority);
                        float moveFracB = ComputeMoveFraction(agentB.Priority, agentA.Priority);
                        float2 corrA = normal * (penetration * moveFracA);
                        float2 corrB = -normal * (penetration * moveFracB);

                        agentA.PenetrationSum += penetration;
                        agentA.CorectionOffset += corrA;
                        agentA.CorrectionCount += 1;

                        agentB.PenetrationSum += penetration;
                        agentB.CorectionOffset += corrB;
                        agentB.CorrectionCount += 1;
                    }
                }
            }
        }
    }

    [BurstCompile]
    public partial struct IJobEntity_WriteHardForces : IJobEntity
    {
        [ReadOnly] public NativeArray<AgentSpatialData> spatialData;

        // The amount the colliders can expand per tick.
        public float CrowdingFactorChangeSpeed;
        public float Iterations;

        public void Execute(ref SimLocalTransform simLocalTransform, ref AgentComponent agent)
        {
            float2 corectionOffset = 0;
            int correctionCount = 0;
            float penetrationSum = 0;

            for (int i = 0; i < agent.Indicies.Length; i++)
            {
                var index = agent.Indicies[i];

                var data = spatialData[index];

                corectionOffset += data.CorectionOffset;
                correctionCount += data.CorrectionCount;

                penetrationSum += data.PenetrationSum;
            }

            agent.Indicies.Clear();

            var pastCrowdingScore = agent.CrowdingFactor;

            if (correctionCount > 0)
            {
                // Calculate relative penetration (compared to capsule size)
                // Should change this from penetartion to area ratios.
                float capsuleSize = agent.Length + agent.Radius * 2;
                float relativePenetration = penetrationSum / capsuleSize;

                // TODO: Make this a value on the agent.
                // NOTE: We can also scale this by the number of collisions. This way, if we only collide twice, we will not scale.
                agent.CrowdingFactor = math.lerp(1, 1.4f, math.saturate(relativePenetration));

                corectionOffset /= correctionCount;

                simLocalTransform.Value.Position += new float3(corectionOffset.x, 0, corectionOffset.y);
            }

            agent.CrowdingFactor = math.lerp(pastCrowdingScore, agent.CrowdingFactor, CrowdingFactorChangeSpeed / Iterations);
        }
    }








    //// We'll embed some helpers into job structs below

    //// 1) Job: rebuild spatial grid (positions -> cells)
    //[BurstCompile]
    //struct BuildGridJob : IJobParallelFor
    //{
    //    [ReadOnly] public NativeArray<float3> Positions;
    //    [ReadOnly] public float CellSize;

    //    public NativeParallelMultiHashMap<int, int>.ParallelWriter CellMap;

    //    public void Execute(int index)
    //    {
    //        float3 pos = Positions[index];

    //        int2 cell = new int2(
    //            (int)math.floor(pos.x / CellSize),
    //            (int)math.floor(pos.z / CellSize)
    //        );

    //        int key = Hash(cell);
    //        CellMap.Add(key, index);
    //    }
    //}

    //// 2) Job: positional overlap resolution (Jacobi style)
    //[BurstCompile]
    //struct ResolveOverlapsJob : IJobParallelFor
    //{
    //    [ReadOnly] public NativeArray<AgentData> Agents;
    //    [ReadOnly] public NativeArray<float3> PositionsRead;
    //    [ReadOnly] public NativeArray<int> SortedIndices;
    //    [ReadOnly] public NativeParallelMultiHashMap<int, int> CellMap;

    //    public NativeArray<float3> PositionsWrite;

    //    public float CellSize;
    //    public float SeparationBias;

    //    public void Execute(int sortedIndexIndex)
    //    {
    //        // Work in priority order, but parallel; we index into sortedIndices
    //        int idxA = SortedIndices[sortedIndexIndex];
    //        var a = Agents[idxA];

    //        float3 posA = PositionsRead[idxA];

    //        float3 correctionSum = float3.zero;
    //        int correctionCount = 0;

    //        int2 cellA = new int2(
    //            (int)math.floor(posA.x / CellSize),
    //            (int)math.floor(posA.z / CellSize)
    //        );

    //        for (int dy = -1; dy <= 1; dy++)
    //        {
    //            for (int dx = -1; dx <= 1; dx++)
    //            {
    //                int2 cell = cellA + new int2(dx, dy);
    //                int key = Hash(cell);

    //                if (!CellMap.TryGetFirstValue(key, out int idxB, out var it))
    //                    continue;

    //                do
    //                {
    //                    if (idxB == idxA)
    //                        continue;

    //                    var b = Agents[idxB];

    //                    float3 posB = PositionsRead[idxB];

    //                    float3 delta = posA - posB;
    //                    float distSq = math.lengthsq(delta);

    //                    float minDist = (a.Radius + b.Radius) * SeparationBias;
    //                    float minDistSq = minDist * minDist;

    //                    if (distSq >= minDistSq)
    //                        continue;

    //                    float dist = math.sqrt(distSq);

    //                    float3 normal;
    //                    if (dist > 1e-4f)
    //                        normal = delta / dist;
    //                    else
    //                        normal = math.normalize(new float3(0.73f, 0f, 0.68f)); // arbitrary stable dir

    //                    float penetration = minDist - dist;

    //                    float moveFracA = ComputeMoveFraction(a.Priority, b.Priority);
    //                    float3 corrA = normal * (penetration * moveFracA);

    //                    correctionSum += corrA;
    //                    correctionCount += 1;

    //                } while (CellMap.TryGetNextValue(out idxB, ref it));
    //            }
    //        }

    //        float3 newPos = posA;
    //        if (correctionCount > 0)
    //        {
    //            float3 avgCorrection = correctionSum / correctionCount;
    //            newPos += avgCorrection;
    //        }

    //        PositionsWrite[idxA] = newPos;
    //    }

    //    static float ComputeMoveFraction(int priA, int priB)
    //    {
    //        // 0 = highest priority -> weight smaller
    //        float wA = 1f / (1f + (float)priA);
    //        float wB = 1f / (1f + (float)priB);
    //        float sum = wA + wB;
    //        return sum > 0f ? wA / sum : 0.5f;
    //    }
    //}

    //// 3) Job: write back to LocalTransform
    //[BurstCompile]
    //struct WriteBackJob : IJobParallelFor
    //{
    //    [ReadOnly] public NativeArray<AgentData> Agents;
    //    [ReadOnly] public NativeArray<float3> FinalPositions;

    //    [NativeDisableParallelForRestriction]
    //    public ComponentLookup<SimLocalTransform> TransformLookup;

    //    public void Execute(int index)
    //    {
    //        var data = Agents[index];
    //        var t = TransformLookup[data.Entity];
    //        t.Value.Position = FinalPositions[index];
    //        TransformLookup[data.Entity] = t;
    //    }
    //}

}