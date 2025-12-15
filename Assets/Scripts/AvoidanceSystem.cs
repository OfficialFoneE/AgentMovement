using Drawing;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;

/// <summary>
/// Velocity-based avoidance system for smooth unit movement in large groups.
/// Uses reciprocal velocity obstacles for cooperative avoidance.
/// Simple O(n²) implementation for clarity - optimize with spatial partitioning later.
/// </summary>
[BurstCompile]
[UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
[UpdateAfter(typeof(StartMovementSystem))]
[UpdateBefore(typeof(EndMovementSystem))]
[UpdateBefore(typeof(NavAgentOverlapResolutionSystem))]
public partial struct AvoidanceSystem : ISystem
{
    // Tunable parameters
    const float AvoidanceHorizon = 2.0f;      // How far ahead to look (seconds)
    const float SeparationWeight = 1.2f;      // Personal space multiplier
    const float VelocitySmoothing = 0.3f;     // How quickly to adapt to new velocities
    const int VelocitySamples = 16;           // Number of directions to sample
    const float TimeStep = 0.1f;              // Fixed timestep (matches your FixedStepSimulationSystemGroup)

    [BurstCompile]
    public void OnCreate(ref SystemState state) { }

    [BurstCompile]
    public void OnDestroy(ref SystemState state) { }

    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        // Gather all agents into arrays for processing
        var agentQuery = SystemAPI.QueryBuilder()
            .WithAll<SimLocalTransform, AgentComponent, AvoidanceVelocity, MaxSpeedComponent, DesiredVelocity>()
            .Build();

        int agentCount = agentQuery.CalculateEntityCount();
        if (agentCount == 0) return;

        var positions = new NativeArray<float2>(agentCount, Allocator.TempJob);
        var velocities = new NativeArray<float2>(agentCount, Allocator.TempJob);
        var desiredVelocities = new NativeArray<float2>(agentCount, Allocator.TempJob);
        var forwards = new NativeArray<float2>(agentCount, Allocator.TempJob);
        var radii = new NativeArray<float>(agentCount, Allocator.TempJob);
        var lengths = new NativeArray<float>(agentCount, Allocator.TempJob);
        var maxSpeeds = new NativeArray<float>(agentCount, Allocator.TempJob);
        var priorities = new NativeArray<int>(agentCount, Allocator.TempJob);
        var newVelocities = new NativeArray<float2>(agentCount, Allocator.TempJob);

        // Job 1: Gather agent data
        var gatherJob = new GatherAgentDataJob
        {
            Positions = positions,
            Velocities = velocities,
            DesiredVelocities = desiredVelocities,
            Forwards = forwards,
            Radii = radii,
            Lengths = lengths,
            MaxSpeeds = maxSpeeds,
            Priorities = priorities,
        };

        // Job 2: Calculate avoidance velocities
        var avoidanceJob = new CalculateAvoidanceJob
        {
            Positions = positions,
            Velocities = velocities,
            DesiredVelocities = desiredVelocities,
            Forwards = forwards,
            Radii = radii,
            Lengths = lengths,
            MaxSpeeds = maxSpeeds,
            Priorities = priorities,
            NewVelocities = newVelocities,
            AvoidanceHorizon = AvoidanceHorizon,
            SeparationWeight = SeparationWeight,
            VelocitySamples = VelocitySamples,
        };

        // Job 3: Apply avoidance velocities
        var applyJob = new ApplyAvoidanceJob
        {
            NewVelocities = newVelocities,
            VelocitySmoothing = VelocitySmoothing,
            TimeStep = TimeStep,
        };

        state.Dependency = gatherJob.ScheduleParallel(state.Dependency);
        state.Dependency = avoidanceJob.Schedule(agentCount, 32, state.Dependency);
        state.Dependency = applyJob.ScheduleParallel(state.Dependency);
        state.Dependency.Complete();

        // Cleanup
        positions.Dispose();
        velocities.Dispose();
        desiredVelocities.Dispose();
        forwards.Dispose();
        radii.Dispose();
        lengths.Dispose();
        maxSpeeds.Dispose();
        priorities.Dispose();
        newVelocities.Dispose();
    }

    [BurstCompile]
    partial struct GatherAgentDataJob : IJobEntity
    {
        [WriteOnly] public NativeArray<float2> Positions;
        [WriteOnly] public NativeArray<float2> Velocities;
        [WriteOnly] public NativeArray<float2> DesiredVelocities;
        [WriteOnly] public NativeArray<float2> Forwards;
        [WriteOnly] public NativeArray<float> Radii;
        [WriteOnly] public NativeArray<float> Lengths;
        [WriteOnly] public NativeArray<float> MaxSpeeds;
        [WriteOnly] public NativeArray<int> Priorities;

        void Execute(
            [EntityIndexInQuery] int index,
            in SimLocalTransform simTransform,
            in AgentComponent agent,
            in AvoidanceVelocity avoidVel,
            in MaxSpeedComponent maxSpeed,
            in DesiredVelocity desiredVel)
        {
            Positions[index] = simTransform.Value.Position.xz;
            Velocities[index] = avoidVel.Value;
            DesiredVelocities[index] = desiredVel.Value;
            Forwards[index] = math.normalize(simTransform.Value.Forward().xz);
            Radii[index] = agent.Radius + agent.Length * 0.5f;
            Lengths[index] = agent.Length;
            MaxSpeeds[index] = maxSpeed.Value;
            Priorities[index] = agent.AvoidancePriority;
        }
    }

    [BurstCompile]
    struct CalculateAvoidanceJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float2> Positions;
        [ReadOnly] public NativeArray<float2> Velocities;
        [ReadOnly] public NativeArray<float2> DesiredVelocities;
        [ReadOnly] public NativeArray<float2> Forwards;
        [ReadOnly] public NativeArray<float> Radii;
        [ReadOnly] public NativeArray<float> Lengths;
        [ReadOnly] public NativeArray<float> MaxSpeeds;
        [ReadOnly] public NativeArray<int> Priorities;

        [WriteOnly] public NativeArray<float2> NewVelocities;

        public float AvoidanceHorizon;
        public float SeparationWeight;
        public int VelocitySamples;

        public void Execute(int i)
        {
            float2 position = Positions[i];
            float2 currentVel = Velocities[i];
            float2 desiredVel = DesiredVelocities[i];
            float maxSpeed = MaxSpeeds[i];
            float radius = Radii[i];
            int priority = Priorities[i];

            // Start with desired velocity as best candidate
            float2 bestVelocity = desiredVel;
            float bestCost = CalculateVelocityCost(i, desiredVel);

            // Sample velocities in a circular pattern
            float angleStep = math.PI * 2.0f / VelocitySamples;

            for (int s = 0; s < VelocitySamples; s++)
            {
                float angle = angleStep * s;
                float2 dir = new float2(math.cos(angle), math.sin(angle));

                // Try multiple speeds: 50%, 75%, 100%
                for (float speedMult = 0.5f; speedMult <= 1.0f; speedMult += 0.25f)
                {
                    float2 testVel = dir * maxSpeed * speedMult;
                    float cost = CalculateVelocityCost(i, testVel);

                    if (cost < bestCost)
                    {
                        bestCost = cost;
                        bestVelocity = testVel;
                    }
                }
            }

            // Also try standing still if things are crowded
            float stillCost = CalculateVelocityCost(i, float2.zero);
            if (stillCost < bestCost)
            {
                bestVelocity = float2.zero;
            }

            NewVelocities[i] = bestVelocity;
        }

        float CalculateVelocityCost(int agentIndex, float2 testVelocity)
        {
            float2 position = Positions[agentIndex];
            float radius = Radii[agentIndex];
            float2 desiredVel = DesiredVelocities[agentIndex];
            int priority = Priorities[agentIndex];

            float cost = 0f;

            // Cost 1: Deviation from desired velocity
            float2 desiredDiff = testVelocity - desiredVel;
            cost += math.lengthsq(desiredDiff) * 1.0f;

            // Cost 2: Check collisions with all other agents
            for (int j = 0; j < Positions.Length; j++)
            {
                if (j == agentIndex) continue;

                float2 otherPos = Positions[j];
                float2 otherVel = Velocities[j];
                float otherRadius = Radii[j];
                int otherPriority = Priorities[j];

                // Quick distance check
                float2 relPos = otherPos - position;
                float distSq = math.lengthsq(relPos);

                float maxDist = AvoidanceHorizon * math.max(MaxSpeeds[agentIndex], MaxSpeeds[j]) + radius + otherRadius;
                if (distSq > maxDist * maxDist)
                    continue;

                // Predict collision using relative velocity
                float2 relVel = testVelocity - otherVel;
                float relSpeed = math.length(relVel);

                if (relSpeed < 0.001f)
                {
                    // Agents moving together - only penalize if too close
                    float dist = math.sqrt(distSq);
                    float minSeparation2 = (radius + otherRadius) * SeparationWeight;

                    if (dist < minSeparation2)
                    {
                        float penetration = minSeparation2 - dist;
                        cost += penetration * penetration * 100.0f;
                    }
                    continue;
                }

                // Time to closest approach
                float timeToClosest = -math.dot(relPos, relVel) / (relSpeed * relSpeed);
                timeToClosest = math.clamp(timeToClosest, 0f, AvoidanceHorizon);

                // Predicted future positions
                float2 futurePos = position + testVelocity * timeToClosest;
                float2 futureOther = otherPos + otherVel * timeToClosest;
                float2 futureDist = futurePos - futureOther;
                float futureDistMag = math.length(futureDist);

                float minSeparation = (radius + otherRadius) * SeparationWeight;

                if (futureDistMag < minSeparation)
                {
                    // Collision predicted!
                    float penetration = minSeparation - futureDistMag;

                    // Earlier collisions are worse
                    float urgency = 1.0f / (timeToClosest + 0.1f);

                    // Priority: lower priority agents should avoid more
                    float priorityFactor = ComputeAvoidanceFactor(priority, otherPriority);

                    cost += penetration * urgency * priorityFactor * 100.0f;
                }
            }

            return cost;
        }

        float ComputeAvoidanceFactor(int myPriority, int otherPriority)
        {
            // 0 = highest priority (should avoid less)
            // 99 = lowest priority (should avoid more)
            // Returns a multiplier: higher = I should avoid more

            float myWeight = 1f / (1f + myPriority);
            float otherWeight = 1f / (1f + otherPriority);
            float sum = myWeight + otherWeight;

            if (sum < 0.001f) return 1.0f;

            // My share of the avoidance burden (0.0 to 2.0, typically around 1.0)
            return (myWeight / sum) * 2.0f;
        }
    }

    [BurstCompile]
    partial struct ApplyAvoidanceJob : IJobEntity
    {
        [ReadOnly] public NativeArray<float2> NewVelocities;
        public float VelocitySmoothing;
        public float TimeStep;

        void Execute(
            [EntityIndexInQuery] int index,
            ref SimLocalTransform simTransform,
            ref AvoidanceVelocity avoidVel,
            ref DesiredVelocity desiredVel)
        {
            float2 computedVelocity = NewVelocities[index];

            // Smooth the velocity change to prevent jitter
            avoidVel.Value = math.lerp(avoidVel.Value, computedVelocity, VelocitySmoothing);

            // Update desired velocity to match avoidance velocity
            desiredVel.Value = avoidVel.Value;

            // Apply movement
            float3 velocity3D = new float3(avoidVel.Value.x, 0, avoidVel.Value.y);
            simTransform.Value.Position += velocity3D * TimeStep;

            // Update rotation to face movement direction
            if (math.lengthsq(avoidVel.Value) > 0.01f)
            {
                float2 forward = math.normalize(avoidVel.Value);
                float angle = math.atan2(forward.x, forward.y);
                simTransform.Value.Rotation = quaternion.RotateY(angle);
            }
        }
    }
}