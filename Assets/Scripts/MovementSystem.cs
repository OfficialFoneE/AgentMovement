using Drawing;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;


public struct CollisionEllipse : IComponentData
{
    /// <summary>Base radii in X (width) and Z (depth).</summary>
    public float2 Radii; // (radiusX, radiusZ)
}

public struct CollisionPriority : IComponentData
{
    /// <summary>
    /// Higher priority = moves LESS when resolving collisions.
    /// e.g. Tank = 5, Marine = 1, Zergling = 0.5
    /// </summary>
    public float Value;
}

public struct DesiredVelocity : IComponentData
{
    /// <summary>Desired velocity in XZ plane for this frame.</summary>
    public float2 Value;
}

/// <summary>
/// Global tuning parameters for separation. Add as a singleton entity.
/// </summary>
public struct EllipseSeparationParams : IComponentData
{
    /// <summary>Global strength of separation push.</summary>
    public float SeparationStrength; // e.g. 1.0–4.0

    /// <summary>
    /// If > 0, damp velocity when in crowded area (jiggle reduction).
    /// </summary>
    public float CrowdDampingFactor; // e.g. 0.8f

    /// <summary>
    /// Neighbor count above which we consider the unit "crowded".
    /// </summary>
    public int CrowdNeighborThreshold; // e.g. 4

    /// <summary>
    /// Suppress backward push relative to desired velocity
    /// (1 = full suppression, 0 = none).
    /// </summary>
    public float BackwardSuppression; // e.g. 0.7f

    /// <summary>
    /// Max distance in XZ we consider for neighbor checks (cheap cull).
    /// </summary>
    public float MaxNeighborDistance; // e.g. 5–10 depending on unit sizes
}

[BurstCompile]
[UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
public partial struct EllipticalSeparationSystem : ISystem
{
    private EntityQuery _unitQuery;

    public void OnCreate(ref SystemState state)
    {
        _unitQuery = state.GetEntityQuery(new EntityQueryDesc
        {
            All = new[]
            {
                ComponentType.ReadWrite<SimLocalTransform>(),
                ComponentType.ReadWrite<DesiredVelocity>(),
                ComponentType.ReadOnly<CollisionEllipse>(),
                ComponentType.ReadOnly<CollisionPriority>()
            }
        });

        state.RequireForUpdate<EllipseSeparationParams>();
    }

    public void OnDestroy(ref SystemState state)
    {
    }

    //[BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        var builder = DrawingManager.GetBuilder(true);

        builder.Preallocate(10000);

        foreach (var (pastSimTransform, simTransform) in SystemAPI.Query<RefRW<PastSimLocalTransform>, RefRO<SimLocalTransform>>())
        {
            pastSimTransform.ValueRW.Value = simTransform.ValueRO.Value;
        }

        if (false)
        {
            builder.DiscardAndDispose();
            return;
        }

        foreach (var (desiredVelocity, simLocalTransform, maxSpeed, destination) in SystemAPI.Query<RefRW<DesiredVelocity>, RefRO<SimLocalTransform>, RefRO<MaxSpeedComponent>, RefRO<DestinationComponent>>())
        {
            var direction = math.normalizesafe(destination.ValueRO.Value - simLocalTransform.ValueRO.Value.Position.xz);

            var distance = math.distance(destination.ValueRO.Value, simLocalTransform.ValueRO.Value.Position.xz);
            if (distance < 0.01f)
            {
                distance = 0;
            }

            desiredVelocity.ValueRW.Value = direction * math.min(maxSpeed.ValueRO.Value, distance);
        }

        int count = _unitQuery.CalculateEntityCount();
        if (count == 0)
            return;

        var entities = _unitQuery.ToEntityArray(Allocator.TempJob);
        var transforms = _unitQuery.ToComponentDataArray<SimLocalTransform>(Allocator.TempJob);
        var ellipses = _unitQuery.ToComponentDataArray<CollisionEllipse>(Allocator.TempJob);
        var priorities = _unitQuery.ToComponentDataArray<CollisionPriority>(Allocator.TempJob);
        var desiredVel = _unitQuery.ToComponentDataArray<DesiredVelocity>(Allocator.TempJob);

        var separationOutput = new NativeArray<float2>(count, Allocator.TempJob);
        var neighborCounts = new NativeArray<int>(count, Allocator.TempJob);

        var parms = SystemAPI.GetSingleton<EllipseSeparationParams>();
        float dt = SystemAPI.Time.DeltaTime;

        // Job: compute separation vectors in array space (index-based).
        var sepJob = new EllipseSeparationJob
        {
            DeltaTime = dt,
            SeparationParams = parms,
            Positions = transforms,
            Ellipses = ellipses,
            Priorities = priorities,
            DesiredVel = desiredVel,
            SeparationOut = separationOutput,
            NeighborCounts = neighborCounts,
            commandBuilder = builder,
        };

        // This is IJob (single-threaded) for simplicity because it's O(N²).
        // You *can* make this parallel with some care, but for clarity we keep it simple.
        sepJob.Run();

        // Apply results back to ECS components.
        var transformLookup = state.GetComponentLookup<SimLocalTransform>(false);
        var velLookup = state.GetComponentLookup<DesiredVelocity>(false);

        for (int i = 0; i < count; i++)
        {
            Entity e = entities[i];

            SimLocalTransform t = transformLookup[e];
            DesiredVelocity dv = velLookup[e];

            float2 separation = separationOutput[i];
            int neighbors = neighborCounts[i];

            // Add separation steering.
            float2 finalVel = dv.Value + separation * parms.SeparationStrength;

            // Optional damping if crowded.
            if (neighbors >= parms.CrowdNeighborThreshold)
            {
                finalVel *= parms.CrowdDampingFactor;
            }

            // Write back velocity.
            dv.Value = finalVel;
            velLookup[e] = dv;

            // Move the unit (XZ only, keep Y).
            float3 pos = t.Value.Position;
            pos.x += finalVel.x * dt;
            pos.z += finalVel.y * dt;
            t.Value.Position = pos;

            var degrees = math.degrees(SignedAngle2D(new float2(0, 1), finalVel));
            t.Value.Rotation = quaternion.AxisAngle(math.up(), math.radians(degrees));

            transformLookup[e] = t;
        }

        entities.Dispose();
        transforms.Dispose();
        ellipses.Dispose();
        priorities.Dispose();
        desiredVel.Dispose();
        separationOutput.Dispose();
        neighborCounts.Dispose();

        builder.Dispose();
    }

    public static float Cross(float2 a, float2 b)
    {
        return a.x * b.y - a.y * b.x;
    }

    public static float SignedAngle2D(float2 a, float2 b)
    {
        var cross = Cross(a, b);
        var angle = math.atan2(math.abs(cross), math.mul(a, b));

        return math.sign(cross) > 0f ? angle * -1f : angle;
    }

    /// <summary>
    /// Computes pairwise separation vectors for units with elliptical collision,
    /// per-unit collision priorities, projected movement, and tangential bias.
    /// </summary>
    [BurstCompile]
    private struct EllipseSeparationJob : IJob
    {
        public float DeltaTime;
        public EllipseSeparationParams SeparationParams;

        [ReadOnly] public NativeArray<SimLocalTransform> Positions;
        [ReadOnly] public NativeArray<CollisionEllipse> Ellipses;
        [ReadOnly] public NativeArray<CollisionPriority> Priorities;
        [ReadOnly] public NativeArray<DesiredVelocity> DesiredVel;

        [WriteOnly] public NativeArray<float2> SeparationOut;
        [WriteOnly] public NativeArray<int> NeighborCounts;

        public CommandBuilder commandBuilder;

        public void Execute()
        {
            int count = Positions.Length;

            float maxSqNeighborDist = SeparationParams.MaxNeighborDistance *
                                      SeparationParams.MaxNeighborDistance;
            float backwardSuppression = SeparationParams.BackwardSuppression;

            for (int i = 0; i < count; i++)
            {
                float3 posI3 = Positions[i].Value.Position;
                float2 posI = new float2(posI3.x, posI3.z);

                float2 velI = DesiredVel[i].Value;
                float2 futurePosI = posI + velI * DeltaTime;

                float2 radiiI = Ellipses[i].Radii;
                float priorityI = math.max(0.0001f, Priorities[i].Value);

                float2 separationAccum = float2.zero;
                int neighborCount = 0;

                for (int j = 0; j < count; j++)
                {
                    if (j == i) continue;

                    float3 posJ3 = Positions[j].Value.Position;
                    float2 posJ = new float2(posJ3.x, posJ3.z);

                    float2 velJ = DesiredVel[j].Value;
                    float2 futurePosJ = posJ + velJ * DeltaTime;

                    float2 radiiJ = Ellipses[j].Radii;
                    float priorityJ = math.max(0.0001f, Priorities[j].Value);

                    // Quick cull by squared distance in world space.
                    float2 worldDelta = futurePosJ - futurePosI;
                    float distSq = math.lengthsq(worldDelta);
                    if (distSq > maxSqNeighborDist)
                        continue;

                    // Combined ellipse radii (Minkowski sum).
                    float2 combinedRadii = radiiI + radiiJ;

                    // If radii are degenerate, skip.
                    if (combinedRadii.x <= 0.0f || combinedRadii.y <= 0.0f)
                        continue;

                    // Normalized offset in ellipse space.
                    float2 dNorm = worldDelta / combinedRadii; // element-wise division
                    float lambda = math.length(dNorm);         // distance in ellipse metric

                    if (lambda >= 1.0f)
                        continue; // no overlap at future positions

                    neighborCount++;

                    // How deep inside: 1 - lambda (0..1)
                    float penetration01 = 1.0f - lambda;

                    // Gradient direction for ellipse: grad ~ (x/a^2, y/b^2)
                    // We use worldDelta with combinedRadii squared.
                    float2 grad = new float2(
                        worldDelta.x / (combinedRadii.x * combinedRadii.x),
                        worldDelta.y / (combinedRadii.y * combinedRadii.y)
                    );

                    float gradLen = math.length(grad);
                    if (gradLen < 1e-6f)
                        continue;

                    float2 dir = grad / gradLen; // normalized separation direction (from i to j)

                    // Base push magnitude (you’ll tune this with SeparationStrength later).
                    // We scale by average radius so bigger units push more.
                    float avgRadius = 0.5f * (combinedRadii.x + combinedRadii.y);
                    float basePush = penetration01 * avgRadius;

                    // Distribute push asymmetrically using priority.
                    // Higher priority = moves less.
                    // weight = 1 / priority so bigger priority => smaller weight.
                    float weightI = 1.0f / priorityI;
                    float weightJ = 1.0f / priorityJ;
                    float weightSum = weightI + weightJ;

                    // How much of the overlap i is responsible for:
                    // fraction of weight that belongs to i.
                    float responsibilityI = weightI / weightSum;
                    // direction of push for i must be AWAY from j:
                    float2 pushI = -dir * (basePush * responsibilityI);

                    // Tangential bias: suppress backward push relative to desired velocity.
                    float2 velIWorld = velI;
                    float velLen = math.length(velIWorld);
                    if (velLen > 1e-5f)
                    {
                        float2 velDir = velIWorld / velLen;
                        // Component of pushI opposite to movement:
                        float backDot = math.dot(pushI, -velDir);
                        if (backDot > 0f)
                        {
                            // Remove a portion of backward component.
                            float2 backComp = -velDir * backDot;
                            pushI -= backComp * backwardSuppression;
                        }
                    }

                    //commandBuilder.xz.Line(posI, posJ);

                    separationAccum += pushI;
                }

                SeparationOut[i] = separationAccum;
                NeighborCounts[i] = neighborCount;
            }
        }
    }
}
