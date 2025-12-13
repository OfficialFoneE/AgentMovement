using Unity.Burst;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;

[BurstCompile]
[UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
public partial struct MovementSystem : ISystem
{
    [BurstCompile]
    public void OnCreate(ref SystemState state) { }

    [BurstCompile]
    public void OnDestroy(ref SystemState state) { }

    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        foreach (var (pastSimTransform, simTransform) in SystemAPI.Query<RefRW<PastSimLocalTransform>, RefRO<SimLocalTransform>>())
        {
            pastSimTransform.ValueRW.Value = simTransform.ValueRO.Value;
        }

        // Update desired velocity.
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

        var delatTime = SystemAPI.Time.DeltaTime;

        // TODO:
        // Run proper avoidance and seperation!

        // Apply unit movement.
        foreach (var (desiredVelocity, simLocalTransform) in SystemAPI.Query<RefRW<DesiredVelocity>, RefRW<SimLocalTransform>>())
        {
            var velocity2D = desiredVelocity.ValueRO.Value;
            var velocity3D = new float3(velocity2D.x, 0, velocity2D.y);
            simLocalTransform.ValueRW.Value.Position += velocity3D * delatTime;
            simLocalTransform.ValueRW.Value.Rotation = quaternion.LookRotation(math.normalizesafe(velocity3D), math.up());
        }
    }
}
