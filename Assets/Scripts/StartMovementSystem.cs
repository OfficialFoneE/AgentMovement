using Unity.Burst;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;

[BurstCompile]
[UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
[UpdateBefore(typeof(EndMovementSystem))]
public partial struct StartMovementSystem : ISystem
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
    }
}
