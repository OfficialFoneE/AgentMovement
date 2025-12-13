using Unity.Burst;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;

[BurstCompile]
[UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
public partial struct EndMovementSystem : ISystem
{
    [BurstCompile]
    public void OnCreate(ref SystemState state) { }

    [BurstCompile]
    public void OnDestroy(ref SystemState state) { }

    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        var delatTime = SystemAPI.Time.DeltaTime;

        // Apply unit movement.
        foreach (var (desiredVelocity, simLocalTransform) in SystemAPI.Query<RefRW<DesiredVelocity>, RefRW<SimLocalTransform>>())
        {
            var velocity2D = desiredVelocity.ValueRO.Value;
            var velocity3D = new float3(velocity2D.x, 0, velocity2D.y);
            simLocalTransform.ValueRW.Value.Position += velocity3D * delatTime;

            // Only rotate if we’re actually moving (prevents twitch).
            if (math.lengthsq(velocity3D) > 1e-6f)
                simLocalTransform.ValueRW.Value.Rotation = quaternion.LookRotation(math.normalize(velocity3D), math.up());
        }
    }
}
