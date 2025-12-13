using Unity.Burst;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using Drawing;

[BurstCompile]
[UpdateInGroup(typeof(SimulationSystemGroup))]
[UpdateBefore(typeof(LocalToWorldSystem))]
public partial struct InterpolationSystem : ISystem
{
    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        var tickDuration = math.saturate(math.fmod(SystemAPI.Time.ElapsedTime, 0.1) / 0.1);

        foreach (var (transform, pastSimTransform, simTransform) in SystemAPI.Query<RefRW<LocalTransform>, RefRO<PastSimLocalTransform>, RefRO<SimLocalTransform>>())
        {
            transform.ValueRW.Position = math.lerp(pastSimTransform.ValueRO.Value.Position, simTransform.ValueRO.Value.Position, (float)tickDuration);
            transform.ValueRW.Rotation = math.slerp(pastSimTransform.ValueRO.Value.Rotation, simTransform.ValueRO.Value.Rotation, (float)tickDuration);
        }
    }
}
