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

[BurstCompile]
[UpdateInGroup(typeof(PresentationSystemGroup))]
public partial struct DrawCollidersSystem : ISystem
{
    public void OnUpdate(ref SystemState state)
    {
        var builder = DrawingManager.GetBuilder(true);

        builder.Preallocate(10000);

        foreach (var (ellipse, simTransform) in SystemAPI.Query<RefRO<AgentComponent>, RefRO<SimLocalTransform>>())
        {
            var t = simTransform.ValueRO.Value;
            //builder.PushMatrix(float4x4.TRS(t.Position, t.Rotation, new float3(ellipse.ValueRO.Radius, 1, ellipse.ValueRO.Radius)));
            builder.xz.Circle(t.Position.xz, ellipse.ValueRO.Radius);
            //builder.PopMatrix();
        }

        builder.Dispose();
    }
}
