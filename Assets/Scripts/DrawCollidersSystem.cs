using Unity.Burst;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using Drawing;

[BurstCompile]
[UpdateInGroup(typeof(PresentationSystemGroup))]
public partial struct DrawCollidersSystem : ISystem
{
    public void OnUpdate(ref SystemState state)
    {
        if (Boostrap.Instance.DrawRadiuses == false)
            return;

        var builder = DrawingManager.GetBuilder(true);

        builder.Preallocate(10000);

        foreach (var (ellipse, simTransform) in SystemAPI.Query<RefRO<AgentComponent>, RefRO<SimLocalTransform>>())
        {
            var p1 = simTransform.ValueRO.Value.Position.xz - math.normalize(simTransform.ValueRO.Value.Forward().xz) * ellipse.ValueRO.Length * 0.5f;
            var p2 = simTransform.ValueRO.Value.Position.xz + math.normalize(simTransform.ValueRO.Value.Forward().xz) * ellipse.ValueRO.Length * 0.5f;
            builder.xz.WirePill(p1, p2, ellipse.ValueRO.BaseRadius, UnityEngine.Color.cyan);
            builder.xz.WirePill(p1, p2, ellipse.ValueRO.Radius, UnityEngine.Color.blue);
        }

        builder.Dispose();
    }
}
