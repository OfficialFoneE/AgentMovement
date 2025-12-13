using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

public struct AgentComponent : IComponentData
{
    public float Length;      // Semi-axis along forward
    public float BaseRadius;  // Semi-axis perpendicular to forward

    public float CrowdingFactor;
    public float Radius => BaseRadius * CrowdingFactor;

    /// 0 = highest priority (moves least), 99 = lowest priority (moves most)
    public int AvoidancePriority;

    public FixedList128Bytes<int> Indicies;

    public void GetMinMax(float2 position, float2 forward, out float2 min, out float2 max)
    {
        var p1 = position - forward * Length;
        var p2 = position + forward * Length;
        min = math.min(p1 - Radius, p2 - Radius);
        max = math.max(p1 + Radius, p2 + Radius);
    }
}

public struct MaxSpeedComponent : IComponentData
{
    public float Value;
}

public struct DesiredVelocity : IComponentData
{
    /// <summary>Desired velocity in XZ plane for this frame.</summary>
    public float2 Value;
}

public struct DestinationComponent : IComponentData
{
    public float2 Value;
}

namespace Unity.Transforms
{
    public struct SimLocalTransform : IComponentData
    {
        public LocalTransform Value;
    }

    public struct PastSimLocalTransform : IComponentData
    {
        public LocalTransform Value;
    }
}
