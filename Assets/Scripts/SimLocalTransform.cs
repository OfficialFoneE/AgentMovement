using Unity.Burst;
using Unity.Entities;

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
