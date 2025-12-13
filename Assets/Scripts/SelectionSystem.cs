using Drawing;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;

[BurstCompile]
[UpdateInGroup(typeof(SimulationSystemGroup))]
public partial struct SelectionSystemSystem : ISystem
{
    public NativeList<Entity> highlightedEntities;
    public NativeList<Entity> selectedEntities;

    public float2 selectionStart;
    public float2 selectionEnd;

    [BurstCompile]
    public void OnCreate(ref SystemState state)
    {
        highlightedEntities = new NativeList<Entity>(10000, Allocator.Persistent);
        selectedEntities = new NativeList<Entity>(10000, Allocator.Persistent);

        selectionStart = 0;
        selectionEnd = 0;
    }

    [BurstCompile]
    public void OnDestroy(ref SystemState state) 
    { 
        highlightedEntities.Dispose(); 
        selectedEntities.Dispose(); 
    }

    public void OnUpdate(ref SystemState state)
    {
        var mousePosition = GetMousePosition();

        if (Input.GetMouseButtonDown(0))
        {
            selectionStart = mousePosition;
        }

        if (Input.GetMouseButton(0))
        {
            selectionEnd = mousePosition;

            ScheduleHighlightJob(ref state);
        }

        if (Input.GetMouseButtonUp(0))
        {
            selectionEnd = mousePosition;

            ScheduleHighlightJob(ref state);
        }

        state.Dependency = new ClearHighlightJob
        {
            highlightedEntities = highlightedEntities,
            selectedEntities = selectedEntities,
            confirmSelection = Input.GetMouseButtonUp(0),
        }.Schedule(state.Dependency);

        if(Input.GetMouseButtonDown(1))
        {
            state.Dependency = new SetDestinationJob
            {
                selectedEntities = selectedEntities,
                destinationLookup = SystemAPI.GetComponentLookup<DestinationComponent>(),
                target = mousePosition,
            }.Schedule(state.Dependency);
        }

        state.Dependency.Complete();
    }

    private float2 GetMousePosition()
    {
        var target = Camera.main.ScreenToWorldPoint(Input.mousePosition);
        return new float2(target.x, target.z);
    }

    private void ScheduleHighlightJob(ref SystemState state)
    {
        var min = math.min(selectionStart, selectionEnd);
        var max = math.max(selectionStart, selectionEnd);

        if (math.distance(min, max) < 0.01f)
            return;

        state.Dependency = new HighlightJob
        {
            min = min,
            max = max,
            highlightedEntities = highlightedEntities.AsParallelWriter(),
        }.ScheduleParallel(state.Dependency);
    }

    [BurstCompile]
    public partial struct HighlightJob : IJobEntity
    {
        public float2 min;
        public float2 max;

        public NativeList<Entity>.ParallelWriter highlightedEntities;

        public void Execute(Entity entity, in LocalTransform localTransform, ref AgentComponent agent)
        {
            var position = localTransform.Position.xz;

            if (math.all(min < position) & math.all(max > position))
            {
                highlightedEntities.AddNoResize(entity);
            }
        }
    }

    [BurstCompile]
    public struct ClearHighlightJob : IJob
    {
        public bool confirmSelection;

        public NativeList<Entity> highlightedEntities;
        [WriteOnly] public NativeList<Entity> selectedEntities;

        public void Execute()
        {
            if(confirmSelection)
            {
                selectedEntities.Clear();
                selectedEntities.AddRange(highlightedEntities.AsArray());
            }

            highlightedEntities.Clear();
        }
    }

    [BurstCompile]
    public struct SetDestinationJob : IJob
    {
        public float2 target;

        [ReadOnly] public NativeList<Entity> selectedEntities;

        public ComponentLookup<DestinationComponent> destinationLookup;

        public void Execute()
        {
            for (int i = 0; i < selectedEntities.Length; i++)
            {
                var destination = destinationLookup.GetRefRW(selectedEntities[i]);

                destination.ValueRW.Value = target;
            }
        }
    }

    [BurstCompile]
    public partial struct PopulateHardForceGridData : IJobEntity
    {
        [ReadOnly] public SpatialGridProperties gridProperties;

        [ReadOnly] public NativeArray<int> cellOffsets;

        [NativeDisableParallelForRestriction]
        [WriteOnly] public NativeArray<AgentSpatialData> spatialData;

        public void Execute(in SimLocalTransform simTranslation, ref AgentComponent agent)
        {
            var agentSpatialData = new AgentSpatialData
            {
                Position = simTranslation.Value.Position.xz,
                Forward = math.normalize(simTranslation.Value.Forward().xz),
                Length = agent.Length,
                Radius = agent.Radius,
                Priority = 50,
                CorectionOffset = 0,
                CorrectionCount = 0,
            };

            agent.GetMinMax(simTranslation.Value.Position.xz, math.normalize(simTranslation.Value.Forward().xz), out var agentMinimum, out var agentMaximum);
            var minCell = gridProperties.GetCellKeyClamped(agentMinimum);
            var maxCell = gridProperties.GetCellKeyClamped(agentMaximum);

            //var min = simTranslation.Value.Position.xz - agent.Radius;
            //var max = simTranslation.Value.Position.xz + agent.Radius;

            //var minCell = gridProperties.GetCellKeyClamped(min);
            //var maxCell = gridProperties.GetCellKeyClamped(max);

            int i = 0;

            for (int y = minCell.y; y <= maxCell.y; y++)
            {
                for (int x = minCell.x; x <= maxCell.x; x++)
                {
                    var cellIndex = gridProperties.GetCellIndex(new int2(x, y));
                    var cellOffset = cellOffsets[cellIndex];

                    if (i + 1 >= agent.Indicies.Capacity)
                    {
                        UnityEngine.Debug.LogError("We hit max capacity, we should not be able to do this...");
                        continue;
                    }

                    var dataIndex = agent.Indicies[i] + cellOffset;

                    agent.Indicies[i++] = dataIndex;

                    spatialData[dataIndex] = agentSpatialData;
                }
            }
        }
    }

    [BurstCompile]
    public struct ProcessHardForces : IJobParallelForDefer
    {
        public SpatialGridProperties gridProperties;

        public float SeparationBias;

        [ReadOnly] public NativeArray<int> cellCounts;
        [ReadOnly] public NativeArray<int> cellOffsets;
        [ReadOnly] public NativeArray<int> activeCells;

        [NativeDisableParallelForRestriction] public NativeArray<AgentSpatialData> spatialData;

        private static float ComputeMoveFraction(float priA, float priB)
        {
            // 0 = highest priority -> weight smaller
            float wA = 1f / (1f + priA);
            float wB = 1f / (1f + priB);
            float sum = wA + wB;
            return sum > 0f ? wA / sum : 0.5f;
        }

        private static (int2, int2) Intersection(int2 minA, int2 maxA, int2 minB, int2 maxB)
        {
            return (math.max(minA, minB), math.min(maxA, maxB));
        }

        public unsafe void Execute(int index)
        {
            // TODO:! We need to sort by priority. So we apply the positions to the important ones first? Although would this actually make a difference?

            var cellIndex = activeCells[index];

            var data = spatialData.GetSubArray(cellOffsets[cellIndex], cellCounts[cellIndex]);

            for (int i = 0; i < data.Length; i++)
            {
                ref var agentA = ref UnsafeUtility.ArrayElementAsRef<AgentSpatialData>(data.GetUnsafePtr(), i);

                agentA.GetMinMax(out var agentAMinimum, out var agentAMaximum);
                var agentAMin = gridProperties.GetCellKeyClamped(agentAMinimum);
                var agentAMax = gridProperties.GetCellKeyClamped(agentAMaximum);

                for (int j = i + 1; j < data.Length; j++)
                {
                    ref var agentB = ref UnsafeUtility.ArrayElementAsRef<AgentSpatialData>(data.GetUnsafePtr(), j);

                    // TODO: These can be pre-calculated
                    agentB.GetMinMax(out var agentBMinimum, out var agentBMaximum);
                    var agentBMin = gridProperties.GetCellKeyClamped(agentBMinimum);
                    var agentBMax = gridProperties.GetCellKeyClamped(agentBMaximum);

                    bool isBottomIntersection = cellIndex != gridProperties.GetCellIndex(Intersection(agentAMin, agentAMax, agentBMin, agentBMax).Item1);

                    // Remove duplicates.
                    if (isBottomIntersection)
                    {
                        continue;
                    }

                    // TODO: Do broad phase check!
                    //float2 delta = agentA.Position - agentB.Position;
                    //float distSq = math.lengthsq(delta);

                    //float minDist = (agentA.Radius + agentB.Radius) * SeparationBias;
                    //float minDistSq = minDist * minDist;

                    //// Broad phase circle.
                    //if (distSq >= minDistSq)
                    //    continue;

                    Capsule2D A = new Capsule2D
                    {
                        position = agentA.Position,
                        forward = agentA.Forward,
                        halfLength = agentA.Length * 0.5f,
                        radius = agentA.Radius * SeparationBias,
                    };
                    Capsule2D B = new Capsule2D
                    {
                        position = agentB.Position,
                        forward = agentB.Forward,
                        halfLength = agentB.Length * 0.5f,
                        radius = agentB.Radius * SeparationBias,
                    };

                    var collision = CapsuleCollision.CheckCollision(A, B);

                    // TODO: If they are right on top do stable direction.
                    //normal = math.normalize(new float2(0.73f, 0.68f)); // arbitrary stable dir
                    if (collision.colliding)
                    {
                        float2 normal = collision.normal;
                        float penetration = collision.penetrationDepth;

                        float moveFracA = ComputeMoveFraction(agentA.Priority, agentB.Priority);
                        float moveFracB = ComputeMoveFraction(agentB.Priority, agentA.Priority);
                        float2 corrA = normal * (penetration * moveFracA);
                        float2 corrB = -normal * (penetration * moveFracB);

                        agentA.PenetrationSum += penetration;
                        agentA.CorectionOffset += corrA;
                        agentA.CorrectionCount += 1;

                        agentB.PenetrationSum += penetration;
                        agentB.CorectionOffset += corrB;
                        agentB.CorrectionCount += 1;
                    }
                }
            }
        }
    }

    [BurstCompile]
    public partial struct IJobEntity_WriteHardForces : IJobEntity
    {
        [ReadOnly] public NativeArray<AgentSpatialData> spatialData;

        // The amount the colliders can expand per tick.
        public float CrowdingFactorChangeSpeed;
        public float Iterations;

        public void Execute(ref SimLocalTransform simLocalTransform, ref AgentComponent agent)
        {
            float2 corectionOffset = 0;
            int correctionCount = 0;
            float penetrationSum = 0;

            for (int i = 0; i < agent.Indicies.Length; i++)
            {
                var index = agent.Indicies[i];

                var data = spatialData[index];

                corectionOffset += data.CorectionOffset;
                correctionCount += data.CorrectionCount;

                penetrationSum += data.PenetrationSum;
            }

            agent.Indicies.Clear();

            var pastCrowdingScore = agent.CrowdingFactor;

            if (correctionCount > 0)
            {
                // Calculate relative penetration (compared to capsule size)
                // Should change this from penetartion to area ratios.
                float capsuleSize = agent.Length + agent.Radius * 2;
                float relativePenetration = penetrationSum / capsuleSize;

                // TODO: Make this a value on the agent.
                // NOTE: We can also scale this by the number of collisions. This way, if we only collide twice, we will not scale.
                agent.CrowdingFactor = math.lerp(1, 1.4f, math.saturate(relativePenetration));

                corectionOffset /= correctionCount;

                simLocalTransform.Value.Position += new float3(corectionOffset.x, 0, corectionOffset.y);
            }

            agent.CrowdingFactor = math.lerp(pastCrowdingScore, agent.CrowdingFactor, CrowdingFactorChangeSpeed / Iterations);
        }
    }








    //// We'll embed some helpers into job structs below

    //// 1) Job: rebuild spatial grid (positions -> cells)
    //[BurstCompile]
    //struct BuildGridJob : IJobParallelFor
    //{
    //    [ReadOnly] public NativeArray<float3> Positions;
    //    [ReadOnly] public float CellSize;

    //    public NativeParallelMultiHashMap<int, int>.ParallelWriter CellMap;

    //    public void Execute(int index)
    //    {
    //        float3 pos = Positions[index];

    //        int2 cell = new int2(
    //            (int)math.floor(pos.x / CellSize),
    //            (int)math.floor(pos.z / CellSize)
    //        );

    //        int key = Hash(cell);
    //        CellMap.Add(key, index);
    //    }
    //}

    //// 2) Job: positional overlap resolution (Jacobi style)
    //[BurstCompile]
    //struct ResolveOverlapsJob : IJobParallelFor
    //{
    //    [ReadOnly] public NativeArray<AgentData> Agents;
    //    [ReadOnly] public NativeArray<float3> PositionsRead;
    //    [ReadOnly] public NativeArray<int> SortedIndices;
    //    [ReadOnly] public NativeParallelMultiHashMap<int, int> CellMap;

    //    public NativeArray<float3> PositionsWrite;

    //    public float CellSize;
    //    public float SeparationBias;

    //    public void Execute(int sortedIndexIndex)
    //    {
    //        // Work in priority order, but parallel; we index into sortedIndices
    //        int idxA = SortedIndices[sortedIndexIndex];
    //        var a = Agents[idxA];

    //        float3 posA = PositionsRead[idxA];

    //        float3 correctionSum = float3.zero;
    //        int correctionCount = 0;

    //        int2 cellA = new int2(
    //            (int)math.floor(posA.x / CellSize),
    //            (int)math.floor(posA.z / CellSize)
    //        );

    //        for (int dy = -1; dy <= 1; dy++)
    //        {
    //            for (int dx = -1; dx <= 1; dx++)
    //            {
    //                int2 cell = cellA + new int2(dx, dy);
    //                int key = Hash(cell);

    //                if (!CellMap.TryGetFirstValue(key, out int idxB, out var it))
    //                    continue;

    //                do
    //                {
    //                    if (idxB == idxA)
    //                        continue;

    //                    var b = Agents[idxB];

    //                    float3 posB = PositionsRead[idxB];

    //                    float3 delta = posA - posB;
    //                    float distSq = math.lengthsq(delta);

    //                    float minDist = (a.Radius + b.Radius) * SeparationBias;
    //                    float minDistSq = minDist * minDist;

    //                    if (distSq >= minDistSq)
    //                        continue;

    //                    float dist = math.sqrt(distSq);

    //                    float3 normal;
    //                    if (dist > 1e-4f)
    //                        normal = delta / dist;
    //                    else
    //                        normal = math.normalize(new float3(0.73f, 0f, 0.68f)); // arbitrary stable dir

    //                    float penetration = minDist - dist;

    //                    float moveFracA = ComputeMoveFraction(a.Priority, b.Priority);
    //                    float3 corrA = normal * (penetration * moveFracA);

    //                    correctionSum += corrA;
    //                    correctionCount += 1;

    //                } while (CellMap.TryGetNextValue(out idxB, ref it));
    //            }
    //        }

    //        float3 newPos = posA;
    //        if (correctionCount > 0)
    //        {
    //            float3 avgCorrection = correctionSum / correctionCount;
    //            newPos += avgCorrection;
    //        }

    //        PositionsWrite[idxA] = newPos;
    //    }

    //    static float ComputeMoveFraction(int priA, int priB)
    //    {
    //        // 0 = highest priority -> weight smaller
    //        float wA = 1f / (1f + (float)priA);
    //        float wB = 1f / (1f + (float)priB);
    //        float sum = wA + wB;
    //        return sum > 0f ? wA / sum : 0.5f;
    //    }
    //}

    //// 3) Job: write back to LocalTransform
    //[BurstCompile]
    //struct WriteBackJob : IJobParallelFor
    //{
    //    [ReadOnly] public NativeArray<AgentData> Agents;
    //    [ReadOnly] public NativeArray<float3> FinalPositions;

    //    [NativeDisableParallelForRestriction]
    //    public ComponentLookup<SimLocalTransform> TransformLookup;

    //    public void Execute(int index)
    //    {
    //        var data = Agents[index];
    //        var t = TransformLookup[data.Entity];
    //        t.Value.Position = FinalPositions[index];
    //        TransformLookup[data.Entity] = t;
    //    }
    //}

}
