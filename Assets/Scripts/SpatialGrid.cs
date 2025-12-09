using Drawing;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;


public struct SpatialGridProperties
{
    public int gridSize;
    public int cellCount;
    public int cellSize;
    private float inverseCellSize;

    public SpatialGridProperties(int2 mapSize, int cellSize = 8)
    {
        gridSize = (int)math.ceil(math.cmax(mapSize) / (float)cellSize);
        cellCount = gridSize * gridSize;

        this.cellSize = cellSize;
        inverseCellSize = math.rcp(cellSize);
    }

    public int2 GetCellKey(float2 position)
    {
        return (int2)(position * inverseCellSize);
    }

    public int2 GetCellKeyClamped(float2 position)
    {
        return math.clamp(GetCellKey(position), 0, gridSize - 1);
    }

    public int GetCellIndex(int2 cellKey)
    {
        return cellKey.y * gridSize + cellKey.x;
    }
}

public partial struct SpatialGrid<T> : System.IDisposable where T : unmanaged
{
    public SpatialGridProperties gridProperties;

    public NativeArray<int> cellCounts;
    public NativeArray<int> cellOffsets;
    public NativeArray<T> cellData;

    public NativeList<int> activeCells;

    public bool IsCreated => cellCounts.IsCreated;

    public SpatialGrid(int2 mapSize, int cellSize, int itemCapacity, Allocator allocator)
    {
        gridProperties = new SpatialGridProperties(mapSize, cellSize);
        cellCounts = new NativeArray<int>(gridProperties.cellCount, allocator);
        cellOffsets = new NativeArray<int>(gridProperties.cellCount + 1, allocator);
        activeCells = new NativeList<int>(gridProperties.cellCount + 1, allocator);
        cellData = new NativeArray<T>(itemCapacity, allocator, NativeArrayOptions.ClearMemory);
    }

    public void Dispose()
    {
        cellCounts.Dispose();
        cellOffsets.Dispose();
        cellData.Dispose();
        activeCells.Dispose();
    }

    [BurstCompile]
    public struct ResetPrefixSum : IJob
    {
        [WriteOnly] public NativeArray<int> cellCounts;
        [WriteOnly] public NativeList<int> activeCells;

        public void Execute()
        {
            for (int i = 0; i < cellCounts.Length; i++)
            {
                cellCounts[i] = 0;
            }

            //unsafe
            //{
            //    UnsafeUtility.MemClear(cellCounts.GetUnsafePtr(), UnsafeUtility.SizeOf<T>() * cellCounts.Length);
            //}
            activeCells.Clear();
        }
    }

    [BurstCompile]
    public struct CalculateCellOffsets : IJob
    {
        [ReadOnly] public NativeArray<int> cellCounts;
        public NativeArray<int> cellOffsets;
        public NativeList<int> activeCells;

        public void Execute()
        {
            cellOffsets[0] = 0;
            for (int cellIndex = 0; cellIndex < cellCounts.Length; cellIndex++)
            {
                cellOffsets[cellIndex + 1] = cellCounts[cellIndex] + cellOffsets[cellIndex];

                if (cellCounts[cellIndex] > 0)
                {
                    activeCells.Add(cellIndex);
                }
            }
        }
    }

    public void Draw()
    {
        var builder = DrawingManager.GetBuilder(true);

        new DrawGridSptailGridJob
        {
            gridProperties = gridProperties,
            CommandBuilder = builder,
        }.Run();

        builder.Dispose();
    }

    [BurstCompile]
    public struct DrawGridSptailGridJob : IJob
    {
        public SpatialGridProperties gridProperties;
        public CommandBuilder CommandBuilder;

        public void Execute()
        {
            float2 mapSize = gridProperties.cellSize * gridProperties.gridSize;

            CommandBuilder.xz.WireGrid(new float2(mapSize * 0.5f), gridProperties.gridSize, mapSize);
        }
    }
}
