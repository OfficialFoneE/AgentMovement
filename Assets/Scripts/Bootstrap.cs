using Unity.Entities;
using Unity.Mathematics;
using Unity.Rendering;
using Unity.Transforms;
using UnityEngine;


public class Boostrap : MonoBehaviour
{
    public static Boostrap Instance;

    public bool DrawRadiuses = true;
    public bool DrawGrid = true;
    public bool RandomRotations = true;

    public Material unitMaterial;
    public Mesh unitMesh;

    public Transform targetDesintation;

    [Header("Properties")]
    public float SeparationStrength = 2.0f;
    public float CrowdDampingFactor = 0.8f;
    public int CrowdNeighborThreshold = 4;
    public float BackwardSuppression = 0.7f;
    public float MaxNeighborDistance = 6.0f;

    public UnitType[] UnitTypes = new UnitType[1] { new UnitType() };

    [System.Serializable]
    public class UnitType
    {
        public float Scale = 1.0f;
        public float Speed = 3.0f;

        [Header("Collision")]
        public float Length = 0.0f;
        public float Radius = 0.5f;
        public int AvoidancePriority = 50;
    }

    private Entity prefab;

    private EntityManager entityManager => World.DefaultGameObjectInjectionWorld.EntityManager;

    private Entity destinationSingleton;
    private Entity seperationSingleton;

    private void Awake()
    {
        Instance = this;
    }

    // Start is called before the first frame update
    private void Start()
    {
        prefab = CreatePrefab();

        for (int i = 0; i < 2; i++)
        {
            SpawnUnit();
        }

        destinationSingleton = entityManager.CreateSingleton(new DestinationSingleton
        {
            destination = float3.zero
        });

        seperationSingleton = entityManager.CreateSingleton(new EllipseSeparationParams
        {
            SeparationStrength = 2.0f,
            CrowdDampingFactor = 0.8f,
            CrowdNeighborThreshold = 4,
            BackwardSuppression = 0.7f,
            MaxNeighborDistance = 6.0f,
        });

        var fixedStepSimulationSystemGroup = World.DefaultGameObjectInjectionWorld.GetOrCreateSystemManaged<FixedStepSimulationSystemGroup>();
        fixedStepSimulationSystemGroup.Timestep = 0.1f; // 10 Hz
    }

    void Update()
    {
        if(Input.GetKey(KeyCode.Space))
        {
            SpawnUnit();
        }

        entityManager.SetComponentData(destinationSingleton, new DestinationSingleton
        {
            destination = targetDesintation.position,
        });
    }

    private Entity CreatePrefab()
    {
        var prefab = entityManager.CreateEntity(
            ComponentType.ReadOnly<Prefab>(),
            ComponentType.ReadOnly<LocalTransform>(),
            ComponentType.ReadOnly<PastSimLocalTransform>(),
            ComponentType.ReadOnly<SimLocalTransform>(),
            ComponentType.ReadWrite<MaxSpeed>(),
            ComponentType.ReadWrite<CollisionEllipse>(), 
            ComponentType.ReadWrite<CollisionPriority>(),
            ComponentType.ReadWrite<DesiredVelocity>(),
            ComponentType.ReadWrite<AgentComponent>());

        var localTransform = new LocalTransform
        {
            Scale = 1,
            Position = 0,
            Rotation = quaternion.identity,
        };

        entityManager.SetComponentData(prefab, localTransform);
        entityManager.SetComponentData(prefab, new SimLocalTransform { Value = localTransform });
        entityManager.SetComponentData(prefab, new PastSimLocalTransform { Value = localTransform });

        entityManager.SetComponentData(prefab, new CollisionEllipse
        {
            Radii = new float2(1f, 1f),
        });

        entityManager.SetComponentData(prefab, new CollisionPriority
        {
            Value = 1,
        });

        var renderMeshDescription = new RenderMeshDescription
        {
            FilterSettings = Unity.Entities.Graphics.RenderFilterSettings.Default,
            LightProbeUsage = UnityEngine.Rendering.LightProbeUsage.Off,
        };

        var renderMeshArray = new RenderMeshArray(new Material[] { unitMaterial }, new Mesh[] { unitMesh });

        RenderMeshUtility.AddComponents(prefab, entityManager, renderMeshDescription, renderMeshArray, MaterialMeshInfo.FromRenderMeshArrayIndices(0, 0));


        return prefab;
    }

    private void SpawnUnit()
    {
        if (UnitTypes.Length == 0)
        {
            UnityEngine.Debug.LogError("Please add a unit type!", this);
            return;
        }

        var unitType = UnitTypes[UnityEngine.Random.Range(0, UnitTypes.Length)];

        var unit = entityManager.Instantiate(prefab);

        var position = new float3(256 + UnityEngine.Random.Range(-0.01f, 0.01f), 0, 256 + UnityEngine.Random.Range(-0.01f, 0.01f));//new float3(UnityEngine.Random.Range(10, 11), 0, UnityEngine.Random.Range(-0.1f, 0.1f));

        var localTransform = new LocalTransform
        {
            Scale = unitType.Scale,
            Position = position,
            Rotation = quaternion.identity,
        };

        if (RandomRotations)
        {
            localTransform.Rotation = quaternion.RotateY(UnityEngine.Random.Range(-math.PI2, math.PI2));
        }

        entityManager.SetComponentData(unit, localTransform);
        entityManager.SetComponentData(unit, new SimLocalTransform { Value = localTransform });
        entityManager.SetComponentData(unit, new PastSimLocalTransform { Value = localTransform });

        entityManager.SetComponentData(unit, new MaxSpeed
        {
            Value = unitType.Speed,
        });

        entityManager.SetComponentData(unit, new AgentComponent
        {
            Length = unitType.Length,
            BaseRadius = unitType.Radius,
            CrowdingFactor = 1,
            AvoidancePriority = unitType.AvoidancePriority,
        });

        //entityManager.SetComponentData(unit, new CollisionEllipse
        //{
        //    Radii = new float2(size, size * 1.5f) * 0.5f,
        //});

        //entityManager.SetComponentData(unit, new CollisionPriority
        //{
        //    Value = math.lerp(5, 0.5f, sizePercent)
        //});
    }

    public void OnValidate()
    {
        if (seperationSingleton != Entity.Null)
        {
            entityManager.SetComponentData(seperationSingleton, new EllipseSeparationParams
            {
                SeparationStrength = SeparationStrength,
                CrowdDampingFactor = CrowdDampingFactor,
                CrowdNeighborThreshold = CrowdNeighborThreshold,
                BackwardSuppression = BackwardSuppression,
                MaxNeighborDistance = MaxNeighborDistance
            });
        }
    }
}
