using Unity.Entities;
using Unity.Mathematics;
using Unity.Rendering;
using Unity.Transforms;
using UnityEngine;


public class Boostrap : MonoBehaviour
{
    public Material unitMaterial;
    public Mesh unitMesh;

    public Material colliderMaterial;
    public Mesh colliderMesh;

    public Transform targetDesintation;

    [Header("Properties")]
    public float SeparationStrength = 2.0f;
    public float CrowdDampingFactor = 0.8f;
    public int CrowdNeighborThreshold = 4;
    public float BackwardSuppression = 0.7f;
    public float MaxNeighborDistance = 6.0f;


    private Entity prefab;

    private EntityManager entityManager => World.DefaultGameObjectInjectionWorld.EntityManager;

    private Entity destinationSingleton;
    private Entity seperationSingleton;

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
        var unit = entityManager.Instantiate(prefab);

        var position = new float3(256, 0, 256);//new float3(UnityEngine.Random.Range(10, 11), 0, UnityEngine.Random.Range(-0.1f, 0.1f));

        float size = UnityEngine.Random.Range(1, 4.0f);
        float sizePercent = (size - 1) / 3.0f;

        var localTransform = new LocalTransform
        {
            Scale = size,
            Position = position,
            Rotation = quaternion.identity,
        };

        entityManager.SetComponentData(unit, localTransform);
        entityManager.SetComponentData(unit, new SimLocalTransform { Value = localTransform });
        entityManager.SetComponentData(unit, new PastSimLocalTransform { Value = localTransform });

        entityManager.SetComponentData(unit, new MaxSpeed
        {
            Value = math.lerp(3f, 1f, sizePercent)
        });

        entityManager.SetComponentData(unit, new CollisionEllipse
        {
            Radii = new float2(size, size /** 1.5f*/) * 0.5f,
        });

        entityManager.SetComponentData(unit, new AgentComponent
        {
            Radius = size/* * 0.5f*/,
            AvoidancePriority = 50,
        });

        entityManager.SetComponentData(unit, new CollisionPriority
        {
            Value = math.lerp(5, 0.5f, sizePercent)
        });
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
