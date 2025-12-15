using UnityEngine;

public class PillVOMultiTester2D : MonoBehaviour
{
    [Header("Spawn")]
    public int agentCount = 40;
    public float spawnRadius = 8f;
    public int randomSeed = 1;

    [Header("Capsule Shape")]
    public float radius = 0.35f;
    public float length = 1.6f; // full segment length (excluding caps)
    public bool varySizes = false;

    [Header("Motion")]
    public float speed = 3.0f;
    public bool seekCenter = true;      // makes them swirl/cross more
    public bool bounceBounds = true;
    public Vector2 boundsHalfExtents = new Vector2(10, 6);

    //In crowds, increasing iterations should reduce “sticking”.
    //If they jitter, increase minResponseTime slightly (e.g. 0.08) and/or clamp maxDeltaSpeed.

    [Header("Avoidance Settings")]
    public float horizonSeconds = 1.0f;
    public float extraSeparation = 0.05f;
    public float maxDeltaSpeed = 10.0f;      // 0 = unlimited
    public float minResponseTime = 0.05f;
    [Range(1, 8)] public int iterations = 3;
    public bool preserveSpeed = true;

    [Header("Debug Draw")]
    public bool drawCapsules = true;
    public bool drawVelocities = true;
    public bool drawPredictedPositions = false; // draws at t=horizon
    public int drawPairDebugForAgent = -1;      // -1 disables (shows closest pair for this agent)

    private PillVOMulti2D.Agent[] _agents;

    void OnEnable()
    {
        ResetSim();
    }

    [ContextMenu("Reset Sim")]
    public void ResetSim()
    {
        Random.InitState(randomSeed);

        _agents = new PillVOMulti2D.Agent[agentCount];

        for (int i = 0; i < agentCount; i++)
        {
            Vector2 pos = (Vector2)transform.position + Random.insideUnitCircle * spawnRadius;

            // initial direction
            Vector2 dir = Random.insideUnitCircle.normalized;
            if (dir.sqrMagnitude < 1e-6f) dir = Vector2.right;

            float r = radius;
            float hl = Mathf.Max(0f, length * 0.5f);

            if (varySizes)
            {
                r *= Random.Range(0.8f, 1.2f);
                hl *= Random.Range(0.7f, 1.3f);
            }

            _agents[i] = new PillVOMulti2D.Agent
            {
                position = pos,
                velocity = dir * speed,
                radius = r,
                halfLength = hl
            };
        }
    }

    void FixedUpdate()
    {
        if (_agents == null || _agents.Length == 0) return;

        // Optional steering to increase interactions
        if (seekCenter)
        {
            Vector2 center = (Vector2)transform.position;
            for (int i = 0; i < _agents.Length; i++)
            {
                Vector2 toC = (center - _agents[i].position);
                if (toC.sqrMagnitude > 1e-6f)
                {
                    Vector2 desired = toC.normalized * speed;
                    // light steering (keeps test stable)
                    _agents[i].velocity = Vector2.Lerp(_agents[i].velocity, desired, 0.08f);
                }
            }
        }

        // Avoidance step
        var settings = new PillVOMulti2D.Settings
        {
            horizonSeconds = horizonSeconds,
            extraSeparation = extraSeparation,
            maxDeltaSpeed = maxDeltaSpeed,
            minResponseTime = minResponseTime,
            iterations = iterations,
            preserveSpeed = preserveSpeed
        };

        PillVOMulti2D.Step(ref _agents, settings);

        // Integrate positions
        float dt = Time.fixedDeltaTime;
        for (int i = 0; i < _agents.Length; i++)
        {
            _agents[i].position += _agents[i].velocity * dt;

            if (bounceBounds)
                BounceInBounds(ref _agents[i], (Vector2)transform.position, boundsHalfExtents);
        }
    }

    void OnDrawGizmos()
    {
        if (_agents == null) return;

        // Draw bounds
        if (bounceBounds)
        {
            Gizmos.color = new Color(1f, 1f, 1f, 0.15f);
            Vector3 c = transform.position;
            Gizmos.DrawWireCube(c, new Vector3(boundsHalfExtents.x * 2, boundsHalfExtents.y * 2, 0));
        }

        for (int i = 0; i < _agents.Length; i++)
        {
            var a = _agents[i];

            if (drawCapsules)
            {
                Gizmos.color = Color.green;
                DrawCapsule2D(a.position, a.velocity, a.halfLength, a.radius);
            }

            if (drawVelocities)
            {
                Gizmos.color = Color.white;
                DrawArrow2D(a.position, a.position + a.velocity * 0.35f);
            }

            if (drawPredictedPositions)
            {
                Gizmos.color = new Color(1f, 0.85f, 0.2f, 0.6f);
                DrawCapsule2D(a.position + a.velocity * horizonSeconds, a.velocity, a.halfLength, a.radius);
            }
        }

        // Optional: show closest predicted pair for one agent index
        if (drawPairDebugForAgent >= 0 && drawPairDebugForAgent < _agents.Length)
        {
            int i = drawPairDebugForAgent;
            int bestJ = -1;
            float bestDist = float.PositiveInfinity;
            Pill2DVO.DebugInfo bestDbg = default;

            var ai = _agents[i];

            for (int j = 0; j < _agents.Length; j++)
            {
                if (j == i) continue;

                var bj = _agents[j];

                Pill2DVO.AvoidPair(
                    new Pill2DVO.Capsule { center = ai.position, velocity = ai.velocity, halfLength = ai.halfLength, radius = ai.radius },
                    new Pill2DVO.Capsule { center = bj.position, velocity = bj.velocity, halfLength = bj.halfLength, radius = bj.radius },
                    horizonSeconds, extraSeparation, maxDeltaSpeed, minResponseTime,
                    out _, out _, out var dbg);

                if (dbg.tMin < 0f) continue;

                if (dbg.minDist < bestDist)
                {
                    bestDist = dbg.minDist;
                    bestJ = j;
                    bestDbg = dbg;
                }
            }

            if (bestJ >= 0)
            {
                Gizmos.color = Color.red;
                Gizmos.DrawSphere(ToV3(bestDbg.pA), 0.06f);
                Gizmos.DrawSphere(ToV3(bestDbg.pB), 0.06f);
                Gizmos.DrawLine(ToV3(bestDbg.pA), ToV3(bestDbg.pB));

                Vector2 mid = 0.5f * (bestDbg.pA + bestDbg.pB);
                DrawArrow2D(mid, mid + bestDbg.normal * 0.6f);

                Gizmos.color = new Color(1f, 0.3f, 0.3f, 1f);
                DrawCapsuleFromEndpoints(bestDbg.a0, bestDbg.a1, ai.radius);
                DrawCapsuleFromEndpoints(bestDbg.b0, bestDbg.b1, _agents[bestJ].radius);
            }
        }
    }

    // --- Helpers ---

    private static void BounceInBounds(ref PillVOMulti2D.Agent a, Vector2 center, Vector2 halfExt)
    {
        Vector2 p = a.position - center;

        if (p.x < -halfExt.x) { p.x = -halfExt.x; a.velocity.x = Mathf.Abs(a.velocity.x); }
        if (p.x > halfExt.x) { p.x = halfExt.x; a.velocity.x = -Mathf.Abs(a.velocity.x); }
        if (p.y < -halfExt.y) { p.y = -halfExt.y; a.velocity.y = Mathf.Abs(a.velocity.y); }
        if (p.y > halfExt.y) { p.y = halfExt.y; a.velocity.y = -Mathf.Abs(a.velocity.y); }

        a.position = center + p;
    }

    private static void DrawCapsule2D(Vector2 center, Vector2 velocity, float halfLen, float radius)
    {
        Vector2 dir = velocity.sqrMagnitude > 1e-8f ? velocity.normalized : Vector2.right;
        Vector2 a = center - dir * halfLen;
        Vector2 b = center + dir * halfLen;

        Gizmos.DrawLine(ToV3(a), ToV3(b));
        Gizmos.DrawWireSphere(ToV3(a), radius);
        Gizmos.DrawWireSphere(ToV3(b), radius);
    }

    private static void DrawCapsuleFromEndpoints(Vector2 a, Vector2 b, float radius)
    {
        Gizmos.DrawLine(ToV3(a), ToV3(b));
        Gizmos.DrawWireSphere(ToV3(a), radius);
        Gizmos.DrawWireSphere(ToV3(b), radius);
    }

    private static void DrawArrow2D(Vector2 from, Vector2 to)
    {
        Gizmos.DrawLine(ToV3(from), ToV3(to));

        Vector2 d = to - from;
        if (d.sqrMagnitude < 1e-8f) return;

        Vector2 dir = d.normalized;
        Vector2 left = new Vector2(-dir.y, dir.x);

        float headLen = 0.12f;
        float headWid = 0.08f;

        Vector2 p1 = to - dir * headLen + left * headWid;
        Vector2 p2 = to - dir * headLen - left * headWid;

        Gizmos.DrawLine(ToV3(to), ToV3(p1));
        Gizmos.DrawLine(ToV3(to), ToV3(p2));
    }

    private static Vector3 ToV3(Vector2 v) => new Vector3(v.x, v.y, 0f);
}
