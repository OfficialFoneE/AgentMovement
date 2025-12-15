using UnityEngine;

public class PillVOScenarioTester2D : MonoBehaviour
{
    public enum Scenario
    {
        RandomDestinations,
        DenseClumpsCrossing,
        MovingClumpThroughIdleClump
    }

    [Header("Scenario")]
    public Scenario scenario = Scenario.RandomDestinations;
    public int randomSeed = 1;

    [Header("Counts")]
    public int agentCount = 60;              // used by RandomDestinations + DenseClumpsCrossing
    public int movingCount = 40;             // used by MovingClumpThroughIdleClump
    public int idleCount = 40;               // used by MovingClumpThroughIdleClump

    [Header("World / Bounds")]
    public bool bounceBounds = true;
    public Vector2 boundsHalfExtents = new Vector2(12, 7);

    [Header("Capsule Shape")]
    public float radius = 0.35f;
    public float length = 1.6f;              // full segment length (excluding caps)
    public bool varySizes = false;

    [Header("Motion")]
    public float speed = 3.5f;
    public float arriveRadius = 0.6f;

    [Header("Priority / Pushability")]
    public float movingPushability = 1f;   // lower = higher priority (moves less)
    public float idlePushability = 4f;     // higher = lower priority (gets pushed more)

    [Header("Avoidance")]
    public float horizonSeconds = 0.9f;
    public float extraSeparation = 0.05f;
    public float maxDeltaSpeed = 10.0f;      // 0 = unlimited
    public float minResponseTime = 0.06f;
    [Range(1, 8)] public int iterations = 4;

    [Header("Realign Back To Target")]
    public float maxAccel = 25f;             // units/sec^2
    [Range(0f, 1f)] public float realignStrength = 0.65f;
    public bool preserveSpeed = true;

    [Header("Capsule Orientation Stability")]
    public float maxTurnDegPerSec = 360f;   // try 180..720

    [Header("Scenario Geometry")]
    public float clumpRadius = 2.2f;
    public float clumpSeparationX = 9.0f;    // for DenseClumpsCrossing
    public float idleClumpRadius = 2.4f;     // for MovingClumpThroughIdleClump

    [Header("Debug Draw")]
    public bool drawCapsules = true;
    public bool drawVelocities = true;
    public bool drawTargets = true;
    public bool drawDesiredVel = false;

    // --- Internal state ---
    private struct Agent
    {
        public Vector2 position;
        public Vector2 velocity;
        public Vector2 facing;       // used to orient capsules when velocity is near-zero
        public Vector2 target;
        public float halfLength;
        public float radius;
        public float invMass;        // 1 for moving, 0 for idle/infinite mass
        public int group;            // for coloring/debug
    }

    private Agent[] _agents;
    private Vector2[] _desiredVels;
    private Vector2 _worldCenter;

    [ContextMenu("Reset Scenario")]
    public void ResetScenario()
    {
        Random.InitState(randomSeed);
        _worldCenter = (Vector2)transform.position;

        switch (scenario)
        {
            case Scenario.RandomDestinations:
                SpawnRandomDestinations(agentCount);
                break;

            case Scenario.DenseClumpsCrossing:
                SpawnDenseClumpsCrossing(agentCount);
                break;

            case Scenario.MovingClumpThroughIdleClump:
                SpawnMovingThroughIdle(movingCount, idleCount);
                break;
        }

        _desiredVels = new Vector2[_agents.Length];
    }

    void OnEnable() => ResetScenario();

    void FixedUpdate()
    {
        if (_agents == null || _agents.Length == 0) return;

        float dt = Time.fixedDeltaTime;
        float maxSteerDv = maxAccel * dt;

        // 1) Compute desired velocities from targets (moving agents only)
        for (int i = 0; i < _agents.Length; i++)
        {
            ref Agent a = ref _agents[i];

            if (a.invMass <= 0f)
            {
                // If you still ever use invMass<=0 as “frozen”, keep this.
                _desiredVels[i] = Vector2.zero;
                continue;
            }

            // If this agent is “idle priority” (pushable) we still want it to settle to zero:
            if (scenario == Scenario.MovingClumpThroughIdleClump && a.group == 1 /* idle group */)
            {
                _desiredVels[i] = Vector2.zero;
                continue;
            }

            // Retarget if arrived (behavior depends on scenario)
            Vector2 toT = a.target - a.position;
            if (toT.sqrMagnitude <= arriveRadius * arriveRadius)
                PickNextTarget(ref a);

            toT = a.target - a.position;
            Vector2 dir = (toT.sqrMagnitude > 1e-6f) ? toT.normalized : a.facing;
            _desiredVels[i] = dir * speed;
        }

        // 2) Avoidance + realign (Jacobi-style iterations)
        Vector2[] deltaV = new Vector2[_agents.Length];

        for (int iter = 0; iter < Mathf.Max(1, iterations); iter++)
        {
            System.Array.Clear(deltaV, 0, deltaV.Length);

            for (int i = 0; i < _agents.Length; i++)
            {
                for (int j = i + 1; j < _agents.Length; j++)
                {
                    ref Agent A = ref _agents[i];
                    ref Agent B = ref _agents[j];

                    // Both idle/infinite mass => no need
                    if (A.invMass <= 0f && B.invMass <= 0f)
                        continue;

                    // Use proxy velocities ONLY for the capsule orientation used by AvoidPair.
                    // IMPORTANT: compute dv against what you passed in (proxy), not against A.velocity.
                    float spA = Mathf.Max(A.velocity.magnitude, 1e-4f);
                    float spB = Mathf.Max(B.velocity.magnitude, 1e-4f);

                    Vector2 vA_proxy = A.facing.normalized * spA;
                    Vector2 vB_proxy = B.facing.normalized * spB;

                    Pill2DVO.AvoidPair(
                        new Pill2DVO.Capsule { center = A.position, velocity = vA_proxy, halfLength = A.halfLength, radius = A.radius },
                        new Pill2DVO.Capsule { center = B.position, velocity = vB_proxy, halfLength = B.halfLength, radius = B.radius },
                        horizonSeconds, extraSeparation, maxDeltaSpeed, minResponseTime,
                        out Vector2 vA_proxyNew,
                        out Vector2 vB_proxyNew,
                        out _);

                    // dv in proxy space (this is what AvoidPair actually solved for)
                    Vector2 dA_half = vA_proxyNew - vA_proxy;
                    Vector2 dB_half = vB_proxyNew - vB_proxy;

                    // full relative correction (equal and opposite)
                    Vector2 full = dB_half - dA_half;

                    float wA = A.invMass;
                    float wB = B.invMass;
                    float wSum = wA + wB;

                    // guard
                    if (wSum > 1e-6f)
                    {
                        float shareA = wA / wSum;
                        float shareB = wB / wSum;

                        // Apply more of the correction to the more “pushable” agent
                        deltaV[i] += (-full * shareA);
                        deltaV[j] += (full * shareB);
                    }
                }
            }

            // Apply avoidance + steering back to desired
            for (int i = 0; i < _agents.Length; i++)
            {
                ref Agent a = ref _agents[i];
                if (a.invMass <= 0f)
                {
                    a.velocity = Vector2.zero;
                    continue;
                }

                a.velocity += deltaV[i];

                // steer back toward desired velocity (limited by maxAccel)
                Vector2 desired = _desiredVels[i];
                Vector2 steer = desired - a.velocity;
                float mag = steer.magnitude;
                if (mag > maxSteerDv && mag > 1e-6f) steer *= (maxSteerDv / mag);

                a.velocity += steer * Mathf.Clamp01(realignStrength);

                // Clamp velocity to max speed.
                // Required for idle agents as they might be pushed very hard due to having less prio.
                {
                    var vmag = a.velocity.magnitude;
                    if(vmag > 1e-6f && vmag > speed)
                    {
                        a.velocity = (a.velocity / vmag) * speed;
                    }
                }

                if (preserveSpeed)
                {
                    float targetSpeed = desired.magnitude;
                    float vmag = a.velocity.magnitude;
                    if (vmag > 1e-6f && targetSpeed > 1e-6f)
                    {
                        a.velocity = a.velocity * (targetSpeed / vmag);
                    }
                }
            }
        }

        // 3) Integrate (moving only) + bounds + update facing
        for (int i = 0; i < _agents.Length; i++)
        {
            ref Agent a = ref _agents[i];

            if (a.invMass > 0f)
                a.position += a.velocity * dt;

            if (bounceBounds)
                BounceInBounds(ref a, _worldCenter, boundsHalfExtents);

            if (a.velocity.sqrMagnitude > 1e-6f)
            {
                Vector2 velDir = a.velocity.normalized;
                a.facing = RotateTowards2D(a.facing, velDir, maxTurnDegPerSec * Mathf.Deg2Rad * dt);
            }

            //float vsq = a.velocity.sqrMagnitude;
            //if (vsq > 1e-6f) a.facing = a.velocity / Mathf.Sqrt(vsq);
        }
    }

    void OnDrawGizmos()
    {
        if (_agents == null) return;

        // bounds
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
                Gizmos.color =
                    a.invMass <= 0f ? new Color(1f, 0.2f, 0.9f, 1f) :          // idle = magenta
                    a.group == 0 ? new Color(0.2f, 1f, 0.2f, 1f) :             // group 0 = green
                                   new Color(0.2f, 0.7f, 1f, 1f);              // group 1 = cyan

                DrawCapsule2D(a.position, a.velocity, a.facing, a.halfLength, a.radius);
            }

            if (drawVelocities)
            {
                Gizmos.color = Color.white;
                DrawArrow2D(a.position, a.position + a.velocity * 0.35f);
            }

            if (drawTargets && a.invMass > 0f)
            {
                Gizmos.color = new Color(1f, 0.85f, 0.2f, 0.9f);
                Gizmos.DrawSphere(ToV3(a.target), 0.08f);
            }

            if (drawDesiredVel && _desiredVels != null && i < _desiredVels.Length)
            {
                Gizmos.color = new Color(1f, 0.6f, 0.1f, 0.8f);
                DrawArrow2D(a.position, a.position + _desiredVels[i] * 0.25f);
            }
        }
    }

    // ---------------- Scenario Spawning ----------------

    private void SpawnRandomDestinations(int count)
    {
        _agents = new Agent[count];

        for (int i = 0; i < count; i++)
        {
            Vector2 pos = _worldCenter + Random.insideUnitCircle * (Mathf.Min(boundsHalfExtents.x, boundsHalfExtents.y) * 0.8f);
            Vector2 dir = Random.insideUnitCircle.normalized;
            if (dir.sqrMagnitude < 1e-6f) dir = Vector2.right;

            float r = radius;
            float hl = Mathf.Max(0f, length * 0.5f);
            if (varySizes)
            {
                r *= Random.Range(0.8f, 1.2f);
                hl *= Random.Range(0.7f, 1.3f);
            }

            _agents[i] = new Agent
            {
                position = pos,
                velocity = dir * speed,
                facing = dir,
                target = RandomPointInBounds(_worldCenter, boundsHalfExtents),
                radius = r,
                halfLength = hl,
                invMass = 1f,
                group = 0
            };
        }
    }

    private void SpawnDenseClumpsCrossing(int count)
    {
        _agents = new Agent[count];

        Vector2 leftCenter = _worldCenter + new Vector2(-clumpSeparationX * 0.5f, 0f);
        Vector2 rightCenter = _worldCenter + new Vector2(+clumpSeparationX * 0.5f, 0f);

        int half = count / 2;

        for (int i = 0; i < count; i++)
        {
            bool leftGroup = i < half;

            Vector2 c = leftGroup ? leftCenter : rightCenter;
            Vector2 pos = c + Random.insideUnitCircle * clumpRadius;

            Vector2 desiredDir = leftGroup ? Vector2.right : Vector2.left;
            Vector2 vel = desiredDir * speed;

            float r = radius;
            float hl = Mathf.Max(0f, length * 0.5f);
            if (varySizes)
            {
                r *= Random.Range(0.9f, 1.15f);
                hl *= Random.Range(0.75f, 1.25f);
            }

            // Targets are “other side”, and we ping-pong when reached (see PickNextTarget).
            Vector2 target = leftGroup
                ? (_worldCenter + new Vector2(+boundsHalfExtents.x * 0.9f, Random.Range(-boundsHalfExtents.y, boundsHalfExtents.y)))
                : (_worldCenter + new Vector2(-boundsHalfExtents.x * 0.9f, Random.Range(-boundsHalfExtents.y, boundsHalfExtents.y)));

            _agents[i] = new Agent
            {
                position = pos,
                velocity = vel,
                facing = desiredDir,
                target = target,
                radius = r,
                halfLength = hl,
                invMass = 1f,
                group = leftGroup ? 0 : 1
            };
        }
    }

    private void SpawnMovingThroughIdle(int movers, int idlers)
    {
        _agents = new Agent[movers + idlers];

        Vector2 moveCenter = _worldCenter + new Vector2(-boundsHalfExtents.x * 0.75f, 0f);
        Vector2 idleCenter = _worldCenter; // center clump

        // moving clump (group 0)
        for (int i = 0; i < movers; i++)
        {
            Vector2 pos = moveCenter + Random.insideUnitCircle * clumpRadius;

            Vector2 desiredDir = Vector2.right;
            Vector2 vel = desiredDir * speed;

            float r = radius;
            float hl = Mathf.Max(0f, length * 0.5f);
            if (varySizes)
            {
                r *= Random.Range(0.9f, 1.15f);
                hl *= Random.Range(0.75f, 1.25f);
            }

            Vector2 target = _worldCenter + new Vector2(+boundsHalfExtents.x * 0.85f, Random.Range(-boundsHalfExtents.y, boundsHalfExtents.y));

            _agents[i] = new Agent
            {
                position = pos,
                velocity = vel,
                facing = desiredDir,
                target = target,
                radius = r,
                halfLength = hl,
                invMass = movingPushability,
                group = 0
            };
        }

        // idle clump (invMass=0, group=1)
        for (int k = 0; k < idlers; k++)
        {
            int i = movers + k;

            Vector2 pos = idleCenter + Random.insideUnitCircle * idleClumpRadius;

            float r = radius;
            float hl = Mathf.Max(0f, length * 0.5f);
            if (varySizes)
            {
                r *= Random.Range(0.9f, 1.15f);
                hl *= Random.Range(0.75f, 1.25f);
            }

            // idle agents: velocity stays 0, target unused, invMass=0
            _agents[i] = new Agent
            {
                position = pos,
                velocity = Vector2.zero,
                facing = Random.insideUnitCircle.sqrMagnitude > 1e-6f ? Random.insideUnitCircle.normalized : Vector2.right,
                target = pos,
                radius = r,
                halfLength = hl,
                invMass = idlePushability,
                group = 1
            };
        }
    }

    private void PickNextTarget(ref Agent a)
    {
        // Scenario-specific “next goal” logic to keep things flowing.
        switch (scenario)
        {
            case Scenario.RandomDestinations:
                a.target = RandomPointInBounds(_worldCenter, boundsHalfExtents);
                break;

            case Scenario.DenseClumpsCrossing:
                // Ping-pong across X (keeps dense crossings continuous)
                float x = a.position.x - _worldCenter.x;
                float sign = (x >= 0f) ? -1f : +1f;
                a.target = _worldCenter + new Vector2(sign * boundsHalfExtents.x * 0.9f, Random.Range(-boundsHalfExtents.y, boundsHalfExtents.y));
                break;

            case Scenario.MovingClumpThroughIdleClump:
                // Movers: keep going left <-> right
                if (a.group == 0)
                {
                    float side = (a.position.x - _worldCenter.x) >= 0f ? -1f : +1f;
                    a.target = _worldCenter + new Vector2(side * boundsHalfExtents.x * 0.85f, Random.Range(-boundsHalfExtents.y, boundsHalfExtents.y));
                }
                break;
        }
    }

    // ---------------- Utility / Gizmos ----------------

    private static Vector2 RandomPointInBounds(Vector2 center, Vector2 halfExt)
    {
        return center + new Vector2(
            Random.Range(-halfExt.x, halfExt.x),
            Random.Range(-halfExt.y, halfExt.y));
    }

    private static void BounceInBounds(ref Agent a, Vector2 center, Vector2 halfExt)
    {
        Vector2 p = a.position - center;

        if (p.x < -halfExt.x) { p.x = -halfExt.x; if (a.invMass > 0f) a.velocity.x = Mathf.Abs(a.velocity.x); }
        if (p.x > halfExt.x) { p.x = halfExt.x; if (a.invMass > 0f) a.velocity.x = -Mathf.Abs(a.velocity.x); }
        if (p.y < -halfExt.y) { p.y = -halfExt.y; if (a.invMass > 0f) a.velocity.y = Mathf.Abs(a.velocity.y); }
        if (p.y > halfExt.y) { p.y = halfExt.y; if (a.invMass > 0f) a.velocity.y = -Mathf.Abs(a.velocity.y); }

        a.position = center + p;
    }

    private static Vector2 RotateTowards2D(Vector2 from, Vector2 to, float maxRadians)
    {
        if (from.sqrMagnitude < 1e-8f) from = Vector2.right;
        if (to.sqrMagnitude < 1e-8f) return from.normalized;

        from.Normalize();
        to.Normalize();

        // signed angle from->to
        float cross = from.x * to.y - from.y * to.x;
        float dot = Mathf.Clamp(Vector2.Dot(from, to), -1f, 1f);
        float angle = Mathf.Atan2(cross, dot);

        float step = Mathf.Clamp(angle, -maxRadians, maxRadians);

        float s = Mathf.Sin(step);
        float c = Mathf.Cos(step);

        return new Vector2(from.x * c - from.y * s, from.x * s + from.y * c).normalized;
    }

    private static void DrawCapsule2D(Vector2 center, Vector2 velocity, Vector2 facing, float halfLen, float radius)
    {
        Vector2 dir = facing.normalized; //(velocity.sqrMagnitude > 1e-8f) ? velocity.normalized : (facing.sqrMagnitude > 1e-8f ? facing.normalized : Vector2.right);
        Vector2 a = center - dir * halfLen;
        Vector2 b = center + dir * halfLen;

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
