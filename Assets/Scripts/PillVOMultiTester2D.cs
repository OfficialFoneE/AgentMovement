using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Assertions;
using Random = UnityEngine.Random;

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
    public int agentCount = 60;
    public int movingCount = 40;
    public int idleCount = 40;

    [Header("World / Bounds")]
    public bool bounceBounds = true;
    public float2 boundsHalfExtents = new float2(12, 7);

    [Header("Capsule Shape")]
    public float radius = 0.35f;
    public float length = 1.6f;
    public bool varySizes = false;

    [Header("Motion")]
    public float speed = 3.5f;
    public float arriveRadius = 0.6f;

    [Header("Priority / Pushability")]
    public float movingPushability = 1f;
    public float idlePushability = 4f;

    [Header("Avoidance")]
    public float horizonSeconds = 0.9f;
    public float extraSeparation = 0.05f;
    public float maxDeltaSpeed = 10.0f; // 0 = unlimited
    public float minResponseTime = 0.06f;
    [Range(1, 8)] public int iterations = 4;

    [Header("Realign Back To Target")]
    public float maxAccel = 25f;
    [Range(0f, 1f)] public float realignStrength = 0.65f;
    public bool preserveSpeed = true;

    [Header("Speed Hold (replaces hard preserveSpeed normalize)")]
    [Range(0f, 1f)] public float speedHoldWhenMisaligned = 0.15f;

    [Header("Capsule Orientation Stability")]
    public float maxTurnDegPerSec = 360f;

    [Header("Scenario Geometry")]
    public float clumpRadius = 2.2f;
    public float clumpSeparationX = 9.0f;
    public float idleClumpRadius = 2.4f;

    [Header("Debug Draw")]
    public bool drawCapsules = true;
    public bool drawVelocities = true;
    public bool drawTargets = true;
    public bool drawDesiredVel = false;

    public enum FacingMode
    {
        LinkedToMoveAxis,
        ToVelocity,
        ToDestination,
        ToPoint
    }

    private struct Agent
    {
        public float2 position;
        public float2 velocity;

        public float2 moveAxis;    // locomotion axis (car constraint axis)
        public float2 facing;      // BODY axis (physical collider axis)

        public FacingMode facingMode;
        public float2 aimPoint;

        public float2 target;
        public float halfLength;
        public float radius;

        public float invMass;      // pushability weight
        public float axisFreedom;  // 0=car, 1=hover
        public int group;
    }

    private static readonly float2 kDefaultDir = new float2(1f, 0f);

    private Agent[] _agents;
    private float2[] _desiredVels;
    private float2[] _deltaV;
    private float2 _worldCenter;

    [ContextMenu("Reset Scenario")]
    public void ResetScenario()
    {
        Random.InitState(randomSeed);
        _worldCenter = new float2(transform.position.x, transform.position.y);

        ValidateSettings_AssertOnly();

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

        _desiredVels = new float2[_agents.Length];
        _deltaV = new float2[_agents.Length];
    }

    void OnEnable() => ResetScenario();

    void FixedUpdate()
    {
        if (_agents == null || _agents.Length == 0) return;

        float dt = Time.fixedDeltaTime;
        int iters = math.max(1, iterations);
        float dtIter = dt / iters;

        float maxSteerDvIter = maxAccel * dtIter;

        // 1) Desired velocities
        for (int i = 0; i < _agents.Length; i++)
        {
            ref Agent a = ref _agents[i];

            if (a.invMass <= 0f)
            {
                _desiredVels[i] = float2.zero;
                continue;
            }

            if (scenario == Scenario.MovingClumpThroughIdleClump && a.group == 1)
            {
                _desiredVels[i] = float2.zero;
                continue;
            }

            float2 toT = a.target - a.position;
            if (math.lengthsq(toT) <= arriveRadius * arriveRadius)
                PickNextTarget(ref a);

            toT = a.target - a.position;

            // No epsilon branches needed: normalizesafe handles zero vectors deterministically.
            float2 fallback = SafeNormal(a.facing);
            float2 dir = math.normalizesafe(toT, fallback);

            _desiredVels[i] = dir * speed;
        }

        // 2) Iterative avoidance + steer + projection
        for (int iter = 0; iter < iters; iter++)
        {
            System.Array.Clear(_deltaV, 0, _deltaV.Length);

            // Pairwise avoidance (multi-circle)
            for (int i = 0; i < _agents.Length; i++)
            {
                for (int j = i + 1; j < _agents.Length; j++)
                {
                    ref Agent A = ref _agents[i];
                    ref Agent B = ref _agents[j];

                    if (A.invMass <= 0f && B.invMass <= 0f)
                        continue;

                    float2 full = ComputeAvoidanceCorrection_MultiCircle(
                        A.position, A.velocity, SafeNormal(A.facing), A.halfLength, A.radius,
                        B.position, B.velocity, SafeNormal(B.facing), B.halfLength, B.radius,
                        horizonSeconds, extraSeparation, maxDeltaSpeed, minResponseTime);

                    float wA = A.invMass;
                    float wB = B.invMass;
                    float wSum = wA + wB;

                    if (wSum > 1e-6f && math.lengthsq(full) > 1e-12f)
                    {
                        float shareA = wA / wSum;
                        float shareB = wB / wSum;

                        _deltaV[i] += (-full * shareA);
                        _deltaV[j] += (full * shareB);
                    }
                }
            }

            // Apply avoidance + goal steering + locomotion projection
            for (int i = 0; i < _agents.Length; i++)
            {
                ref Agent a = ref _agents[i];

                if (a.invMass <= 0f)
                {
                    a.velocity = float2.zero;
                    continue;
                }

                float2 desired = _desiredVels[i];

                float2 vCmd = a.velocity + _deltaV[i];

                float2 steer = desired - vCmd;
                float smag = math.length(steer);
                if (smag > maxSteerDvIter && smag > 1e-6f)
                    steer *= (maxSteerDvIter / smag);

                vCmd += steer * math.saturate(realignStrength);

                float vmag = math.length(vCmd);
                if (vmag > speed && vmag > 1e-6f)
                    vCmd = (vCmd / vmag) * speed;

                ProjectLocomotion(ref a, vCmd, desired, dtIter);
            }
        }

        // 3) Integrate + bounds
        for (int i = 0; i < _agents.Length; i++)
        {
            ref Agent a = ref _agents[i];

            if (a.invMass > 0f)
                a.position += a.velocity * dt;

            if (bounceBounds)
                BounceInBounds(ref a, _worldCenter, boundsHalfExtents);
        }
    }

    private void ProjectLocomotion(ref Agent a, float2 vCmd, float2 desiredVel, float dt)
    {
        float maxTurnRad = maxTurnDegPerSec * math.radians(1f) * dt;
        float maxDv = maxAccel * dt;

        float axisFreedom01 = math.saturate(a.axisFreedom);

        // --- 1) Update moveAxis ---
        float2 axisDesired =
            (axisFreedom01 < 1e-3f)
                ? math.normalizesafe(desiredVel, SafeNormal(a.moveAxis)) // car aims body toward goal
                : math.normalizesafe(vCmd, SafeNormal(desiredVel));      // hover aims body toward command

        float axisTurn = math.lerp(maxTurnRad, math.PI, axisFreedom01);
        a.moveAxis = RotateTowards2D(a.moveAxis, axisDesired, axisTurn);

        // --- 2) Accel constrain ---
        float2 v = a.velocity;
        float2 dv = vCmd - v;

        float2 axis = SafeNormal(a.moveAxis);
        float2 dvParallel = math.dot(dv, axis) * axis;
        float2 dvPerp = dv - dvParallel;

        float2 dvAllowed = dvParallel + dvPerp * axisFreedom01;

        float dvmag = math.length(dvAllowed);
        if (dvmag > maxDv && dvmag > 1e-6f)
            dvAllowed *= (maxDv / dvmag);

        v += dvAllowed;

        float vmag = math.length(v);
        if (vmag > speed && vmag > 1e-6f)
            v = (v / vmag) * speed;

        // Speed hold controller
        if (preserveSpeed)
        {
            float desiredSpeed = math.length(desiredVel);
            if (desiredSpeed > 1e-6f)
            {
                float2 desiredDir = desiredVel / desiredSpeed;

                float curSpeed = math.length(v);
                float2 curDir = (curSpeed > 1e-6f) ? (v / curSpeed) : desiredDir;

                float align = math.dot(curDir, desiredDir);

                float t = math.saturate((align + 1f) * 0.5f);
                float targetSpeed = math.lerp(speedHoldWhenMisaligned * desiredSpeed, desiredSpeed, t);

                float ds = targetSpeed - curSpeed;
                float dsStep = math.clamp(ds, -maxDv, maxDv);

                v += curDir * dsStep;

                float vmag2 = math.length(v);
                if (vmag2 > speed && vmag2 > 1e-6f)
                    v = (v / vmag2) * speed;
            }
        }

        a.velocity = v;

        // --- 3) Facing update (BODY axis used by physical collider) ---
        float2 faceDesired;
        switch (a.facingMode)
        {
            default:
            case FacingMode.LinkedToMoveAxis:
                faceDesired = a.moveAxis;
                break;

            case FacingMode.ToVelocity:
                faceDesired = math.normalizesafe(a.velocity, a.moveAxis);
                break;

            case FacingMode.ToDestination:
                faceDesired = math.normalizesafe(a.target - a.position, a.moveAxis);
                break;

            case FacingMode.ToPoint:
                faceDesired = math.normalizesafe(a.aimPoint - a.position, a.moveAxis);
                break;
        }

        a.facing = RotateTowards2D(a.facing, SafeNormal(faceDesired), maxTurnRad);

        if (a.facingMode == FacingMode.LinkedToMoveAxis)
            a.facing = a.moveAxis;
    }

    private float2 ComputeAvoidanceCorrection_MultiCircle(
        float2 cA, float2 vA, float2 axisA, float halfLenA, float rA,
        float2 cB, float2 vB, float2 axisB, float halfLenB, float rB,
        float horizon, float extraSep, float maxDelta, float minRespTime)
    {
        float R = (rA + rB + extraSep);

        int nA = GetCircleSampleCount(halfLenA, rA);
        int nB = GetCircleSampleCount(halfLenB, rB);

        float2 vRel = vB - vA;

        float bestDelta = 0f;
        float2 bestN = float2.zero;

        for (int ia = 0; ia < nA; ia++)
        {
            float ta = (nA == 1) ? 0.5f : (ia / (float)(nA - 1));
            float offA = math.lerp(-halfLenA, +halfLenA, ta);
            float2 pA = cA + axisA * offA;

            for (int ib = 0; ib < nB; ib++)
            {
                float tb = (nB == 1) ? 0.5f : (ib / (float)(nB - 1));
                float offB = math.lerp(-halfLenB, +halfLenB, tb);
                float2 pB = cB + axisB * offB;

                ComputeDiscAvoidance(pA, pB, vRel, R, horizon, minRespTime, out float2 n, out float delta);

                if (delta > bestDelta)
                {
                    bestDelta = delta;
                    bestN = n;
                }
            }
        }

        if (bestDelta <= 0f || math.lengthsq(bestN) < 1e-8f)
            return float2.zero;

        if (maxDelta > 0f)
            bestDelta = math.min(bestDelta, maxDelta);

        return bestN * bestDelta;
    }

    private static int GetCircleSampleCount(float halfLen, float radius)
    {
        float fullLen = math.max(0f, halfLen) * 2f;
        if (fullLen <= radius * 1.25f) return 3;
        if (fullLen <= radius * 3.5f) return 5;
        return 7;
    }

    private static void ComputeDiscAvoidance(
        float2 pA, float2 pB,
        float2 vRel,
        float R,
        float horizon,
        float minRespTime,
        out float2 n,
        out float deltaSpeed)
    {
        float2 p = pB - pA;
        float dist = math.length(p);

        if (dist < R)
        {
            n = math.normalizesafe(p, kDefaultDir);
            float tau = math.max(minRespTime, 1e-4f);
            deltaSpeed = (R - dist) / tau;
            return;
        }

        float v2 = math.lengthsq(vRel);
        if (v2 < 1e-10f)
        {
            n = float2.zero;
            deltaSpeed = 0f;
            return;
        }

        float t = -math.dot(p, vRel) / v2;
        if (t <= 0f || t > horizon)
        {
            n = float2.zero;
            deltaSpeed = 0f;
            return;
        }

        float2 closest = p + vRel * t;
        float d = math.length(closest);

        if (d >= R)
        {
            n = float2.zero;
            deltaSpeed = 0f;
            return;
        }

        n = math.normalizesafe(closest, SafePerp(SafeNormal(vRel)));
        float tau2 = math.max(minRespTime, t);

        float vRelN = math.dot(vRel, n);
        float requiredOut = (R - d) / tau2;

        deltaSpeed = requiredOut - vRelN;
        if (deltaSpeed < 0f) deltaSpeed = 0f;
    }

    private static float2 SafePerp(float2 v) => new float2(-v.y, v.x);

    private static float2 SafeNormal(float2 v) => math.normalizesafe(v, kDefaultDir);

    private static float2 RotateTowards2D(float2 from, float2 to, float maxRadians)
    {
        from = math.normalizesafe(from, kDefaultDir);
        to = math.normalizesafe(to, from);

        float cross = from.x * to.y - from.y * to.x;
        float dot = math.clamp(math.dot(from, to), -1f, 1f);
        float angle = math.atan2(cross, dot);

        float step = math.clamp(angle, -maxRadians, maxRadians);

        float s = math.sin(step);
        float c = math.cos(step);

        // rotation preserves normalization
        return new float2(from.x * c - from.y * s, from.x * s + from.y * c);
    }

    // ---------------- Scenario Spawning ----------------

    private void InitLocomotion(ref Agent a, float2 initialMoveDir)
    {
        bool isCar = Random.value < 0.5f;
        a.axisFreedom = isCar ? 0f : 1f;

        a.moveAxis = math.normalizesafe(initialMoveDir, kDefaultDir);

        if (isCar)
        {
            a.facingMode = FacingMode.LinkedToMoveAxis;
            a.facing = a.moveAxis;
        }
        else
        {
            int r = Random.Range(0, 3);
            a.facingMode = (r == 0) ? FacingMode.ToVelocity :
                           (r == 1) ? FacingMode.ToDestination :
                                      FacingMode.ToPoint;

            a.facing = a.moveAxis;
            a.aimPoint = RandomPointInBounds(_worldCenter, boundsHalfExtents);
        }
    }

    private void SpawnRandomDestinations(int count)
    {
        _agents = new Agent[count];

        for (int i = 0; i < count; i++)
        {
            Vector2 rnd = Random.insideUnitCircle;
            float2 pos = _worldCenter + new float2(rnd.x, rnd.y) * (math.min(boundsHalfExtents.x, boundsHalfExtents.y) * 0.8f);

            Vector2 rndDir = Random.insideUnitCircle;
            float2 dir = math.normalizesafe(new float2(rndDir.x, rndDir.y), kDefaultDir);

            float r = radius;
            float hl = math.max(0f, length * 0.5f);
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

            InitLocomotion(ref _agents[i], dir);
        }
    }

    private void SpawnDenseClumpsCrossing(int count)
    {
        _agents = new Agent[count];

        float2 leftCenter = _worldCenter + new float2(-clumpSeparationX * 0.5f, 0f);
        float2 rightCenter = _worldCenter + new float2(+clumpSeparationX * 0.5f, 0f);

        int half = count / 2;

        for (int i = 0; i < count; i++)
        {
            bool leftGroup = i < half;

            Vector2 rnd = Random.insideUnitCircle;
            float2 c = leftGroup ? leftCenter : rightCenter;
            float2 pos = c + new float2(rnd.x, rnd.y) * clumpRadius;

            float2 desiredDir = leftGroup ? new float2(1, 0) : new float2(-1, 0);
            float2 vel = desiredDir * speed;

            float r = radius;
            float hl = math.max(0f, length * 0.5f);
            if (varySizes)
            {
                r *= Random.Range(0.9f, 1.15f);
                hl *= Random.Range(0.75f, 1.25f);
            }

            float2 target = leftGroup
                ? (_worldCenter + new float2(+boundsHalfExtents.x * 0.9f, Random.Range(-boundsHalfExtents.y, boundsHalfExtents.y)))
                : (_worldCenter + new float2(-boundsHalfExtents.x * 0.9f, Random.Range(-boundsHalfExtents.y, boundsHalfExtents.y)));

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

            InitLocomotion(ref _agents[i], desiredDir);
        }
    }

    private void SpawnMovingThroughIdle(int movers, int idlers)
    {
        _agents = new Agent[movers + idlers];

        float2 moveCenter = _worldCenter + new float2(-boundsHalfExtents.x * 0.75f, 0f);
        float2 idleCenter = _worldCenter;

        // movers
        for (int i = 0; i < movers; i++)
        {
            Vector2 rnd = Random.insideUnitCircle;
            float2 pos = moveCenter + new float2(rnd.x, rnd.y) * clumpRadius;

            float2 desiredDir = new float2(1, 0);
            float2 vel = desiredDir * speed;

            float r = radius;
            float hl = math.max(0f, length * 0.5f);
            if (varySizes)
            {
                r *= Random.Range(0.9f, 1.15f);
                hl *= Random.Range(0.75f, 1.25f);
            }

            float2 target = _worldCenter + new float2(+boundsHalfExtents.x * 0.85f, Random.Range(-boundsHalfExtents.y, boundsHalfExtents.y));

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

            InitLocomotion(ref _agents[i], desiredDir);
        }

        // idlers (pushable)
        for (int k = 0; k < idlers; k++)
        {
            int i = movers + k;

            Vector2 rnd = Random.insideUnitCircle;
            float2 pos = idleCenter + new float2(rnd.x, rnd.y) * idleClumpRadius;

            float r = radius;
            float hl = math.max(0f, length * 0.5f);
            if (varySizes)
            {
                r *= Random.Range(0.9f, 1.15f);
                hl *= Random.Range(0.75f, 1.25f);
            }

            Vector2 rndFace = Random.insideUnitCircle; // FIX: sample once
            float2 face = math.normalizesafe(new float2(rndFace.x, rndFace.y), kDefaultDir);

            _agents[i] = new Agent
            {
                position = pos,
                velocity = float2.zero,
                facing = face,
                target = pos,
                radius = r,
                halfLength = hl,
                invMass = idlePushability,
                group = 1
            };

            InitLocomotion(ref _agents[i], _agents[i].facing);
        }
    }

    private void PickNextTarget(ref Agent a)
    {
        if (a.facingMode == FacingMode.ToPoint && Random.value < 0.35f)
            a.aimPoint = RandomPointInBounds(_worldCenter, boundsHalfExtents);

        switch (scenario)
        {
            case Scenario.RandomDestinations:
                a.target = RandomPointInBounds(_worldCenter, boundsHalfExtents);
                break;

            case Scenario.DenseClumpsCrossing:
                float x = a.position.x - _worldCenter.x;
                float sign = (x >= 0f) ? -1f : +1f;
                a.target = _worldCenter + new float2(sign * boundsHalfExtents.x * 0.9f, Random.Range(-boundsHalfExtents.y, boundsHalfExtents.y));
                break;

            case Scenario.MovingClumpThroughIdleClump:
                if (a.group == 0)
                {
                    float side = (a.position.x - _worldCenter.x) >= 0f ? -1f : +1f;
                    a.target = _worldCenter + new float2(side * boundsHalfExtents.x * 0.85f, Random.Range(-boundsHalfExtents.y, boundsHalfExtents.y));
                }
                break;
        }
    }

    // ---------------- Utility / Gizmos ----------------

    private static float2 RandomPointInBounds(float2 center, float2 halfExt)
    {
        return center + new float2(
            Random.Range(-halfExt.x, halfExt.x),
            Random.Range(-halfExt.y, halfExt.y));
    }

    private static void BounceInBounds(ref Agent a, float2 center, float2 halfExt)
    {
        float2 p = a.position - center;

        if (p.x < -halfExt.x) { p.x = -halfExt.x; if (a.invMass > 0f) a.velocity.x = math.abs(a.velocity.x); }
        if (p.x > halfExt.x) { p.x = halfExt.x; if (a.invMass > 0f) a.velocity.x = -math.abs(a.velocity.x); }
        if (p.y < -halfExt.y) { p.y = -halfExt.y; if (a.invMass > 0f) a.velocity.y = math.abs(a.velocity.y); }
        if (p.y > halfExt.y) { p.y = halfExt.y; if (a.invMass > 0f) a.velocity.y = -math.abs(a.velocity.y); }

        a.position = center + p;
    }

    private static void DrawCapsule2D(float2 center, float2 direction, float halfLen, float radius)
    {
        float2 dir = math.normalizesafe(direction, kDefaultDir);
        float2 a = center - dir * halfLen;
        float2 b = center + dir * halfLen;

        Gizmos.DrawLine(ToV3(a), ToV3(b));
        Gizmos.DrawWireSphere(ToV3(a), radius);
        Gizmos.DrawWireSphere(ToV3(b), radius);
    }

    private static void DrawArrow2D(float2 from, float2 to)
    {
        Gizmos.DrawLine(ToV3(from), ToV3(to));

        float2 d = to - from;
        if (math.lengthsq(d) < 1e-12f) return;

        float2 dir = math.normalizesafe(d, kDefaultDir);
        float2 left = new float2(-dir.y, dir.x);

        const float headLen = 0.12f;
        const float headWid = 0.08f;

        float2 p1 = to - dir * headLen + left * headWid;
        float2 p2 = to - dir * headLen - left * headWid;

        Gizmos.DrawLine(ToV3(to), ToV3(p1));
        Gizmos.DrawLine(ToV3(to), ToV3(p2));
    }

    private static Vector3 ToV3(float2 v) => new Vector3(v.x, v.y, 0f);

    void OnDrawGizmos()
    {
        if (_agents == null) return;

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
                    a.invMass <= 0f ? new Color(1f, 0.2f, 0.9f, 1f) :
                    a.group == 0 ? new Color(0.2f, 1f, 0.2f, 1f) :
                                   new Color(0.2f, 0.7f, 1f, 1f);

                DrawCapsule2D(a.position, a.facing, a.halfLength, a.radius);

                Gizmos.color = new Color(1f, 0.3f, 1f, 0.9f);
                DrawArrow2D(a.position, a.position + SafeNormal(a.facing) * 0.6f);
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

    [System.Diagnostics.Conditional("UNITY_ASSERTIONS")]
    private void ValidateSettings_AssertOnly()
    {
        Assert.IsTrue(speed >= 0f);
        Assert.IsTrue(maxAccel >= 0f);
        Assert.IsTrue(iterations >= 1);
        Assert.IsTrue(radius > 0f);
        Assert.IsTrue(length >= 0f);
        Assert.IsTrue(horizonSeconds > 0f);
        Assert.IsTrue(minResponseTime > 0f);
        Assert.IsTrue(speedHoldWhenMisaligned >= 0f && speedHoldWhenMisaligned <= 1f);
    }
}
