using Unity.Mathematics;
using UnityEngine;
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
    public int agentCount = 60;              // used by RandomDestinations + DenseClumpsCrossing
    public int movingCount = 40;             // used by MovingClumpThroughIdleClump
    public int idleCount = 40;               // used by MovingClumpThroughIdleClump

    [Header("World / Bounds")]
    public bool bounceBounds = true;
    public float2 boundsHalfExtents = new float2(12, 7);

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

    [Header("Speed Hold (replaces hard preserveSpeed normalize)")]
    [Range(0f, 1f)] public float speedHoldWhenMisaligned = 0.15f; // 0..1, how much speed we keep when moving opposite desired

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
    public enum FacingMode
    {
        LinkedToMoveAxis,   // car
        ToVelocity,         // rocket points where it's moving
        ToDestination,      // points to its target destination
        ToPoint             // points to some arbitrary aim point
    }

    private struct Agent
    {
        public float2 position;
        public float2 velocity;

        public float2 moveAxis;      // NEW: locomotion axis (car constraint axis)
        public float2 facing;        // keep: visual/aim direction

        public FacingMode facingMode; // NEW
        public float2 aimPoint;      // NEW: used by FacingMode.ToPoint

        public float2 target;
        public float halfLength;
        public float radius;

        public float invMass;         // pushability weight (your existing meaning)
        public float axisFreedom;     // NEW: 0 = car, 1 = hover (can randomize)
        public int group;
    }

    private Agent[] _agents;
    private float2[] _desiredVels;
    private float2 _worldCenter;

    [ContextMenu("Reset Scenario")]
    public void ResetScenario()
    {
        Random.InitState(randomSeed);
        _worldCenter = (Vector2)(transform.position);

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
    }

    void OnEnable() => ResetScenario();

    void FixedUpdate()
    {
        if (_agents == null || _agents.Length == 0) return;

        float dt = Time.fixedDeltaTime;
        int iters = Mathf.Max(1, iterations);
        float dtIter = dt / iters;

        // IMPORTANT: when you run projection iters times, budgets must be per-iter.
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

            float2 fallbackDir = (math.lengthsq(a.facing) > 1e-6f) ? a.facing : a.moveAxis;
            float2 dir = (math.lengthsq(toT) > 1e-6f) ? math.normalize(toT) : SafeNormal(fallbackDir);

            _desiredVels[i] = dir * speed;
        }

        // 2) Iterative avoidance + steer + projection
        float2[] deltaV = new float2[_agents.Length];

        for (int iter = 0; iter < iters; iter++)
        {
            System.Array.Clear(deltaV, 0, deltaV.Length);

            // Pairwise avoidance (multi-circle)
            for (int i = 0; i < _agents.Length; i++)
            {
                for (int j = i + 1; j < _agents.Length; j++)
                {
                    ref Agent A = ref _agents[i];
                    ref Agent B = ref _agents[j];

                    if (A.invMass <= 0f && B.invMass <= 0f)
                        continue;

                    // Compute ONE correction vector for the whole capsule pair
                    float2 full = ComputeAvoidanceCorrection_MultiCircle(
                        A.position, A.velocity, SafeNormal(A.facing), A.halfLength, A.radius,
                        B.position, B.velocity, SafeNormal(B.facing), B.halfLength, B.radius,
                        horizonSeconds, extraSeparation, maxDeltaSpeed, minResponseTime);

                    // Distribute by pushability (invMass is your pushability weight)
                    float wA = A.invMass;
                    float wB = B.invMass;
                    float wSum = wA + wB;

                    if (wSum > 1e-6f && math.lengthsq(full) > 1e-12f)
                    {
                        float shareA = wA / wSum;
                        float shareB = wB / wSum;

                        deltaV[i] += (-full * shareA);
                        deltaV[j] += (full * shareB);
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

                // Command velocity
                float2 vCmd = a.velocity + deltaV[i];

                // Goal steering (per-iter accel budget)
                float2 steer = desired - vCmd;
                float smag = math.length(steer);
                if (smag > maxSteerDvIter && smag > 1e-6f)
                    steer *= (maxSteerDvIter / smag);

                vCmd += steer * Mathf.Clamp01(realignStrength);

                // Optional clamp for stability
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

            // NOTE: do NOT overwrite a.facing here.
            // Facing is managed ONLY inside ProjectLocomotion based on FacingMode.
        }
    }

    private void ProjectLocomotion(ref Agent a, float2 vCmd, float2 desiredVel, float dt)
    {
        float maxTurnRad = maxTurnDegPerSec * Mathf.Deg2Rad * dt;
        float maxDv = maxAccel * dt;

        float axisFreedom01 = Mathf.Clamp01(a.axisFreedom);

        // --- 1) Update moveAxis ---
        // For car-like agents (axisFreedom ~ 0), aim the body axis toward the GOAL, not the command.
        // This lets avoidance temporarily push velocity backwards without immediately flipping the body.
        float2 axisDesired;

        if (axisFreedom01 < 1e-3f)
        {
            axisDesired =
                (math.lengthsq(desiredVel) > 1e-6f) ? desiredVel :
                (math.lengthsq(a.moveAxis) > 1e-6f) ? a.moveAxis :
                vCmd;
        }
        else
        {
            axisDesired =
                (math.lengthsq(vCmd) > 1e-6f) ? vCmd :
                (math.lengthsq(desiredVel) > 1e-6f) ? desiredVel :
                a.moveAxis;
        }

        axisDesired = SafeNormal(axisDesired);

        float axisTurn = Mathf.Lerp(maxTurnRad, Mathf.PI, axisFreedom01);
        a.moveAxis = RotateTowards2D(a.moveAxis, axisDesired, axisTurn);

        // --- 2) Accel constrain: car only along moveAxis, hover full 2D ---
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

        // clamp max speed
        float vmag = math.length(v);
        if (vmag > speed && vmag > 1e-6f)
            v = (v / vmag) * speed;

        // Speed hold controller (your improved preserveSpeed behavior)
        if (preserveSpeed)
        {
            float desiredSpeed = math.length(desiredVel);
            if (desiredSpeed > 1e-6f)
            {
                float2 desiredDir = desiredVel / desiredSpeed;

                float curSpeed = math.length(v);
                float2 curDir = (curSpeed > 1e-6f) ? (v / curSpeed) : desiredDir;

                float align = math.dot(curDir, desiredDir);

                float t = Mathf.Clamp01((align + 1f) * 0.5f);
                float targetSpeed = Mathf.Lerp(speedHoldWhenMisaligned * desiredSpeed, desiredSpeed, t);

                float ds = targetSpeed - curSpeed;
                float dsStep = Mathf.Clamp(ds, -maxDv, maxDv);

                v += curDir * dsStep;

                float vmag2 = math.length(v);
                if (vmag2 > speed && vmag2 > 1e-6f)
                    v = (v / vmag2) * speed;
            }
        }

        a.velocity = v;

        // --- 3) Facing update (this defines BODY axis used by the physical collider) ---
        float2 faceDesired;

        switch (a.facingMode)
        {
            default:
            case FacingMode.LinkedToMoveAxis:
                faceDesired = a.moveAxis;
                break;

            case FacingMode.ToVelocity:
                faceDesired = (math.lengthsq(a.velocity) > 1e-6f) ? a.velocity : a.moveAxis;
                break;

            case FacingMode.ToDestination:
                faceDesired = (a.target - a.position);
                if (math.lengthsq(faceDesired) < 1e-6f) faceDesired = a.moveAxis;
                break;

            case FacingMode.ToPoint:
                faceDesired = (a.aimPoint - a.position);
                if (math.lengthsq(faceDesired) < 1e-6f) faceDesired = a.moveAxis;
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

        // Sample counts adapt to length vs radius (kept small for stability)
        int nA = GetCircleSampleCount(halfLenA, rA);
        int nB = GetCircleSampleCount(halfLenB, rB);

        float2 vRel = vB - vA;

        float bestDelta = 0f;
        float2 bestN = float2.zero;

        // Offsets along capsule axis from -halfLen..+halfLen
        for (int ia = 0; ia < nA; ia++)
        {
            float ta = (nA == 1) ? 0.5f : (ia / (float)(nA - 1));
            float offA = Mathf.Lerp(-halfLenA, +halfLenA, ta);
            float2 pA = cA + axisA * offA;

            for (int ib = 0; ib < nB; ib++)
            {
                float tb = (nB == 1) ? 0.5f : (ib / (float)(nB - 1));
                float offB = Mathf.Lerp(-halfLenB, +halfLenB, tb);
                float2 pB = cB + axisB * offB;

                // Disc-disc VO check (returns required outward deltaSpeed along normal)
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
            bestDelta = Mathf.Min(bestDelta, maxDelta);

        return bestN * bestDelta;
    }

    private static int GetCircleSampleCount(float halfLen, float radius)
    {
        // 3 for normal pills; 5 if noticeably longer. Clamp to stay cheap.
        float fullLen = Mathf.Max(0f, halfLen) * 2f;
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

        // If already overlapping (or extremely close), push apart immediately.
        if (dist < R)
        {
            n = (dist > 1e-6f) ? (p / dist) : new float2(1, 0);
            float tau = Mathf.Max(minRespTime, 1e-4f);
            deltaSpeed = (R - dist) / tau;
            return;
        }

        float v2 = math.lengthsq(vRel);
        if (v2 < 1e-10f)
        {
            // Not moving relative to each other; no predicted collision if currently separated.
            n = float2.zero;
            deltaSpeed = 0f;
            return;
        }

        // Time of closest approach in [0, horizon]
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

        // Need to increase separation along n over a response time tau
        n = (d > 1e-6f) ? (closest / d) : SafePerp(SafeNormal(vRel));
        float tau2 = Mathf.Max(minRespTime, t);

        float vRelN = math.dot(vRel, n);
        float requiredOut = (R - d) / tau2;

        deltaSpeed = requiredOut - vRelN;
        if (deltaSpeed < 0f) deltaSpeed = 0f;
    }

    private static float2 SafePerp(float2 v) => new float2(-v.y, v.x);


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

                //DrawCapsule2D(a.position, a.moveAxis, a.halfLength, a.radius);
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

    private static float2 SafeNormal(float2 v)
    {
        float sq = math.lengthsq(v);
        return sq > 1e-8f ? v / Mathf.Sqrt(sq) : new float2(1, 0);
    }


    // ---------------- Scenario Spawning ----------------

    private void InitLocomotion(ref Agent a, float2 initialMoveDir)
    {
        // Randomize “type”: 50% car-like, 50% hover-like (tweak however you want)
        bool isCar = Random.value < 0.5f;
        a.axisFreedom = isCar ? 0f : 1f;

        a.moveAxis = (math.lengthsq(initialMoveDir) > 1e-6f) ? math.normalize(initialMoveDir) : new float2(1, 0);

        if (isCar)
        {
            a.facingMode = FacingMode.LinkedToMoveAxis;
            a.facing = a.moveAxis;
        }
        else
        {
            // Random hover facing behavior
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
            float2 pos = _worldCenter + (float2)Random.insideUnitCircle * (Mathf.Min(boundsHalfExtents.x, boundsHalfExtents.y) * 0.8f);
            float2 dir = Random.insideUnitCircle.normalized;
            if (math.lengthsq(dir) < 1e-6f) dir = new float2(1, 0);

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

            float2 c = leftGroup ? leftCenter : rightCenter;
            float2 pos = c + (float2)Random.insideUnitCircle * clumpRadius;

            float2 desiredDir = leftGroup ? new float2(1, 0) : new float2(-1, 0);
            float2 vel = desiredDir * speed;

            float r = radius;
            float hl = Mathf.Max(0f, length * 0.5f);
            if (varySizes)
            {
                r *= Random.Range(0.9f, 1.15f);
                hl *= Random.Range(0.75f, 1.25f);
            }

            // Targets are “other side”, and we ping-pong when reached (see PickNextTarget).
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
        float2 idleCenter = _worldCenter; // center clump

        // moving clump (group 0)
        for (int i = 0; i < movers; i++)
        {
            float2 pos = moveCenter + (float2)Random.insideUnitCircle * clumpRadius;

            float2 desiredDir = new float2(1, 0);
            float2 vel = desiredDir * speed;

            float r = radius;
            float hl = Mathf.Max(0f, length * 0.5f);
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

        // idle clump (invMass=0, group=1)
        for (int k = 0; k < idlers; k++)
        {
            int i = movers + k;

            float2 pos = idleCenter + (float2)Random.insideUnitCircle * idleClumpRadius;

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
                velocity = float2.zero,
                facing = math.lengthsq(Random.insideUnitCircle) > 1e-6f ? Random.insideUnitCircle.normalized : new float2(1, 0),
                target = pos,
                radius = r,
                halfLength = hl,
                invMass = idlePushability,
                group = 1
            };

            InitLocomotion(ref _agents[i], _agents[i].facing); // or Random.insideUnitCircle
        }
    }

    private void PickNextTarget(ref Agent a)
    {
        if (a.facingMode == FacingMode.ToPoint && Random.value < 0.35f)
            a.aimPoint = RandomPointInBounds(_worldCenter, boundsHalfExtents);

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
                a.target = _worldCenter + new float2(sign * boundsHalfExtents.x * 0.9f, Random.Range(-boundsHalfExtents.y, boundsHalfExtents.y));
                break;

            case Scenario.MovingClumpThroughIdleClump:
                // Movers: keep going left <-> right
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

        if (p.x < -halfExt.x) { p.x = -halfExt.x; if (a.invMass > 0f) a.velocity.x = Mathf.Abs(a.velocity.x); }
        if (p.x > halfExt.x) { p.x = halfExt.x; if (a.invMass > 0f) a.velocity.x = -Mathf.Abs(a.velocity.x); }
        if (p.y < -halfExt.y) { p.y = -halfExt.y; if (a.invMass > 0f) a.velocity.y = Mathf.Abs(a.velocity.y); }
        if (p.y > halfExt.y) { p.y = halfExt.y; if (a.invMass > 0f) a.velocity.y = -Mathf.Abs(a.velocity.y); }

        a.position = center + p;
    }

    private static float2 RotateTowards2D(float2 from, float2 to, float maxRadians)
    {
        if (math.lengthsq(from) < 1e-8f) from = new float2(1, 0);
        if (math.lengthsq(to) < 1e-8f) return math.normalize(from);

        from = math.normalize(from);
        to = math.normalize(to);

        float cross = from.x * to.y - from.y * to.x;
        float dot = Mathf.Clamp(math.dot(from, to), -1f, 1f);
        float angle = Mathf.Atan2(cross, dot);

        float step = Mathf.Clamp(angle, -maxRadians, maxRadians);

        float s = Mathf.Sin(step);
        float c = Mathf.Cos(step);

        return new float2(from.x * c - from.y * s, from.x * s + from.y * c);
    }

    private static void DrawCapsule2D(float2 center, float2 direction, float halfLen, float radius)
    {
        float2 dir = math.normalize(direction); //(velocity.sqrMagnitude > 1e-8f) ? velocity.normalized : (facing.sqrMagnitude > 1e-8f ? facing.normalized : new float2(1, 0));
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
        if (math.lengthsq(d) < 1e-8f) return;

        float2 dir = math.normalize(d);
        float2 left = new float2(-dir.y, dir.x);

        float headLen = 0.12f;
        float headWid = 0.08f;

        float2 p1 = to - dir * headLen + left * headWid;
        float2 p2 = to - dir * headLen - left * headWid;

        Gizmos.DrawLine(ToV3(to), ToV3(p1));
        Gizmos.DrawLine(ToV3(to), ToV3(p2));
    }

    private static float3 ToV3(float2 v) => new float3(v.x, v.y, 0f);
}
