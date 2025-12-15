using UnityEngine;

public static class Pill2DVO
{
    public struct Capsule
    {
        public Vector2 center;     // capsule center (midpoint of the segment)
        public Vector2 velocity;   // 2D velocity
        public float halfLength;   // half of the capsule segment length (excluding radius)
        public float radius;       // capsule radius

        public Vector2 DirectionOr(Vector2 fallbackDir)
        {
            float sq = velocity.sqrMagnitude;
            return sq > 1e-8f ? velocity / Mathf.Sqrt(sq) : fallbackDir;
        }
    }

    public struct DebugInfo
    {
        public bool willCollide;
        public float tMin;

        // Segment endpoints at tMin
        public Vector2 a0, a1, b0, b1;

        // Closest points at tMin
        public Vector2 pA, pB;

        public Vector2 normal;     // from A to B at closest approach
        public float minDist;
        public float requiredDeltaSpeed;

        public Vector2 newVelA, newVelB;
    }

    /// <summary>
    /// Computes new velocities for two moving 2D capsules (pills) to avoid a predicted collision.
    /// This is a simple reciprocal push along the separation normal at the predicted closest approach.
    /// </summary>
    public static void AvoidPair(
        Capsule a,
        Capsule b,
        float horizonSeconds,
        float extraSeparation,
        float maxDeltaSpeed,
        float minResponseTime,
        out Vector2 outVelA,
        out Vector2 outVelB,
        out DebugInfo dbg)
    {
        dbg = default;

        // Defaults: keep current velocities unless we decide to adjust.
        outVelA = a.velocity;
        outVelB = b.velocity;

        horizonSeconds = Mathf.Max(0f, horizonSeconds);
        float combinedRadius = a.radius + b.radius + Mathf.Max(0f, extraSeparation);
        float combinedRadiusSq = combinedRadius * combinedRadius;

        // If horizon is 0, just do an instantaneous overlap check at t=0.
        int searchIters = horizonSeconds > 0f ? 10 : 0;

        // Heading fallback if velocity is zero.
        Vector2 dirA0 = a.DirectionOr(Vector2.right);
        Vector2 dirB0 = b.DirectionOr(Vector2.right);

        // Find time within [0, horizon] that minimizes distance between the *moving* segments.
        float lo = 0f, hi = horizonSeconds;
        float bestT = 0f;
        float bestDistSq = float.PositiveInfinity;

        // Always evaluate endpoints too (helps robustness).
        EvaluateAt(0f, a, b, dirA0, dirB0, ref bestT, ref bestDistSq, out _);
        EvaluateAt(horizonSeconds, a, b, dirA0, dirB0, ref bestT, ref bestDistSq, out _);

        for (int i = 0; i < searchIters; i++)
        {
            float t1 = Mathf.Lerp(lo, hi, 1f / 3f);
            float t2 = Mathf.Lerp(lo, hi, 2f / 3f);

            float d1 = DistSqAt(t1, a, b, dirA0, dirB0);
            float d2 = DistSqAt(t2, a, b, dirA0, dirB0);

            if (d1 <= d2) hi = t2;
            else lo = t1;
        }

        float tMin = 0.5f * (lo + hi);
        Vector2 pA, pB;
        float distSq = DistSqAt(tMin, a, b, dirA0, dirB0, out Vector2 a0, out Vector2 a1, out Vector2 b0, out Vector2 b1, out pA, out pB);

        // Update best if needed
        if (distSq < bestDistSq)
        {
            bestDistSq = distSq;
            bestT = tMin;
        }

        // Recompute at bestT to fill debug precisely
        float finalDistSq = DistSqAt(bestT, a, b, dirA0, dirB0,
            out dbg.a0, out dbg.a1, out dbg.b0, out dbg.b1, out dbg.pA, out dbg.pB);

        dbg.tMin = bestT;

        if (finalDistSq > combinedRadiusSq)
        {
            dbg.willCollide = false;
            dbg.minDist = Mathf.Sqrt(finalDistSq);
            dbg.newVelA = outVelA;
            dbg.newVelB = outVelB;
            return;
        }

        dbg.willCollide = true;

        Vector2 sep = dbg.pB - dbg.pA;
        float dist = sep.magnitude;
        dbg.minDist = dist;

        Vector2 n;
        if (dist > 1e-6f)
        {
            n = sep / dist;
        }
        else
        {
            // Perfect overlap of closest points: pick something deterministic.
            Vector2 vRel = b.velocity - a.velocity;
            if (vRel.sqrMagnitude > 1e-8f)
            {
                // Normal perpendicular to relative motion (push sideways).
                Vector2 v = vRel.normalized;
                n = new Vector2(-v.y, v.x);
            }
            else
            {
                n = Vector2.up;
            }
        }

        dbg.normal = n;

        float needed = combinedRadius - dist; // how much extra distance we need at closest approach
        if (needed <= 0f)
        {
            dbg.requiredDeltaSpeed = 0f;
            dbg.newVelA = outVelA;
            dbg.newVelB = outVelB;
            return;
        }

        float responseTime = Mathf.Max(minResponseTime, bestT);
        responseTime = Mathf.Max(responseTime, 1e-4f);

        // Increase separation by adding relative speed along n for the remaining time.
        float deltaSpeed = needed / responseTime;
        if (maxDeltaSpeed > 0f) deltaSpeed = Mathf.Min(deltaSpeed, maxDeltaSpeed);

        dbg.requiredDeltaSpeed = deltaSpeed;

        // Reciprocal push: half applied to each.
        Vector2 delta = n * (0.5f * deltaSpeed);
        outVelA = a.velocity - delta;
        outVelB = b.velocity + delta;

        dbg.newVelA = outVelA;
        dbg.newVelB = outVelB;
    }

    // --- Internals ---

    private static float DistSqAt(float t, Capsule a, Capsule b, Vector2 dirA, Vector2 dirB)
    {
        return DistSqAt(t, a, b, dirA, dirB, out _, out _, out _, out _, out _, out _);
    }

    private static float DistSqAt(
        float t,
        Capsule a,
        Capsule b,
        Vector2 dirA,
        Vector2 dirB,
        out Vector2 a0, out Vector2 a1,
        out Vector2 b0, out Vector2 b1,
        out Vector2 pA, out Vector2 pB)
    {
        Vector2 ca = a.center + a.velocity * t;
        Vector2 cb = b.center + b.velocity * t;

        a0 = ca - dirA * a.halfLength;
        a1 = ca + dirA * a.halfLength;
        b0 = cb - dirB * b.halfLength;
        b1 = cb + dirB * b.halfLength;

        ClosestPointsOnSegments(a0, a1, b0, b1, out pA, out pB);
        return (pB - pA).sqrMagnitude;
    }

    // Real-Time Collision Detection (Christer Ericson) style segment-segment closest points.
    private static void ClosestPointsOnSegments(
        Vector2 p1, Vector2 q1,
        Vector2 p2, Vector2 q2,
        out Vector2 c1,
        out Vector2 c2)
    {
        Vector2 d1 = q1 - p1;   // direction of segment S1
        Vector2 d2 = q2 - p2;   // direction of segment S2
        Vector2 r = p1 - p2;
        float a = Vector2.Dot(d1, d1); // squared length of S1
        float e = Vector2.Dot(d2, d2); // squared length of S2
        float f = Vector2.Dot(d2, r);

        const float EPS = 1e-8f;

        float s, t;

        if (a <= EPS && e <= EPS)
        {
            // Both segments degenerate into points
            s = t = 0f;
            c1 = p1;
            c2 = p2;
            return;
        }

        if (a <= EPS)
        {
            // First segment degenerates into a point
            s = 0f;
            t = Mathf.Clamp01(f / e);
        }
        else
        {
            float c = Vector2.Dot(d1, r);
            if (e <= EPS)
            {
                // Second segment degenerates into a point
                t = 0f;
                s = Mathf.Clamp01(-c / a);
            }
            else
            {
                float b = Vector2.Dot(d1, d2);
                float denom = a * e - b * b;

                if (denom != 0f) s = Mathf.Clamp01((b * f - c * e) / denom);
                else s = 0f;

                float tNom = (b * s + f);
                if (tNom < 0f)
                {
                    t = 0f;
                    s = Mathf.Clamp01(-c / a);
                }
                else if (tNom > e)
                {
                    t = 1f;
                    s = Mathf.Clamp01((b - c) / a);
                }
                else
                {
                    t = tNom / e;
                }
            }
        }

        c1 = p1 + d1 * s;
        c2 = p2 + d2 * t;
    }

    private static void EvaluateAt(
        float t,
        Capsule a,
        Capsule b,
        Vector2 dirA,
        Vector2 dirB,
        ref float bestT,
        ref float bestDistSq,
        out (Vector2 pA, Vector2 pB) closestPts)
    {
        float dsq = DistSqAt(t, a, b, dirA, dirB, out _, out _, out _, out _, out Vector2 pA, out Vector2 pB);
        closestPts = (pA, pB);
        if (dsq < bestDistSq)
        {
            bestDistSq = dsq;
            bestT = t;
        }
    }
}
