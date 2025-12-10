using Unity.Mathematics;
using static Unity.Mathematics.math;

[System.Serializable]
public struct Ellipse2D
{
    public float2 position;
    public float2 radii;  // half width (rx, ry)
    public float2 forward; // normalized orientation forward (major axis)
}

public struct EllipseCollisionResult
{
    public bool intersecting;
    public float2 normal;
    public float penetrationDepth;
}

public static class EllipseCollision
{
    public static EllipseCollisionResult IntersectEllipses(
           in Ellipse2D A,
           in Ellipse2D B)
    {
        EllipseCollisionResult result = default;

        float2 delta = B.position - A.position;
        float distSq = lengthsq(delta);

        // Handle same position
        if (distSq < 1e-10f)
        {
            float2 fallback = normalize(A.forward);
            if (!all(isfinite(fallback))) fallback = float2(1, 0);
            result.intersecting = true;
            result.normal = fallback;
            result.penetrationDepth = max(cmax(A.radii), cmax(B.radii));
            return result;
        }

        float2 initialNormal = normalize(delta);

        // Approx penetration using support function (ellipse -> circle projection method)
        float rA = ProjectRadius(A, initialNormal);
        float rB = ProjectRadius(B, -initialNormal);
        float centerDist = sqrt(distSq);
        float penetration = rA + rB - centerDist;

        if (penetration <= 0f)
        {
            result.intersecting = false;
            return result;
        }

        // -------------------------
        // Refinement step
        // -------------------------
        // Push the normal slightly toward the true collision direction
        float2 refinedNormal = RefineNormal(A, B, initialNormal);

        // Recalculate penetration depth using refined normal
        rA = ProjectRadius(A, refinedNormal);
        rB = ProjectRadius(B, -refinedNormal);
        penetration = rA + rB - centerDist;
        penetration = max(penetration, 0f); // clamp for
        refinedNormal = normalize(refinedNormal);

        result.intersecting = true;
        result.normal = refinedNormal;
        result.penetrationDepth = penetration;
        return result;
    }

    // Projection of ellipse radius along direction n
    private static float ProjectRadius(in Ellipse2D e, float2 n)
    {
        float2 f = normalize(e.forward);
        float2 r = new float2(-f.y, f.x);

        float nx = dot(n, f);
        float ny = dot(n, r);

        float rx = e.radii.x;
        float ry = e.radii.y;

        float denom = sqrt((nx * nx) / (rx * rx) + (ny * ny) / (ry * ry));
        if (denom < 1e-9f) return 0f;
        return 1f / denom;
    }

    // Normal correction -> 1 or 2 iterations is enough
    private static float2 RefineNormal(in Ellipse2D A, in Ellipse2D B, float2 n)
    {
        for (int i = 0; i < 2; i++) // keeping it ultra cheap
        {
            float rA = ProjectRadius(A, n);
            float rB = ProjectRadius(B, -n);

            float2 pA = A.position + n * rA;
            float2 pB = B.position - n * rB;

            float2 correction = normalize(pB - pA);
            if (all(isfinite(correction)))
                n = correction;
        }
        return normalize(n);
    }
}

//public struct Ellipse2D
//{
//    public float2 center;
//    public float2 forwardDirection;   // Unit forward vector
//    public float forwardRadius;      // Semi-axis along forward
//    public float rightRadius;        // Semi-axis perpendicular to forward

//    private static float2 Right(float2 forward) => new float2(-forward.y, forward.x); // 2D perpendicular

//    private static float2 ToLocalBasis(float2 v, float2 forward, float2 right)
//    {
//        return new float2(math.dot(v, right), math.dot(v, forward));
//    }

//    private static float2 ToWorldPoint(float2 localScaled, Ellipse2D E, float2 forward, float2 right)
//    {
//        return E.center + (right * localScaled.x) + (forward * localScaled.y);
//    }

//    public static bool Overlap(
//        Ellipse2D A,
//        Ellipse2D B,
//        out float2 normal,
//        out float penetration)
//    {
//        normal = float2.zero;
//        penetration = 0f;

//        float2 deltaAB = A.center - B.center;
//        float sqDist = math.lengthsq(deltaAB);

//        // Degenerate case: same center
//        if (sqDist < 1e-10f)
//        {
//            float2 righA = Right(A.forwardDirection);
//            normal = math.normalize(righA + A.forwardDirection);
//            penetration = math.max(A.forwardRadius, A.rightRadius);
//            return true;
//        }

//        float2 direction = math.normalize(deltaAB);

//        // ---------------------------------------------------------
//        // STEP 1 - Find supporting point on B’s ellipse
//        // ---------------------------------------------------------
//        float2 rightB = Right(B.forwardDirection);
//        float2 dirLocalB = ToLocalBasis(direction, B.forwardDirection, rightB);

//        // Scale inside normalized ellipse space
//        float2 normalizedLocalDirB = dirLocalB / new float2(B.rightRadius, B.forwardRadius);
//        float stretchB = math.length(normalizedLocalDirB);

//        // Closest point on B boundary along direction
//        float2 PB = B.center + (direction / stretchB);

//        // ---------------------------------------------------------
//        // STEP 2 - Check if PB is inside A’s ellipse
//        // ---------------------------------------------------------
//        float2 rightA = Right(A.forwardDirection);
//        float2 localPB_A = ToLocalBasis(PB - A.center, A.forwardDirection, rightA);

//        float2 pbNormalizedA = localPB_A / new float2(A.rightRadius, A.forwardRadius);
//        float insideMetricA = math.length(pbNormalizedA);

//        if (insideMetricA >= 1f)
//            return false; // No overlap

//        // ---------------------------------------------------------
//        // STEP 3 - Project to A boundary to find penetration
//        // ---------------------------------------------------------
//        float2 dirToPB_A = math.normalize(PB - A.center);
//        float2 dirToPBLocalA = ToLocalBasis(dirToPB_A, A.forwardDirection, rightA);

//        float2 dirNormalizedLocalA =
//            dirToPBLocalA / new float2(A.rightRadius, A.forwardRadius);

//        float stretchToSurfaceA = math.length(dirNormalizedLocalA);
//        float2 PA = A.center + (dirToPB_A / stretchToSurfaceA);

//        // ---------------------------------------------------------
//        // STEP 4 - Final normal + penetration
//        // ---------------------------------------------------------
//        float2 correction = PB - PA;
//        float correctionLength = math.length(correction);

//        if (correctionLength < 1e-10f)
//            return false;

//        normal = correction / correctionLength;
//        penetration = correctionLength;

//        return true;
//    }
//}
