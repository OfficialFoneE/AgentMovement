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
    public static EllipseCollisionResult IntersectEllipses(in Ellipse2D A, in Ellipse2D B)
    {
        EllipseCollisionResult result = default;

        float2 delta = B.position - A.position;
        float distSq = lengthsq(delta);

        // If same position -> fallback case
        if (distSq < 1e-8f)
        {
            result.intersecting = true;

            // Pick a stable fallback direction:
            float2 normal = math.normalize(new float2(0.73f, 0.68f));//normalize(A.forward);
            if (any(isnan(normal)) || all(normal == float2(0)))
                normal = float2(1, 0);

            float2 avgRadii = (A.radii + B.radii) * 0.5f;
            float penetrationMag = max(avgRadii.x, avgRadii.y);

            result.normal = normal;
            result.penetrationDepth = penetrationMag;
            return result;
        }

        float2 n = normalize(delta);

        // Generate basis for each ellipse from its forward vector
        float2 A_right = float2(-A.forward.y, A.forward.x);
        float2 B_right = float2(-B.forward.y, B.forward.x);

        // Transform direction into ellipse local axis
        float2 nA = float2(dot(n, A.forward), dot(n, A_right));
        float2 nB = float2(dot(n, B.forward), dot(n, B_right));

        // Effective "radius" in the direction of n (ellipse support function)
        float rA = 1f / sqrt((nA.x * nA.x) / (A.radii.x * A.radii.x) +
                             (nA.y * nA.y) / (A.radii.y * A.radii.y));

        float rB = 1f / sqrt((nB.x * nB.x) / (B.radii.x * B.radii.x) +
                             (nB.y * nB.y) / (B.radii.y * B.radii.y));

        float centerDist = sqrt(distSq);
        float penetration = rA + rB - centerDist;

        if (penetration > 0f)
        {
            result.intersecting = true;
            result.normal = n; // direction from A -> B
            result.penetrationDepth = penetration;
        }

        return result;
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
