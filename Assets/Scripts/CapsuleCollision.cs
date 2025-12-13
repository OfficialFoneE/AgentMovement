using Unity.Mathematics;

public struct CollisionResult
{
    public bool colliding;
    public float2 normal;         // Points from capsule2 toward capsule1
    public float penetrationDepth;
}

public class CapsuleCollision
{
    // Main collision detection function
    public static CollisionResult CheckCollision(Capsule2D capsule1, Capsule2D capsule2)
    {
        CollisionResult result = new CollisionResult();

        // Get the line segments for both capsules
        capsule1.GetEndPoints(out float2 a1, out float2 a2);
        capsule2.GetEndPoints(out float2 b1, out float2 b2);

        // Find closest points between the two line segments
        ClosestPointsOnSegments(a1, a2, b1, b2, out float2 closestA, out float2 closestB);

        // Calculate distance between closest points
        float2 diff = closestA - closestB;
        float distSq = diff.x * diff.x + diff.y * diff.y;
        float combinedRadius = capsule1.radius + capsule2.radius;

        // Check if collision occurs
        if (distSq >= combinedRadius * combinedRadius)
        {
            result.colliding = false;
            return result;
        }

        result.colliding = true;
        float dist = (float)math.sqrt(distSq);

        // Calculate basic normal and penetration
        if (dist > 1e-6f)
        {
            result.normal = diff / dist;
            result.penetrationDepth = combinedRadius - dist;
        }
        else
        {
            // Capsules are directly on top of each other - use direction between centers
            float2 centerDiff = capsule1.position - capsule2.position;
            float centerDist = math.length(centerDiff);
            result.normal = centerDist > 1e-6f ? math.normalize(centerDiff) : new float2(1, 0);
            result.penetrationDepth = combinedRadius;
        }

        // Apply smoothing to prevent flat-edge clustering
        float2 originalNormal = result.normal;
        result.normal = SmoothNormal(result.normal, closestA, closestB, capsule1, capsule2);

        // Recalculate penetration depth along the new normal if it changed
        if (math.abs(result.normal.x - originalNormal.x) > 1e-6f ||
            math.abs(result.normal.y - originalNormal.y) > 1e-6f)
        {
            result.penetrationDepth = RecalculatePenetrationDepth(capsule1, capsule2, result.normal);
        }

        return result;
    }

    // Smooth the normal based on where collision occurs to prevent clustering on flat ends
    private static float2 SmoothNormal(float2 normal, float2 closestA, float2 closestB,
                                        Capsule2D cap1, Capsule2D cap2)
    {
        // Calculate how close we are to the end caps
        float endFactorA = GetEndCapFactor(closestA, cap1);
        float endFactorB = GetEndCapFactor(closestB, cap2);

        // If collision is near the end caps, blend toward center-to-center direction
        float maxEndFactor = math.max(endFactorA, endFactorB);

        if (maxEndFactor > 0.7f) // Near the ends
        {
            float2 centerDir = math.normalize(cap1.position - cap2.position);

            // Smoothly blend between collision normal and center direction
            float blend = (maxEndFactor - 0.7f) / 0.3f; // 0.7 to 1.0 maps to 0 to 1
            blend = math.min(blend * 0.5f, 0.5f); // Cap blend at 50%

            float2 blended = normal * (1 - blend) + centerDir * blend;
            return math.normalize(blended);
        }

        return normal;
    }

    // Recalculate penetration depth along a given normal direction
    private static float RecalculatePenetrationDepth(Capsule2D cap1, Capsule2D cap2, float2 normal)
    {
        // Project both capsules onto the normal axis and find overlap
        ProjectCapsuleOntoAxis(cap1, normal, out float min1, out float max1);
        ProjectCapsuleOntoAxis(cap2, normal, out float min2, out float max2);

        // Calculate overlap along the axis
        float overlap1 = max1 - min2; // cap1's max overlaps cap2's min
        float overlap2 = max2 - min1; // cap2's max overlaps cap1's min

        // Return the smaller overlap (actual penetration)
        return math.min(overlap1, overlap2);
    }

    // Project a capsule onto an axis and get its min/max extents
    private static void ProjectCapsuleOntoAxis(Capsule2D capsule, float2 axis, out float min, out float max)
    {
        capsule.GetEndPoints(out float2 p1, out float2 p2);

        // Project both endpoints onto the axis
        float proj1 = math.dot(p1, axis);
        float proj2 = math.dot(p2, axis);

        // The capsule extends by its radius in both directions
        min = math.min(proj1, proj2) - capsule.radius;
        max = math.max(proj1, proj2) + capsule.radius;
    }

    // Calculate how close a point is to the end caps (0 = center, 1 = at end)
    private static float GetEndCapFactor(float2 point, Capsule2D capsule)
    {
        //capsule.GetEndPoints(out float2 p1, out float2 p2);

        // Project point onto the capsule's axis
        float2 axis = capsule.forward;
        float2 toPoint = point - capsule.position;
        float projection = math.abs(math.dot(toPoint, axis));

        // Normalize to 0-1 range (0 at center, 1 at ends)
        return capsule.halfLength > 0 ? math.min(projection / capsule.halfLength, 1.0f) : 0;
    }

    // Find closest points between two line segments
    private static void ClosestPointsOnSegments(float2 a1, float2 a2, float2 b1, float2 b2,
                                                out float2 closestA, out float2 closestB)
    {
        float2 d1 = a2 - a1;
        float2 d2 = b2 - b1;
        float2 r = a1 - b1;

        float a = math.dot(d1, d1);
        float e = math.dot(d2, d2);
        float f = math.dot(d2, r);

        float s, t;

        // Check if either or both segments degenerate into points
        if (a <= 1e-6f && e <= 1e-6f)
        {
            closestA = a1;
            closestB = b1;
            return;
        }

        if (a <= 1e-6f)
        {
            s = 0.0f;
            t = math.saturate(f / e);
        }
        else
        {
            float c = math.dot(d1, r);
            if (e <= 1e-6f)
            {
                t = 0.0f;
                s = math.saturate(-c / a);
            }
            else
            {
                float b = math.dot(d1, d2);
                float denom = a * e - b * b;

                if (denom != 0.0f)
                    s = math.saturate((b * f - c * e) / denom);
                else
                    s = 0.0f;

                t = (b * s + f) / e;

                if (t < 0.0f)
                {
                    t = 0.0f;
                    s = math.saturate(-c / a);
                }
                else if (t > 1.0f)
                {
                    t = 1.0f;
                    s = math.saturate((b - c) / a);
                }
            }
        }

        closestA = a1 + d1 * s;
        closestB = b1 + d2 * t;
    }
}
