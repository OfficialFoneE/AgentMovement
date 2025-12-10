using UnityEngine;
using Unity.Mathematics;
using static Unity.Mathematics.math;
using UnityEditor.Build.Pipeline;
using Drawing;

public class EllipseDebugDrawer2 : Drawing.MonoBehaviourGizmos
{
    public Capsule ellipseA;
    public Capsule ellipseB;

    public Color ellipseColorA = Color.green;
    public Color ellipseColorB = Color.cyan;
    public Color collisionColor = Color.red;

    // Number of segments to draw ellipse outline
    private const int Segments = 64;

    public override void DrawGizmos()
    {
        ellipseA.Draw();
        ellipseB.Draw();

        var results = CapsuleCollision.CheckCollision(ellipseA, ellipseB);

        if (results.colliding)
        {
            Drawing.Draw.xz.Line(ellipseA.position, ellipseA.position + results.normal * results.penetrationDepth);
        }
    }

}


public struct CrowdingInfo
{
    public float expansionFactor;  // Multiply radius by this (1.0 = no expansion, 1.5 = 50% bigger)
    public int collisionCount;
    public float totalPenetration;
    public float angularSpread;    // 0 to 1, where 1 = collisions from all directions
}

//// Calculate crowding
//var crowding = CapsuleCollision.CalculateCrowding(myUnit, collisions.ToArray());

//// Apply expansion if crowded
//if (crowding.expansionFactor > 1.0f)
//{
//    // Temporarily expand the capsule
//    var expandedUnit = myUnit;
//    expandedUnit.radius *= crowding.expansionFactor;

//    // Re-resolve collisions with expanded capsule
//    foreach (var other in otherUnits)
//    {
//        var result = CapsuleCollision.CheckCollision(expandedUnit, other);
//        if (result.colliding)
//        {
//            // Push other units away more aggressively
//            other.position -= result.normal * result.penetrationDepth;
//        }
//    }
//}

public class CrowdingCollision
{
    // Calculate crowding/pressure on a capsule from multiple collisions
    public static CrowdingInfo CalculateCrowding(Capsule capsule, CollisionResult[] collisions)
    {
        CrowdingInfo info = new CrowdingInfo();
        info.expansionFactor = 1.0f;

        if (collisions == null || collisions.Length == 0)
            return info;

        // Filter to only valid collisions
        int validCount = 0;
        float totalPenetration = 0;
        float avgPenetration = 0;

        foreach (var col in collisions)
        {
            if (col.colliding)
            {
                validCount++;
                totalPenetration += col.penetrationDepth;
            }
        }

        if (validCount == 0)
            return info;

        avgPenetration = totalPenetration / validCount;

        // Calculate angular spread of collisions (how surrounded are we?)
        float angularSpread = CalculateAngularSpread(capsule, collisions);

        // Calculate relative penetration (compared to capsule size)
        float capsuleSize = capsule.halfLength * 2 + capsule.radius * 2;
        float relativePenetration = avgPenetration / capsuleSize;

        // Only expand if we have significant, deep collisions from multiple directions
        float crowdingScore = 0;

        // Factor 1: Number of collisions (more collisions = more crowded)
        float countFactor = math.min(validCount / 8.0f, 1.0f); // Saturate at 8 collisions

        // Factor 2: Angular spread (surrounded = crowded)
        float spreadFactor = angularSpread;

        // Factor 3: Relative penetration depth (deep = truly stuck)
        float depthFactor = math.min(relativePenetration * 4.0f, 1.0f); // Saturate at 25% of size

        // Combine factors - all must be present for crowding
        // Use multiplication so if any factor is low, crowding is low
        crowdingScore = countFactor * spreadFactor * depthFactor;

        // Apply crowding threshold - only expand if truly crowded
        if (crowdingScore > 0.3f)
        {
            // Expansion scales with crowding, max 50% expansion
            info.expansionFactor = 1.0f + (crowdingScore - 0.3f) / 0.7f * 0.5f;
        }

        info.collisionCount = validCount;
        info.totalPenetration = totalPenetration;
        info.angularSpread = angularSpread;

        return info;
    }

    // Calculate how evenly distributed collisions are around the capsule (0 = one side, 1 = all sides)
    private static float CalculateAngularSpread(Capsule capsule, CollisionResult[] collisions)
    {
        if (collisions.Length < 2)
            return 0;

        // Collect angles of collision normals relative to capsule
        int validCount = 0;
        float[] angles = new float[collisions.Length];

        foreach (var col in collisions)
        {
            if (col.colliding)
            {
                // Get angle of normal pointing toward this capsule
                Vector2 dir = -col.normal; // Reverse because normal points away
                angles[validCount] = (float)math.atan2(dir.y, dir.x);
                validCount++;
            }
        }

        if (validCount < 2)
            return 0;

        // Sort angles
        System.Array.Sort(angles, 0, validCount);

        // Find largest gap between consecutive angles
        float maxGap = 0;
        for (int i = 0; i < validCount; i++)
        {
            int next = (i + 1) % validCount;
            float gap = angles[next] - angles[i];

            // Handle wraparound
            if (i == validCount - 1)
                gap = (float)(2 * math.PI) - angles[i] + angles[0];

            if (gap < 0)
                gap += (float)(2 * math.PI);

            maxGap = math.max(maxGap, gap);
        }

        // Convert max gap to spread score
        // If max gap is 2pi, all collisions are from one direction (spread = 0)
        // If max gap is 2pi/n, collisions are evenly distributed (spread = 1)
        float idealGap = (float)(2 * math.PI / validCount);
        float spreadScore = 1.0f - (maxGap - idealGap) / (float)(2 * math.PI - idealGap);

        return math.max(0, math.min(1, spreadScore));
    }
}





    [System.Serializable]
public struct Capsule
{
    public float2 position;  // Center position
    public float2 forward;  // Must be normalised!
    public float rotation;    // Rotation in radians
    public float halfLength;  // Half the length of the capsule's line segment
    public float radius;      // Radius around the line segment

    public Capsule(float2 pos, float2 forward, float rot, float halfLen, float rad)
    {
        position = pos;
        this.forward = forward;
        rotation = rot;
        halfLength = halfLen;
        radius = rad;
    }

    // Get the two end points of the capsule's central line segment
    public void GetEndPoints(out float2 p1, out float2 p2)
    {
        p1 = position - forward * halfLength;
        p2 = position + forward * halfLength;
    }

    public void Draw()
    {
        GetEndPoints(out var p1, out var p2);
        Drawing.Draw.xz.WirePill(p1, p2, radius);
    }
}

public struct CollisionResult
{
    public bool colliding;
    public float2 normal;         // Points from capsule2 toward capsule1
    public float penetrationDepth;
}





public class CapsuleCollision
{
    // Main collision detection function
    public static CollisionResult CheckCollision(Capsule capsule1, Capsule capsule2)
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
                                        Capsule cap1, Capsule cap2)
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
    private static float RecalculatePenetrationDepth(Capsule cap1, Capsule cap2, float2 normal)
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
    private static void ProjectCapsuleOntoAxis(Capsule capsule, float2 axis, out float min, out float max)
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
    private static float GetEndCapFactor(float2 point, Capsule capsule)
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
            t = saturate(f / e);
        }
        else
        {
            float c = math.dot(d1, r);
            if (e <= 1e-6f)
            {
                t = 0.0f;
                s = saturate(-c / a);
            }
            else
            {
                float b = math.dot(d1, d2);
                float denom = a * e - b * b;

                if (denom != 0.0f)
                    s = saturate((b * f - c * e) / denom);
                else
                    s = 0.0f;

                t = (b * s + f) / e;

                if (t < 0.0f)
                {
                    t = 0.0f;
                    s = saturate(-c / a);
                }
                else if (t > 1.0f)
                {
                    t = 1.0f;
                    s = saturate((b - c) / a);
                }
            }
        }

        closestA = a1 + d1 * s;
        closestB = b1 + d2 * t;
    }

    private static float Clamp01(float value)
    {
        if (value < 0.0f) return 0.0f;
        if (value > 1.0f) return 1.0f;
        return value;
    }
}
