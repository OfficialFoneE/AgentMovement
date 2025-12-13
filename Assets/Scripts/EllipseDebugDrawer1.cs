using Unity.Mathematics;
using UnityEngine;

public class EllipseDebugDrawer2 : Drawing.MonoBehaviourGizmos
{
    public Capsule2D ellipseA;
    public Capsule2D ellipseB;

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
    public static CrowdingInfo CalculateCrowding(Capsule2D capsule, CollisionResult[] collisions)
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
    private static float CalculateAngularSpread(Capsule2D capsule, CollisionResult[] collisions)
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
