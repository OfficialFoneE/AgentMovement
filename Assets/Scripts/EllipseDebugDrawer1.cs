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

    void OnDrawGizmos()
    {
        //ellipseA.Draw();
        //ellipseB.Draw();

        //var results = CapsuleCollision.CheckCollision(ellipseA, ellipseB);

        //if (results.colliding)
        //{
        //    Drawing.Draw.xz.Line(ellipseA.position, ellipseA.position + results.normal * results.penetrationDepth);
        //}

        //ellipseA.forward = normalize(ellipseA.forward);
        //ellipseB.forward = normalize(ellipseB.forward);

        //DrawEllipse(ellipseA, ellipseColorA);
        //DrawEllipse(ellipseB, ellipseColorB);

        //var result = EllipseCollision.IntersectEllipses(ellipseA, ellipseB);

        //if (result.intersecting)
        //{
        //    Gizmos.color = collisionColor;

        //    float2 contactPoint = ellipseA.position + result.normal * ellipseA.radii.x;
        //    // This is a rough contact location for visualization

        //    Gizmos.DrawLine(ToFloat3(ellipseA.position), ToFloat3((ellipseA.position + result.normal * result.penetrationDepth)));

        //    Gizmos.DrawSphere(ToFloat3(contactPoint), 0.05f);
        //}
    }

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

    //float3 ToFloat3(float2 v) => new float3(v.x, 0, v.y);

    //void DrawEllipse(Ellipse2D ellipse, Color color)
    //{
    //    Gizmos.color = color;

    //    float2 forward = normalize(ellipse.forward);
    //    float2 right = new float2(-forward.y, forward.x);

    //    float angleStep = 2f * Mathf.PI / Segments;
    //    float2 prevPoint = ellipse.radii.x * forward + ellipse.radii.y * right + ellipse.position;

    //    for (int i = 1; i <= Segments; i++)
    //    {
    //        float angle = i * angleStep;

    //        // Parametric ellipse
    //        float2 local = float2(cos(angle) * ellipse.radii.x, sin(angle) * ellipse.radii.y);
    //        float2 world = ellipse.position + local.x * forward + local.y * right;

    //        Gizmos.DrawLine(ToFloat3(prevPoint), ToFloat3(world));
    //        prevPoint = world;
    //    }

    //    // Axis visualization (optional)
    //    Gizmos.color = Color.yellow;
    //    Gizmos.DrawLine(ToFloat3(ellipse.position), ToFloat3((ellipse.position + forward * ellipse.radii.x)));
    //    Gizmos.color = Color.magenta;
    //    Gizmos.DrawLine(ToFloat3(ellipse.position), ToFloat3((ellipse.position + right * ellipse.radii.y)));
    //}
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
        result.normal = SmoothNormal(result.normal, closestA, closestB, capsule1, capsule2);

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
