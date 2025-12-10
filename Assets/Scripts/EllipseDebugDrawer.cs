using UnityEngine;
using Unity.Mathematics;
using static Unity.Mathematics.math;
using UnityEditor.Build.Pipeline;

public class EllipseDebugDrawer : MonoBehaviour
{
    public Ellipse2D ellipseA;
    public Ellipse2D ellipseB;

    public Color ellipseColorA = Color.green;
    public Color ellipseColorB = Color.cyan;
    public Color collisionColor = Color.red;

    // Number of segments to draw ellipse outline
    private const int Segments = 64;

    void OnDrawGizmos()
    {
        ellipseA.forward = normalize(ellipseA.forward);
        ellipseB.forward = normalize(ellipseB.forward);

        DrawEllipse(ellipseA, ellipseColorA);
        DrawEllipse(ellipseB, ellipseColorB);

        var result = EllipseCollision.IntersectEllipses(ellipseA, ellipseB);

        if (result.intersecting)
        {
            Gizmos.color = collisionColor;

            float2 contactPoint = ellipseA.position + result.normal * ellipseA.radii.x;
            // This is a rough contact location for visualization

            Gizmos.DrawLine(ToFloat3(ellipseA.position), ToFloat3((ellipseA.position + result.normal * result.penetrationDepth)));

            Gizmos.DrawSphere(ToFloat3(contactPoint), 0.05f);
        }
    }

    float3 ToFloat3(float2 v) => new float3(v.x, 0, v.y);

    void DrawEllipse(Ellipse2D ellipse, Color color)
    {
        Gizmos.color = color;

        float2 forward = normalize(ellipse.forward);
        float2 right = new float2(-forward.y, forward.x);

        float angleStep = 2f * Mathf.PI / Segments;
        float2 prevPoint = ellipse.radii.x * forward + ellipse.radii.y * right + ellipse.position;

        for (int i = 1; i <= Segments; i++)
        {
            float angle = i * angleStep;

            // Parametric ellipse
            float2 local = float2(cos(angle) * ellipse.radii.x, sin(angle) * ellipse.radii.y);
            float2 world = ellipse.position + local.x * forward + local.y * right;

            Gizmos.DrawLine(ToFloat3(prevPoint), ToFloat3(world));
            prevPoint = world;
        }

        // Axis visualization (optional)
        Gizmos.color = Color.yellow;
        Gizmos.DrawLine(ToFloat3(ellipse.position), ToFloat3((ellipse.position + forward * ellipse.radii.x)));
        Gizmos.color = Color.magenta;
        Gizmos.DrawLine(ToFloat3(ellipse.position), ToFloat3((ellipse.position + right * ellipse.radii.y)));
    }
}
