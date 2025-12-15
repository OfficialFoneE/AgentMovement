using UnityEngine;

public class PillAvoidancePairTester2D : MonoBehaviour
{
    [Header("Pill A")]
    public Transform A;
    public float radiusA = 0.5f;
    public float lengthA = 2.0f;          // full segment length (excluding caps)
    public Vector2 velocityA = new Vector2(2, 0);

    [Header("Pill B")]
    public Transform B;
    public float radiusB = 0.5f;
    public float lengthB = 2.0f;
    public Vector2 velocityB = new Vector2(-2, 0);

    [Header("Avoidance")]
    public float horizonSeconds = 1.0f;
    public float extraSeparation = 0.05f;
    public float maxDeltaSpeed = 10.0f;    // clamp on avoidance impulse (0 = unlimited)
    public float minResponseTime = 0.05f;  // prevents huge dv when tMin ~ 0

    [Header("Sim")]
    public bool simulateInPlayMode = true;

    private Pill2DVO.DebugInfo _dbg;

    void FixedUpdate()
    {
        if (!simulateInPlayMode || A == null || B == null) return;

        var capA = new Pill2DVO.Capsule
        {
            center = (Vector2)A.position,
            velocity = velocityA,
            halfLength = Mathf.Max(0f, lengthA * 0.5f),
            radius = Mathf.Max(0f, radiusA),
        };

        var capB = new Pill2DVO.Capsule
        {
            center = (Vector2)B.position,
            velocity = velocityB,
            halfLength = Mathf.Max(0f, lengthB * 0.5f),
            radius = Mathf.Max(0f, radiusB),
        };

        Pill2DVO.AvoidPair(
            capA, capB,
            horizonSeconds,
            extraSeparation,
            maxDeltaSpeed,
            minResponseTime,
            out velocityA,
            out velocityB,
            out _dbg);

        // Move them (for testing). Replace with your own movement logic.
        A.position += (Vector3)(velocityA * Time.fixedDeltaTime);
        B.position += (Vector3)(velocityB * Time.fixedDeltaTime);
    }

    void OnDrawGizmos()
    {
        if (A == null || B == null) return;

        var capA = new Pill2DVO.Capsule
        {
            center = (Vector2)A.position,
            velocity = velocityA,
            halfLength = Mathf.Max(0f, lengthA * 0.5f),
            radius = Mathf.Max(0f, radiusA),
        };

        var capB = new Pill2DVO.Capsule
        {
            center = (Vector2)B.position,
            velocity = velocityB,
            halfLength = Mathf.Max(0f, lengthB * 0.5f),
            radius = Mathf.Max(0f, radiusB),
        };

        // Recompute debug for gizmos (so it works in edit mode too)
        Pill2DVO.AvoidPair(
            capA, capB,
            horizonSeconds,
            extraSeparation,
            maxDeltaSpeed,
            minResponseTime,
            out Vector2 newVelA,
            out Vector2 newVelB,
            out _dbg);

        // Draw current capsules
        Gizmos.color = Color.green;
        DrawCapsule2D(capA.center, capA.velocity, capA.halfLength, capA.radius);

        Gizmos.color = Color.magenta;
        DrawCapsule2D(capB.center, capB.velocity, capB.halfLength, capB.radius);

        // Draw velocity vectors (current)
        Gizmos.color = Color.white;
        DrawArrow2D(capA.center, capA.center + capA.velocity * 0.5f);
        DrawArrow2D(capB.center, capB.center + capB.velocity * 0.5f);

        // Draw new velocity vectors
        Gizmos.color = Color.cyan;
        DrawArrow2D(capA.center, capA.center + newVelA * 0.5f);
        DrawArrow2D(capB.center, capB.center + newVelB * 0.5f);

        // Draw capsules at closest approach time
        float t = _dbg.tMin;

        Gizmos.color = new Color(1f, 0.85f, 0.2f, 1f); // yellow-ish
        DrawCapsule2D(capA.center + capA.velocity * t, capA.velocity, capA.halfLength, capA.radius);
        DrawCapsule2D(capB.center + capB.velocity * t, capB.velocity, capB.halfLength, capB.radius);

        // Closest points + normal
        Gizmos.color = _dbg.willCollide ? Color.red : Color.gray;
        Gizmos.DrawSphere(ToV3(_dbg.pA), 0.06f);
        Gizmos.DrawSphere(ToV3(_dbg.pB), 0.06f);
        Gizmos.DrawLine(ToV3(_dbg.pA), ToV3(_dbg.pB));

        Gizmos.color = Color.red;
        Vector2 mid = 0.5f * (_dbg.pA + _dbg.pB);
        DrawArrow2D(mid, mid + _dbg.normal * 0.5f);
    }

    // --- Gizmo helpers ---

    private static void DrawCapsule2D(Vector2 center, Vector2 velocity, float halfLen, float radius)
    {
        Vector2 dir = velocity.sqrMagnitude > 1e-8f ? velocity.normalized : Vector2.right;
        Vector2 a = center - dir * halfLen;
        Vector2 b = center + dir * halfLen;

        Gizmos.DrawLine(ToV3(a), ToV3(b));
        Gizmos.DrawWireSphere(ToV3(a), radius);
        Gizmos.DrawWireSphere(ToV3(b), radius);
    }

    private static void DrawArrow2D(Vector2 from, Vector2 to)
    {
        Gizmos.DrawLine(ToV3(from), ToV3(to));

        Vector2 d = (to - from);
        if (d.sqrMagnitude < 1e-8f) return;

        Vector2 dir = d.normalized;
        Vector2 left = new Vector2(-dir.y, dir.x);

        float headLen = 0.12f;
        float headWid = 0.08f;

        Vector2 p1 = to - dir * headLen + left * headWid;
        Vector2 p2 = to - dir * headLen - left * headWid;

        Gizmos.DrawLine(ToV3(to), ToV3(p1));
        Gizmos.DrawLine(ToV3(to), ToV3(p2));
    }

    private static Vector3 ToV3(Vector2 v) => new Vector3(v.x, v.y, 0f);
}
