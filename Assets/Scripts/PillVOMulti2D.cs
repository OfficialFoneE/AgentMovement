using System.Collections.Generic;
using UnityEngine;

public static class PillVOMulti2D
{
    public struct Agent
    {
        public Vector2 position;   // capsule center
        public Vector2 velocity;   // current velocity
        public float halfLength;
        public float radius;
    }

    public struct Settings
    {
        public float horizonSeconds;     // predict window
        public float extraSeparation;    // buffer
        public float maxDeltaSpeed;      // clamp per-pair response (0 = unlimited)
        public float minResponseTime;    // stabilizer when tMin ~ 0
        public int iterations;           // solver passes (1-6 typical)
        public bool preserveSpeed;       // keep original speed magnitude after avoidance
    }

    /// <summary>
    /// Multi-agent avoidance step. O(N^2) pairs, accumulates reciprocal pushes, applies in iterations.
    /// Mutates agents[i].velocity in place.
    /// </summary>
    public static void Step(ref Agent[] agents, in Settings s)
    {
        if (agents == null || agents.Length <= 1) return;

        int n = agents.Length;
        int iters = Mathf.Max(1, s.iterations);

        // For speed preservation option.
        float[] baseSpeed = null;
        if (s.preserveSpeed)
        {
            baseSpeed = new float[n];
            for (int i = 0; i < n; i++) baseSpeed[i] = agents[i].velocity.magnitude;
        }

        // Jacobi-style iterations: compute all deltas from current velocities, then apply.
        Vector2[] deltaV = new Vector2[n];

        for (int iter = 0; iter < iters; iter++)
        {
            System.Array.Clear(deltaV, 0, n);

            for (int i = 0; i < n; i++)
            {
                for (int j = i + 1; j < n; j++)
                {
                    var a = agents[i];
                    var b = agents[j];

                    // Pair solve -> new velocities
                    Pill2DVO.AvoidPair(
                        new Pill2DVO.Capsule
                        {
                            center = a.position,
                            velocity = a.velocity,
                            halfLength = a.halfLength,
                            radius = a.radius
                        },
                        new Pill2DVO.Capsule
                        {
                            center = b.position,
                            velocity = b.velocity,
                            halfLength = b.halfLength,
                            radius = b.radius
                        },
                        s.horizonSeconds,
                        s.extraSeparation,
                        s.maxDeltaSpeed,
                        s.minResponseTime,
                        out Vector2 newVA,
                        out Vector2 newVB,
                        out _);

                    // Accumulate *changes* from this pair (so multiple neighbors add up)
                    deltaV[i] += (newVA - a.velocity);
                    deltaV[j] += (newVB - b.velocity);
                }
            }

            // Apply accumulated deltas.
            for (int i = 0; i < n; i++)
            {
                agents[i].velocity += deltaV[i];

                if (s.preserveSpeed)
                {
                    float target = baseSpeed[i];
                    Vector2 v = agents[i].velocity;
                    float mag = v.magnitude;

                    if (mag > 1e-6f && target > 1e-6f)
                        agents[i].velocity = v * (target / mag);
                }
            }
        }
    }
}
