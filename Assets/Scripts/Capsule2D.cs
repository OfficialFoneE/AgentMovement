using Unity.Mathematics;

[System.Serializable]
public struct Capsule2D
{
    public float2 position;  // Center position
    public float2 forward;  // Must be normalised!
    public float rotation;    // Rotation in radians
    public float halfLength;  // Half the length of the capsule's line segment
    public float radius;      // Radius around the line segment

    public Capsule2D(float2 pos, float2 forward, float rot, float halfLen, float rad)
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
