// BuildRobotArm.cs
// Procedurally builds a UR10 arm in the scene using the SAME geometry as the
// assignment model (the p0..p8 home points from the MATLAB script), so what
// you see in Unity is the arm your IK was derived for.
//
// Joint names are J1..J6, which is exactly what ZedArmUDP.cs looks for, so
// the receiver binds to this arm automatically. Scale: 1 Unity unit = 1 m.
//
// Geometry (mm), straight-up home pose, matching the MATLAB home points:
//   base -> shoulder height 128        (p0 -> p1)
//   shoulder jog out 176               (p1 -> p2)
//   upper arm up 612.7                 (p2 -> p3)
//   elbow jog in to 12.1               (p3 -> p4)
//   forearm up 571.6                   (p4 -> p5)
//   wrist jog out to 163.9             (p5 -> p6)  <- wrist centre (red ball)
//   wrist link up 115.7                (p6 -> p7)
//   tool out to 256.1                  (p7 -> p8)
//
// MATLAB frame is Z-up, Unity is Y-up, so MATLAB (x, y, z) maps to
// Unity (x, z, y). The home pose points straight up along Unity Y.
//
// Setup: add this component to an empty GameObject at the origin and press
// Play. Combine with ZedArmUDP on another GameObject to drive it.

using UnityEngine;

public class BuildRobotArm : MonoBehaviour
{
    [Header("Appearance")]
    public float linkRadius  = 0.035f;
    public float jointRadius = 0.055f;
    public Color linkColor        = new Color(0.55f, 0.57f, 0.60f);
    public Color jointColor       = new Color(0.95f, 0.55f, 0.15f);
    public Color wristCentreColor = Color.red;

    void Start()
    {
        Build();
    }

    void Build()
    {
        // ---- home-pose points in Unity metres (from the MATLAB p0..p8) ----
        Vector3 pBase        = V(0.0f,    0.0f);      // p0
        Vector3 pShoulder    = V(0.0f,    128.0f);    // p1, J2 pivot
        Vector3 pShoulderOut = V(176.0f,  128.0f);    // p2
        Vector3 pUpperTop    = V(176.0f,  740.7f);    // p3
        Vector3 pElbowIn     = V(12.1f,   740.7f);    // p4, J3 pivot
        Vector3 pForearmTop  = V(12.1f,   1312.3f);   // p5, J4 pivot
        Vector3 pWristCentre = V(163.9f,  1312.3f);   // p6, J5 pivot (the IK target point)
        Vector3 pWristTop    = V(163.9f,  1428.0f);   // p7, J6 pivot
        Vector3 pToolTip     = V(256.1f,  1428.0f);   // p8

        // ---- joint chain: each joint is the parent of the next ----
        Transform j1 = MakeJoint("J1", pBase,        transform);
        Transform j2 = MakeJoint("J2", pShoulder,    j1);
        Transform j3 = MakeJoint("J3", pElbowIn,     j2);
        Transform j4 = MakeJoint("J4", pForearmTop,  j3);
        Transform j5 = MakeJoint("J5", pWristCentre, j4);
        Transform j6 = MakeJoint("J6", pWristTop,    j5);

        // ---- visual links, parented under the joint that moves them ----
        Segment(j1, pBase,        pShoulder,    "base");
        Segment(j2, pShoulder,    pShoulderOut, "shoulder jog");
        Segment(j2, pShoulderOut, pUpperTop,    "upper arm");
        Segment(j3, pUpperTop,    pElbowIn,     "elbow jog");
        Segment(j3, pElbowIn,     pForearmTop,  "forearm");
        Segment(j4, pForearmTop,  pWristCentre, "wrist jog");
        Segment(j5, pWristCentre, pWristTop,    "wrist link");
        Segment(j6, pWristTop,    pToolTip,     "tool");

        // ---- joint markers ----
        Ball(j1, pBase,        jointRadius, jointColor, "j1 marker");
        Ball(j2, pShoulder,    jointRadius, jointColor, "j2 marker");
        Ball(j3, pElbowIn,     jointRadius, jointColor, "j3 marker");
        Ball(j4, pForearmTop,  jointRadius, jointColor, "j4 marker");
        Ball(j6, pWristTop,    jointRadius, jointColor, "j6 marker");

        // the wrist centre: the point your IK actually positions. Red so you
        // can see it land on targets.
        Ball(j5, pWristCentre, jointRadius * 1.3f, wristCentreColor, "wrist centre");

        // ---- gripper: two fingers at the tool tip, driven by ZedArmUDP ----
        Finger(j6, pToolTip, +1);   // GripR
        Finger(j6, pToolTip, -1);   // GripL

        Debug.Log("BuildRobotArm: UR10 built with assignment geometry, joints J1..J6 + gripper.");
    }

    // MATLAB (x_mm, z_mm) in the arm's vertical plane -> Unity metres, Y up.
    static Vector3 V(float x_mm, float z_mm)
    {
        return new Vector3(x_mm / 1000f, z_mm / 1000f, 0f);
    }

    Transform MakeJoint(string name, Vector3 worldPos, Transform parent)
    {
        var g = new GameObject(name);
        g.transform.position = worldPos;
        g.transform.SetParent(parent, true);   // keep world position
        return g.transform;
    }

    void Segment(Transform parent, Vector3 a, Vector3 b, string name)
    {
        var cyl = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        cyl.name = name;
        Vector3 dir = b - a;
        cyl.transform.position = (a + b) * 0.5f;
        cyl.transform.rotation = Quaternion.FromToRotation(Vector3.up, dir.normalized);
        cyl.transform.localScale =
            new Vector3(linkRadius * 2f, dir.magnitude * 0.5f, linkRadius * 2f);
        cyl.transform.SetParent(parent, true);
        Paint(cyl, linkColor);
    }

    void Finger(Transform parent, Vector3 tip, int side)
    {
        var f = GameObject.CreatePrimitive(PrimitiveType.Cube);
        f.name = side > 0 ? "GripR" : "GripL";
        // fingers sit just past the tool tip, separated sideways;
        // ZedArmUDP slides their local z to open and close
        f.transform.position = tip + new Vector3(0.03f, 0f, side * 0.045f);
        f.transform.localScale = new Vector3(0.06f, 0.014f, 0.014f);
        f.transform.SetParent(parent, true);
        Paint(f, jointColor);
    }

    void Ball(Transform parent, Vector3 pos, float radius, Color c, string name)
    {
        var s = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        s.name = name;
        s.transform.position = pos;
        s.transform.localScale = Vector3.one * radius * 2f;
        s.transform.SetParent(parent, true);
        Paint(s, c);
    }

    static void Paint(GameObject g, Color c)
    {
        var r = g.GetComponent<Renderer>();
        if (r != null) r.material.color = c;
    }
}
