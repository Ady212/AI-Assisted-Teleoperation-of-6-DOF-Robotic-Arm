// ReachEnvelope.cs
// Draws the robot's reachable space around the base, so you can SEE how
// low and high it can go and why a floor object is out of range.
//
// Two things are drawn, because they are different:
//   1) PHYSICAL REACH  (grey wireframe rings): the shell the arm can
//      actually touch, from the IK limits. Inner + outer radius, over
//      a range of heights.
//   2) MAPPING ZONE    (green translucent band): the sub-region your
//      teleoperation mapping actually commands. This sits INSIDE the
//      physical reach. The gap between the two is why the floor, though
//      physically reachable-ish, is never commanded.
//
// Also draws a marker at the live target height (the red gripper/wrist
// centre) so you can read where "floor level" falls on the scale.
//
// Setup: add to an empty GameObject "ReachViz" at the origin, press Play.
// Toggle the checkboxes in the Inspector to show/hide each layer.
//
// NOTE: the numbers below mirror the sender's constants. If you change
// R_MIN/R_MAX/Z_MIN/Z_MAX or C_MM/MAX_OFFSET in the Python, update them
// here too so the picture stays truthful.

using System.Collections.Generic;
using UnityEngine;

[ExecuteAlways]
public class ReachEnvelope : MonoBehaviour
{
    [Header("Show / hide")]
    public bool showPhysicalReach = true;
    public bool showMappingZone   = true;
    public bool showFloorLine     = true;

    [Header("Physical reach (metres, from IK limits)")]
    // L2 + D4 - margin and |L2 - D4| + margin, converted to metres,
    // plus the base/shoulder height offset.
    public float shoulderHeight = 0.1273f;   // d1
    public float reachOuter     = 1.124f;    // (612.7 + 571.6 - 60) mm
    public float reachInner     = 0.101f;    // (|612.7 - 571.6| + 60) mm
    public float physLowZ       = -0.05f;    // Z_MIN in metres
    public float physHighZ      = 0.90f;     // Z_MAX in metres

    [Header("Mapping zone (metres) - the band you actually command")]
    public Vector3 workspaceCentre = new Vector3(0f, 0.65f, 0.35f); // C_MM
    public float mappingRadius = 0.42f;      // MAX_OFFSET in metres

    [Header("Appearance")]
    public Color physicalColor = new Color(0.6f, 0.6f, 0.65f, 1f);
    public Color mappingColor  = new Color(0.20f, 0.80f, 0.35f, 0.18f);
    public int ringSegments = 48;

    Transform wristCentre;   // the red ball, for the live height marker
    GameObject mappingSphere;

    void OnEnable()  { BuildMappingSphere(); }
    void OnDisable() { if (mappingSphere) DestroyImmediate(mappingSphere); }

    void BuildMappingSphere()
    {
        if (mappingSphere != null) return;
        mappingSphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        mappingSphere.name = "MappingZone(viz)";
        mappingSphere.transform.SetParent(transform, false);
        // strip the collider so it never interferes with grasping
        var col = mappingSphere.GetComponent<Collider>();
        if (col) DestroyImmediate(col);
        var mr = mappingSphere.GetComponent<MeshRenderer>();
        var mat = new Material(Shader.Find("Sprites/Default"));
        mat.color = mappingColor;
        mr.sharedMaterial = mat;
    }

    void Update()
    {
        // keep the translucent sphere sized and placed like the mapping ball
        if (mappingSphere != null)
        {
            mappingSphere.SetActive(showMappingZone);
            mappingSphere.transform.position = workspaceCentre;
            mappingSphere.transform.localScale = Vector3.one * mappingRadius * 2f;
        }
        if (wristCentre == null)
        {
            var w = GameObject.Find("wrist centre");
            if (w != null) wristCentre = w.transform;
        }
    }

    // Gizmos draw the wireframe rings (visible in Scene view, and in Game
    // view if Gizmos are enabled there).
    void OnDrawGizmos()
    {
        if (showPhysicalReach)
        {
            Gizmos.color = physicalColor;
            // a stack of ring pairs (inner + outer) at several heights
            int layers = 7;
            for (int i = 0; i < layers; i++)
            {
                float z = Mathf.Lerp(physLowZ, physHighZ, i / (float)(layers - 1));
                // at this height, the reachable annulus radius shrinks near
                // the top/bottom of the sphere of reach
                float dz = z - shoulderHeight;
                float maxR = Mathf.Sqrt(Mathf.Max(reachOuter*reachOuter - dz*dz, 0f));
                float minR = ReachInnerAt(dz);
                DrawRing(maxR, z);
                if (minR > 0.02f) DrawRing(minR, z);
            }
            // vertical extent markers
            DrawRing(0.01f, physLowZ);
            DrawRing(0.01f, physHighZ);
        }

        if (showFloorLine)
        {
            Gizmos.color = new Color(1f, 0.3f, 0.2f, 1f);
            Vector3 a = new Vector3(-1.3f, 0f, 0f);
            Vector3 b = new Vector3( 1.3f, 0f, 0f);
            Gizmos.DrawLine(a, b);
            Gizmos.DrawLine(new Vector3(0f, 0f, -1.3f), new Vector3(0f, 0f, 1.3f));
        }

        if (wristCentre != null)
        {
            Gizmos.color = Color.red;
            // horizontal disc at the current target height
            DrawRing(reachOuter, wristCentre.position.y);
        }
    }

    float ReachInnerAt(float dz)
    {
        float v = reachInner*reachInner - dz*dz;
        return v > 0f ? Mathf.Sqrt(v) : 0f;
    }

    void DrawRing(float radius, float y)
    {
        Vector3 prev = new Vector3(radius, y, 0f);
        for (int i = 1; i <= ringSegments; i++)
        {
            float a = i / (float)ringSegments * Mathf.PI * 2f;
            Vector3 p = new Vector3(Mathf.Cos(a) * radius, y, Mathf.Sin(a) * radius);
            Gizmos.DrawLine(prev, p);
            prev = p;
        }
    }
}
