// PickAndPlaceScene.cs
// Builds a floor, a table and a graspable cylinder, and handles grasping:
//   gripper closes near the cylinder  -> cylinder attaches to the wrist
//   gripper opens                     -> cylinder releases and falls
//
// Grasp detection reads the finger positions (GripL / GripR) directly,
// so no changes are needed in ZedArmUDP.cs or BuildRobotArm.cs.
//
// Setup: add this component to an empty GameObject (e.g. "PickScene")
// and press Play alongside the arm.
//
// Calibration trick (important): the exact reachable spot in Unity
// depends on your joint sign settings, so instead of guessing where to
// put the table, drive the arm to a comfortable pose, then right-click
// this component's header in the Inspector and choose
// "Place cylinder at gripper". The cylinder teleports into the fingers
// and the table slides underneath it. One click, guaranteed reachable.

using System.Globalization;
using System.Net.Sockets;
using System.Text;
using UnityEngine;

public class PickAndPlaceScene : MonoBehaviour
{
    [Header("Cylinder position broadcast (for DMP pick)")]
    public int broadcastPort = 5007;   // Python listens here for the position

    [Header("Table")]
    public Vector3 tableCentre = new Vector3(0f, 0.175f, 0.60f);
    public Vector3 tableSize   = new Vector3(0.80f, 0.35f, 0.50f);

    [Header("Cylinder object")]
    public float cylinderRadius = 0.03f;    // 6 cm diameter
    public float cylinderHeight = 0.12f;    // 12 cm tall
    public Color freeColor = new Color(0.85f, 0.45f, 0.10f);
    public Color heldColor = new Color(0.20f, 0.80f, 0.30f);

    [Header("Grasping")]
    public float graspRadius = 0.12f;        // gripper must be this close
    public float closedSeparation = 0.045f;  // fingers closer = "closed"

    [Header("Status (read-only)")]
    public bool held;

    Transform gripL, gripR;
    Transform table, cyl;
    Rigidbody cylRb;
    Renderer cylRend;
    UdpClient udp;
    float lastBroadcast;

    void Start()
    {
        udp = new UdpClient();
        BuildFloor();
        BuildTable();
        BuildCylinder();
    }

    void BuildFloor()
    {
        var f = GameObject.CreatePrimitive(PrimitiveType.Cube);
        f.name = "Floor";
        f.transform.position = new Vector3(0f, -0.05f, 0f);
        f.transform.localScale = new Vector3(6f, 0.1f, 6f);
        f.GetComponent<Renderer>().material.color = new Color(0.35f, 0.35f, 0.38f);
    }

    void BuildTable()
    {
        var t = GameObject.CreatePrimitive(PrimitiveType.Cube);
        t.name = "Table";
        t.transform.position = tableCentre;
        t.transform.localScale = tableSize;
        t.GetComponent<Renderer>().material.color = new Color(0.55f, 0.42f, 0.28f);
        table = t.transform;
    }

    void BuildCylinder()
    {
        var c = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        c.name = "PickCylinder";
        float topY = tableCentre.y + tableSize.y * 0.5f;
        c.transform.position = new Vector3(tableCentre.x,
                                           topY + cylinderHeight * 0.5f,
                                           tableCentre.z);
        // Unity's cylinder primitive is 2 m tall at scale 1, so the
        // y scale is HALF the desired height
        c.transform.localScale = new Vector3(cylinderRadius * 2f,
                                             cylinderHeight * 0.5f,
                                             cylinderRadius * 2f);
        cylRend = c.GetComponent<Renderer>();
        cylRend.material.color = freeColor;

        cylRb = c.AddComponent<Rigidbody>();
        cylRb.isKinematic = true;   // sits still until first grabbed/released
        cyl = c.transform;
    }

    void Update()
    {
        // the fingers are created at runtime by BuildRobotArm,
        // so keep looking until they exist
        if (gripL == null || gripR == null)
        {
            var l = GameObject.Find("GripL"); if (l != null) gripL = l.transform;
            var r = GameObject.Find("GripR"); if (r != null) gripR = r.transform;
            if (gripL == null || gripR == null) return;
        }

        Vector3 graspPoint = (gripL.position + gripR.position) * 0.5f;
        float separation = Vector3.Distance(gripL.position, gripR.position);
        bool closed = separation < closedSeparation;

        if (!held && closed &&
            Vector3.Distance(cyl.position, graspPoint) < graspRadius)
        {
            Grab();
        }
        else if (held && !closed)
        {
            Release();
        }

        // ---- broadcast the cylinder position (10 Hz) for the DMP pick ----
        if (cyl != null && Time.time - lastBroadcast > 0.1f)
        {
            lastBroadcast = Time.time;
            var ci = CultureInfo.InvariantCulture;
            string msg = "{\"cx\":" + cyl.position.x.ToString("F4", ci)
                       + ",\"cy\":" + cyl.position.y.ToString("F4", ci)
                       + ",\"cz\":" + cyl.position.z.ToString("F4", ci)
                       + ",\"held\":" + (held ? "true" : "false") + "}";
            byte[] b = Encoding.UTF8.GetBytes(msg);
            try { udp.Send(b, b.Length, "127.0.0.1", broadcastPort); } catch { }
        }
    }

    void OnDestroy()
    {
        try { if (udp != null) udp.Close(); } catch { }
    }

    void Grab()
    {
        held = true;
        cylRb.isKinematic = true;                 // physics off while carried
        cyl.SetParent(gripL.parent, true);        // rides with the wrist (J6)
        cylRend.material.color = heldColor;
    }

    void Release()
    {
        held = false;
        cyl.SetParent(null, true);
        cylRb.isKinematic = false;                // gravity takes it
        cylRb.velocity = Vector3.zero;
        cylRend.material.color = freeColor;
    }

    [ContextMenu("Place cylinder at gripper")]
    void PlaceAtGripper()
    {
        if (gripL == null || gripR == null) return;
        Vector3 graspPoint = (gripL.position + gripR.position) * 0.5f;
        // cylinder into the fingers, table underneath it
        cyl.SetParent(null, true);
        cyl.position = graspPoint;
        table.position = new Vector3(graspPoint.x, tableCentre.y, graspPoint.z);
        held = false;
        cylRb.isKinematic = false;   // it will drop the short way onto the table
        cylRb.velocity = Vector3.zero;
        cylRend.material.color = freeColor;
    }

    [ContextMenu("Reset cylinder to table")]
    void ResetCylinder()
    {
        cyl.SetParent(null, true);
        held = false;
        float topY = table.position.y + tableSize.y * 0.5f;
        cyl.position = new Vector3(table.position.x,
                                   topY + cylinderHeight * 0.5f,
                                   table.position.z);
        cyl.rotation = Quaternion.identity;
        cylRb.isKinematic = true;
        cylRend.material.color = freeColor;
    }
}
