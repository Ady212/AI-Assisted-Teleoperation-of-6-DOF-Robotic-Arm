// OcMarker.cs
// Shows a floating marker at the live target position (Oc) that the arm
// is trying to reach, plus a faint line from the wrist centre to it.
//
// Two modes:
//   - Follow the wrist centre (default): the marker sits on the red ball,
//     confirming where the IK actually placed the arm each frame.
//   - Listen for Oc directly: if you later add "ocx/ocy/ocz" fields to the
//     UDP packet, set listenForOc = true and it shows the COMMANDED target,
//     so any gap between commanded and achieved becomes visible.
//
// Setup: add to an empty GameObject "OcMarker". Press Play.

using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

public class OcMarker : MonoBehaviour
{
    [Header("Marker")]
    public float markerSize = 0.06f;
    public Color markerColor = new Color(1f, 0.85f, 0.1f);

    [Header("Commanded-Oc mode (optional)")]
    public bool listenForOc = false;      // needs ocx/ocy/ocz in the packet
    public int listenPort = 5006;          // a SEPARATE port from the arm

    Transform wristCentre;
    GameObject marker;
    LineRenderer line;

    UdpClient client;
    Thread thread;
    volatile string latest;

    [Serializable] class OcPacket { public float ocx, ocy, ocz; }

    void Start()
    {
        marker = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        marker.name = "Oc(marker)";
        marker.transform.localScale = Vector3.one * markerSize;
        var mr = marker.GetComponent<MeshRenderer>();
        mr.material.color = markerColor;
        Destroy(marker.GetComponent<Collider>());

        var lgo = new GameObject("Oc(line)");
        line = lgo.AddComponent<LineRenderer>();
        line.widthMultiplier = 0.006f;
        line.material = new Material(Shader.Find("Sprites/Default"));
        line.startColor = line.endColor = new Color(1f, 0.85f, 0.1f, 0.6f);
        line.positionCount = 2;

        if (listenForOc)
        {
            client = new UdpClient(listenPort);
            thread = new Thread(Loop) { IsBackground = true };
            thread.Start();
        }
    }

    void Loop()
    {
        var ep = new IPEndPoint(IPAddress.Any, listenPort);
        while (true)
        {
            try { latest = Encoding.UTF8.GetString(client.Receive(ref ep)); }
            catch { break; }
        }
    }

    void Update()
    {
        if (wristCentre == null)
        {
            var w = GameObject.Find("wrist centre");
            if (w != null) wristCentre = w.transform;
            else return;
        }

        Vector3 pos = wristCentre.position;
        if (listenForOc && !string.IsNullOrEmpty(latest))
        {
            try
            {
                var p = JsonUtility.FromJson<OcPacket>(latest);
                // convert mm (robot frame x,y,z) to Unity metres (x, z, y)
                pos = new Vector3(p.ocx, p.ocz, p.ocy) / 1000f;
            }
            catch { }
        }

        marker.transform.position = pos;
        line.SetPosition(0, wristCentre.position);
        line.SetPosition(1, pos);
    }

    void OnDestroy()
    {
        try { if (client != null) client.Close(); } catch { }
    }
}
