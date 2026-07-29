# AI-Assisted Teleoperation of a 6-DOF Robotic Arm

**MSc thesis project — work in progress.**

Controlling a robotic arm by tracking a person's arm with a camera. I stand in
front of a ZED stereo camera, it tracks my arm pose, and a UR10 robot (simulated
in Unity) copies my hand position in real time. My left hand opens and closes the
gripper with a raise gesture. The longer-term goal is to record demonstrations and
have the arm learn and replay a pick-and-place motion on its own.

This repo holds the operator-side program (`zed_ik_udp_sender.py`), which does the
tracking, the maths, and the streaming to the simulator.

## How it works

```
ZED camera ──► arm pose ──► map hand into robot workspace ──► inverse kinematics ──► joint angles ──► UDP ──► Unity (UR10)
                                                                                          │
left arm raise gesture ──────────────────────────────────────────────────────────────────┴──► gripper open / close
```

- **Right arm** drives position. Wherever I move my hand inside a "ball" workspace
  around a centre point, the robot's end point moves to match (absolute mapping).
- **Left arm** drives the gripper. Raise my wrist above my shoulder to close, lower
  it to open, with a hysteresis band so it doesn't flicker.
- Joint angles are solved with a closed-form inverse kinematics for the UR10's
  first three joints, then streamed to Unity over UDP as JSON.

## Fighting the jitter

Raw camera keypoints wobble, which makes the robot twitchy. Two things I added to
fix that, both toggleable so I can A/B test and measure the difference:

- **Elbow-stabilised wrist** — the elbow tracks more reliably than the wrist, so I
  rebuild the wrist off the elbow plus a fixed forearm length, throwing away the
  jitter that acts along the arm.
- **One-Euro filter** — an adaptive filter that smooths hard when I'm still and
  eases off when I move fast, so I don't trade jitter for lag.

There's a live **jitter readout** on the video (mm, measured at rest) so I can put
an actual number on how much each option helps instead of just eyeballing it.

## Running it

Needs a ZED camera + the ZED SDK (`pyzed`), plus `numpy` and `opencv-python`, and
the Unity scene listening on UDP port 5005.

```bash
python zed_ik_udp_sender.py
```

Keys in the video window: `q` quit · `f` freeze the arm · `r` start/stop recording a demo.

## Status / to do

- [x] Real-time arm tracking → IK → UDP to Unity
- [x] Raise-gesture gripper with hysteresis
- [x] Elbow stabilisation + One-Euro filter with live jitter measurement
- [x] Demonstration recorder (saves end-point + grip to CSV)
- [ ] DMP learning + replay of a recorded pick motion *(in progress)*
- [ ] Shift the learned grab when the object moves
- [ ] Quantify end-to-end latency and tracking accuracy

## Note

This is active thesis work, so it's still changing and some parts are experimental.
Built with a ZED 2 camera, a simulated UR10 in Unity, and the BODY_38 tracking model.
