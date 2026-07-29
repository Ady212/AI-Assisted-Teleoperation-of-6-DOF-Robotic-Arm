########################################################################
# zed_ik_udp_sender.py
#
# *** MSc thesis - work in progress ***
# AI-assisted teleoperation of a 6-DOF robotic arm from vision-based
# human arm tracking. This is the operator-side program: it reads my arm
# pose from a ZED stereo camera, works out where the robot's end point
# should be, solves the inverse kinematics, and streams the joint angles
# to a Unity simulation of a UR10 over UDP. Still actively developing and
# tuning, so some parts (the DMP pick, A/B toggles) are experimental.
#
# How it works, end to end:
#   Right arm -> wrist position -> map into robot workspace -> IK -> joints
#   Left arm  -> raise gesture  -> open / close the gripper
#
# The control scheme is absolute positional mimicry: wherever I move my
# hand inside a "ball" workspace around a centre point, the robot's end
# point moves to match. I went with this because for a front-facing
# pick-and-place task it's the most intuitive to operate, an earlier
# cylindrical-rotation scheme I tried wasn't needed in a front workspace.
#
# Gripper gesture: raise my left wrist above my left shoulder to close,
# lower it back down to open. There's a 10 cm hysteresis band so it
# doesn't flicker on the boundary.
#
# Two things I added to fight keypoint jitter (both toggleable so I can
# A/B test them and actually measure the difference):
#   - Elbow-stabilised wrist: the elbow tracks more reliably than the
#     wrist, so I rebuild the wrist off the elbow + a fixed forearm length.
#     (This came out of a suggestion from my supervisor, Dr. Omerdic.)
#   - One-Euro filter: an adaptive filter that smooths hard when I'm still
#     and lightly when I move fast (Casiez et al. 2012).
# A live jitter readout (mm, at rest) lets me put a number on it rather
# than just eyeballing it.
#
# There's also a demonstration recorder (press 'r') that logs the end
# point and grip to a CSV, which feeds a separate DMP script I'm building
# to learn and replay a pick motion. The cylinder position from Unity is
# snapshotted at record time so the learned grab can shift if the object
# moves.
#
# Keys in the video window:  q quit   f freeze arm   r record demo
########################################################################

import json
import os
import socket
import time
import numpy as np
import cv2
import pyzed.sl as sl

# ---------------------- configuration ----------------------
UNITY_IP   = "127.0.0.1"
UNITY_PORT = 5005
CYL_PORT   = 5007          # Unity sends the cylinder position here

# UR10 constants (mm)
D3 = 163.9
D4 = 571.6
L2 = 612.7

ELBOW = -1                 # elbow-up vs elbow-down IK solution
ALPHA = 0.2                # joint angle smoothing

# workspace mapping
L_HUMAN = 0.65
L_ROBOT_SPAN = 0.40
S = L_ROBOT_SPAN / L_HUMAN
C_MM = np.array([0.0, 650.0, 350.0])
MAX_OFFSET = 420.0

# position filter
POS_ALPHA   = 0.35
DEADBAND_MM = 6.0
MAX_STEP_MM = 40.0

# gripper raise gesture
GRIP_MODE        = "hold"   # "hold": closed while raised. "toggle": each raise flips.
GRIP_RAISE_ABOVE = 0.05     # close when wrist this far ABOVE the shoulder (m)
GRIP_LOWER_BELOW = -0.05    # open (or re-arm) when this far BELOW (m)
GRIP_ALPHA       = 0.25     # gripper motion smoothing

CONF_MIN = 40.0

# tracking model: "ACCURATE" (steadier, slower) or "FAST"
TRACK_MODEL = "ACCURATE"

# --- accuracy improvements (toggle for A/B testing) ---
USE_ELBOW_STABILISE = True   # blend wrist with elbow-reconstructed estimate
ELBOW_BLEND = 0.45           # 0 = raw wrist only, 1 = elbow estimate only
USE_ONE_EURO = True          # adaptive filter instead of fixed POS_ALPHA
OE_MINCUTOFF = 1.0           # lower = smoother at rest
OE_BETA = 0.02               # higher = less lag when moving fast

# demonstration recording
REC_DIR = "demos" 


def camera_to_robot(reach_cam):
    # swap the camera's axes into the robot's frame:
    return np.array([
         reach_cam[0],     # my right      -> robot x
         reach_cam[2],     # toward camera  -> robot y (forward)
         reach_cam[1],     # up            -> robot z
    ])


def ur10_ik3(oc_mm):
    # Closed-form inverse kinematics for the first 3 joints of the UR10:
    # given where I want the wrist centre (oc_mm), solve for th1, th2, th3.
    # Returns ok=False if the target is out of reach so the caller can hold.
    xc, yc, zc = float(oc_mm[0]), float(oc_mm[1]), float(oc_mm[2])
    rz = zc

    den = xc**2 + yc**2
    rad = den - D3**2
    if rad < 0.0 or abs(yc) < 1e-6:
        return False, 0.0, 0.0, 0.0

    ct1 = (xc*D3 + yc*np.sqrt(rad)) / den
    ct1 = max(min(ct1, 1.0), -1.0)
    st1 = (D3 - xc*ct1) / yc
    th1 = np.arctan2(st1, ct1)

    ry = yc*ct1 - xc*st1

    ct3 = (ry**2 + rz**2 - L2**2 - D4**2) / (2.0*L2*D4)
    if abs(ct3) > 1.0 + 1e-9:
        return False, 0.0, 0.0, 0.0
    ct3 = max(min(ct3, 1.0), -1.0)
    st3 = ELBOW*np.sqrt(1.0 - ct3**2)
    th3 = np.arctan2(st3, ct3)

    k1 = L2 + D4*ct3
    k2 = D4*st3
    ct2 = (rz*k1 - ry*k2) / (k1**2 + k2**2)
    st2 = (-ry - k2*ct2) / k1
    th2 = np.arctan2(st2, ct2)

    return True, th1, th2, th3


class OneEuro:
    """One-Euro filter (Casiez, Roussel, Vogel 2012).
    A fixed smoothing factor always trades jitter against lag. This one
    adapts: it smooths hard when my hand is still (kills jitter) and eases
    off when I move fast (kills lag), so I get the best of both."""
    def __init__(self, mincutoff=1.0, beta=0.02, dcutoff=1.0):
        self.mincutoff = mincutoff; self.beta = beta; self.dcutoff = dcutoff
        self.x_prev = None; self.dx_prev = None; self.t_prev = None
    @staticmethod
    def _alpha(cutoff, dt):
        tau = 1.0 / (2.0 * np.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)
    def __call__(self, x, t):
        if self.x_prev is None:
            self.x_prev = x; self.dx_prev = np.zeros_like(x); self.t_prev = t
            return x
        dt = max(t - self.t_prev, 1e-3)
        dx = (x - self.x_prev) / dt
        a_d = self._alpha(self.dcutoff, dt)
        dx_hat = a_d * dx + (1 - a_d) * self.dx_prev
        cutoff = self.mincutoff + self.beta * np.linalg.norm(dx_hat)
        a = self._alpha(cutoff, dt)
        x_hat = a * x + (1 - a) * self.x_prev
        self.x_prev = x_hat; self.dx_prev = dx_hat; self.t_prev = t
        return x_hat


_forearm_len = {"L": None}   # running estimate of the operator's forearm length

def stabilise_wrist(wrist, elbow, shoulder, blend):
    """Steady the wrist point using the elbow as an anchor.

    My forearm length doesn't actually change, so if the wrist keeps
    wobbling in and out relative to the elbow, that's noise. I learn the
    forearm length over time (heavily smoothed so it stays put), then
    rebuild the wrist as elbow + forearm_direction * that_fixed_length.
    That throws away the jitter acting along the arm, and the elbow is a
    steadier point to begin with. I blend it back with the raw wrist so I
    don't over-constrain genuine sideways movement."""
    fore = wrist - elbow
    L = np.linalg.norm(fore)
    if L < 1e-6:
        return wrist
    # learn the forearm length slowly so it acts as a stable reference
    if _forearm_len["L"] is None:
        _forearm_len["L"] = L
    else:
        _forearm_len["L"] = 0.98 * _forearm_len["L"] + 0.02 * L
    Lfixed = _forearm_len["L"]
    est = elbow + (fore / L) * Lfixed     # fixed length removes along-arm jitter
    return (1.0 - blend) * wrist + blend * est


# BODY_38 bone pairs to draw as a skeleton (index pairs)
_BONES = [
    ("PELVIS","SPINE_1"),("SPINE_1","SPINE_2"),("SPINE_2","SPINE_3"),
    ("SPINE_3","NECK"),("NECK","NOSE"),
    ("NECK","LEFT_CLAVICLE"),("LEFT_CLAVICLE","LEFT_SHOULDER"),
    ("LEFT_SHOULDER","LEFT_ELBOW"),("LEFT_ELBOW","LEFT_WRIST"),
    ("NECK","RIGHT_CLAVICLE"),("RIGHT_CLAVICLE","RIGHT_SHOULDER"),
    ("RIGHT_SHOULDER","RIGHT_ELBOW"),("RIGHT_ELBOW","RIGHT_WRIST"),
    ("PELVIS","LEFT_HIP"),("LEFT_HIP","LEFT_KNEE"),("LEFT_KNEE","LEFT_ANKLE"),
    ("PELVIS","RIGHT_HIP"),("RIGHT_HIP","RIGHT_KNEE"),("RIGHT_KNEE","RIGHT_ANKLE"),
]

def draw_skeleton(frame, body, conf, CONF_MIN):
    """Draw the tracked skeleton on the video, coloured by confidence so I
    can see at a glance which joints are solid and which are jittering."""
    kp2d = body.keypoint_2d
    def pt(name):
        i = getattr(sl.BODY_38_PARTS, name).value
        p = kp2d[i]
        if not (np.isfinite(p[0]) and np.isfinite(p[1])):
            return None, 0.0
        return (int(p[0]), int(p[1])), conf[i]
    for a, b in _BONES:
        pa, ca = pt(a); pb, cb = pt(b)
        if pa is None or pb is None:
            continue
        good = ca > CONF_MIN and cb > CONF_MIN
        col = (0, 220, 0) if good else (0, 140, 220)
        cv2.line(frame, pa, pb, col, 2)
    for name in ("RIGHT_WRIST","RIGHT_SHOULDER"):
        p, c = pt(name)
        if p is not None:
            cv2.circle(frame, p, 8, (0, 0, 255), -1)   # control points in red


def main():
    print("ZED arm tracking -> IK -> UDP to Unity.  keys: q quit  f freeze  r record")

    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.camera_fps = 60
    init_params.coordinate_units = sl.UNIT.METER
    init_params.depth_mode = sl.DEPTH_MODE.NEURAL
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP

    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        print("Camera failed to open")
        return

    zed.enable_positional_tracking(sl.PositionalTrackingParameters())

    body_param = sl.BodyTrackingParameters()
    body_param.enable_tracking = True
    body_param.enable_body_fitting = True
    body_param.detection_model = (sl.BODY_TRACKING_MODEL.HUMAN_BODY_ACCURATE
                                  if TRACK_MODEL == "ACCURATE"
                                  else sl.BODY_TRACKING_MODEL.HUMAN_BODY_FAST)
    body_param.body_format = sl.BODY_FORMAT.BODY_38
    zed.enable_body_tracking(body_param)

    body_runtime = sl.BodyTrackingRuntimeParameters()
    body_runtime.detection_confidence_threshold = 40

    # keypoint indices. If any name errors on your SDK version, list them with:
    # python -c "import pyzed.sl as sl; print([p.name for p in sl.BODY_38_PARTS])"
    RW  = sl.BODY_38_PARTS.RIGHT_WRIST.value
    RS  = sl.BODY_38_PARTS.RIGHT_SHOULDER.value
    RE  = sl.BODY_38_PARTS.RIGHT_ELBOW.value
    LW  = sl.BODY_38_PARTS.LEFT_WRIST.value
    LS  = sl.BODY_38_PARTS.LEFT_SHOULDER.value

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    cyl_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    cyl_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        cyl_sock.bind(("127.0.0.1", CYL_PORT))
        cyl_sock.setblocking(False)
    except OSError:
        print("warning: could not bind cylinder port %d" % CYL_PORT)
    cyl_unity = None           # latest cylinder position from Unity (m)
    cyl_at_rec = None          # snapshot taken when recording starts
    smoothed = None
    oc_filt = None
    frozen = False
    status = "waiting for body"

    grip_closed = False        # gesture state (with hysteresis)
    grip_smooth = 0.0          # 0 = open, 1 = closed, smoothed
    grip_dh_disp = 0.0         # wrist height above shoulder, for the overlay
    raise_armed = True         # used in toggle mode

    recording = False
    rec_rows = []
    rec_t0 = 0.0

    oe = OneEuro(OE_MINCUTOFF, OE_BETA)
    jitter_buf = []            # recent oc values, for the at-rest jitter readout
    jitter_mm = 0.0

    bodies = sl.Bodies()
    image = sl.Mat()

    while True:
        if zed.grab() != sl.ERROR_CODE.SUCCESS:
            continue

        zed.retrieve_image(image, sl.VIEW.LEFT)
        zed.retrieve_bodies(bodies, body_runtime)

        # drain any cylinder-position packets from Unity
        while True:
            try:
                data, _ = cyl_sock.recvfrom(512)
                m = json.loads(data.decode("utf-8"))
                cyl_unity = (float(m["cx"]), float(m["cy"]), float(m["cz"]))
            except (BlockingIOError, OSError, ValueError, KeyError):
                break

        if len(bodies.body_list) > 0:
            body = bodies.body_list[0]
            kp = body.keypoint
            conf = body.keypoint_confidence

            # ---------- right arm: position channel ----------
            wrist = np.array(kp[RW], dtype=float)
            shoulder = np.array(kp[RS], dtype=float)
            valid = (np.all(np.isfinite(wrist)) and np.all(np.isfinite(shoulder))
                     and conf[RW] > CONF_MIN and conf[RS] > CONF_MIN)

            if valid and not frozen:
                w = wrist
                if USE_ELBOW_STABILISE:
                    elbow = np.array(kp[RE], dtype=float)
                    if np.all(np.isfinite(elbow)):
                        w = stabilise_wrist(wrist, elbow, shoulder, ELBOW_BLEND)
                reach = w - shoulder
                reach_robot = camera_to_robot(reach)

                offset = S * 1000.0 * reach_robot
                n = np.linalg.norm(offset)
                if n > MAX_OFFSET:
                    offset *= MAX_OFFSET / n
                oc_raw = C_MM + offset

                if USE_ONE_EURO:
                    oc_filt = oe(oc_raw, time.time())
                elif oc_filt is None:
                    oc_filt = oc_raw.copy()
                else:
                    d = oc_raw - oc_filt
                    dist = np.linalg.norm(d)
                    if dist > DEADBAND_MM:
                        if dist > MAX_STEP_MM:
                            d *= MAX_STEP_MM / dist
                        oc_filt = oc_filt + POS_ALPHA * d

                # live jitter readout (std of target over the last ~1 s)
                jitter_buf.append(oc_filt.copy())
                if len(jitter_buf) > 60:
                    jitter_buf.pop(0)
                if len(jitter_buf) > 5:
                    arr = np.array(jitter_buf)
                    jitter_mm = float(np.linalg.norm(arr.std(axis=0)))

                ok, th1, th2, th3 = ur10_ik3(oc_filt)
                if ok:
                    new = np.array([th1, th2, th3])
                    if smoothed is None:
                        smoothed = new
                    else:
                        smoothed = ALPHA*new + (1.0 - ALPHA)*smoothed
                    status = "tracking  Oc=({:.0f},{:.0f},{:.0f})mm".format(*oc_filt)
                else:
                    status = "target unreachable, holding pose"
            elif not valid:
                status = "low confidence, holding pose"

            # ---------- left arm: gripper channel (raise gesture) ----------
            lw = np.array(kp[LW], dtype=float)
            ls = np.array(kp[LS], dtype=float)
            hand_ok = (np.all(np.isfinite(lw)) and np.all(np.isfinite(ls))
                       and conf[LW] > CONF_MIN and conf[LS] > CONF_MIN)
            if hand_ok:
                dh = lw[1] - ls[1]      # camera Y is up: + = wrist above shoulder
                grip_dh_disp = dh
                if GRIP_MODE == "hold":
                    if grip_closed and dh < GRIP_LOWER_BELOW:
                        grip_closed = False
                    elif (not grip_closed) and dh > GRIP_RAISE_ABOVE:
                        grip_closed = True
                else:                    # toggle mode
                    if raise_armed and dh > GRIP_RAISE_ABOVE:
                        grip_closed = not grip_closed
                        raise_armed = False
                    elif dh < GRIP_LOWER_BELOW:
                        raise_armed = True
            # if the arm is not visible, the last grip state is held

        else:
            status = "no body detected"

        grip_target = 1.0 if grip_closed else 0.0
        grip_smooth += GRIP_ALPHA * (grip_target - grip_smooth)

        # ---------- demo recording ----------
        if recording and oc_filt is not None:
            rec_rows.append((time.time() - rec_t0,
                             oc_filt[0], oc_filt[1], oc_filt[2],
                             grip_smooth))

        # ---------- send ----------
        if smoothed is not None:
            packet = {
                "j1": float(smoothed[0]),
                "j2": float(smoothed[1]),
                "j3": float(smoothed[2]),
                "j4": 0.0, "j5": 0.0, "j6": 0.0,
                "grip": float(grip_smooth),
                "frozen": frozen,
            }
            sock.sendto(json.dumps(packet).encode("utf-8"), (UNITY_IP, UNITY_PORT))

        # ---------- overlay ----------
        frame = image.get_data()
        label = ("FROZEN" if frozen else status)
        cv2.putText(frame, label, (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                    (0, 0, 255) if frozen else (0, 255, 0), 2)
        if smoothed is not None:
            deg = np.degrees(smoothed)
            cv2.putText(frame, "th1 %.1f  th2 %.1f  th3 %.1f deg" % tuple(deg),
                        (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
        cv2.putText(frame, "grip dh %+.2fm  %s" % (grip_dh_disp,
                    "CLOSED" if grip_closed else "OPEN"),
                    (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (0, 140, 255) if grip_closed else (255, 255, 255), 2)
        cv2.putText(frame, "jitter %.2f mm  [elbow:%s euro:%s]" %
                    (jitter_mm, "on" if USE_ELBOW_STABILISE else "off",
                     "on" if USE_ONE_EURO else "off"),
                    (20, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 255, 200), 2)
        if len(bodies.body_list) > 0:
            draw_skeleton(frame, bodies.body_list[0],
                          bodies.body_list[0].keypoint_confidence, CONF_MIN)
        if recording:
            cv2.putText(frame, "REC %d" % len(rec_rows), (20, 160),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        cv2.imshow("ZED | IK sender", frame)

        key = cv2.waitKey(1)
        if key == ord('q'):
            break
        if key == ord('f'):
            frozen = not frozen
        if key == ord('r'):
            if not recording:
                recording = True
                rec_rows = []
                rec_t0 = time.time()
                cyl_at_rec = cyl_unity
                if cyl_at_rec is None:
                    print("recording started (WARNING: no cylinder position "
                          "received from Unity, demo will lack pick data)")
                else:
                    print("recording started, cylinder at (%.3f, %.3f, %.3f)"
                          % cyl_at_rec)
            else:
                recording = False
                os.makedirs(REC_DIR, exist_ok=True)
                fname = os.path.join(REC_DIR,
                        time.strftime("demo_%Y%m%d_%H%M%S.csv"))
                c = cyl_at_rec if cyl_at_rec is not None else (float("nan"),)*3
                with open(fname, "w") as fh:
                    fh.write("t,x_mm,y_mm,z_mm,grip,cylx,cyly,cylz\n")
                    for row in rec_rows:
                        fh.write("%.4f,%.2f,%.2f,%.2f,%.3f,%.4f,%.4f,%.4f\n"
                                 % (row + c))
                print("saved %s (%d samples)" % (fname, len(rec_rows)))

    image.free(sl.MEM.CPU)
    zed.disable_body_tracking()
    zed.disable_positional_tracking()
    zed.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
