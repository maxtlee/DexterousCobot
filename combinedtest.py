import json
import numpy as np
import cv2
from pathlib import Path
from standardbots import StandardBotsRobot, models
from collections import deque
import base64
from time import sleep, monotonic

import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib import gridspec

from ah_wrapper import AHSerialClient

import pyrealsense2 as rs

armHome = (-60*3.14/180, -25*3.14/180, 113*3.14/180, 126*3.14/180, -323*3.14/180, 147*3.14/180)

handOpen = [2.5, 2.5, 2.5, 2.5, 2.5, -35]
handMiddle = [30, 30, 30, 30, 2.5, -99]
handClosed = [45, 45, 45, 45, 22, -99]

# ballColor as per-channel HSV [min H,S,V], [max H,S,V] (OpenCV scale:
# H 0-179, S/V 0-255); a pixel is a ball pixel when min <= channel <= max
# for all 3 HSV channels.
ballColor = ([9,110,20], [75,205,196])

ballDiameter = 0.065  # m (standard tennis ball)
ballRadius = ballDiameter / 2

# The hand-measured offsets below (grasp point and camera, measured in the
# same session) turned out to be expressed in a tooltip frame whose X/Y axis
# labels are yawed 90 deg from the frame the robot actually reports.
# Diagnosed 2026-06-12: pushing the ball radially toward the base column made
# the computed base position move tangentially (a pure vertical-axis yaw
# error), while floor-plane and hand-projection tests had already pinned the
# camera tilt and ruled out every other axis. tooltipFrameFix remaps every
# hand-measured vector into the true frame. If a radial-motion test still
# shows tangential drift (mirrored direction), the sign is wrong: use -90.
tooltipFrameYaw = np.deg2rad(-90)
tooltipFrameFix = np.array(
    [[np.cos(tooltipFrameYaw), -np.sin(tooltipFrameYaw), 0],
     [np.sin(tooltipFrameYaw),  np.cos(tooltipFrameYaw), 0],
     [0, 0, 1]])

# Custom tooltip offset: the physical tool point (grasp point) in the frame
# that get_arm_position() reports tooltip_position in. The robot keeps
# reporting/targeting its own tooltip frame; getToolPointInBase() /
# toolPointTargetToTooltip() apply this offset in code. Because camInTooltip
# stays relative to the *reported* tooltip, changing this offset does not
# invalidate the hand-eye calibration. Measured: Y+0.12 m, Z-0.10 m.
tooltipOffset = np.eye(4)
tooltipOffset[:3, 3] = tooltipFrameFix @ [-0.05, 0.12, -0.13]

# Hand-eye extrinsics: pose of the RealSense *color* camera in the frame that
# get_arm_position() reports tooltip_position in (4x4 homogeneous, meters).
# Produced by calibrate_handeye.py, which writes camInTooltip.npy next to this
# file. Until that exists, fall back to the hand-measured mounting offset:
# Y-0.03 m, Z-0.07 m, roll -120 deg about the (measured) tooltip X axis; the
# camera is mounted flipped 180 deg about its own optical (Z) axis.
rollRad = np.deg2rad(-120)
camInTooltipMeasured = np.eye(4)
camInTooltipMeasured[:3, :3] = tooltipFrameFix @ np.array(
    [[1, 0, 0],
     [0, np.cos(rollRad), -np.sin(rollRad)],
     [0, np.sin(rollRad),  np.cos(rollRad)]]
) @ np.diag([-1.0, -1.0, 1.0])  # Rz(180): the 180-deg sensor flip
camInTooltipMeasured[:3, 3] = tooltipFrameFix @ [0.0, -0.03, -0.07]

camInTooltipFile = Path(__file__).with_name("camInTooltip.npy")
if camInTooltipFile.exists():
    camInTooltip = np.load(camInTooltipFile)
else:
    print("camInTooltip.npy not found; using hand-measured camera offset "
          "(run calibrate_handeye.py for a calibrated one)")
    camInTooltip = camInTooltipMeasured

# Valid pickup region (axis-aligned box, base frame), taught by sweeping the
# ball through valid/invalid positions with collect_ball_region.py. Without
# it every reachable position counts as valid (the routine warns at startup).
ballRegionFile = Path(__file__).with_name("ballRegion.json")
if ballRegionFile.exists():
    _region = json.loads(ballRegionFile.read_text())
    ballRegionMin = np.array(_region["min"])
    ballRegionMax = np.array(_region["max"])
else:
    ballRegionMin = ballRegionMax = None

def isValidBallPosition(p):
    """True when p (base-frame xyz, m) is inside the taught valid region."""
    if ballRegionMin is None:
        return True
    return bool(np.all(p >= ballRegionMin) and np.all(p <= ballRegionMax))

metersPerUnit = {
    models.LinearUnitKind.Millimeters: 0.001,
    models.LinearUnitKind.Centimeters: 0.01,
    models.LinearUnitKind.Meters: 1.0,
    models.LinearUnitKind.Inches: 0.0254,
    models.LinearUnitKind.Feet: 0.3048,
}

# Camera settings the robot used to apply to this camera, now applied directly
# to the RealSense RGB sensor. These are UVC controls and the values are already
# on the RealSense scale (it is the same camera / same control ranges).
cameraSettings = {
    "brightness": 0,
    "contrast": 40,
    "exposure": 100,          # manual exposure; auto-exposure is turned off to apply it
    "sharpness": 50,
    "hue": 0,
    "white_balance": 4600,
    "auto_white_balance": True,
}

# --- pickup-routine tuning
waitingStableS = 0.5          # ball must be valid + stationary this long
stationaryTolM = 0.02         # max kf wander allowed within that window
reachedTolRad = np.deg2rad(3.0)   # per-joint |current - target| = "arrived"
retargetTolRad = np.deg2rad(1.0)  # re-send when the target moves this much
handActuateS = 1.5            # settle time after each hand command
routineSpeedScale = 0.3       # conservative speed for autonomous motion
homeTimeoutS = 20.0

def main():
    """Infinite pickup cycle: wait for a valid, stationary ball -> move to it
    with continuous Kalman-based retargeting -> grasp -> deliver home -> drop
    -> wait again. Aborts home whenever the ball estimate leaves the taught
    valid region (or becomes unreachable)."""
    if ballRegionMin is None:
        print("WARNING: no ballRegion.json — every reachable position counts "
              "as valid. Teach the region with collect_ball_region.py.")

    with ArmClient() as arm, CamClient() as cam, HandClient() as hand:
        initArm(arm)
        moveHand(hand, handOpen)
        goHome(arm)
        kf = BallKalman()
        history = deque()       # (t, kf position) while continuously valid
        lastSent = None
        state = "waiting"
        lastT = monotonic()
        print("waiting for a ball...")

        while True:
            lastT, detected, joints = perceiveBall(arm, cam, kf, lastT)
            pos = kf.position
            target = None
            if pos is not None and isValidBallPosition(pos):
                target = ballJointTarget(pos, joints)

            if state == "waiting":
                if detected and target is not None:
                    history.append((lastT, pos.copy()))
                    while history and lastT - history[0][0] > waitingStableS + 0.2:
                        history.popleft()
                    pts = np.array([p for _, p in history])
                    if (lastT - history[0][0] >= waitingStableS
                            and np.linalg.norm(pts.max(0) - pts.min(0)) < stationaryTolM):
                        print(f"ball stable at {np.round(pos, 3)} -> moving")
                        state, lastSent = "moving", None
                else:
                    history.clear()

            elif state == "moving":
                if target is None:
                    print("ball left the valid region -> returning home")
                    goHome(arm)
                    kf, lastSent, state = BallKalman(), None, "waiting"
                    history.clear()
                    print("waiting for a ball...")
                elif armReached(joints, target):
                    graspAndDeliver(arm, hand)
                    kf, lastSent, state = BallKalman(), None, "waiting"
                    history.clear()
                    print("waiting for a ball...")
                elif (lastSent is None
                        or np.max(np.abs(target - lastSent)) > retargetTolRad):
                    if moveArm(arm, target, speedScale=routineSpeedScale):
                        lastSent = target

def perceiveBall(arm, cam, kf, lastT):
    """One vision tick: predict the kf and fold in a detection when there is
    one. Measurements are accepted while the arm moves: there is no hardware
    sync between frame and pose, so a mid-motion measurement is stale by up
    to a frame time (a few mm at the routine's speed scale) — but folding
    them in is what lets the target keep refining as the camera closes in,
    and lets the routine follow a ball that is moved mid-approach. Detections
    clipped by the frame edge are rejected in fitCircleInArray; on any miss
    (occlusion by the hand included) the zero-velocity kf just coasts.

    Returns (timestamp, updated: bool, current joints)."""
    for _ in range(2):  # flush so the frame is current
        frame, depthM, intr = getFrames(cam)
    T, joints = getArmState(arm)  # right after the grab: minimal frame/pose skew

    now = monotonic()
    kf.predict(now - lastT)

    fit = fitCircleInArray(maskBall(frame))
    if fit is None:
        return now, False, joints

    pCam = ballCenterInCam(fit[0], fit[1], depthM, intr)
    kf.update((T @ camInTooltip @ np.append(pCam, 1.0))[:3])
    return now, True, joints

def armReached(current, target, tol=reachedTolRad):
    """True when every joint is within tol of the target (2*pi-wrapped)."""
    d = np.abs(np.asarray(current) - np.asarray(target)) % (2 * np.pi)
    return bool(np.max(np.minimum(d, 2 * np.pi - d)) < tol)

def goHome(arm):
    moveArm(arm, armHome, speedScale=routineSpeedScale)
    waitUntilReached(arm, armHome)

def waitUntilReached(arm, target, timeoutS=homeTimeoutS):
    t0 = monotonic()
    while monotonic() - t0 < timeoutS:
        if armReached(getArmJoints(arm), target):
            return True
        sleep(0.1)
    print(f"warning: arm did not reach the target within {timeoutS:.0f} s")
    return False

def graspAndDeliver(arm, hand):
    print("at the ball -> grasping")
    moveHand(hand, handMiddle)
    sleep(handActuateS)
    moveHand(hand, handClosed)
    sleep(handActuateS)
    print("delivering home")
    goHome(arm)
    moveHand(hand, handOpen)
    sleep(handActuateS)
    print("dropped — reset the ball to run another cycle")

def getTennisBallPose(arm, cam, debug=False):
    """Absolute 6-DoF pose of the tennis ball in the robot base frame.

    Returns (position, orientation) — position as an xyz vector in meters,
    orientation as a quaternion xyzw — or None when no ball is visible.

    Chain: tuned HSV mask -> circle fit -> 3D center in the camera frame
    (stereo depth, apparent size as fallback) -> camInTooltip hand-eye
    extrinsics -> current tooltip pose -> base frame.

    The frame grab and the arm-pose read happen back to back, but there is no
    hardware sync: call this while the arm is stationary (or moving slowly)
    or the camera pose will be stale by a frame time.
    """
    frame, depthM, intr = getFrames(cam)
    tooltipInBase = getTooltipInBase(arm)

    mask = maskBall(frame)
    fit = fitCircleInArray(mask)

    if debug:
        cv2.imwrite("frame.jpeg", frame)
        out = np.zeros_like(frame)
        out[mask > 0] = (0, 0, 255)
        if fit is not None:
            (cx, cy), d = fit
            cv2.circle(out, (round(cx), round(cy)), round(d / 2), (0, 255, 0), 1)
        cv2.imwrite("filtered.jpeg", out)

    if fit is None:
        return None

    centerCam = ballCenterInCam(fit[0], fit[1], depthM, intr)
    baseFromCam = tooltipInBase @ camInTooltip
    position = (baseFromCam @ np.append(centerCam, 1.0))[:3]

    # TODO orientation: a tennis ball is rotationally symmetric except for its
    # seam, so orientation is unobservable with this pipeline. Identity for now;
    # estimating the seam from the mask interior would go here.
    orientation = np.array([0.0, 0.0, 0.0, 1.0])  # quaternion xyzw

    return position, orientation

def maskBall(frame):
    """Binary ball mask (uint8 0/255) from the tuned HSV window. frame is RGB."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
    return cv2.inRange(hsv, np.array(ballColor[0], np.uint8),
                            np.array(ballColor[1], np.uint8))

# Returns a location (x,y) and diameter
def fitCircleInArray(pixelArray):
    """Fit a circle to the largest blob of a binary mask.

    Returns ((x, y), diameter) in pixels, or None when nothing ball-like is
    found. Open/close morphology kills the speckle the HSV window lets
    through; a circularity gate rejects non-ball blobs (carpet streaks). The
    minimum enclosing circle sets the size: unlike an area-equivalent radius
    it stays close to the true diameter when the gripper occludes part of
    the ball. Balls clipped by the frame edge are rejected outright: a
    truncated disc fits a circle with a corrupted center and diameter, and
    with apparent-size ranging a wrong diameter means a wrong distance.
    """
    mask = (np.asarray(pixelArray) > 0).astype(np.uint8) * 255
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)   # drop speckle
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # bridge the seam line

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not contours:
        return None
    contour = max(contours, key=cv2.contourArea)

    area = cv2.contourArea(contour)
    if area < 60:  # ~the ball at 4 m; smaller is noise
        return None
    circularity = 4 * np.pi * area / cv2.arcLength(contour, True) ** 2
    if circularity < 0.1:  # 1.0 = perfect circle; a half-occluded disc is ~0.75
        return None

    (x, y), radius = cv2.minEnclosingCircle(contour)
    h, w = mask.shape
    if (x - radius < 2 or y - radius < 2
            or x + radius > w - 2 or y + radius > h - 2):
        return None  # clipped by the frame edge
    return ((x, y), 2 * radius)

def ballCenterInCam(center, diameterPx, depthM, intr):
    """3D ball center in the color-camera frame (m; +X right, +Y down, +Z out).

    Primary: median stereo depth over the middle of the detected disc, pushed
    half a ball deeper (depth sees the front surface, we want the center).
    Fallback when depth has no valid samples there — ball nearer than the
    D415 min-z (~30 cm) during a grasp approach, or an IR-dark patch — is the
    apparent size: with the true diameter known, range = fx * D / d_px.
    """
    cx, cy = center

    # Depth is disabled for now because it is inaccurate at the desired range
    # depthSurf = sampleBallDepth(depthM, center, diameterPx / 2)
    # if depthSurf is not None:
    #     # The sampled disc sees the front cap at z = Zcenter - R*cos(theta);
    #     # the median over the inner 55% works out to Zcenter - ~0.92 R.
    #     return np.array(rs.rs2_deproject_pixel_to_point(
    #         intr, [cx, cy], depthSurf + 0.92 * ballRadius))

    # print("ball depth unavailable; falling back to apparent-size range")
    ray = np.array(rs.rs2_deproject_pixel_to_point(intr, [cx, cy], 1.0))
    dist = intr.fx * ballDiameter / diameterPx  # range to the ball center
    return ray * (dist / np.linalg.norm(ray))

def sampleBallDepth(depthM, center, radiusPx):
    """Median valid depth (m) over the inner 55% of the detected circle.

    Staying inside the circle avoids edge pixels where stereo depth bleeds
    into the background. Returns None when too few pixels carry depth.
    """
    cx, cy = center
    r = max(2.0, 0.55 * radiusPx)
    ys, xs = np.ogrid[:depthM.shape[0], :depthM.shape[1]]
    samples = depthM[(xs - cx) ** 2 + (ys - cy) ** 2 <= r * r]
    samples = samples[samples > 0]
    if samples.size < 10:
        return None
    return float(np.median(samples))

class BallKalman:
    """Zero-velocity (random-walk) Kalman filter for the ball center (meters).

    Feed it base-frame measurements: the ball sits still in the world while
    camera-frame measurements jump whenever the arm moves, so the base frame
    is the one where "the ball is not moving" is actually true.

    The motion model assumes zero velocity: predict() leaves the position
    estimate where it is and only inflates its covariance. During detection
    dropouts the estimate therefore stays put instead of coasting away on a
    velocity fitted to position jitter — with ~5 mm of jitter at ~15 Hz the
    apparent velocity noise is ~0.1 m/s, swamping any real motion of a parked
    ball. A ball that does move is still tracked (motion enters through the
    measurement updates), just with lag, and it cannot be extrapolated
    through an occlusion.

    Call predict(dt) once per cycle with the elapsed time, then update(z)
    when there is a detection; on a miss just skip update.

    Tuning: sigmaMeas is the per-axis std of one measurement (~2 mm observed
    at 0.3 m with stereo depth; raise it when the apparent-size fallback is
    in play). sigmaWalk (m/sqrt(s)) says how far the "stationary" ball may
    credibly drift per unit time; it sets the smoothing/lag trade-off. The
    defaults give a steady-state gain of ~0.4 at 15 Hz: jitter is roughly
    halved, and a ball carried at 0.1 m/s trails by ~1 cm. Lower sigmaWalk
    for a parked ball (more smoothing), raise it to track hand motion.
    """

    def __init__(self, sigmaWalk=0.01, sigmaMeas=0.005):
        self.sigmaWalk = sigmaWalk
        self.sigmaMeas = sigmaMeas
        self.x = None  # [px, py, pz]
        self.P = None  # 3x3 position covariance

    def predict(self, dt):
        """Advance the estimate by dt seconds: position is unchanged (zero-
        velocity model), only the uncertainty grows. No-op before the first
        update."""
        if self.x is None:
            return
        self.P = self.P + self.sigmaWalk ** 2 * dt * np.eye(3)

    def update(self, z):
        """Fold in a measured base-frame ball position (3-vector, meters)."""
        z = np.asarray(z, dtype=float)
        if self.x is None:
            self.x = z.copy()
            self.P = self.sigmaMeas ** 2 * np.eye(3)
            return
        # H = I: the full state is measured directly.
        S = self.P + self.sigmaMeas ** 2 * np.eye(3)
        K = self.P @ np.linalg.inv(S)
        self.x = self.x + K @ (z - self.x)
        self.P = self.P - K @ self.P
        self.P = (self.P + self.P.T) / 2  # keep symmetric against roundoff

    @property
    def position(self):
        """Estimated ball center (3-vector, m), or None before the first update."""
        return None if self.x is None else self.x.copy()

    @property
    def sigma(self):
        """Per-axis position std (3-vector, m), or None before the first update."""
        return None if self.P is None else np.sqrt(np.diag(self.P))

def getArmState(arm):
    """(tooltip pose as 4x4 base-frame transform, joints as 6-vector of
    radians) from a single get_arm_position call."""
    combined = arm.movement.position.get_arm_position().ok()
    tooltip = combined.tooltip_position
    if (tooltip is None or tooltip.position is None
            or tooltip.orientation is None or tooltip.orientation.quaternion is None):
        raise RuntimeError("arm did not report a tooltip pose with a quaternion")

    scale = metersPerUnit.get(tooltip.position.unit_kind, 1.0)
    q = tooltip.orientation.quaternion
    T = np.eye(4)
    T[:3, :3] = quatToMat(q.x, q.y, q.z, q.w)
    T[:3, 3] = np.array([tooltip.position.x, tooltip.position.y, tooltip.position.z]) * scale
    return T, np.array(combined.joint_rotations, dtype=float)

def getTooltipInBase(arm):
    """Current tooltip pose as a 4x4 base-frame transform (meters)."""
    return getArmState(arm)[0]

def getToolPointInBase(arm):
    """Current physical tool point (tooltipOffset applied) as a 4x4 base-frame
    transform (meters)."""
    return getTooltipInBase(arm) @ tooltipOffset

def toolPointTargetToTooltip(target):
    """Reported-tooltip pose that puts the physical tool point at `target`
    (4x4 base-frame pose). Pass the result to move_tooltip."""
    return target @ np.linalg.inv(tooltipOffset)

# Arm geometry for ballJointTarget, fitted against the robot's own FK
# endpoint (/api/v1/poses/joint-pose) on 2026-06-12 over a 36-pose grid with
# the wrist held level (J1+J2+J3 = 180 deg, J4 = -270 deg, J5 = 180 deg).
# In that configuration the tooltip orientation is the identity (level) to
# 0.21 deg, J0 is a pure base-Z rotation, and the J1/J2/J3 pitch axes are
# parallel: the tooltip in the frame rotating with J0 is
#   radial   r = a2*sin(J1+b1) + a3*sin(J1+J2+b2) + rConst
#   vertical z = a2*cos(J1+b1) + a3*cos(J1+J2+b2) + zConst
#   lateral  y = yConst
# In-plane fit residual < 3 um; the lateral constant wobbles ~3 mm across the
# workspace (factory-calibrated, slightly non-ideal axes) — that is the
# accuracy floor of this model.
armLevelSum = np.pi          # J1+J2+J3 that keeps the wrist level
armUpperArm = 0.591138       # a2 (m), J1 -> J2
armForearm = 0.549556        # a3 (m), J2 -> J3
armJ1Zero = 0.00538457       # b1 (rad), joint-zero calibration offset
armJ12Zero = -0.00413454     # b2 (rad)
armRadialConst = 0.162179    # rConst (m), tooltip chain (without tooltipOffset)
armLateralConst = 0.193491   # yConst (m)
armVerticalConst = 0.021837  # zConst (m)

def ballJointTarget(ball, currentJoints,
                    j4=np.deg2rad(-270), j5=np.deg2rad(180)):
    """6-joint target (radians) placing the grasp point at `ball` (base frame, m).

    The wrist is fully constrained by design: J5 and J4 are held at the given
    values (defaults match armA/armB) and J3 takes whatever angle keeps the
    arm level (J1+J2+J3 = armLevelSum). Under that constraint the grasp point
    sits at a constant offset from the wrist in the frame rotating with J0,
    so the wrist position is backcalculated from the ball position, and
    J0/J1/J2 follow from the lateral-offset shoulder solution plus standard
    planar two-link IK. Of the up-to-4 solutions (shoulder front/back x
    elbow up/down, each also wrapped by 2*pi toward the current pose) the one
    with the smallest total absolute joint difference from currentJoints is
    returned, or None when the ball is out of reach.

    Joint limits are not checked here; the robot rejects an infeasible target
    when it is commanded.
    """
    ball = np.asarray(ball, dtype=float)
    current = np.asarray(currentJoints, dtype=float)

    # Grasp-point chain constants: tooltipOffset's translation is expressed in
    # the level tooltip frame, which is axis-aligned with the rotating frame.
    rT = armRadialConst + tooltipOffset[0, 3]
    yL = armLateralConst + tooltipOffset[1, 3]
    zC = armVerticalConst + tooltipOffset[2, 3]

    candidates = []
    lat2 = ball[0] ** 2 + ball[1] ** 2 - yL ** 2
    if lat2 > 0:
        azimuth = np.arctan2(ball[1], ball[0])
        for rSign in (1.0, -1.0):  # grasp point in front of / behind the J0 axis
            r = rSign * np.sqrt(lat2)
            q0 = azimuth - np.arctan2(yL, r)
            rr, zz = r - rT, ball[2] - zC
            cosElbow = ((rr ** 2 + zz ** 2 - armUpperArm ** 2 - armForearm ** 2)
                        / (2 * armUpperArm * armForearm))
            if abs(cosElbow) > 1:
                continue
            for elbow in (1.0, -1.0):
                g = elbow * np.arccos(cosElbow)  # angle J2 (forearm vs upper arm)
                u1 = (np.arctan2(rr, zz)
                      - np.arctan2(armForearm * np.sin(g),
                                   armUpperArm + armForearm * np.cos(g)))
                q1 = u1 - armJ1Zero
                q2 = g + armJ1Zero - armJ12Zero
                q3 = armLevelSum - q1 - q2  # level the wrist
                candidates.append([q0, q1, q2, q3, j4, j5])

    best, bestCost = None, np.inf
    for cand in candidates:
        q = np.array(cand)
        # A joint shifted by 2*pi is the same physical pose; compare the
        # representative nearest the current angle (J4/J5 stay as given).
        q[:4] -= 2 * np.pi * np.round((q[:4] - current[:4]) / (2 * np.pi))
        cost = np.abs(q - current).sum()
        if cost < bestCost:
            best, bestCost = q, cost
    return best

def getArmJoints(arm):
    """Current joint rotations J0..J5 (radians) as a 6-vector."""
    return getArmState(arm)[1]

def quatToMat(x, y, z, w):
    n = np.sqrt(x*x + y*y + z*z + w*w)
    x, y, z, w = x/n, y/n, z/n, w/n
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)],
    ])

def getFrames(cam):
    """One synchronized (RGB frame, depth-in-meters array, color intrinsics).

    Depth is aligned into the color frame, so a pixel in the mask indexes
    straight into the depth array and deprojects with the color intrinsics.
    """
    color = depth = None
    while not (color and depth):
        frames = cam.align.process(cam.pipeline.wait_for_frames())
        color = frames.get_color_frame()
        depth = frames.get_depth_frame()
    frame = np.asanyarray(color.get_data())
    depthM = np.asanyarray(depth.get_data()) * cam.depthScale
    intr = color.profile.as_video_stream_profile().get_intrinsics()
    return frame, depthM, intr

def getCameraFrame(cam):
    """RGB frame only (kept for tune_ball_color.py)."""
    return getFrames(cam)[0]

_lastRejection = None

def moveArm(arm, position, speedScale=None):
    """Send a joint-space target. Returns True when the robot accepted it.

    Re-sending while a previous move is executing is also how the routine
    retargets mid-flight: the API has no cancel endpoint (verified:
    routine-editor/stop demands a running routine, emergency-stop faults the
    arm), so the new target either preempts the old one or — if the firmware
    queues/rejects mid-motion sends — takes effect when the current leg ends
    (run check_preemption.py to see which). Rejections are printed once per
    distinct message (not per retry tick) and reported as False.

    speedScale (0..1) scales the default speed profile; the pickup routine
    uses a conservative value.
    """
    global _lastRejection
    profile = None
    if speedScale is not None:
        profile = models.SpeedProfile(scaling_factor=float(speedScale))
    body = models.ArmPositionUpdateRequest(
        kind=models.ArmPositionUpdateRequestKindEnum.JointRotation,
        joint_rotation=models.ArmJointRotations(
            joints=tuple(float(j) for j in position)),
        speed_profile=profile,
    )
    response = arm.movement.position.set_arm_position(body)
    try:
        response.ok()
        _lastRejection = None
        return True
    except Exception:
        message = getattr(response.data, "message", response.data)
        if message != _lastRejection:
            print("moveArm rejected:", message)
            _lastRejection = message
        return False

def initArm(arm):
    arm.movement.brakes.unbrake().ok()
    arm.status.control.set_configuration_control_state(models.RobotControlMode(kind=models.RobotControlModeEnum.Api)).ok()
    arm.recovery.recover.recover().ok()

def moveHand(hand, position):
    hand.set_position(positions=position, reply_mode=2)
    hand.send_command()

class HandClient:
    def __enter__(self):
        self.hand = AHSerialClient(write_thread=False)
        return self.hand

    def __exit__(self,a,b,c):
        self.hand.close()

class ArmClient:
    def __enter__(self):
        self.sdk = StandardBotsRobot(
            url='http://192.168.1.3:3000',
            token=Path("/home/max/Documents/.robot_token").read_text().strip(),
            robot_kind=StandardBotsRobot.RobotKind.Live,
        )
        return self.sdk

    def __exit__(self, a, b, c):
        self.sdk.movement.brakes.brake().ok()
        self.sdk._request_manager.close()

def applyCameraConfig(profile, settings=cameraSettings):
    """Apply the camera settings (UVC controls) to the RealSense RGB sensor.

    Mirrors what the robot used to set when the camera was connected through it.
    Each value is clamped into the option's supported range; auto-exposure is
    disabled so the manual exposure takes effect, and white balance honours the
    auto_white_balance flag.
    """
    color = next(s for s in profile.get_device().query_sensors()
                 if s.get_info(rs.camera_info.name) == "RGB Camera")

    def setOpt(option, value):
        if not color.supports(option):
            return
        rng = color.get_option_range(option)
        value = min(max(value, rng.min), rng.max)  # clamp into supported range
        try:
            color.set_option(option, float(value))
        except Exception as e:
            print(f"camera: could not set {option}: {e}")

    # Manual exposure only takes effect once auto-exposure is off.
    setOpt(rs.option.enable_auto_exposure, 0)
    setOpt(rs.option.exposure, settings["exposure"])

    # White balance: honour the auto flag; set a manual value only when auto off.
    if settings["auto_white_balance"]:
        setOpt(rs.option.enable_auto_white_balance, 1)
    else:
        setOpt(rs.option.enable_auto_white_balance, 0)
        setOpt(rs.option.white_balance, settings["white_balance"])

    setOpt(rs.option.brightness, settings["brightness"])
    setOpt(rs.option.contrast, settings["contrast"])
    setOpt(rs.option.sharpness, settings["sharpness"])
    setOpt(rs.option.hue, settings["hue"])


class CamClient:
    def __enter__(self):
        self.pipeline = rs.pipeline()
        config = rs.config()
        # Same 640x480 RGB the HSV window was tuned on, plus the depth stream.
        config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        profile = self.pipeline.start(config)
        applyCameraConfig(profile)
        self.align = rs.align(rs.stream.color)
        self.depthScale = profile.get_device().first_depth_sensor().get_depth_scale()
        for _ in range(15):  # let auto white balance and the depth filters settle
            self.pipeline.wait_for_frames()
        return self

    def __exit__(self, a, b, c):
        self.pipeline.stop()

if __name__ == "__main__":
    main()
