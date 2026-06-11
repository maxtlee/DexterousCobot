import numpy as np
import cv2
from pathlib import Path
from standardbots import StandardBotsRobot, models
from collections import deque
import base64
from time import sleep

import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib import gridspec

from ah_wrapper import AHSerialClient

import pyrealsense2 as rs

armA = (85*3.14/180, 16*3.14/180, 144*3.14/180, 22*3.14/180, -270*3.14/180, 180*3.14/180)
armB = (95*3.14/180, 16*3.14/180, 144*3.14/180, 22*3.14/180, -270*3.14/180, 180*3.14/180)

handOpen = [40, 40, 40, 40, 20, -60]
handClosed = [70, 70, 70, 70, 50, -80]

# ballColor as per-channel HSV [min H,S,V], [max H,S,V] (OpenCV scale:
# H 0-179, S/V 0-255); a pixel is a ball pixel when min <= channel <= max
# for all 3 HSV channels.
ballColor = ([18,110,34], [56,205,196])

ballDiameter = 0.065  # m (standard tennis ball)
ballRadius = ballDiameter / 2

# Custom tooltip offset: the physical tool point (grasp point) in the frame
# that get_arm_position() reports tooltip_position in. The robot keeps
# reporting/targeting its own tooltip frame; getToolPointInBase() /
# toolPointTargetToTooltip() apply this offset in code. Because camInTooltip
# stays relative to the *reported* tooltip, changing this offset does not
# invalidate the hand-eye calibration.
tooltipOffset = np.eye(4)
tooltipOffset[:3, 3] = [0.0, 0.12, -0.10]

# Hand-eye extrinsics: pose of the RealSense *color* camera in the frame that
# get_arm_position() reports tooltip_position in (4x4 homogeneous, meters).
# Produced by calibrate_handeye.py, which writes camInTooltip.npy next to this
# file. Until that exists, fall back to the hand-measured mounting offset:
# Y-0.03 m, Z-0.07 m, roll -120 deg about the tooltip X axis; the camera is
# mounted flipped 180 deg about its own optical (Z) axis.
rollRad = np.deg2rad(-120)
camInTooltipMeasured = np.eye(4)
camInTooltipMeasured[:3, :3] = np.array(
    [[1, 0, 0],
     [0, np.cos(rollRad), -np.sin(rollRad)],
     [0, np.sin(rollRad),  np.cos(rollRad)]]
) @ np.diag([-1.0, -1.0, 1.0])  # Rz(180): the 180-deg sensor flip
camInTooltipMeasured[:3, 3] = [0.0, -0.03, -0.07]

camInTooltipFile = Path(__file__).with_name("camInTooltip.npy")
if camInTooltipFile.exists():
    camInTooltip = np.load(camInTooltipFile)
else:
    print("camInTooltip.npy not found; using hand-measured camera offset "
          "(run calibrate_handeye.py for a calibrated one)")
    camInTooltip = camInTooltipMeasured

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

def main():
    with ArmClient() as arm, CamClient() as cam:
        # initArm(arm)
        # moveHand(hand, handOpen)

        pose = getTennisBallPose(arm, cam, debug=True)
        if pose is None:
            print("tennis ball: not found")
        else:
            position, orientation = pose
            print("tennis ball position (m, robot base frame):",
                  np.array2string(position, precision=4, suppress_small=True))
            print("tennis ball orientation (quaternion xyzw, placeholder):", orientation)

        # moveHand(hand, handClosed)

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
    the ball.
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

def getTooltipInBase(arm):
    """Current tooltip pose as a 4x4 base-frame transform (meters)."""
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
    return T

def getToolPointInBase(arm):
    """Current physical tool point (tooltipOffset applied) as a 4x4 base-frame
    transform (meters)."""
    return getTooltipInBase(arm) @ tooltipOffset

def toolPointTargetToTooltip(target):
    """Reported-tooltip pose that puts the physical tool point at `target`
    (4x4 base-frame pose). Pass the result to move_tooltip."""
    return target @ np.linalg.inv(tooltipOffset)

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

def moveArm(arm, position):
    body = models.ArmPositionUpdateRequest(
        kind=models.ArmPositionUpdateRequestKindEnum.JointRotation,
        joint_rotation=models.ArmJointRotations(joints=position),
    )
    response = arm.movement.position.set_arm_position(body)
    try:
        print(response.ok())
    except Exception:
        print(response.data.message)

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
