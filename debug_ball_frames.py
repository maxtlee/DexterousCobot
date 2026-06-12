#!/usr/bin/env python3
"""Coordinate-frame debugger: live tennis-ball position in every frame.

Prints one line per reading with the detected ball center expressed in the
camera, reported-tooltip, tool-point (tooltipOffset applied), and robot base
frames, so each link of the chain

    pixel + depth -> camera --camInTooltip--> tooltip --tooltipOffset--> tool
                                tooltip --arm pose--> base

can be checked in isolation:
  * ball still, arm moving: base must stay put. Drift that grows with arm
    rotation means camInTooltip is off (rerun calibrate_handeye.py).
  * everything still: cam/tooltip/tool/base must all be steady; jitter here
    is detection/depth noise, before any transform is involved.
  * ball at a spot you can measure: base should match the tape measure, and
    tool reads (0, 0, 0) exactly when the grasp point touches the ball center.

The kf column is a zero-velocity (random-walk) Kalman estimate of the
base-frame position (combinedtest.BallKalman) with its largest per-axis std;
on missed detections the estimate stays put while the std grows. Compare kf
against raw base to judge how much smoothing/lag the current tuning
(sigmaWalk/sigmaMeas) gives.

The jt column is the 6-joint movement target (degrees, J0..J5) that would
put the grasp point at the kf estimate, from combinedtest.ballJointTarget:
wrist level (J1+J2+J3 = 180 deg), J4 = -270 deg, J5 = 180 deg, nearest of
the IK solutions to the arm's current joints. Display only — this script
still never commands motion.

SAFETY: read-only — never commands motion and never brakes/unbrakes the arm.
Freedrive/jog the arm while it runs.

Usage:
    .venv/bin/python debug_ball_frames.py    (Ctrl+C to stop)
"""

import time

import numpy as np

from calibrate_handeye import ArmReader
from combinedtest import (BallKalman, CamClient, ballJointTarget, getArmJoints,
                          getFrames, getTooltipInBase, maskBall,
                          fitCircleInArray, ballCenterInCam, camInTooltip,
                          tooltipOffset)

toolFromTooltip = np.linalg.inv(tooltipOffset)

def toFrame(T, p):
    return (T @ np.append(p, 1.0))[:3]

def fmt(p):
    return np.array2string(p, precision=3, suppress_small=True,
                           floatmode="fixed")

def jointTargetStr(arm, kf):
    """Joint target (deg) that would put the grasp point at the kf estimate."""
    target = ballJointTarget(kf.position, getArmJoints(arm))
    if target is None:
        return "jt: unreachable"
    return "jt" + np.array2string(np.degrees(target), precision=1,
                                  suppress_small=True, floatmode="fixed") + " deg"

def main():
    print("all positions in meters; Ctrl+C to stop")
    kf = BallKalman()
    lastT = time.monotonic()
    with ArmReader() as arm, CamClient() as cam:
        while True:
            # Frame first, pose second, back to back (same caveat as
            # getTennisBallPose: no hardware sync, so trust readings taken
            # while the arm is still or moving slowly). The flush keeps the
            # frame current — at this loop rate the pipeline queue would
            # otherwise serve ever-staler images.
            for _ in range(2):
                frame, depthM, intr = getFrames(cam)
            tooltipInBase = getTooltipInBase(arm)

            now = time.monotonic()
            kf.predict(now - lastT)
            lastT = now

            fit = fitCircleInArray(maskBall(frame))
            if fit is None:
                if kf.position is None:
                    print("ball: not found")
                else:  # zero-velocity model: estimate holds, sigma grows
                    print(f"ball: not found  kf base{fmt(kf.position)} "
                          f"+-{kf.sigma.max():.3f}  {jointTargetStr(arm, kf)}")
                continue

            pCam = ballCenterInCam(fit[0], fit[1], depthM, intr)
            pTooltip = toFrame(camInTooltip, pCam)
            pTool = toFrame(toolFromTooltip, pTooltip)
            pBase = toFrame(tooltipInBase, pTooltip)
            kf.update(pBase)

            print(f"cam{fmt(pCam)}  tooltip{fmt(pTooltip)}  tool{fmt(pTool)}  "
                  f"base{fmt(pBase)}  kf{fmt(kf.position)} "
                  f"+-{kf.sigma.max():.3f}  {jointTargetStr(arm, kf)}")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
