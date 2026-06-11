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

SAFETY: read-only — never commands motion and never brakes/unbrakes the arm.
Freedrive/jog the arm while it runs.

Usage:
    .venv/bin/python debug_ball_frames.py    (Ctrl+C to stop)
"""

import numpy as np

from calibrate_handeye import ArmReader
from combinedtest import (CamClient, getFrames, getTooltipInBase, maskBall,
                          fitCircleInArray, ballCenterInCam, camInTooltip,
                          tooltipOffset)

toolFromTooltip = np.linalg.inv(tooltipOffset)

def toFrame(T, p):
    return (T @ np.append(p, 1.0))[:3]

def fmt(p):
    return np.array2string(p, precision=3, suppress_small=True,
                           floatmode="fixed")

def main():
    print("all positions in meters; Ctrl+C to stop")
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

            fit = fitCircleInArray(maskBall(frame))
            if fit is None:
                print("ball: not found")
                continue

            pCam = ballCenterInCam(fit[0], fit[1], depthM, intr)
            pTooltip = toFrame(camInTooltip, pCam)
            pTool = toFrame(toolFromTooltip, pTooltip)
            pBase = toFrame(tooltipInBase, pTooltip)

            print(f"cam{fmt(pCam)}  tooltip{fmt(pTooltip)}  tool{fmt(pTool)}  "
                  f"base{fmt(pBase)}  range {np.linalg.norm(pCam):.3f} m")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
