#!/usr/bin/env python3
"""Does set_arm_position preempt, queue, or reject when sent mid-motion?

COMMANDS MOTION: the arm yaws J0 by +12 deg and back at low speed. Clear the
space the arm sweeps through and keep a hand near the e-stop.

The pickup routine re-sends the joint target whenever the Kalman estimate
shifts it; what that does mid-flight depends on undocumented firmware
behavior for a set_arm_position received while a previous one executes:

  preempt — the new target replaces the old one mid-motion. Best case: the
            routine retargets with no extra latency.
  queue   — accepted, but the old move finishes first. The routine still
            converges (armReached compares against the live target, so it
            never grasps at a stale position); worst-case retarget latency
            is the remaining duration of the current leg.
  reject  — errors while moving. Same latency as queue: the routine prints
            'moveArm rejected' once and retries every tick until the leg
            ends. If this is the verdict and the latency hurts, the fix is
            to command short legs (cap per-send joint travel) so the arm
            goes idle — and retargetable — every fraction of a second.

Method: command J0 +12 deg at speed scale 0.2; 0.4 s into the move command
the start pose back; classify from the acceptance and the peak J0 excursion.

Usage:
    .venv/bin/python check_preemption.py    (Enter to confirm, Ctrl+C aborts)
"""

import numpy as np
from time import sleep, monotonic

from combinedtest import (ArmClient, armReached, getArmJoints, initArm,
                          moveArm, waitUntilReached)

deltaRad = np.deg2rad(30)
speedScale = 0.05
resendAfterS = 0.1
watchTimeoutS = 15.0


def returnToStart(arm, start):
    while not moveArm(arm, start, speedScale=speedScale):
        sleep(0.2)  # rejected while still moving: retry until idle
    waitUntilReached(arm, start)


def main():
    input("the arm will yaw J0 +12 deg and back at low speed — clear the "
          "area and press Enter to start (Ctrl+C aborts) > ")
    with ArmClient() as arm:
        initArm(arm)
        start = getArmJoints(arm)
        away = start.copy()
        away[0] += deltaRad

        if not moveArm(arm, away, speedScale=speedScale):
            print("could not start the test move")
            return
        sleep(resendAfterS)
        midJ0 = getArmJoints(arm)[0] - start[0]
        if midJ0 > 0.9 * deltaRad:
            print(f"inconclusive: J0 was already {np.degrees(midJ0):.1f} deg "
                  f"of {np.degrees(deltaRad):.0f} at re-send time — raise "
                  "deltaRad or lower speedScale and rerun")
            returnToStart(arm, start)
            return

        accepted = moveArm(arm, start, speedScale=speedScale)
        peak = midJ0
        t0 = monotonic()
        while monotonic() - t0 < watchTimeoutS:
            joints = getArmJoints(arm)
            peak = max(peak, joints[0] - start[0])
            if armReached(joints, start if accepted else away):
                break
            sleep(0.02)

        if not accepted:
            print("VERDICT: reject — set_arm_position errors while a move "
                  "executes; the routine finishes each leg before a retarget "
                  "takes effect (consider capping leg length)")
            returnToStart(arm, start)
        elif peak < deltaRad - np.deg2rad(3):
            print(f"VERDICT: preempt — turned around at {np.degrees(peak):.1f} "
                  f"of {np.degrees(deltaRad):.0f} deg; mid-flight retargeting "
                  "works, the routine is optimal as written")
        else:
            print(f"VERDICT: queue — reached {np.degrees(peak):.1f} deg before "
                  "returning; the routine finishes each leg before a retarget "
                  "takes effect (consider capping leg length)")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
