#!/usr/bin/env python3
"""Teach the robot which ball positions are "valid" pickup targets.

You move the ball around while the robot observes (the arm itself must stay
parked — this script NEVER commands motion). Record timed bursts of VALID
positions (sweep the ball through the whole region the arm may pick from)
and INVALID positions (sweep it through everywhere it must NOT trigger a
pickup: too close to the base, off the carpet, near obstacles...).

The valid region is fitted as an axis-aligned box in the robot base frame:
percentile bounds of the valid samples plus a margin. Invalid samples are not
carved out of the box — they are checked against it, and any conflicts
(invalid samples inside the box) are reported so you can re-teach with a
tighter sweep. Keep regions box-shaped or teach the smaller box that excludes
the trouble spot.

Output: ballRegion.json (next to this script), loaded by combinedtest.py for
isValidBallPosition().

Usage:
    .venv/bin/python collect_ball_region.py
Commands at the prompt:
    v  record an 8 s burst of VALID ball positions (repeatable)
    i  record an 8 s burst of INVALID ball positions (repeatable)
    s  show sample counts and the current box fit
    c  compute, save ballRegion.json, and exit
    q  abort without saving
"""

import json
import time
from datetime import datetime
from pathlib import Path

import numpy as np

burstS = 8.0
marginM = 0.03           # box expansion beyond the valid-sample percentiles
percentile = (2.0, 98.0)  # trims detection-glitch outliers

regionPath = Path(__file__).resolve().parent / "ballRegion.json"


def computeRegion(validPts, invalidPts, margin=marginM):
    """Fit the valid box and score it. Returns (boxMin, boxMax, report str)."""
    validPts = np.asarray(validPts)
    lo = np.percentile(validPts, percentile[0], axis=0) - margin
    hi = np.percentile(validPts, percentile[1], axis=0) + margin

    lines = [f"valid box (base frame, m): min {np.round(lo, 3)}  max {np.round(hi, 3)}"]
    inside = np.all((validPts >= lo) & (validPts <= hi), axis=1)
    lines.append(f"valid samples inside box: {inside.sum()}/{len(validPts)} "
                 f"(rest are trimmed outliers)")
    if len(invalidPts):
        invalidPts = np.asarray(invalidPts)
        conflicts = np.all((invalidPts >= lo) & (invalidPts <= hi), axis=1)
        frac = conflicts.mean()
        lines.append(f"INVALID samples inside box: {conflicts.sum()}/{len(invalidPts)} "
                     f"({frac:.0%})")
        if frac > 0.05:
            lines.append("WARNING: the box overlaps taught-invalid territory. A box"
                         " cannot exclude pockets inside the valid sweep — re-teach"
                         " with a valid sweep that stays clear of the invalid zone.")
    else:
        lines.append("no invalid samples taught — box is unchecked against negatives")
    return lo, hi, "\n".join(lines)


def recordBurst(arm, cam, label):
    """Sample ball positions for burstS seconds; returns list of base-frame xyz."""
    from combinedtest import (getFrames, getTooltipInBase, maskBall,
                              fitCircleInArray, ballCenterInCam, camInTooltip)

    print(f"recording {label} for {burstS:.0f} s — keep the ball moving "
          f"through {label} positions...")
    samples = []
    t0 = time.monotonic()
    while time.monotonic() - t0 < burstS:
        frame, depthM, intr = getFrames(cam)
        tooltipInBase = getTooltipInBase(arm)
        fit = fitCircleInArray(maskBall(frame))
        if fit is None:
            continue
        pCam = ballCenterInCam(fit[0], fit[1], depthM, intr)
        pBase = (tooltipInBase @ camInTooltip @ np.append(pCam, 1.0))[:3]
        samples.append(pBase.tolist())
    print(f"  captured {len(samples)} samples")
    if len(samples) < 20:
        print("  (few samples — is the ball visible? check HSV tuning)")
    return samples


def main():
    from calibrate_handeye import ArmReader
    from combinedtest import CamClient, getTooltipInBase

    valid, invalid = [], []
    with ArmReader() as arm, CamClient() as cam:
        ttStart = getTooltipInBase(arm)
        print("park the arm where it will WAIT during the pickup routine and "
              "do not move it while teaching.")
        while True:
            cmd = input(f"[{len(valid)} valid / {len(invalid)} invalid] "
                        "v=valid burst  i=invalid burst  s=stats  c=save  q=abort > "
                        ).strip().lower()
            if cmd == "v":
                valid += recordBurst(arm, cam, "VALID")
            elif cmd == "i":
                invalid += recordBurst(arm, cam, "INVALID")
            elif cmd == "s" and valid:
                print(computeRegion(valid, invalid)[2])
            elif cmd == "c":
                if len(valid) < 50:
                    print(f"only {len(valid)} valid samples; record more bursts first")
                    continue
                drift = np.linalg.norm(getTooltipInBase(arm)[:3, 3] - ttStart[:3, 3])
                if drift > 0.005:
                    print(f"WARNING: arm moved {drift * 1e3:.0f} mm during teaching; "
                          "samples may be inconsistent")
                lo, hi, report = computeRegion(valid, invalid)
                print(report)
                regionPath.write_text(json.dumps({
                    "created": datetime.now().isoformat(timespec="seconds"),
                    "min": lo.tolist(), "max": hi.tolist(),
                    "marginM": marginM, "nValid": len(valid),
                    "nInvalid": len(invalid),
                }, indent=2))
                print(f"saved {regionPath}")
                return
            elif cmd == "q":
                print("aborted, nothing saved")
                return


if __name__ == "__main__":
    main()
