#!/usr/bin/env python3
"""Hand-eye calibration for the RealSense D415 on the RO1 arm (eye-in-hand).

Solves for camInTooltip: the pose of the RealSense color camera in the frame
that get_arm_position() reports tooltip_position in. The result is written to
camInTooltip.npy, which combinedtest.py loads at import.

SAFETY: this script NEVER commands robot motion and never brakes/unbrakes the
arm. You move the arm yourself between captures (freedrive / jog from the
Standard Bots interface at http://192.168.1.3:3000); the script only reads the
tooltip pose. A capture is rejected if the arm moved while it was being taken.

Setup:
  * Print a checkerboard, tape it perfectly flat to something rigid, and keep
    it COMPLETELY STILL for the whole session (the math assumes a static board).
  * Measure a square edge with calipers and pass it via --square (in meters!);
    an error here scales directly into the calibrated camera offset.
  * Defaults assume 9x6 INNER corners (a 10x7-squares board). One dimension
    should be even and the other odd so the corner ordering cannot flip.

Procedure: move the arm so the camera sees the whole board, let it settle,
capture; repeat from 10-15 viewpoints. Vary ORIENTATION as much as reach
allows (tilt/roll the camera around the board, near and far, board in
different parts of the image) — pure translations do not constrain hand-eye.

Usage:
    .venv/bin/python calibrate_handeye.py --square 0.025
    .venv/bin/python calibrate_handeye.py --square 0.025 --no-gui
    .venv/bin/python calibrate_handeye.py --recompute handeye_captures/<dir>/poses.json
    .venv/bin/python calibrate_handeye.py --selftest    # no hardware needed

Keys (GUI window): SPACE capture | u undo | c calibrate+save | q/Esc abort
Console mode:      Enter capture | u undo | c calibrate+save | q abort

Outputs:
  * camInTooltip.npy (next to this script; loaded by combinedtest.py)
  * handeye_captures/<timestamp>/poses.json    raw pose pairs (for --recompute)
  * handeye_captures/<timestamp>/capture_*.jpeg  what each capture saw
  * handeye_captures/<timestamp>/result.txt    matrix + quality report
"""

import argparse
import json
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

scriptDir = Path(__file__).resolve().parent
resultNpyPath = scriptDir / "camInTooltip.npy"

# Capture is rejected when the two tooltip reads around the frame grab differ
# by more than this (the arm was still settling, or someone was pushing it).
stillToleranceM = 0.002
stillToleranceDeg = 0.3

handEyeMethods = {
    "park": cv2.CALIB_HAND_EYE_PARK,
    "tsai": cv2.CALIB_HAND_EYE_TSAI,
    "horaud": cv2.CALIB_HAND_EYE_HORAUD,
    "andreff": cv2.CALIB_HAND_EYE_ANDREFF,
    "daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS,
}
primaryMethod = "park"


# ---------------------------------------------------------------- pose math

def rtToMat(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t).ravel()
    return T

def rotAngleDeg(R):
    """Rotation angle of a rotation matrix, degrees."""
    return float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))))

def poseDelta(T1, T2):
    """(translation distance m, rotation angle deg) between two poses."""
    return (float(np.linalg.norm(T1[:3, 3] - T2[:3, 3])),
            rotAngleDeg(T1[:3, :3].T @ T2[:3, :3]))


# ------------------------------------------------------------ board detection

def boardObjectPoints(cols, rows, square):
    """Inner-corner grid on the board plane (z=0), ordered like the detector."""
    pts = np.zeros((rows * cols, 3), np.float32)
    pts[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square
    return pts

def detectBoard(gray, cols, rows, fast=False):
    """Inner corners as (N,1,2) float32, or None. fast= is a preview-quality
    detector for the live view; captures use the accurate SB detector."""
    if fast:
        found, corners = cv2.findChessboardCorners(
            gray, (cols, rows), flags=cv2.CALIB_CB_FAST_CHECK)
        return corners if found else None
    found, corners = cv2.findChessboardCornersSB(
        gray, (cols, rows),
        flags=cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_ACCURACY)
    return corners if found else None

def intrinsicsToK(intr):
    """RealSense color intrinsics -> (camera matrix, distortion coeffs)."""
    K = np.array([[intr.fx, 0, intr.ppx],
                  [0, intr.fy, intr.ppy],
                  [0, 0, 1]], dtype=np.float64)
    dist = np.array(intr.coeffs[:5], dtype=np.float64)
    if "inverse" in str(intr.model) and np.any(dist != 0):
        print(f"warning: distortion model {intr.model} with nonzero coeffs; "
              "solvePnP expects forward Brown-Conrady")
    return K, dist

def boardPoseInCam(corners, objPts, K, dist):
    """Board pose in the camera frame (4x4) and mean reprojection error (px)."""
    ok, rvec, tvec = cv2.solvePnP(objPts, corners, K, dist,
                                  flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None, None
    projected, _ = cv2.projectPoints(objPts, rvec, tvec, K, dist)
    err = float(np.linalg.norm((projected - corners).reshape(-1, 2), axis=1).mean())
    R, _ = cv2.Rodrigues(rvec)
    return rtToMat(R, tvec), err


# ------------------------------------------------------------------- solving

def solveHandEye(tooltipPoses, boardPoses):
    """Run every OpenCV hand-eye method; returns {name: camInTooltip or None}."""
    Rg = [T[:3, :3] for T in tooltipPoses]
    tg = [T[:3, 3] for T in tooltipPoses]
    Rt = [T[:3, :3] for T in boardPoses]
    tt = [T[:3, 3] for T in boardPoses]
    solutions = {}
    for name, method in handEyeMethods.items():
        try:
            R, t = cv2.calibrateHandEye(Rg, tg, Rt, tt, method=method)
            solutions[name] = rtToMat(R, t)
        except cv2.error as e:
            print(f"method {name} failed: {str(e).splitlines()[-1]}")
            solutions[name] = None
    return solutions

def buildReport(tooltipPoses, boardPoses, solutions):
    """Quality report: cross-method agreement, rotation diversity, and the
    physical consistency check (the static board's pose in the base frame must
    come out identical from every capture)."""
    X = solutions[primaryMethod]
    lines = []

    angles = [rotAngleDeg(tooltipPoses[i][:3, :3].T @ tooltipPoses[j][:3, :3])
              for i in range(len(tooltipPoses)) for j in range(i + 1, len(tooltipPoses))]
    lines.append(f"poses: {len(tooltipPoses)}, max relative tooltip rotation "
                 f"{max(angles):.1f} deg")
    if max(angles) < 20:
        lines.append("WARNING: little rotation between poses; hand-eye is "
                     "poorly constrained. Tilt the camera more between captures.")

    lines.append(f"\nmethod agreement (vs {primaryMethod}):")
    for name, sol in solutions.items():
        if sol is None:
            lines.append(f"  {name:<10} failed")
        else:
            dt, dr = poseDelta(X, sol)
            lines.append(f"  {name:<10} dt {dt * 1000:6.2f} mm   dR {dr:5.2f} deg")

    boardInBase = [Tb @ X @ Tc for Tb, Tc in zip(tooltipPoses, boardPoses)]
    ts = np.array([T[:3, 3] for T in boardInBase])
    devs = np.linalg.norm(ts - ts.mean(axis=0), axis=1) * 1000
    rots = [rotAngleDeg(boardInBase[0][:3, :3].T @ T[:3, :3]) for T in boardInBase]
    lines.append("\nconsistency (board pose in base frame should be identical "
                 "in every capture):")
    lines.append(f"  translation deviation: mean {devs.mean():.2f} mm, "
                 f"max {devs.max():.2f} mm")
    lines.append(f"  rotation spread vs first capture: max {max(rots):.2f} deg")
    if devs.max() > 10:
        lines.append("  WARNING: poor consistency — did the board move, is "
                     "--square right, were captures blurry?")
    return "\n".join(lines)

def pasteReady(X):
    rows = [", ".join(f"{v: .8f}" for v in row) for row in X]
    return ("camInTooltip = np.array([\n    ["
            + "],\n    [".join(rows) + "],\n])")

def calibrateAndSave(tooltipPoses, boardPoses, sessionDir):
    if len(tooltipPoses) < 3:
        print(f"only {len(tooltipPoses)} capture(s); need at least 3 "
              "(10-15 recommended)")
        return None
    if len(tooltipPoses) < 8:
        print(f"note: {len(tooltipPoses)} captures is thin; 10-15 give a "
              "much more trustworthy result")

    solutions = solveHandEye(tooltipPoses, boardPoses)
    X = solutions[primaryMethod]
    if X is None:
        print(f"primary method {primaryMethod} failed")
        return None

    report = buildReport(tooltipPoses, boardPoses, solutions)
    text = (f"hand-eye calibration {datetime.now().isoformat(timespec='seconds')}\n"
            f"camInTooltip ({primaryMethod}):\n{np.array_str(X, precision=6)}\n\n"
            f"{report}\n\npaste into combinedtest.py (or rely on "
            f"camInTooltip.npy, loaded automatically):\n\n{pasteReady(X)}\n")
    print("\n" + text)

    np.save(resultNpyPath, X)
    print(f"saved {resultNpyPath}")
    if sessionDir is not None:
        (sessionDir / "result.txt").write_text(text)
        print(f"saved {sessionDir / 'result.txt'}")
    return X


# ------------------------------------------------------------- capture session

class ArmReader:
    """Read-only arm connection. Unlike combinedtest.ArmClient this never
    brakes the arm on exit — the whole point is that the user is moving it."""

    def __enter__(self):
        from standardbots import StandardBotsRobot
        self.sdk = StandardBotsRobot(
            url='http://192.168.1.3:3000',
            token=Path("/home/max/Documents/.robot_token").read_text().strip(),
            robot_kind=StandardBotsRobot.RobotKind.Live,
        )
        return self.sdk

    def __exit__(self, a, b, c):
        self.sdk._request_manager.close()

def writeJson(sessionDir, board, intr, captures):
    data = {
        "created": datetime.now().isoformat(timespec="seconds"),
        "board": board,
        "intrinsics": None if intr is None else {
            "width": intr.width, "height": intr.height,
            "fx": intr.fx, "fy": intr.fy, "ppx": intr.ppx, "ppy": intr.ppy,
            "model": str(intr.model), "coeffs": list(intr.coeffs),
        },
        "captures": captures,
    }
    (sessionDir / "poses.json").write_text(json.dumps(data, indent=2))

def tryCapture(arm, cam, objPts, args, sessionDir, captures, attempt):
    """One capture: tooltip pose + board pose, with a stillness check.
    Appends to captures and rewrites poses.json. Returns the frame intrinsics
    (so the session can record them) or None when rejected."""
    from combinedtest import getFrames, getTooltipInBase

    tooltip1 = getTooltipInBase(arm)
    for _ in range(5):  # flush queued frames so the image is from *now*
        frame, _, intr = getFrames(cam)
    tooltip2 = getTooltipInBase(arm)

    dt, dr = poseDelta(tooltip1, tooltip2)
    if dt > stillToleranceM or dr > stillToleranceDeg:
        print(f"rejected: arm moved during capture ({dt * 1000:.1f} mm, "
              f"{dr:.2f} deg) — let it settle and retry")
        return None

    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    corners = detectBoard(gray, args.cols, args.rows)
    bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    if corners is None:
        cv2.imwrite(str(sessionDir / "last_failed.jpeg"), bgr)
        print(f"rejected: no {args.cols}x{args.rows} board found "
              f"(see {sessionDir / 'last_failed.jpeg'})")
        return None

    K, dist = intrinsicsToK(intr)
    boardPose, reprojErr = boardPoseInCam(corners, objPts, K, dist)
    if boardPose is None:
        print("rejected: solvePnP failed")
        return None

    image = f"capture_{attempt:03d}.jpeg"
    cv2.drawChessboardCorners(bgr, (args.cols, args.rows), corners, True)
    cv2.imwrite(str(sessionDir / image), bgr)

    captures.append({
        "image": image,
        "tooltipInBase": tooltip2.tolist(),
        "boardInCam": boardPose.tolist(),
        "reprojErrPx": reprojErr,
    })
    board = {"cols": args.cols, "rows": args.rows, "squareM": args.square}
    writeJson(sessionDir, board, intr, captures)
    print(f"capture {len(captures)}: board at "
          f"{np.linalg.norm(boardPose[:3, 3]):.3f} m, reproj err "
          f"{reprojErr:.2f} px -> {image}")
    return intr

def captureSession(args):
    from combinedtest import CamClient, getFrames

    sessionDir = scriptDir / "handeye_captures" / datetime.now().strftime("%Y%m%d_%H%M%S")
    sessionDir.mkdir(parents=True)
    objPts = boardObjectPoints(args.cols, args.rows, args.square)
    captures = []
    attempt = 0
    print(f"session dir: {sessionDir}")
    print("move the arm by hand/jog between captures; this script never "
          "commands motion.")

    with ArmReader() as arm, CamClient() as cam:
        if args.no_gui:
            while True:
                cmd = input(f"[{len(captures)} captures] Enter=capture  "
                            "u=undo  c=calibrate  q=abort > ").strip().lower()
                if cmd == "":
                    attempt += 1
                    tryCapture(arm, cam, objPts, args, sessionDir, captures, attempt)
                elif cmd == "u" and captures:
                    print(f"dropped {captures.pop()['image']}")
                elif cmd == "c":
                    break
                elif cmd == "q":
                    print(f"aborted; {len(captures)} captures kept in "
                          f"{sessionDir} (usable via --recompute)")
                    return
        else:
            window = "hand-eye calibration"
            cv2.namedWindow(window)
            while True:
                frame, _, _ = getFrames(cam)
                gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
                bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                preview = detectBoard(gray, args.cols, args.rows, fast=True)
                if preview is not None:
                    cv2.drawChessboardCorners(bgr, (args.cols, args.rows),
                                              preview, True)
                status = "board OK" if preview is not None else "no board"
                cv2.putText(bgr, f"{len(captures)} captures | {status} | "
                            "SPACE=capture u=undo c=calibrate q=abort",
                            (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0, 255, 0), 1, cv2.LINE_AA)
                cv2.imshow(window, bgr)
                key = cv2.waitKey(1) & 0xFF
                if key == ord(' '):
                    attempt += 1
                    tryCapture(arm, cam, objPts, args, sessionDir, captures, attempt)
                elif key == ord('u') and captures:
                    print(f"dropped {captures.pop()['image']}")
                elif key == ord('c'):
                    break
                elif key in (ord('q'), 27):
                    print(f"aborted; {len(captures)} captures kept in "
                          f"{sessionDir} (usable via --recompute)")
                    cv2.destroyAllWindows()
                    return
            cv2.destroyAllWindows()

    tooltipPoses = [np.array(c["tooltipInBase"]) for c in captures]
    boardPoses = [np.array(c["boardInCam"]) for c in captures]
    calibrateAndSave(tooltipPoses, boardPoses, sessionDir)

def recompute(jsonPath):
    data = json.loads(Path(jsonPath).read_text())
    captures = data["captures"]
    print(f"recomputing from {len(captures)} captures in {jsonPath}")
    tooltipPoses = [np.array(c["tooltipInBase"]) for c in captures]
    boardPoses = [np.array(c["boardInCam"]) for c in captures]
    calibrateAndSave(tooltipPoses, boardPoses, Path(jsonPath).parent)


# -------------------------------------------------------------------- selftest

def selftest():
    """End-to-end check with synthetic data; no hardware, no files changed
    outside a temp dir."""
    import tempfile

    rng = np.random.default_rng(0)
    def randomRotation(scale):
        axis = rng.normal(size=3)
        R, _ = cv2.Rodrigues(axis / np.linalg.norm(axis) * rng.uniform(0.2, scale))
        return R

    xTrue = rtToMat(randomRotation(0.5), [0.03, -0.05, 0.08])
    boardInBase = rtToMat(randomRotation(0.5), [0.45, 0.10, 0.02])
    tooltipPoses, boardPoses = [], []
    for _ in range(12):
        Tb = rtToMat(randomRotation(1.2), rng.uniform(-0.3, 0.3, 3) + [0.3, 0, 0.4])
        tooltipPoses.append(Tb)
        boardPoses.append(np.linalg.inv(xTrue) @ np.linalg.inv(Tb) @ boardInBase)

    global resultNpyPath
    savedNpyPath = resultNpyPath
    with tempfile.TemporaryDirectory() as tmp:
        resultNpyPath = Path(tmp) / "camInTooltip.npy"
        try:
            sessionDir = Path(tmp)
            writeJson(sessionDir, {"cols": 9, "rows": 6, "squareM": 0.025},
                      None, [{"image": "", "tooltipInBase": t.tolist(),
                              "boardInCam": b.tolist(), "reprojErrPx": 0.0}
                             for t, b in zip(tooltipPoses, boardPoses)])
            recompute(sessionDir / "poses.json")  # exercises the full solve path
            X = np.load(resultNpyPath)
        finally:
            resultNpyPath = savedNpyPath
    dt, dr = poseDelta(X, xTrue)
    assert dt < 1e-6 and dr < 1e-4, f"hand-eye recovery off: {dt * 1000} mm, {dr} deg"

    # Board detection + PnP wiring on a rendered board (9x6 inner corners).
    cols, rows, sq = 9, 6, 40
    pattern = (np.indices((rows + 1, cols + 1)).sum(axis=0) % 2)
    img = np.pad(np.kron(pattern, np.ones((sq, sq))) * 255, sq,
                 constant_values=255).astype(np.uint8)
    corners = detectBoard(img, cols, rows)
    assert corners is not None and len(corners) == cols * rows, "board detection failed"
    K = np.array([[800.0, 0, img.shape[1] / 2], [0, 800.0, img.shape[0] / 2], [0, 0, 1]])
    T, err = boardPoseInCam(corners, boardObjectPoints(cols, rows, 0.025), K, np.zeros(5))
    assert T is not None and err < 0.5, f"PnP reprojection error {err} px"

    print("\nselftest passed (known hand-eye recovered to "
          f"{dt * 1e6:.2f} um / {dr:.2e} deg; board PnP reproj {err:.3f} px)")


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--square", type=float,
                        help="checkerboard square edge in METERS (measure it!)")
    parser.add_argument("--cols", type=int, default=9,
                        help="inner corners per row (default 9)")
    parser.add_argument("--rows", type=int, default=6,
                        help="inner corners per column (default 6)")
    parser.add_argument("--no-gui", action="store_true",
                        help="console capture loop instead of a preview window")
    parser.add_argument("--recompute", metavar="POSES_JSON",
                        help="re-solve from a saved session; no hardware needed")
    parser.add_argument("--selftest", action="store_true",
                        help="synthetic end-to-end check; no hardware needed")
    args = parser.parse_args()

    if args.selftest:
        selftest()
        return
    if args.recompute:
        recompute(args.recompute)
        return
    if args.square is None:
        parser.error("--square is required for a capture session "
                     "(square edge in meters, e.g. --square 0.025)")
    if args.cols == args.rows or (args.cols + args.rows) % 2 == 0:
        print("warning: use a board with one even and one odd corner count "
              f"(got {args.cols}x{args.rows}); symmetric boards can flip the "
              "corner ordering between views and ruin the calibration")
    captureSession(args)


if __name__ == "__main__":
    main()
