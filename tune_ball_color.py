#!/usr/bin/env python3
"""Interactive GUI tuner for the 6 ``ballColor`` values in combinedtest.py.

Opens an OpenCV window showing the live camera stream and the filtered (masked)
stream side by side, with the 6 parameters exposed as trackbars: centre R/G/B
and tolerance R/G/B. Drag the sliders and watch the mask update live.

The mask is computed the same way combinedtest.main() does so the values you
pick transfer straight back into ``ballColor``::

    isBall = abs(channel - centre) < tol   for all 3 channels

The subtraction is done in int16 so it does not wrap around for dark pixels.

Usage:
    .venv/bin/python tune_ball_color.py            # use the robot camera
    .venv/bin/python tune_ball_color.py --demo     # no hardware; synthetic image
    .venv/bin/python tune_ball_color.py --selftest # render one frame to PNG, exit

Keys (focus the window):
    s            save current values (prints + writes ball_color_tuned.txt)
    r            reset sliders to the starting values
    p            pause / resume camera polling
    q / Esc      quit
"""

import argparse
import threading
import time

import cv2
import numpy as np

# Starting values mirror ballColor in combinedtest.py
# (tolerance = 2; tol = [33, 38, 20] * tolerance).
START_CENTER = [153, 152, 97]
START_TOL = [66, 76, 40]

WINDOW = "ball-color tuner"
TRACKBARS = [
    ("center R", 255),
    ("center G", 255),
    ("center B", 255),
    ("tol R", 255),
    ("tol G", 255),
    ("tol B", 255),
]


# --------------------------------------------------------------------------- #
# Mask + image helpers
# --------------------------------------------------------------------------- #
def compute_mask(frame, center, tol):
    """Boolean mask of ball pixels for an RGB uint8 frame.

    Matches combinedtest.main(): a pixel is kept when |channel - centre| < tol
    for all three channels. The subtraction is done in int16 so it does not
    wrap around for dark pixels.
    """
    diff = np.abs(frame.astype(np.int16) - np.array(center, dtype=np.int16))
    return (diff < np.array(tol)).all(axis=2)


def make_demo_frame(t):
    """Synthetic RGB frame: a ball near the target colour on a textured
    background, drifting in a circle so motion/filtering is visible."""
    h, w = 360, 480
    yy, xx = np.mgrid[0:h, 0:w]
    bg = np.stack(
        [
            (80 + 40 * np.sin(xx / 45.0)),
            (70 + 40 * np.cos(yy / 38.0)),
            (60 + 30 * np.sin((xx + yy) / 60.0)),
        ],
        axis=2,
    )
    cx = int(w / 2 + w * 0.28 * np.cos(t))
    cy = int(h / 2 + h * 0.28 * np.sin(t * 1.3))
    r = 55
    ball = (xx - cx) ** 2 + (yy - cy) ** 2 < r * r
    img = bg.astype(np.uint8)
    img[ball] = np.array(START_CENTER, dtype=np.uint8)
    noise = (np.random.randn(h, w, 1) * 6).astype(np.int16)
    return np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)


def resize_to_width(img, width):
    h, w = img.shape[:2]
    if w == width:
        return img
    return cv2.resize(img, (width, max(1, int(round(h * width / w)))),
                      interpolation=cv2.INTER_AREA)


def label(img_bgr, text):
    """Draw a label with a dark backing strip onto a BGR image (in place)."""
    cv2.rectangle(img_bgr, (0, 0), (img_bgr.shape[1], 22), (0, 0, 0), -1)
    cv2.putText(img_bgr, text, (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (255, 255, 255), 1, cv2.LINE_AA)
    return img_bgr


def build_display(frame_rgb, center, tol, fps, paused, disp_w):
    """Return (composite BGR image, coverage%) for one frame."""
    mask = compute_mask(frame_rgb, center, tol)
    coverage = float(mask.mean()) * 100.0
    filtered_rgb = np.where(mask[..., None], frame_rgb, np.uint8(0)).astype(np.uint8)

    live = label(cv2.cvtColor(resize_to_width(frame_rgb, disp_w), cv2.COLOR_RGB2BGR),
                 "LIVE")
    filt = label(cv2.cvtColor(resize_to_width(filtered_rgb, disp_w), cv2.COLOR_RGB2BGR),
                 "FILTERED  coverage %.1f%%" % coverage)
    combo = np.hstack([live, filt])

    bar = "ballColor = ([%d,%d,%d], [%d,%d,%d])  %.0f fps%s" % (
        center[0], center[1], center[2], tol[0], tol[1], tol[2],
        fps, "  [PAUSED]" if paused else "")
    foot = np.zeros((26, combo.shape[1], 3), dtype=np.uint8)
    cv2.putText(foot, bar, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                (180, 255, 180), 1, cv2.LINE_AA)
    # swatch of the current centre colour (centre is RGB -> BGR for display)
    cv2.rectangle(foot, (combo.shape[1] - 30, 4), (combo.shape[1] - 6, 22),
                  (int(center[2]), int(center[1]), int(center[0])), -1)
    return np.vstack([combo, foot]), coverage


def format_ball_color(center, tol):
    return "ballColor = ([%d,%d,%d], [%d,%d,%d])" % (
        center[0], center[1], center[2], tol[0], tol[1], tol[2])


def save_values(center, tol):
    line = format_ball_color(center, tol)
    try:
        with open("ball_color_tuned.txt", "w") as f:
            f.write(line + "\n")
        print("saved -> ball_color_tuned.txt :", line)
    except OSError as e:
        print("save failed:", e)
    return line


# --------------------------------------------------------------------------- #
# Camera source (background thread so the UI stays responsive)
# --------------------------------------------------------------------------- #
class CameraThread(threading.Thread):
    def __init__(self, demo):
        super().__init__(daemon=True)
        self.demo = demo
        self.lock = threading.Lock()
        self._frame = None
        self.error = None
        self.running = True
        self.paused = False

    def _set(self, frame):
        with self.lock:
            self._frame = frame

    def get(self):
        with self.lock:
            return self._frame

    def stop(self):
        self.running = False

    def run(self):
        if self.demo:
            t0 = time.time()
            while self.running:
                if not self.paused:
                    self._set(make_demo_frame(time.time() - t0))
                time.sleep(0.03)
            return
        try:
            from combinedtest import ArmClient, getCameraFrame
        except Exception as e:  # pragma: no cover - hardware deps
            self.error = "import combinedtest failed: %s" % e
            return
        try:
            with ArmClient() as arm:
                while self.running:
                    if self.paused:
                        time.sleep(0.05)
                        continue
                    try:
                        self._set(getCameraFrame(arm))
                        self.error = None
                    except Exception as e:
                        self.error = "camera read failed: %s" % e
                        time.sleep(0.2)
        except Exception as e:  # pragma: no cover - hardware deps
            self.error = "robot connection failed: %s" % e


# --------------------------------------------------------------------------- #
# Trackbars
# --------------------------------------------------------------------------- #
def create_trackbars():
    starts = START_CENTER + START_TOL
    for (name, vmax), val in zip(TRACKBARS, starts):
        cv2.createTrackbar(name, WINDOW, val, vmax, lambda v: None)


def read_trackbars():
    pos = [cv2.getTrackbarPos(name, WINDOW) for name, _ in TRACKBARS]
    return pos[0:3], pos[3:6]


def set_trackbars(center, tol):
    for (name, _), val in zip(TRACKBARS, list(center) + list(tol)):
        cv2.setTrackbarPos(name, WINDOW, val)


def waiting_screen(width, message):
    img = np.zeros((120, max(width, 480), 3), dtype=np.uint8)
    cv2.putText(img, message, (10, 64), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                (200, 200, 200), 1, cv2.LINE_AA)
    return img


# --------------------------------------------------------------------------- #
# Main
# --------------------------------------------------------------------------- #
def run_selftest(demo):
    frame = make_demo_frame(0.0)
    combo, coverage = build_display(frame, list(START_CENTER), list(START_TOL),
                                    15.0, False, 480)
    out = "tuner_selftest.png"
    cv2.imwrite(out, combo)
    print("wrote %s  shape=%s  coverage=%.2f%%" % (out, combo.shape, coverage))
    print(format_ball_color(list(START_CENTER), list(START_TOL)))
    print("selftest OK")


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--demo", action="store_true",
                    help="use a synthetic moving image instead of the robot camera")
    ap.add_argument("--selftest", action="store_true",
                    help="render one frame to tuner_selftest.png and exit (no window)")
    ap.add_argument("--width", type=int, default=480,
                    help="display width of each panel in pixels (default 480)")
    args = ap.parse_args()

    if args.selftest:
        run_selftest(args.demo)
        return

    cam = CameraThread(args.demo)
    cam.start()

    try:
        cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
    except cv2.error as e:
        raise SystemExit(
            "Could not open a GUI window (%s).\nThis build of OpenCV may lack "
            "HighGUI/display support, or there is no display available.\n"
            "Run with --demo on a machine with a display, or use --selftest "
            "to render to a PNG." % e)
    create_trackbars()

    last = time.time()
    fps = 0.0
    center, tol = list(START_CENTER), list(START_TOL)
    try:
        while True:
            center, tol = read_trackbars()

            frame = cam.get()
            now = time.time()
            dt = now - last
            last = now
            if dt > 0:
                fps = 0.85 * fps + 0.15 * (1.0 / dt)

            if frame is None:
                cv2.imshow(WINDOW, waiting_screen(
                    2 * args.width, cam.error or "waiting for camera..."))
            else:
                combo, _ = build_display(frame, center, tol, fps,
                                         cam.paused, args.width)
                cv2.imshow(WINDOW, combo)

            key = cv2.waitKey(15) & 0xFF
            if key in (ord("q"), 27):  # q or Esc
                break
            elif key == ord("s"):
                save_values(center, tol)
            elif key == ord("p"):
                cam.paused = not cam.paused
            elif key == ord("r"):
                set_trackbars(START_CENTER, START_TOL)

            # window closed via the [x] button
            if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                break
    except KeyboardInterrupt:
        pass
    finally:
        cam.stop()
        cam.join(timeout=1.0)
        cv2.destroyAllWindows()
        print(format_ball_color(center, tol))


if __name__ == "__main__":
    main()
