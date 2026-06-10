#!/usr/bin/env python3
"""Interactive GUI tuner for the 6 ``ballColor`` values in combinedtest.py.

Opens an OpenCV window showing the live camera stream and the filtered (masked)
stream side by side, with the 6 parameters exposed as trackbars: min H/S/V and
max H/S/V. Drag the sliders and watch the mask update live.

The camera frame is converted RGB -> HSV (OpenCV scale: H 0-179, S/V 0-255)
before thresholding. The mask is computed the same way combinedtest.main() does
so the values you pick transfer straight back into ``ballColor`` (stored as HSV
``([min...], [max...])``)::

    isBall = (min <= channel <= max)   for all 3 HSV channels

Usage:
    .venv/bin/python tune_ball_color.py            # use the RealSense camera
    .venv/bin/python tune_ball_color.py --demo     # no hardware; synthetic image
    .venv/bin/python tune_ball_color.py --selftest # render one frame to PNG, exit

Keys (focus the window):
    s            save current values (prints + writes ball_color_tuned.txt)
    r            reset sliders to the starting values
    p            pause / resume camera polling
    q / Esc      quit
"""

import argparse
import os
import threading
import time

import cv2
import numpy as np

# Starting values mirror ballColor in combinedtest.py (per-channel HSV min/max,
# OpenCV scale: H 0-179, S/V 0-255).
START_MIN = [20, 40, 80]
START_MAX = [40, 200, 240]
# Colour the synthetic --demo ball is painted with (RGB); its HSV lands inside
# the default window above.
DEMO_BALL_RGB = [153, 152, 97]

WINDOW = "ball-color tuner"
TRACKBARS = [
    ("min H", 179),
    ("min S", 255),
    ("min V", 255),
    ("max H", 179),
    ("max S", 255),
    ("max V", 255),
]


# --------------------------------------------------------------------------- #
# Mask + image helpers
# --------------------------------------------------------------------------- #
def compute_mask(frame_rgb, lo, hi):
    """Boolean mask of ball pixels for an RGB uint8 frame.

    Matches combinedtest.main(): the frame is converted to HSV and a pixel is
    kept when lo <= channel <= hi for all three HSV channels (inclusive).
    """
    hsv = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2HSV)
    return ((hsv >= np.array(lo)) & (hsv <= np.array(hi))).all(axis=2)


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
    img[ball] = np.array(DEMO_BALL_RGB, dtype=np.uint8)
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


def build_display(frame_rgb, lo, hi, fps, paused, disp_w):
    """Return (composite BGR image, coverage%) for one frame."""
    mask = compute_mask(frame_rgb, lo, hi)
    coverage = float(mask.mean()) * 100.0
    filtered_rgb = np.where(mask[..., None], frame_rgb, np.uint8(0)).astype(np.uint8)

    live = label(cv2.cvtColor(resize_to_width(frame_rgb, disp_w), cv2.COLOR_RGB2BGR),
                 "LIVE")
    filt = label(cv2.cvtColor(resize_to_width(filtered_rgb, disp_w), cv2.COLOR_RGB2BGR),
                 "FILTERED  coverage %.1f%%" % coverage)
    combo = np.hstack([live, filt])

    bar = "ballColor(HSV) = ([%d,%d,%d], [%d,%d,%d])  %.0f fps%s" % (
        lo[0], lo[1], lo[2], hi[0], hi[1], hi[2],
        fps, "  [PAUSED]" if paused else "")
    foot = np.zeros((26, combo.shape[1], 3), dtype=np.uint8)
    cv2.putText(foot, bar, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                (180, 255, 180), 1, cv2.LINE_AA)
    # swatches of the min (left) and max (right) HSV bounds, shown as BGR
    w = combo.shape[1]
    lo_bgr = cv2.cvtColor(np.uint8([[lo]]), cv2.COLOR_HSV2BGR)[0, 0]
    hi_bgr = cv2.cvtColor(np.uint8([[hi]]), cv2.COLOR_HSV2BGR)[0, 0]
    cv2.rectangle(foot, (w - 58, 4), (w - 34, 22),
                  tuple(int(c) for c in lo_bgr), -1)
    cv2.rectangle(foot, (w - 30, 4), (w - 6, 22),
                  tuple(int(c) for c in hi_bgr), -1)
    return np.vstack([combo, foot]), coverage


def format_ball_color(lo, hi):
    return "ballColor = ([%d,%d,%d], [%d,%d,%d])  # HSV min,max" % (
        lo[0], lo[1], lo[2], hi[0], hi[1], hi[2])


def save_values(lo, hi):
    line = format_ball_color(lo, hi)
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
            from combinedtest import CamClient, getCameraFrame
        except Exception as e:  # pragma: no cover - hardware deps
            self.error = "import combinedtest failed: %s" % e
            return
        try:
            with CamClient() as cam:
                while self.running:
                    if self.paused:
                        time.sleep(0.05)
                        continue
                    try:
                        # copy: getCameraFrame returns a view into the RealSense
                        # frame buffer, which the SDK recycles on the next
                        # wait_for_frames(); copy so this GUI keeps a stable image.
                        self._set(getCameraFrame(cam).copy())
                        self.error = None
                    except Exception as e:
                        self.error = "camera read failed: %s" % e
                        time.sleep(0.2)
        except Exception as e:  # pragma: no cover - hardware deps
            self.error = "camera open failed: %s" % e


# --------------------------------------------------------------------------- #
# Trackbars
# --------------------------------------------------------------------------- #
def create_trackbars():
    starts = START_MIN + START_MAX
    for (name, vmax), val in zip(TRACKBARS, starts):
        cv2.createTrackbar(name, WINDOW, val, vmax, lambda v: None)


def read_trackbars():
    pos = [cv2.getTrackbarPos(name, WINDOW) for name, _ in TRACKBARS]
    return pos[0:3], pos[3:6]


def set_trackbars(lo, hi):
    for (name, _), val in zip(TRACKBARS, list(lo) + list(hi)):
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
    combo, coverage = build_display(frame, list(START_MIN), list(START_MAX),
                                    15.0, False, 480)
    out = "tuner_selftest.png"
    cv2.imwrite(out, combo)
    print("wrote %s  shape=%s  coverage=%.2f%%" % (out, combo.shape, coverage))
    print(format_ball_color(list(START_MIN), list(START_MAX)))
    print("selftest OK")


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--demo", action="store_true",
                    help="use a synthetic moving image instead of the RealSense camera")
    ap.add_argument("--selftest", action="store_true",
                    help="render one frame to tuner_selftest.png and exit (no window)")
    ap.add_argument("--width", type=int, default=480,
                    help="display width of each panel in pixels (default 480)")
    args = ap.parse_args()

    if args.selftest:
        run_selftest(args.demo)
        return

    # The OpenCV/Qt GUI aborts the whole process (uncatchable) if it cannot
    # reach a display, so check for one first and fail with a clear message.
    if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        raise SystemExit(
            "No display available (DISPLAY/WAYLAND_DISPLAY unset). The GUI needs "
            "a desktop session; over SSH use X11 forwarding (ssh -X), or run on "
            "the machine's own display.\n"
            "To exercise the pipeline without a window, use --selftest.")

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
    lo, hi = list(START_MIN), list(START_MAX)
    try:
        while True:
            lo, hi = read_trackbars()

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
                combo, _ = build_display(frame, lo, hi, fps,
                                         cam.paused, args.width)
                cv2.imshow(WINDOW, combo)

            key = cv2.waitKey(15) & 0xFF
            if key in (ord("q"), 27):  # q or Esc
                break
            elif key == ord("s"):
                save_values(lo, hi)
            elif key == ord("p"):
                cam.paused = not cam.paused
            elif key == ord("r"):
                set_trackbars(START_MIN, START_MAX)

            # window closed via the [x] button
            if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                break
    except KeyboardInterrupt:
        pass
    finally:
        cam.stop()
        cam.join(timeout=1.0)
        cv2.destroyAllWindows()
        print(format_ball_color(lo, hi))


if __name__ == "__main__":
    main()
