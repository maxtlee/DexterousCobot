import time
from pathlib import Path
import numpy as np
import cv2
from standardbots import StandardBotsRobot

sdk = StandardBotsRobot(
    url='http://192.168.1.3:3000',
    token=Path("/home/max/Documents/.robot_token").read_text().strip(),
    robot_kind=StandardBotsRobot.RobotKind.Live,
)

OUTPUT_PATH = 'capture.mp4'
TARGET_FPS = 30.0  # assumes constant frame rate and is technically incorrect

SOI = b'\xff\xd8'
EOI = b'\xff\xd9'

def iter_jpeg_frames(http_resp):
    buf = b''
    for chunk in http_resp.stream(8192):
        buf += chunk
        while True:
            start = buf.find(SOI)
            end = buf.find(EOI, start + 2)
            if start == -1 or end == -1:
                break
            yield buf[start:end + 2]
            buf = buf[end + 2:]

with sdk.connection():
    resp = sdk.camera.data.get_camera_stream()
    raw = resp.response
    writer = None
    t0 = None
    n = 0
    try:
        for jpeg in iter_jpeg_frames(raw):
            img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                continue
            if writer is None:
                h, w = img.shape[:2]
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                writer = cv2.VideoWriter(OUTPUT_PATH, fourcc, TARGET_FPS, (w, h))
                t0 = time.monotonic()
            writer.write(img)
            n += 1
    except KeyboardInterrupt:
        pass
    finally:
        if writer is not None:
            writer.release()
            dt = time.monotonic() - t0
            print(f'wrote {n} frames to {OUTPUT_PATH} in {dt:.2f}s '
                  f'(actual avg {n/dt:.2f} fps, file fps {TARGET_FPS})')
        raw.release_conn()
