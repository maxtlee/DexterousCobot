
from combinedtest import (getFrames, CamClient)
import cv2

while True:
    with CamClient() as cam:
        frame, depthM, intr = getFrames(cam)
        cv2.imwrite("frame.jpeg", frame)
