import numpy as np
import cv2
from standardbots import StandardBotsRobot, models
from collections import deque
import base64
from time import sleep

import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib import gridspec

from ah_wrapper import AHSerialClient

def main():

    arm = StandardBotsRobot(
        url='http://192.168.1.3:3000', 
        token='3citgsf7-gycosg-uy730cec-4pr51c', 
        robot_kind=StandardBotsRobot.RobotKind.Live,
    )

    request = models.CameraFrameRequest(
        camera_settings=models.CameraSettings(
            brightness=0,
            contrast=50,
            exposure=350,
            sharpness=50,
            hue=0,
            whiteBalance=4600,
            autoWhiteBalance=True,
        )
    )

    hand = AHSerialClient(write_thread=False)

    try:
        with arm.connection():
            res = arm.camera.data.get_color_frame(request)
            raw_data = res.response.data
            base64_data = raw_data.decode().split(",")[1]
            image_data = base64.b64decode(base64_data)
            np_byte_array = np.frombuffer(image_data, dtype=np.uint8)
            pixel_array_bgr = cv2.imdecode(np_byte_array, cv2.IMREAD_COLOR)
            pixel_array_rgb = cv2.cvtColor(pixel_array_bgr, cv2.COLOR_BGR2RGB)
            
            print(pixel_array_rgb)

            flat = pixel_array_rgb.flatten()

            print(flat)

            avg = np.mean(flat)

            print(avg)

    finally:
        hand.close()

if __name__ == "__main__":
    main()
