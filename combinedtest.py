import numpy as np
import cv2
from pathlib import Path
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
        token=Path("/home/max/Documents/.robot_token").read_text().strip(),
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
        target_position1 = (85*3.14/180, 16*3.14/180, 144*3.14/180, 22*3.14/180, -270*3.14/180, 180*3.14/180)
        body1 = models.ArmPositionUpdateRequest(
            kind=models.ArmPositionUpdateRequestKindEnum.JointRotation,
            joint_rotation=models.ArmJointRotations(joints=target_position1),
        )
        target_position2 = (95*3.14/180, 16*3.14/180, 144*3.14/180, 22*3.14/180, -270*3.14/180, 180*3.14/180)
        body2 = models.ArmPositionUpdateRequest(
            kind=models.ArmPositionUpdateRequestKindEnum.JointRotation,
            joint_rotation=models.ArmJointRotations(joints=target_position2),
        )
        
        hand.set_position(positions=[40, 40, 40, 40, 20, -60], reply_mode=2)
        hand.send_command()

        with arm.connection():
            arm.movement.brakes.unbrake().ok()
            arm.status.control.set_configuration_control_state(models.RobotControlMode(kind=models.RobotControlModeEnum.Api)).ok()
            arm.recovery.recover.recover().ok()

            state = False
            while (brightness := getBrightness(arm, request)) > 20:
                print(brightness)
                response = arm.movement.position.set_arm_position(body1 if state else body2)
                try:
                    print(response.ok())
                except Exception:
                    print(response.data.message)
                #    break
                state = not state

            hand.set_position([70, 70, 70, 70, 50, -80], reply_mode=2)
            hand.send_command()
    finally:
        arm.movement.brakes.brake().ok()
        hand.close()

def getBrightness(arm, request): 
    res = arm.camera.data.get_color_frame(request)
    raw_data = res.response.data
    base64_data = raw_data.decode().split(",")[1]
    image_data = base64.b64decode(base64_data)
    np_byte_array = np.frombuffer(image_data, dtype=np.uint8)
    pixel_array_bgr = cv2.imdecode(np_byte_array, cv2.IMREAD_COLOR)
    pixel_array_rgb = cv2.cvtColor(pixel_array_bgr, cv2.COLOR_BGR2RGB)
    flat = pixel_array_rgb.flatten()
    avg = np.mean(flat)
    return avg


if __name__ == "__main__":
    main()
