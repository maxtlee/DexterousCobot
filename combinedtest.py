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

armA = (85*3.14/180, 16*3.14/180, 144*3.14/180, 22*3.14/180, -270*3.14/180, 180*3.14/180)
armB = (95*3.14/180, 16*3.14/180, 144*3.14/180, 22*3.14/180, -270*3.14/180, 180*3.14/180)

handOpen = [40, 40, 40, 40, 20, -60]
handClosed = [70, 70, 70, 70, 50, -80]

groundColor = ([75,70,68], [20,15,15])
tolerance = 2
# ballColor = ([193,192,127], [33*tolerance,38*tolerance,20*tolerance])
ballColor = ([153,152,97], [33*tolerance,38*tolerance,20*tolerance])

defaultCameraRequest = models.CameraFrameRequest(
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

def main():
    with ArmClient() as arm:
        # initArm(arm)
        # moveHand(hand, handOpen)

        # print(getBrightness(arm))

        frame = getCameraFrame(arm)
        # print(frame.shape)
        # print(frame[360,640])

        # isBall = np.vectorize(lambda pix: (abs(pix[0] - ballColor[0][0]) < ballColor[1][0]) and (abs(pix[1] - ballColor[0][1]) < ballColor[1][1]) and (abs(pix[2] - ballColor[0][2]) < ballColor[1][2]))
        # isBallFrame = isBall(pix=(frame[:,:,0],frame[:,:,1],frame[:,:,2]))
        frame0 = frame[:,:,0]
        isBall0 = abs(frame0 - ballColor[0][0]) < ballColor[1][0]
        frame1 = frame[:,:,1]
        isBall1 = abs(frame1 - ballColor[0][1]) < ballColor[1][1]
        frame2 = frame[:,:,2]
        isBall2 = abs(frame2 - ballColor[0][2]) < ballColor[1][2]

        isBallFrame = isBall2 & isBall1 & isBall0

        out = np.where(isBallFrame[...,None], np.array([np.uint8(0),np.uint8(0),np.uint8(255)]), np.array([np.uint8(0),np.uint8(0),np.uint8(0)]))

        cv2.imwrite("frame.jpeg", frame)
        cv2.imwrite("filtered.jpeg", out)

        # moveHand(hand, handClosed)

def getCameraFrame(arm, cameraRequest = defaultCameraRequest): 
    res = arm.camera.data.get_color_frame(cameraRequest)
    raw_data = res.response.data
    base64_data = raw_data.decode().split(",")[1]
    image_data = base64.b64decode(base64_data)
    np_byte_array = np.frombuffer(image_data, dtype=np.uint8)
    pixel_array_bgr = cv2.imdecode(np_byte_array, cv2.IMREAD_COLOR)
    pixel_array_rgb = cv2.cvtColor(pixel_array_bgr, cv2.COLOR_BGR2RGB)
    return pixel_array_rgb

def getBrightness(arm, cameraRequest = defaultCameraRequest
    ): 
    pixel_array_rgb = getCameraFrame(arm, cameraRequest)
    flat = pixel_array_rgb.flatten()
    avg = np.mean(flat)
    return avg

def getTennisBallLoc3D(arm, cameraRequest = defaultCameraRequest, 
        tennisBallAcceptColors = ballColor
    ): 
    pixel_array_rgb = getCameraFrame(arm, cameraRequest)
    # pixel_array_tennis_ball = 
    pixel_array_tennis_ball = pixel_array_rgb > tennisBallAcceptColors[0] and pixel_array_rgb < tennisBallAcceptColors[1] #TODO: FIX SYNTAX
    tennisBallLoc2D = fitCircleInArray(pixel_array_tennis_ball)
    tennisBallOffset = 0 #TODO: Implement
    tennisBallLoc3D = arm.getCamPose().TransformBy(tennisBallOffset) #TODO: Fix Syntax
    return tennisBallLoc3D

# Returns a location (x,y) and diameter
def fitCircleInArray(pixelArray):
    return ((0,0),0) #TODO: Implmement


def moveArm(arm, position):
    body = models.ArmPositionUpdateRequest(
        kind=models.ArmPositionUpdateRequestKindEnum.JointRotation,
        joint_rotation=models.ArmJointRotations(joints=position),
    )
    response = arm.movement.position.set_arm_position(body)
    try:
        print(response.ok())
    except Exception:
        print(response.data.message)

def initArm(arm):
    arm.movement.brakes.unbrake().ok()
    arm.status.control.set_configuration_control_state(models.RobotControlMode(kind=models.RobotControlModeEnum.Api)).ok()
    arm.recovery.recover.recover().ok()

def moveHand(hand, position):
    hand.set_position(positions=position, reply_mode=2)
    hand.send_command()

class HandClient:
    def __enter__(self):
        self.hand = AHSerialClient(write_thread=False)
        return self.hand

    def __exit__(self,a,b,c):
        self.hand.close()

class ArmClient:
    def __enter__(self):
        self.sdk = StandardBotsRobot(
            url='http://192.168.1.3:3000', 
            token=Path("/home/max/Documents/.robot_token").read_text().strip(),
            robot_kind=StandardBotsRobot.RobotKind.Live,
        )
        return self.sdk

    def __exit__(self, a, b, c):
        self.sdk.movement.brakes.brake().ok()
        self.sdk._request_manager.close()

if __name__ == "__main__":
    main()
