from pathlib import Path
from standardbots import StandardBotsRobot, models
from time import sleep

sdk = StandardBotsRobot(url='http://192.168.1.3:3000', token=Path("/home/max/Documents/.robot_token").read_text().strip(), robot_kind=StandardBotsRobot.RobotKind.Live,)

def out(response):
    try:
        data = response.ok()
        print(data, "\n")
    except Exception:
        print(response.data.message, "\n")  

with sdk.connection():
    
    # unbrake robot
    sdk.movement.brakes.unbrake().ok()
    
    # set to api control mode
    with sdk.connection():
      sdk.status.control.set_configuration_control_state(models.RobotControlMode(kind=models.RobotControlModeEnum.Api)).ok()
    response = sdk.status.control.get_configuration_state_control()
    out(response)

    response = sdk.movement.position.get_arm_position()
    out(response)

    print("moving arm now!!!\n")

    sleep(1)

    target_position = (90*3.14/180, 16*3.14/180, 144*3.14/180, 22*3.14/180, -271*3.14/180, 225*3.14/180)
    body = models.ArmPositionUpdateRequest(
        kind=models.ArmPositionUpdateRequestKindEnum.JointRotation,
        joint_rotation=models.ArmJointRotations(joints=target_position),
    )

    response = sdk.movement.position.set_arm_position(body=body)
    out(response)

    print("moving arm again!  >:0\n")

    sleep(1)

    target_position = (90*3.14/180, 16*3.14/180, 144*3.14/180, 22*3.14/180, -271*3.14/180, 135*3.14/180)
    body = models.ArmPositionUpdateRequest(
        kind=models.ArmPositionUpdateRequestKindEnum.JointRotation,
        joint_rotation=models.ArmJointRotations(joints=target_position),
    )

    response = sdk.movement.position.set_arm_position(body=body)
    out(response)

    print("still moving arm... \n")

    sleep(1)

    target_position = (90*3.14/180, 16*3.14/180, 144*3.14/180, 22*3.14/180, -271*3.14/180, 180*3.14/180)
    body = models.ArmPositionUpdateRequest(
        kind=models.ArmPositionUpdateRequestKindEnum.JointRotation,
        joint_rotation=models.ArmJointRotations(joints=target_position),
    )

    response = sdk.movement.position.set_arm_position(body=body)
    out(response)

    print("moving arm up up up!!! \n")

    sleep(1)

    target_position = (90*3.14/180, 16*3.14/180, 80*3.14/180, 22*3.14/180, -271*3.14/180, 180*3.14/180)
    body = models.ArmPositionUpdateRequest(
        kind=models.ArmPositionUpdateRequestKindEnum.JointRotation,
        joint_rotation=models.ArmJointRotations(joints=target_position),
    )

    response = sdk.movement.position.set_arm_position(body=body)
    out(response)

    print("moving arm back :D \n")

    sleep(1)

    target_position = (90*3.14/180, 16*3.14/180, 150*3.14/180, 22*3.14/180, -271*3.14/180, 180*3.14/180)
    body = models.ArmPositionUpdateRequest(
        kind=models.ArmPositionUpdateRequestKindEnum.JointRotation,
        joint_rotation=models.ArmJointRotations(joints=target_position),
    )

    response = sdk.movement.position.set_arm_position(body=body)
    out(response)

    print("rebraking\n")

    sleep(1)

    sdk.movement.brakes.brake().ok()




#     # Example how to move tooltip to target position
#     target_tooltip_position = models.Position(x=0.1, y=0.2, z=1.0)
#     body = models.ArmPositionUpdateRequest(
#         kind=models.ArmPositionUpdateRequestKindEnum.TooltipPosition,
#         tooltip_position=models.PositionAndOrientation(
#             position=target_tooltip_position,
#             orientation=models.Orientation(
#                 kind=models.OrientationKindEnum.Quaternion,
#                 quaternion=models.Quaternion(0.0, 0.0, 0.0, 1.0),
#             ),
#         ),
#     )
#     sdk.movement.position.set_arm_position(body=body).ok()

    # Example how to rotate joints in radians.
    # target_position corresponds to (J0, J1, J2, J3, J4, J5)
    







# from standardbots import models, StandardBotsRobot

# sdk = StandardBotsRobot(
#   url='https://mybot.sb.app',
#   token='token',
#   robot_kind=StandardBotsRobot.RobotKind.Live,
# )

# with sdk.connection():
#   sdk.movement.brakes.unbrake().ok()
#   sdk.movement.position.move(
#     position=models.Position(
#       unit_kind=models.LinearUnitKind.Meters,
#       x=0.1,
#       y=0.2,
#       z=1.0,
#     ),
#     orientation=models.Orientation(
#       kind=models.OrientationKindEnum.Quaternion,
#       quaternion=models.Quaternion(0.0, 0.0, 0.0, 1.0),
#     ),
#   ).ok()