from standardbots import StandardBotsRobot, models

sdk = StandardBotsRobot(url='http://192.168.1.3:3000', token='3citgsf7-gycosg-uy730cec-4pr51c', robot_kind=StandardBotsRobot.RobotKind.Live,)

with sdk.connection():
    response = sdk.status.control.get_configuration_state_control()
    try:
        data = response.ok()
        print(data)
    except Exception:
        print(response.data.message)   
    sdk.movement.brakes.brake().ok()
    sdk.equipment.get_gripper_configuration().ok()
    with sdk.connection():
      sdk.status.control.set_configuration_control_state(models.RobotControlMode(kind=models.RobotControlModeEnum.Api)).ok()
    response = sdk.status.control.get_configuration_state_control()
    try:
        data = response.ok()
        print(data)
    except Exception:
        print(response.data.message)   
 
