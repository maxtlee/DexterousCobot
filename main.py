from standardbots import StandardBotsRobot, models

sdk = StandardBotsRobot(url='http://192.168.1.3:3000', token='3citgsf7-gycosg-uy730cec-4pr51c', robot_kind=StandardBotsRobot.RobotKind.Live,)

with sdk.connection():
    
    # brake robot
    sdk.movement.brakes.brake().ok()
    
    # set to api control mode
    with sdk.connection():
      sdk.status.control.set_configuration_control_state(models.RobotControlMode(kind=models.RobotControlModeEnum.Api)).ok()

    # double check control mode and print
    response = sdk.status.control.get_configuration_state_control()
    try:
        data = response.ok()
        print(data)
    except Exception:
        print(response.data.message)   
 
