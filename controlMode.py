from pathlib import Path
from standardbots import StandardBotsRobot

sdk = StandardBotsRobot(
    url='http://192.168.1.3:3000',
    token=Path("/home/max/Documents/.robot_token").read_text().strip(),
    robot_kind=StandardBotsRobot.RobotKind.Live,
)

with sdk.connection():
    response = sdk.status.control.get_configuration_state_control()
    try:
        data = response.ok()
        print(data)
    except Exception:
        print(response.data.message)
