
from pathlib import Path
from standardbots import StandardBotsRobot, models
import base64

sdk = StandardBotsRobot(
    url='http://192.168.1.3:3000', 
    token=Path("/home/max/Documents/.robot_token").read_text().strip(),
    robot_kind=StandardBotsRobot.RobotKind.Live,
)


body = models.CameraFrameRequest(
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

with sdk.connection():
    res = sdk.camera.data.get_color_frame(body)

raw_data = res.response.data

# Extract the base64 encoded data
base64_data = raw_data.decode().split(",")[1]

# Decode the base64 data
image_data = base64.b64decode(base64_data)

# Write the frame as jpeg
with open("frame.jpeg", "wb") as f:
    f.write(image_data)
