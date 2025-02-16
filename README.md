# retico-coppelia
A ReTiCo module for the CoppeliaSim robot simulation software.

---

## Installation and requirements
See [CoppeliaSim](https://www.coppeliarobotics.com/) to install the simulator. 

### Required Packages
- numpy
- pyzmq
- pillow
- opencv-python

Running `pip install numpy pyzmq pillow opencv-python` will install each of these.

Additionally, to communicate with the CoppeliaSim software, CoppeliaSim's remote API client must be installed:   
`pip install coppeliasim-zmqremoteapi-client`  
  
More information about CoppeliaSim's remote API client can be found [here](https://manual.coppeliarobotics.com/en/zmqRemoteApiOverview.htm).

---

## Example
```python
import sys, os
import time
import numpy as np

prefix = '<path-to-retico-modules>'

os.environ['RETICO'] = prefix + "retico-core"
os.environ['RETICOV'] = prefix + "retico-vision"
os.environ['CPLIA'] = "retico-coppelia"

sys.path.append(os.environ['RETICO'])
sys.path.append(os.environ['RETICOV'])
sys.path.append(os.environ['CPLIA'])

from retico_core import UpdateType, UpdateMessage
from retico_core.debug import DebugModule
from retico_coppelia.coppelia import CoppeliaModule, JointPositionIU, JointVelocityIU
from retico_coppelia.coppelia_cozmo_util import *
from retico_coppelia.coppelia_cozmo import CoppeliaCozmoModule, CoppeliaCozmoIU
from retico_coppelia.coppelia_cozmo_state import CozmoStateModule
from retico_coppelia.coppelia_camera import CoppeliaCameraModule

# Specify the scene to load and manipulate in CoppeliaSim
scene = '<path-to-this-repository>/retico-coppelia/example/coppelia_cozmo_example.ttt'

# Specify the vision sensor/camera whose feed to view
sensor_path = '/cozmo/camera_joint/Vision_sensor'

# Specify the path to a given cozmo robot within the scene.
# This is the robot that will be controlled.
cozmo_path = '/cozmo'

coppelia = CoppeliaModule(scene=scene, start_scene=False)
cozmo = CoppeliaCozmoModule(cozmo_path=cozmo_path, scene=scene, start_scene=True)
state = CozmoStateModule(cozmo.robot, pub_ip='localhost')
cam = CoppeliaCameraModule(scene=scene, sensor_path=sensor_path, visualizer=True)
debug = DebugModule(print_payload_only=True)

state.subscribe(debug)

cozmo.run()
state.run()
cam.run()
debug.run()

# input to CoppeliaCozmoModule
cozmo_inputs = [
    ({"turn": [Radians(2 * np.pi), Rads(np.pi), True]}, UpdateType.ADD),  # [angle, rate, wait_until_completed], update_type
    ({"drive": [Millimeters(400), MMPS(200), False]}, UpdateType.ADD),  # [distance, rate, wait_until_completed], update_type
    ({"lift": [1, Rads(1), True]}, UpdateType.ADD),  # [position, rate, wait_until_completed], update_type
    ({"look": [1, Rads(1), True]}, UpdateType.ADD),
    ({"lift": [0, DPS(15), False]}, UpdateType.ADD),
    ({"look": [0, DPS(15), False]}, UpdateType.ADD),
    ({"turn": [Degrees(-360), DPS(180), True]}, UpdateType.ADD),
    ({"drive": [Millimeters(-400), MMPS(200), True]}, UpdateType.ADD)
]

# input to CoppeliaModule
inputs = [
    ({"/LBRiiwa14R820/joint": np.radians(90)}, UpdateType.ADD),  # {joint_path: magnitude}, update_type
    ({"/LBRiiwa14R820/link2_resp/joint": np.radians(90)}, UpdateType.ADD),
    ({"/LBRiiwa14R820/link3_resp/joint": np.radians(180)}, UpdateType.ADD),
    ({"/LBRiiwa14R820/link4_resp/joint": np.radians(-90)}, UpdateType.ADD),
    ({"/LBRiiwa14R820/link5_resp/joint": np.radians(90)}, UpdateType.ADD),
    ({"/LBRiiwa14R820/link6_resp/joint": np.radians(90)}, UpdateType.ADD),
    ({"/LBRiiwa14R820/link7_resp/joint": np.radians(-90)}, UpdateType.ADD),
    ({"/LBRiiwa14R820/joint": np.radians(-90)}, UpdateType.ADD),
]

time.sleep(2)  # Wait for simulation to start and settle

iu_counter = 0
cozmo_prefix = [None]
coppelia_prefix = [None]
i = 0
for payload, ut in cozmo_inputs:
    iu = CoppeliaCozmoIU(iuid=iu_counter, previous_iu=cozmo_prefix[-1], payload=payload)
    um = UpdateMessage.from_iu(iu, ut)
    cozmo.process_update(um)
    iu_counter += 1
    cozmo_prefix.append(iu)

    if i < 4:
        iu = JointPositionIU(iuid=iu_counter, previous_iu=coppelia_prefix[-1], payload=inputs[i][0])
    else:
        iu = JointVelocityIU(iuid=iu_counter, previous_iu=coppelia_prefix[-1], payload=inputs[i][0])
    um = UpdateMessage.from_iu(iu, inputs[i][1])
    coppelia.process_update(um)
    iu_counter += 1
    coppelia_prefix.append(iu)
    i += 1

input()

coppelia.stop()
cozmo.stop()
state.stop()
cam.stop()
debug.stop()
```
---

## Module Synopses

### CoppeliaModule  

### CoppeliaCameraModule  

### CoppeliaCozmoModule

### CozmoStateModule