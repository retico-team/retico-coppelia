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

**!! ONLY USE BULLET 2.83 with CoppeliaCozmoModule. CoppeliaSim's other physics engines do not interact properly with the provided Cozmo model. !!**

---

## Example
### Coppelia Runner
```python
import sys, os
import threading
import time
import numpy as np

prefix = '<path-to-Retico-repositories>'

os.environ['RETICO'] = prefix + "retico-core"
os.environ['RETICOV'] = prefix + "retico-vision"
os.environ["ZMQ"] = prefix + "retico-zmq"
os.environ['CPLIA'] = "retico-coppelia"

sys.path.append(os.environ['RETICO'])
sys.path.append(os.environ['RETICOV'])
sys.path.append(os.environ["ZMQ"])
sys.path.append(os.environ['CPLIA'])

from retico_core import UpdateType, UpdateMessage
from retico_core.debug import DebugModule
from retico_core.text import SpeechRecognitionIU
from retico_zmq.zmq import ReaderSingleton
from retico_coppelia.coppelia import CoppeliaModule, JointPositionIU
from retico_coppelia.coppelia_cozmo_util import *
from retico_coppelia.coppelia_cozmo import CoppeliaCozmoModule
from retico_coppelia.coppelia_cozmo_state import CozmoStateModule
from retico_coppelia.coppelia_camera import CoppeliaCameraModule
from asr2cozmo import ASR2CozmoModule

scene = './coppelia_cozmo_example.ttt'
sensor_path = '/cozmo/camera_joint/Vision_sensor'
cozmo_path = '/cozmo'

ip = "<ip-of-machine-running-asr_runner>"
asr = ReaderSingleton(ip=ip, port="12345")
asr.add(topic="asr", target_iu_type=SpeechRecognitionIU)

asr2cozmo = ASR2CozmoModule()
coppelia = CoppeliaModule(scene=scene, start_scene=False)
cozmo = CoppeliaCozmoModule(cozmo_path=cozmo_path, scene=scene, start_scene=True)
state = CozmoStateModule(cozmo.robot, pub_ip='localhost')  # pub_ip is the ip of the machine running the simulation
cam = CoppeliaCameraModule(scene=scene, sensor_path=sensor_path, visualizer=True)
# debug = DebugModule(print_payload_only=True)
debug = DebugModule(print_payload_only=False)

asr.subscribe(asr2cozmo)
asr.subscribe(debug)
asr2cozmo.subscribe(cozmo)
state.subscribe(debug)

asr.run()
asr2cozmo.run()
coppelia.run()
cozmo.run()
state.run()
cam.run()
debug.run()

# input to CoppeliaModule
inputs = [
    ({"/LBRiiwa14R820/joint": np.radians(30)}, UpdateType.ADD),  # {joint_path: magnitude}, update_type
    ({"/LBRiiwa14R820/link2_resp/joint": np.radians(30)}, UpdateType.ADD),
    ({"/LBRiiwa14R820/link3_resp/joint": np.radians(90)}, UpdateType.ADD),
    ({"/LBRiiwa14R820/link4_resp/joint": np.radians(-30)}, UpdateType.ADD),
    ({"/LBRiiwa14R820/link5_resp/joint": np.radians(30)}, UpdateType.ADD),
    ({"/LBRiiwa14R820/link6_resp/joint": np.radians(30)}, UpdateType.ADD),
    ({"/LBRiiwa14R820/link7_resp/joint": np.radians(-30)}, UpdateType.ADD),
    ({"/LBRiiwa14R820/joint": np.radians(-30)}, UpdateType.ADD),

    ({"/LBRiiwa14R820/joint": np.radians(30)}, UpdateType.ADD),
    ({"/LBRiiwa14R820/link7_resp/joint": np.radians(30)}, UpdateType.ADD),
    ({"/LBRiiwa14R820/link6_resp/joint": np.radians(-30)}, UpdateType.ADD),
    ({"/LBRiiwa14R820/link5_resp/joint": np.radians(-30)}, UpdateType.ADD),
    ({"/LBRiiwa14R820/link4_resp/joint": np.radians(30)}, UpdateType.ADD),
    ({"/LBRiiwa14R820/link3_resp/joint": np.radians(-90)}, UpdateType.ADD),
    ({"/LBRiiwa14R820/link2_resp/joint": np.radians(-30)}, UpdateType.ADD),
    ({"/LBRiiwa14R820/joint": np.radians(-30)}, UpdateType.ADD),
]

def _loop(inputs, _looping):
    while _looping:
        iu_counter = 0
        coppelia_prefix = [None]
        i = 0
        for payload, ut in inputs:
            iu = JointPositionIU(iuid=iu_counter, previous_iu=coppelia_prefix[-1], payload=inputs[i][0])
            um = UpdateMessage.from_iu(iu, inputs[i][1])
            coppelia.process_update(um)
            iu_counter += 1
            coppelia_prefix.append(iu)
            i = (i + 1) % 16

_looping = True
t = threading.Thread(target=_loop, args=[inputs, _looping], daemon=True)
t.start()

input()
_looping = False

asr.stop()
asr2cozmo.stop()
coppelia.stop()
cozmo.stop()
state.stop()
cam.stop()
debug.stop()
```

### ASR Runner
```python
import sys, os

prefix = '<path-to-Retico-repositories>'
sys.path.append(prefix+'retico-core')
sys.path.append(prefix+'retico-whisperasr')
sys.path.append(prefix+'retico-zmq')

from retico_core.audio import MicrophoneModule
from retico_core.debug import DebugModule
from retico_zmq.zmq import WriterSingleton, ZeroMQWriter
from retico_whisperasr import WhisperASRModule

ip = '<local-ip>'
WriterSingleton(ip=ip, port='12345')
zmq_writer = ZeroMQWriter(topic='asr')

mic = MicrophoneModule(rate=16000)
asr = WhisperASRModule()
debug = DebugModule(print_payload_only=True)

mic.subscribe(asr)
asr.subscribe(zmq_writer)
asr.subscribe(debug)

mic.run()
asr.run()
zmq_writer.run()
debug.run()

input()

mic.stop()
asr.stop()
zmq_writer.stop()
debug.stop()
```
---

## Module Synopses

### coppelia.CoppeliaModule
This is a general-purpose module for controlling joints within a CoppeliaSim scene.
It accepts Position, Velocity, and Force IUs defined in `coppelia.py` that contain
dicts pairing strings with float values. The strings should be paths within the scene
to the joint that will be manipulated, and the float values are the target values to 
apply to the respective joints. This module assumes very little about the contents of 
the given scene, requiring only that the joints being manipulated are correctly 
configured for the types of IUs being used to manipulate them.

### coppelia_camera.CoppeliaCameraModule
This module grabs the feed from any vision sensor within a running CoppeliaSim scene and 
converts it into PIL Image objects. 

### coppelia_cozmo.CoppeliaCozmoModule
The CoppeliaCozmoModule provides bindings for a Cozmo robot within the CoppeliaSim simulator.
It accepts CoppeliaCozmoIUs which pair a string action-term with a list of values specifying 
a change in position/distance, speed of the transition to the specified position, and whether 
Cozmo should wait to start other actions until the current one has been completed. There are 
several different classes provided in `coppelia_cozmo_util.py` for specifying positions/distance
and speed. For more detail and a usage example, see the documentation for CoppeliaCozmoIU in
`coppelia_cozmo.py`.

### coppelia_cozmo_state.CozmoStateModule
This module monitors and sends the state of a Cozmo robot within a currently running
simulation scene as CozmoStateIUs (found in `coppelia_cozmo_state.py`). It takes a Cozmo robot and sets up a ZMQ channel 
for retrieving updates published by that robot's script within CoppeliaSim.