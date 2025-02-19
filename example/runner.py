import sys, os
import threading
import time
import numpy as np

prefix = '/home/prigby/Retico/'

os.environ['RETICO'] = prefix + "retico-core"
os.environ['RETICOV'] = prefix + "retico-vision"
os.environ["ZMQ"] = prefix + "retico-zmq"
os.environ['CPLIA'] = "/home/prigby/SLIM/repos/retico-coppelia"

sys.path.append(os.environ['RETICO'])
sys.path.append(os.environ['RETICOV'])
sys.path.append(os.environ["ZMQ"])
sys.path.append(os.environ['CPLIA'])

from retico_core import UpdateType, UpdateMessage
from retico_core.debug import DebugModule
from retico_core.text import SpeechRecognitionIU
from retico_zmq.zmq import ReaderSingleton
from retico_coppelia.coppelia import CoppeliaModule, JointPositionIU, JointVelocityIU
from retico_coppelia.coppelia_cozmo_util import *
from retico_coppelia.coppelia_cozmo import CoppeliaCozmoModule
from retico_coppelia.coppelia_cozmo_state import CozmoStateModule
from retico_coppelia.coppelia_camera import CoppeliaCameraModule
from asr2cozmo import ASR2CozmoModule

scene = '/home/prigby/SLIM/repos/retico-coppelia/example/coppelia_cozmo_example.ttt'
sensor_path = '/cozmo/camera_joint/Vision_sensor'
cozmo_path = '/cozmo'

ip = "192.168.1.41"
# asr = ReaderSingleton(ip=ip, port="12345")
# asr.add(topic="asr", target_iu_type=SpeechRecognitionIU)

asr2cozmo = ASR2CozmoModule()
coppelia = CoppeliaModule(scene=scene, start_scene=False)
cozmo = CoppeliaCozmoModule(cozmo_path=cozmo_path, scene=scene, start_scene=True)
state = CozmoStateModule(cozmo.robot, pub_ip='localhost')
cam = CoppeliaCameraModule(scene=scene, sensor_path=sensor_path, visualizer=True)
# debug = DebugModule(print_payload_only=True)
debug = DebugModule(print_payload_only=False)

# asr.subscribe(asr2cozmo)
# asr.subscribe(debug)
asr2cozmo.subscribe(cozmo)
# state.subscribe(debug)

# asr.run()
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

# time.sleep(2)  # Wait for simulation to start and settle

def _loop(inputs, _looping):
    while _looping:
        iu_counter = 0
        cozmo_prefix = [None]
        coppelia_prefix = [None]
        i = 0
        for payload, ut in inputs:
            iu = JointPositionIU(iuid=iu_counter, previous_iu=coppelia_prefix[-1], payload=inputs[i][0])
            um = UpdateMessage.from_iu(iu, inputs[i][1])
            coppelia.process_update(um)
            iu_counter += 1
            coppelia_prefix.append(iu)
            i = (i + 1) % 16
            # time.sleep(1)

_looping = True
t = threading.Thread(target=_loop, args=[inputs, _looping], daemon=True)
t.start()

input()
_looping = False

# asr.stop()
asr2cozmo.stop()
coppelia.stop()
cozmo.stop()
state.stop()
cam.stop()
debug.stop()
