import time
import threading
import zmq
import numpy as np
from collections import deque
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from retico_core.abstract import AbstractProducingModule
from retico_coppelia.coppelia_cozmo import Cozmo
# from retico_core.robot import RobotStateIU
from retico_coppelia.coppelia_cozmo_util import CozmoStateIU


class CozmoStateModule(AbstractProducingModule):
    @staticmethod
    def name():
        return "Cozmo State Module"

    @staticmethod
    def description():
        return "A module that tracks the state of a CoppeliaSim Cozmo robot"

    @staticmethod
    def output_iu():
        return CozmoStateIU

    def __init__(self, robot: Cozmo, pub_ip, port=20001, **kwargs):
        super().__init__(**kwargs)
        self.robot = robot
        self.port = port
        self.pub_ip = pub_ip
        self.num_states = 0
        self.num_frames = 0
        self.state_queue = deque(maxlen=5)
        self._update = False

        client = RemoteAPIClient()
        self._sim = client.require('sim')

        print('Connecting to simulation...')
        context = zmq.Context()
        self.subscriber = context.socket(zmq.SUB)
        self.subscriber.connect(f"tcp://{pub_ip}:{port}")
        self.subscriber.setsockopt_string(zmq.SUBSCRIBE, '') # Subscribes to all messages

    def process_update(self, um):
        if len(self.state_queue) == 0: return

        state = self.state_queue.popleft()
        output_iu = self.create_iu(None)
        output_iu.set_state(state)
        self.num_states += 1
        self.append(output_iu)

    def _state_listener(self, **kwargs):
        while self._update:
            try:

                # if np.round(time.time() % 2, 1) < 0.1:  # Get only about half of the frames per ten seconds
                state = self.subscriber.recv_json(zmq.NOBLOCK)
                print(f"Received state: {state}")
                # robot = kwargs['robot']
                # state = robot.get_robot_state_dict()
                # state.update({"left_wheel_speed": data['left_wheel_speed']})
                # state.update({"right_wheel_speed": data['right_wheel_speed']})
                # state.update({"battery_voltage": str(robot.battery_voltage)})
                # state.update({"robot_id": str(robot.robot_id)})
                # state.update({"time": str(time.time())})
                # state.update({'face_count': str(robot.world.visible_face_count())})
                self.state_queue.append(state)

            except zmq.Again:
                pass

    def setup(self):
        self.robot.set_zmq_port(self.port)
        print(f"Connected to publisher at {self.pub_ip}:{self.port}")
        self._update = True
        threading.Thread(target=self._state_listener, daemon=True).start()

    def shutdown(self):
        self._update = False
