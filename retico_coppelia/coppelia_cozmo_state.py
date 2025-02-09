import base64
import time
import threading

import cv2
import zmq
import numpy as np
from collections import deque
from PIL import Image
from retico_core import abstract
# from retico_core.abstract import AbstractProducingModule, UpdateMessage
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from retico_coppelia.coppelia_cozmo import Cozmo
# from retico_core.robot import RobotStateIU
from retico_coppelia.coppelia_cozmo_util import CozmoStateIU


class CozmoStateModule(abstract.AbstractProducingModule):
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

        # self.res = self._sim.getVisionSensorRes(self.robot.)

        print('Connecting to simulation...')
        context = zmq.Context()
        self.subscriber = context.socket(zmq.SUB)
        self.subscriber.connect(f"tcp://{pub_ip}:{port}")
        self.subscriber.setsockopt_string(zmq.SUBSCRIBE, '') # Subscribes to all messages

    def process_update(self, um):
        if len(self.state_queue) == 0: return

        state = self.state_queue.popleft()
        self.num_states += 1
        output_iu = self.create_iu(None)
        output_iu.set_state(state)
        um = abstract.UpdateMessage.from_iu(output_iu, abstract.UpdateType.ADD)
        self.append(um)

    def _state_listener(self):
        while self._update:
            try:
                state = self.subscriber.recv_json(zmq.NOBLOCK)
                stringified = state['image']
                res = state['image_res']

                img_buffer = base64.b64decode(stringified)
                img = np.frombuffer(img_buffer, dtype=np.uint8).reshape(res[1], res[0], 3)
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                img = cv2.flip(img, 0)
                frame = Image.fromarray(img)

                cv2.imshow("cozmo_image", frame)

                state.update({"image": frame})
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
