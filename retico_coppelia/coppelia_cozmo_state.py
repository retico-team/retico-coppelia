import threading
import zmq
from collections import deque
from retico_core.abstract import AbstractProducingModule, UpdateMessage, UpdateType, IncrementalUnit
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from retico_coppelia.coppelia_cozmo import Cozmo
# from retico_coppelia.coppelia_cozmo_util import CozmoStateIU


class CozmoStateIU(IncrementalUnit):
    """Attributes:

    :param creator (AbstractModule): The module that created this IU
    :param previous_iu (IncrementalUnit): A link to the IU created before the
        current one.
    :param grounded_in (IncrementalUnit): A link to the IU this IU is based on.
    :param created_at (float): The UNIX timestamp of the moment the IU is created.
    :param state (dict): The state of the robot
    """

    @staticmethod
    def type():
        return "Cozmo State IU"

    def __init__(self, creator=None, iuid=0, previous_iu=None, grounded_in=None, state=None,
                 **kwargs):
        super().__init__(creator=creator, iuid=iuid, previous_iu=previous_iu,
                         grounded_in=grounded_in, payload=state)
        self.payload = state

    def set_state(self, state):
        """Sets the state of the robot"""
        self.payload = state

    def __str__(self):
        out = "\n{"
        for key, val in self.payload.items():
            out += "\n\t" + key + ": " + str(val) + ","
        out += "\n}"
        return out

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
        um = UpdateMessage.from_iu(output_iu, UpdateType.ADD)
        self.append(um)

    def _state_listener(self):
        """Looping/threaded function for fetching state packages from CoppeliaSim."""

        while self._update:
            try:  # Try to receive state package from CoppeliaSim as python dict/json
                state = self.subscriber.recv_json(zmq.NOBLOCK)
                self.state_queue.append(state)

            except zmq.Again:  # No package
                pass

    def setup(self):
        self.robot.set_zmq_port(self.port)  # Binds a specific simulation robot to the port to communicate on
        print(f"Connected to publisher at {self.pub_ip}:{self.port}")
        self._update = True
        threading.Thread(target=self._state_listener, daemon=True).start()

    def shutdown(self):
        self._update = False
