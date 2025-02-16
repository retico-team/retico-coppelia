import time
import retico_core
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from retico_coppelia.coppelia_cozmo_util import *

class Cozmo:
    """An object for interfacing with a Cozmo robot within CoppeliaSim."""

    def __init__(self, cozmo_path, scene, start_scene=False):
        self.start_scene = start_scene
        self._sim = RemoteAPIClient().require('sim')

        if self.start_scene:
            self._sim.loadScene(scene)

        # Initialize scene script
        self._script_handle = self._sim.getScript(self._sim.scripttype_simulation, cozmo_path + '/Script')
        self._sim.initScript(self._script_handle)

        if self.start_scene:
            print("Starting simulation...")
            self._sim.startSimulation()

    def shutdown(self):
        """Stops the simulation if self was used to start the simulation."""
        if self.start_scene:
            print("Stopping simulation...")
            self._sim.stopSimulation()

    def set_zmq_port(self, port):
        """Sets the port for Cozmo to use for ZMQ messaging.

        :param port: The port that the publisher will publish to
        """
        self._sim.callScriptFunction(
            "bind_zmq",
            self._script_handle,
            port
        )

    def wait_until_completed(self):
        """Hangs execution of function calls from a Cozmo object until the corresponding CoppeliaSim robot has stopped
        moving.
        """
        while self._sim.callScriptFunction("is_moving", self._script_handle):
            time.sleep(0.1)

    def turn_in_place(self, angle: Angle, speed: AngularSpeed):
        """Calls the Cozmo robot script function turn_in_place() within CoppeliaSim.

        :param angle: An Angle object denoting how far to turn. The Angle is relative to the position of Cozmo at call
        time.
        :param speed: The speed at which Cozmo should turn.
        :return: A reference to self, allowing wait_until_completed() to be called to prevent parallel actions.
        """
        self._sim.callScriptFunction("turn_in_place", self._script_handle, angle.to_radians(), speed.to_rads())
        return self

    def set_head_angle(self, height: float, speed: AngularSpeed):
        """Calls the Cozmo robot script function set_head_angle() within CoppeliaSim.

        :param height: The position or angle that Cozmo's head should be set to. Can be a float from 0 to 1, and
        represents a percentage of Cozmo's full head range.
        :param speed: An AngularSpeed object, denoting how fast Cozmo should change its head position.
        :return: A reference to self, allowing wait_until_completed() to be called to prevent parallel actions.
        """
        self._sim.callScriptFunction("set_head_angle", self._script_handle, height, speed.to_rads())
        return self

    def set_lift_height(self, height: float, speed: AngularSpeed):
        """Calls the Cozmo robot script function set_lift_height() within CoppeliaSim.

        :param height: The position that Cozmo's lift should be set to. Can be a float from 0 to 1, and represents a
        percentage of Cozmo's full lift range.
        :param speed: An AngularSpeed object, denoting how fast Cozmo should change its lift position.
        :return: A reference to self, allowing wait_until_completed() to be called to prevent parallel actions.
        """
        self._sim.callScriptFunction("set_lift_height", self._script_handle, height, speed.to_rads())
        return self

    def drive_straight(self, distance: Distance, speed: MMPS):
        """Calls the Cozmo robot script function drive_straight() within CoppeliaSim.

        :param distance: A Distance object, denoting how far Cozmo should travel in a straight line.
        :param speed: An MMPS object, denoting how fast Cozmo should travel.
        :return: A reference to self, allowing wait_until_completed() to be called to prevent parallel actions.
        """
        self._sim.callScriptFunction("drive_straight", self._script_handle, distance.to_mm(), speed.to_mmps())
        return self
    

class CoppeliaCozmoIU(retico_core.abstract.IncrementalUnit):
    """Incremental Unit for sending information to a CoppeliaCozmoModule.

    Payload should be a dict with the form {str: list}:\n
    {'<command>': [<angle | distance | position>, <speed>, <wait_status>],
    ...,
    '<command>': [<angle | distance | position>, <speed>, <wait_status>]}

    Payloads can contain up to four key-value pairs if all commands are given. Valid commands are:\n
    - 'turn'
    - 'look'
    - 'lift'
    - 'drive'

    Example:\n
    {
        'turn': [Degrees(180), DPS(90), True],\n
        'look': [0.75, Rads(0.5), False],\n
        'lift': [1, DPS(15), False],\n
        'drive': [Millimeters(200), MMPS(50), True]
    }
    """

    @staticmethod
    def type():
        return "CoppeliaCozmoIU"

    def __init__(self, creator=None, iuid=0, previous_iu=None, grounded_in=None, payload: dict[str: list]=None, **kwargs):
        if payload is not None:
            for command in payload.keys():
                if command not in ['turn', 'look', 'lift', 'drive']:
                    raise Exception(f"Invalid key {command}.")
                elif len(payload[command]) < 3:
                    raise Exception("Too few fields in value.")
        else:
            payload = {}

        super().__init__(creator=creator, iuid=iuid, previous_iu=previous_iu, grounded_in=grounded_in, payload=payload)

    def set_turn(self, angle: Angle, speed: AngularSpeed, wait_status: bool=True):
        """Sets the parameters for Cozmo to turn.

        :param angle: A relative angle of rotation of type Angle from coppelia_cozmo_util. Can be Rads or DPS.
        :param speed: The speed at which Cozmo will turn. Can be of type AngularSpeed.
        :param wait_status: A boolean indicating whether the action is blocking or not. Subsequent actions will wait
        until the current one is completed.
        :return:
        """
        self.payload['turn'] = [angle, speed, wait_status]

    def set_look(self, height: float, speed: AngularSpeed, wait_status: bool=True):
        """Sets the parameters for Cozmo to look up or down.

        :param height: How high Cozmo will look. Ranges from 0 to 1, and indicates the percentage of Cozmo's look range.
        :param speed: How fast Cozmo should change its head position.
        :param wait_status: A boolean indicating whether the action is blocking or not. Subsequent actions will wait
        until the current one is completed.
        :return:
        """
        self.payload['look'] = [height, speed, wait_status]

    def set_lift(self, height: float, speed: AngularSpeed, wait_status: bool=True):
        """Sets the parameters for Cozmo to move its lift.

        :param height: How high Cozmo will move its lift. Ranges from 0 to 1, and indicates the percentage of Cozmo's
        lift range.
        :param speed: How fast Cozmo should change its lift position.
        :param wait_status: A boolean indicating whether the action is blocking or not. Subsequent actions will wait
        until the current one is completed.
        :return:
        """
        self.payload['lift'] = [height, speed, wait_status]

    def set_drive(self, distance: Distance, speed: MMPS, wait_status: bool=True):
        """Sets the parameters for Cozmo to drive straight.

        :param distance: How far Cozmo will drive in a straight line. Can be of type Distance from coppelia_cozmo_util.
        :param speed: The speed at which Cozmo will cover the specified distance. Must be of type MMPS.
        :param wait_status: A boolean indicating whether the action is blocking or not. Subsequent actions will wait
        until the current one is completed.
        :return:
        """
        self.payload['drive'] = [distance, speed, wait_status]


class CoppeliaCozmoModule(retico_core.AbstractConsumingModule):
    """A Retico module for controlling a Cozmo robot inside a CoppeliaSim scene."""

    @staticmethod
    def name():
        return "CoppeliaCozmoModule"

    @staticmethod
    def description():
        return "An interfacing module for a CoppeliaSim Cozmo robot"

    @staticmethod
    def input_ius():
        return [CoppeliaCozmoIU]

    @staticmethod
    def output_iu():
        return None

    def __init__(self, cozmo_path, scene, start_scene=False, **kwargs):
        super().__init__(**kwargs)
        self.robot = Cozmo(cozmo_path, scene, start_scene)
        self.queue = []

    def process_update(self, update_message):
        for iu, ut in update_message:
            if ut == retico_core.abstract.UpdateType.ADD:
                self.queue.append(iu)
        self.process_iu()

    def process_iu(self):
        if len(self.queue) == 0: return

        iu = self.queue.pop(0)
        for key, value in iu.payload.items():
            if "turn" == key:
                if value[2]:
                    self.robot.turn_in_place(angle=value[0], speed=value[1]).wait_until_completed()
                else:
                    self.robot.turn_in_place(angle=value[0], speed=value[1])
            elif "look" == key:
                if value[2]:
                    self.robot.set_head_angle(height=value[0], speed=value[1]).wait_until_completed()
                else:
                    self.robot.set_head_angle(height=value[0], speed=value[1])
            elif "lift" == key:
                if value[2]:
                    self.robot.set_lift_height(height=value[0], speed=value[1]).wait_until_completed()
                else:
                    self.robot.set_lift_height(height=value[0], speed=value[1])
            elif "drive" == key:
                if value[2]:
                    self.robot.drive_straight(distance=value[0], speed=value[1]).wait_until_completed()
                else:
                    self.robot.drive_straight(distance=value[0], speed=value[1])

    def shutdown(self):
        self.robot.shutdown()
