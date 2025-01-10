import threading
import time
import numpy as np
import retico_core
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from retico_core.text import TextIU

from retico_coppelia.coppelia_cozmo_util import (
    Angle, Distance, Speed, Degrees, DegreesPerSecond, Radians, RadiansPerSecond, Millimeters, MillimetersPerSecond
)

class Cozmo:
    class CozmoThread:
        def __init__(self, cozmo, thread=None, threads=None):
            if thread is None and threads is None:
                raise Exception("Must provide either thread or threads")
            if thread is not None and threads is not None:
                raise Exception("Cannot provide both thread and threads")

            self.cozmo = cozmo
            self.thread = thread
            self.threads = threads


        def start(self):
            if self.thread is not None:
                self.thread.start()
                self.thread.join()
            else:
                for thread in self.threads.values():
                    thread.start()
                    thread.join()

        def _lock(self):
            if self.cozmo._simlock:
                raise Exception("Fatal error: multiple sources tried to use the sim lock.")

            self.cozmo._simlock = True

        def _unlock(self):
            if not self.cozmo._simlock:
                raise Exception("Fatal error: something unexpectedly tampered with the sim lock.")

            self.cozmo._simlock = False

        def wait_until_completed(self):
            self._lock()

            if self.thread is not None:
                while self.thread.is_alive(): continue
            else:
                while any(thread.is_alive() for thread in self.threads.values()): continue
            self._unlock()

    MAX_LIFT_POS = MAX_HEAD_POS = 0.785
    MIN_LIFT_POS = MIN_HEAD_POS = 0

    def __init__(self, cozmo_path, scene, start_scene=False):
        self.start_scene = start_scene

        self._sim = RemoteAPIClient().require('sim')
        if self.start_scene:
            self._sim.loadScene(scene)

        # Simulation lock for blocking/waiting on actions
        self._simlock = False

        # Initialize scene script
        self._script_handle = self._sim.getScript(self._sim.scripttype_simulation, cozmo_path + '/Script')
        self._sim.initScript(self._script_handle)

        # Initialize simulation clients
        self._sims = {}
        for name in ['wheels', 'lift', 'head']:
            self._sims[name] = RemoteAPIClient().require('sim')

        self._joint_handles = {}
        for name in ['left_front', 'left_back', 'right_front', 'right_back']:
            self._joint_handles[name] = self._sim.getObject(cozmo_path + '/' + name + '_joint')
        self._joint_handles['head'] = self._sim.getObject(cozmo_path + '/camera_joint')
        self._joint_handles['lift'] = self._sim.getObject(cozmo_path + '/arm_joint')

        if self.start_scene:
            print("Starting simulation...")
            self._sim.startSimulation()

    def shutdown(self):
        if self.start_scene:
            print("Stopping simulation...")
            self._sim.stopSimulation()

    def wait_until_completed(self):
        while self._sim.callScriptFunction("is_moving", self._script_handle):
            time.sleep(0.1)

    def turn_in_place(self, angle: Angle, speed: Speed):
        while self._simlock: continue
        self._sim.callScriptFunction("turn_in_place", self._script_handle, angle.to_radians(), speed.to_rads())

        return self

    def set_head_angle(self, angle: float, speed: Speed):
        while self._simlock: continue
        self._sim.callScriptFunction("set_head_angle", self._script_handle, angle, speed.to_rads())

        return self

    def set_lift_height(self, height: float, speed: Speed):
        while self._simlock: continue
        self._sim.callScriptFunction("set_lift_height", self._script_handle, height, speed.to_rads())

        return self

    def drive_straight(self, distance: Distance, speed: Speed):
        while self._simlock: continue
        self._sim.callScriptFunction("drive_straight", self._script_handle, distance.to_millimeters(), speed.to_mmps())

        return self

class CoppeliaCozmoRobot(retico_core.AbstractModule):
    @staticmethod
    def name():
        return "CoppeliaCozmoRobot"

    @staticmethod
    def description():
        return "An interfacing module for a CoppeliaSim Cozmo robot"

    @staticmethod
    def input_ius():
        return [TextIU]

    @staticmethod
    def output_iu():
        return None

    def __init__(self, cozmo_path, scene, start_scene=False, **kwargs):
        super().__init__(**kwargs)

        self.robot = Cozmo(cozmo_path, scene, start_scene)
        self.max_head_angle = 0.785
        self.max_lift_height = 0.785

        self.queue = []

    def process_update(self, update_message):
        for iu, ut in update_message:
            if ut == retico_core.abstract.UpdateType.ADD:
                self.queue.append(iu)
        self.process_iu()

    def process_iu(self):
        if len(self.queue) == 0: return

        iu = self.queue.pop(0)
        command = iu.payload
        if "turn left" in command:
            self.robot.turn_in_place(angle=Radians(2 * np.pi), speed=RadiansPerSecond(np.pi / 2)).wait_until_completed()
        if "turn right" in command:
            self.robot.turn_in_place(angle=Radians(2 * -np.pi), speed=RadiansPerSecond(np.pi / 2)).wait_until_completed()
        if "look up" in command:
            self.robot.set_head_angle(angle=self.max_head_angle, speed=RadiansPerSecond(1)).wait_until_completed()
        if "look down" in command:
            self.robot.set_head_angle(angle=0, speed=RadiansPerSecond(1)).wait_until_completed()
        if "lift up" in command:
            self.robot.set_lift_height(height=self.max_lift_height, speed=RadiansPerSecond(0.2)).wait_until_completed()
        if "lift down" in command:
            self.robot.set_lift_height(height=0, speed=RadiansPerSecond(1)).wait_until_completed()
        if "drive forward" in command:
            self.robot.drive_straight(distance=Millimeters(150), speed=MillimetersPerSecond(50)).wait_until_completed()
        if "drive backward" in command:
            self.robot.drive_straight(distance=Millimeters(-150), speed=MillimetersPerSecond(50)).wait_until_completed()

    def shutdown(self):
        self.robot.shutdown()