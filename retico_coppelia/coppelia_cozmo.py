import threading
import time

import retico_core
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from retico_core.text import TextIU
from retico_coppelia.coppelia_cozmo_util import (
    AngleDistance, Speed, Degrees, DegreesPerSecond, Radians, RadiansPerSecond, Millimeters, MillimetersPerSecond
)

class Cozmo:
    class CozmoThread:
        def __init__(self, cozmo, thread):
            self.cozmo = cozmo
            self.thread = thread

        def start(self):
            self.thread.start()

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
            self.thread.join()
            self._unlock()


    MAX_LIFT_POS = MAX_HEAD_POS = 0.785
    MIN_LIFT_POS = MIN_HEAD_POS = 0

    def __init__(self, cozmo_path, scene, start_scene=False):
        self._sim = RemoteAPIClient().require('sim')
        self._sim.loadScene(scene)

        # Simulation lock for blocking/waiting on actions
        self._simlock = False

        # Initialize simulation clients
        self._sims = {}
        for name in ['wheels', 'lift', 'head']:
            self._sims[name] = RemoteAPIClient().require('sim')

        # Simulation joint handles
        self._left_front_motor = self._sim.getObject(cozmo_path + '/left_front_joint')
        self._left_back_motor = self._sim.getObject(cozmo_path + '/left_back_joint')
        self._right_front_motor = self._sim.getObject(cozmo_path + '/right_front_joint')
        self._right_back_motor = self._sim.getObject(cozmo_path + '/right_back_joint')
        self._head = self._sim.getObject(cozmo_path + '/camera_joint')
        self._lift = self._sim.getObject(cozmo_path + '/arm_joint')

        # Initialize joint velocities
        self._sim.setJointTargetVelocity(self._left_front_motor, 0)
        self._sim.setJointTargetVelocity(self._left_back_motor, 0)
        self._sim.setJointTargetVelocity(self._right_front_motor, 0)
        self._sim.setJointTargetVelocity(self._right_back_motor, 0)

        if start_scene:
            print("Starting simulation...")
            self._sim.startSimulation()

    def shutdown(self):
        self._sim.stopSimulation()

    def turn_in_place(self, angle: AngleDistance, speed: Speed):
        while self._simlock: continue

        thread = threading.Thread(
            target=self._turn_in_place,
            args=[
                self._sims['wheels'],
                [
                    self._left_front_motor,
                    self._left_back_motor,
                    self._right_front_motor,
                    self._right_back_motor
                ],
                angle,
                speed
            ]
        )
        thread = self.CozmoThread(self, thread)
        thread.start()

        time.sleep(1)  # TODO remove??

        return thread

    @staticmethod
    def _turn_in_place(sim, joints, angle: AngleDistance, speed: Speed):
        if angle.magnitude == 0: return

        if angle.magnitude > 0:
            # sim.setJointTargetVelocity(joints[0], -speed.rate)
            # sim.setJointTargetVelocity(joints[1], -speed.rate)
            # sim.setJointTargetVelocity(joints[2], speed.rate)
            # sim.setJointTargetVelocity(joints[3], speed.rate)
            sim.setJointTargetVelocity(joints[0], -3.14)
            sim.setJointTargetVelocity(joints[1], -3.14)
            sim.setJointTargetVelocity(joints[2], 3.14)
            sim.setJointTargetVelocity(joints[3], 3.14)
        else:
            # sim.setJointTargetVelocity(joints[0], speed.rate)
            # sim.setJointTargetVelocity(joints[1], speed.rate)
            # sim.setJointTargetVelocity(joints[2], -speed.rate)
            # sim.setJointTargetVelocity(joints[3], -speed.rate)
            sim.setJointTargetVelocity(joints[0], 3.14)
            sim.setJointTargetVelocity(joints[1], 3.14)
            sim.setJointTargetVelocity(joints[2], -3.14)
            sim.setJointTargetVelocity(joints[3], -3.14)

        # Calculate how long to turn joints...
        time.sleep(angle.get_wait_time(speed))

        sim.setJointTargetVelocity(joints[0], 0)
        sim.setJointTargetVelocity(joints[1], 0)
        sim.setJointTargetVelocity(joints[2], 0)
        sim.setJointTargetVelocity(joints[3], 0)

    def set_head_angle(self, angle: AngleDistance, speed: Speed):
        while self._simlock: continue

        thread = threading.Thread(
            target=self._set_head_angle,
            args=[
                self._sims['head'],
                self._head,
                angle,
                speed
            ]
        )
        thread = self.CozmoThread(self, thread)
        thread.start()

        return thread

    @staticmethod
    def _set_head_angle(sim, joint, angle: AngleDistance, speed: Speed):
        sim.setJointTargetPosition(joint, angle.magnitude)  # TODO utilize speed for movement

        # Calculate time for joint to move and wait before terminating
        time.sleep(angle.get_wait_time(speed))

    def set_lift_height(self, height: float, speed: Speed):
        while self._simlock: continue

        thread = threading.Thread(
            target=self._set_lift_height,
            args=[
                self._sims['lift'],
                self._lift,
                height,
                speed
            ]
        )
        thread = self.CozmoThread(self, thread)
        thread.start()

        return thread

    @staticmethod
    def _set_lift_height(sim, joint, height: float, speed: Speed):
        if height < Cozmo.MIN_LIFT_POS:
            height = 0
        elif height > Cozmo.MAX_LIFT_POS:
            height = Cozmo.MAX_LIFT_POS

        sim.setJointTargetPosition(joint, height)  # TODO utilize speed for movement

        # Calculate time for joint to move and wait before terminating
        angle = Radians(height)
        time.sleep(angle.magnitude / speed.rate)

    def drive_straight(self, distance: AngleDistance, speed: Speed):
        while self._simlock: continue

        thread = threading.Thread(
            target=self._drive_straight,
            args=[
                self._sims['wheels'],
                [self._left_motor, self._right_motor],
                distance,
                speed
            ]
        )
        thread = self.CozmoThread(self, thread)
        thread.start()

        return thread

    @staticmethod
    def _drive_straight(sim, joints, distance: AngleDistance, speed: Speed):
        sim.setJointTargetVelocity(joints[0], speed.rate)
        sim.setJointTargetVelocity(joints[1], speed.rate)

        # Calculate how long to turn joints...
        time.sleep(distance.get_wait_time(speed))

        sim.setJointTargetVelocity(joints[0], 0)
        sim.setJointTargetVelocity(joints[1], 0)


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
                # self.process_iu(iu)
                self.queue.append(iu)
        self.process_iu()

    def process_iu(self):
        if len(self.queue) == 0: return

        iu = self.queue.pop(0)
        command = iu.payload
        if "turn left" in command:
            self.robot.turn_in_place(angle=Degrees(15), speed=DegreesPerSecond(3.14)).wait_until_completed()
        if "turn right" in command:
            self.robot.turn_in_place(angle=Degrees(-15), speed=DegreesPerSecond(3.14)).wait_until_completed()
        if "look up" in command:
            self.robot.set_head_angle(angle=Radians(self.max_head_angle), speed=RadiansPerSecond(1)).wait_until_completed()
        if "look down" in command:
            self.robot.set_head_angle(angle=Radians(0), speed=RadiansPerSecond(1)).wait_until_completed()
        if "lift up" in command:
            self.robot.set_lift_height(height=self.max_lift_height, speed=RadiansPerSecond(1)).wait_until_completed()
        if "lift down" in command:
            self.robot.set_lift_height(height=0, speed=RadiansPerSecond(1)).wait_until_completed()
        if "drive forward" in command:
            self.robot.drive_straight(distance=Millimeters(50), speed=MillimetersPerSecond(50)).wait_until_completed()
        if "drive backward" in command:
            self.robot.drive_straight(distance=Millimeters(-50), speed=MillimetersPerSecond(50)).wait_until_completed()

    def shutdown(self):
        self.robot.shutdown()