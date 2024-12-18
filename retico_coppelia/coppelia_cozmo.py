import threading
import time

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import retico_core
from retico_core.text import TextIU


class Cozmo:
    class CozmoThread:
        def __init__(self, target):
            self._thread = threading.Thread(target=target)

        def start(self):
            self._thread.start()

        def wait_until_completed(self):
            self._thread.join()


    def __init__(self, cozmo_path, scene, start_scene=False):
        self._sim = RemoteAPIClient().require('sim')
        self._sim.loadScene(scene)

        self._active_joint = None
        self._waiting = False

        # Simulation joint handles
        self._left_motor = self._sim.getObject(cozmo_path + '/left_wheel_joint')
        self._right_motor = self._sim.getObject(cozmo_path + '/right_wheel_joint')
        self._head = self._sim.getObject(cozmo_path + '/camera_joint')
        self._lift = self._sim.getObject(cozmo_path + '/arm_joint')

        # Get initial joint position values
        self._left_wheel_radians = self._sim.getJointPosition(self._left_motor)
        self._right_wheel_radians = self._sim.getJointPosition(self._right_motor)

        if start_scene:
            print("Starting simulation...")
            self._sim.startSimulation()

    def wait_until_completed(self):
        if self._active_joint is None:
            raise Exception("No action to await.")
        if self._waiting:
            raise Exception("Tried to await an action while already awaiting.")

        self._waiting = True
        def func():
            sim = RemoteAPIClient().require('sim')
            target = sim.getJointTargetPosition(self._active_joint)
            while self._waiting:
                if abs(sim.getJointPosition(self._active_joint) - target) < 0.05:
                    self._active_joint = None
                    self._waiting = False
                    return
                print("Diff:", target - sim.getJointPosition(self._active_joint))
                print("Target Position:", target)
                print("Current Position:", sim.getJointPosition(self._active_joint))
                time.sleep(0.5)

        thread = threading.Thread(target=func)
        thread.start()
        thread.join()


    def turn_in_place(self, radians: float, speed: float):
        # def func():
        sim = RemoteAPIClient().require('sim')

        self._left_wheel_radians -= radians
        self._right_wheel_radians += radians


        # sets active joint for wait_until_completed()
        self._active_joint = self._right_motor

        # sim.setJointTargetVelocity(self._left_motor, speed)
        # sim.setJointTargetVelocity(self._right_motor, speed)
        sim.setJointTargetPosition(self._left_motor, self._left_wheel_radians)
        sim.setJointTargetPosition(self._right_motor, self._right_wheel_radians)
        # thread = self.CozmoThread(target=func)
        # thread.start()
        # return thread
        return self

    def set_head_angle(self, angle: float):
        # def func():
        sim = RemoteAPIClient().require('sim')

        # sets values for wait_until_completed()
        self._active_joint = self._head

        sim.setJointTargetPosition(self._head, angle)
        # thread = self.CozmoThread(target=func)
        # thread.start()
        # return thread
        return self

    def set_lift_height(self, height: float):
        # def func():
        sim = RemoteAPIClient().require('sim')

        # sets active joint for wait_until_completed()
        self._active_joint = self._lift

        sim.setJointTargetPosition(self._lift, height)
        # thread = self.CozmoThread(target=func)
        # thread.start()
        # return thread
        return self

    def drive_straight(self, distance: float, speed: float):
        # def func():
        sim = RemoteAPIClient().require('sim')

        # subtract distance instead of add to handle polarity so it works with CoppeliaSim Cozmo rigging
        self._left_wheel_radians -= distance  # TODO convert distance to a joint position / degrees
        self._right_wheel_radians -= distance  # TODO convert distance to a joint position / degrees

        # sim.setJointTargetVelocity(self._left_motor, speed)
        # sim.setJointTargetVelocity(self._right_motor, speed)

        # sets active joint for wait_until_completed()
        self._active_joint = self._right_motor

        sim.setJointTargetPosition(self._left_motor, self._left_wheel_radians)
        sim.setJointTargetPosition(self._right_motor, self._right_wheel_radians)

        # thread = self.CozmoThread(target=func)
        # thread.start()
        # return thread
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

    def process_update(self, update_message):
        for iu, ut in update_message:
            if ut == retico_core.abstract.UpdateType.ADD:
                self.process_iu(iu)

    def process_iu(self, iu):
        command = iu.payload
        if "turn left" in command:
            self.robot.turn_in_place(radians=3.14, speed=3).wait_until_completed()
            # self.robot.turn_in_place(degrees=15, speed=3)
        if "turn right" in command:
            self.robot.turn_in_place(radians=-3.14, speed=-3).wait_until_completed()
            # self.robot.turn_in_place(degrees=-15, speed=-3)
        if "look up" in command:
            self.robot.set_head_angle(angle=self.max_head_angle).wait_until_completed()
            # self.robot.set_head_angle(angle=self.head_angle)
        if "look down" in command:
            self.robot.set_head_angle(angle=0).wait_until_completed()
            # self.robot.set_head_angle(angle=self.head_angle)
        if "lift up" in command:
            self.robot.set_lift_height(height=self.max_lift_height).wait_until_completed()
            # self.robot.set_lift_height(height=1)
        if "lift down" in command:
            self.robot.set_lift_height(height=0).wait_until_completed()
            # self.robot.set_lift_height(height=0)
        if "drive forward" in command:
            self.shutdown()
            self.robot.drive_straight(distance=3.14, speed=3).wait_until_completed()
            # self.robot.drive_straight(distance=200, speed=3)
        if "drive backward" in command:
            self.shutdown()
            self.robot.drive_straight(distance=-3.14, speed=-3).wait_until_completed()
            # self.robot.drive_straight(distance=-200, speed=-3)

