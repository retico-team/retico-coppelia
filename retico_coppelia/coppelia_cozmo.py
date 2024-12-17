import threading
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
        self.sim = RemoteAPIClient().require('sim')
        self.sim.loadScene(scene)

        # Simulation joint handles
        self._left_motor = self.sim.getObject(cozmo_path + '/left_wheel_joint')
        self._right_motor = self.sim.getObject(cozmo_path + '/right_wheel_joint')
        self._head = self.sim.getObject(cozmo_path + '/camera_joint')
        self._lift = self.sim.getObject(cozmo_path + '/arm_joint')

        if start_scene:
            print("Starting simulation...")
            self.sim.startSimulation()

    def turn_in_place(self, angle: float, speed: float):
        def func():
            sim = RemoteAPIClient().require('sim')
            # sim.setJointTargetVelocity(self._left_motor, speed)
            # sim.setJointTargetVelocity(self._right_motor, speed)
            sim.setJointTargetPosition(self._left_motor, -angle)
            sim.setJointTargetPosition(self._right_motor, angle)
        thread = self.CozmoThread(target=func)
        thread.start()
        return thread

    def set_head_angle(self, angle: float):
        def func():
            sim = RemoteAPIClient().require('sim')
            sim.setJointTargetPosition(self._head, angle)
        thread = self.CozmoThread(target=func)
        thread.start()
        return thread

    def set_lift_height(self, height: float):
        def func():
            sim = RemoteAPIClient().require('sim')
            sim.setJointTargetPosition(self._lift, height)
        thread = self.CozmoThread(target=func)
        thread.start()
        return thread

    def drive_straight(self, distance: float, speed: float):
        def func():
            sim = RemoteAPIClient().require('sim')
            # sim.setJointTargetVelocity(self._left_motor, speed)
            # sim.setJointTargetVelocity(self._right_motor, speed)
            sim.setJointTargetPosition(self._left_motor, distance)  # TODO convert distance to a joint position
            sim.setJointTargetPosition(self._right_motor, distance)  # TODO convert distance to a joint position

        thread = self.CozmoThread(target=func)
        thread.start()
        return thread


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
        self.head_angle = 15

    def process_update(self, update_message):
        for iu, ut in update_message:
            if ut == retico_core.abstract.UpdateType.ADD:
                self.process_iu(iu)

    def process_iu(self, iu):
        command = iu.payload
        if "turn left" in command:
            self.robot.turn_in_place(angle=15, speed=3).wait_until_completed()
            # self.robot.turn_in_place(angle=15, speed=3)
        if "turn right" in command:
            self.robot.turn_in_place(angle=-15, speed=-3).wait_until_completed()
            # self.robot.turn_in_place(angle=-15, speed=-3)
        if "look up" in command:
            self.head_angle = abs(self.head_angle)
            self.robot.set_head_angle(angle=self.head_angle).wait_until_completed()
            # self.robot.set_head_angle(angle=self.head_angle)
        if "look down" in command:
            self.head_angle = -abs(self.head_angle)
            self.robot.set_head_angle(angle=self.head_angle).wait_until_completed()
            # self.robot.set_head_angle(angle=self.head_angle)
        if "lift up" in command:
            self.robot.set_lift_height(height=1).wait_until_completed()
            # self.robot.set_lift_height(height=1)
        if "lift down" in command:
            self.robot.set_lift_height(height=0).wait_until_completed()
            # self.robot.set_lift_height(height=0)
        if "drive forward" in command:
            self.shutdown()
            self.robot.drive_straight(distance=15, speed=3).wait_until_completed()
            # self.robot.drive_straight(distance=15, speed=3)
        if "drive backward" in command:
            self.shutdown()
            self.robot.drive_straight(distance=-15, speed=-3).wait_until_completed()
            # self.robot.drive_straight(distance=-15, speed=-3)

