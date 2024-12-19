import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import retico_core
from retico_core.text import TextIU


class Cozmo:
    MIN_WHEEL_POS = -np.pi
    MAX_WHEEL_POS = np.pi

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

        # Initialize joint positions
        self._sim.setJointTargetPosition(self._left_motor, 0)
        self._sim.setJointTargetVelocity(self._left_motor, 0)
        self._sim.setJointTargetPosition(self._right_motor, 0)
        self._sim.setJointTargetVelocity(self._right_motor, 0)

        # Virtual joint positions
        self._left_wheel_position = 0
        self._right_wheel_position = 0

        if start_scene:
            print("Starting simulation...")
            self._sim.startSimulation()

    def shutdown(self):
        self._sim.setJointPosition(self._left_motor, 0)
        self._sim.setJointPosition(self._right_motor, 0)

        self._sim.stopSimulation()

    def wait_until_completed(self):
        if self._active_joint is None:
            raise Exception("No action to await.")
        if self._waiting:
            raise Exception("Tried to await an action while already awaiting.")

        self._waiting = True
        sim = RemoteAPIClient().require('sim')
        target = sim.getJointTargetPosition(self._active_joint)
        while self._waiting:
            # print("Diff:", target - sim.getJointPosition(self._active_joint))
            print("Target Position:", target)
            # print("Current Position L:", sim.getJointPosition(self._left_motor))
            print("Current Position R:", sim.getJointPosition(self._right_motor))
            # print("Current Vel L:", sim.getJointVelocity(self._left_motor))
            # print("Current Vel R:", sim.getJointVelocity(self._right_motor))
            if abs(sim.getJointPosition(self._active_joint) - target) < 0.05:
                self._active_joint = None
                self._waiting = False
                print("REACHED TARGET")
                return

    def _update_wheel(self, position, radians: float):
        # position = (position + radians) % (2 * np.pi)
        position += radians
        # if position > 2 * np.pi:
        #     position = position - (2 * np.pi)
        # elif position < 0:
        #     position = (2 * np.pi) + position

        if position > self.MAX_WHEEL_POS:
            position = self.MIN_WHEEL_POS + (position - self.MAX_WHEEL_POS)
        elif position < self.MIN_WHEEL_POS:
            position = self.MAX_WHEEL_POS - abs(position - self.MIN_WHEEL_POS)


        return position

    def turn_in_place(self, radians: float, speed: float):

        sim_left = RemoteAPIClient().require('sim')
        sim_right = RemoteAPIClient().require('sim')

        print("turning from:", sim_right.getJointPosition(self._right_motor))

        # Update virtual joint positions
        self._left_wheel_position = self._update_wheel(self._left_wheel_position, -radians)
        self._right_wheel_position = self._update_wheel(self._right_wheel_position, radians)

        sim_left.setJointTargetVelocity(self._left_motor, -speed)
        sim_right.setJointTargetVelocity(self._right_motor, speed)

        sim_left.setJointTargetPosition(self._left_motor, self._left_wheel_position)
        sim_right.setJointTargetPosition(self._right_motor, self._right_wheel_position)

        # sets active joint for wait_until_completed()
        self._active_joint = self._right_motor

        return self

    def set_head_angle(self, angle: float):
        sim = RemoteAPIClient().require('sim')

        sim.setJointTargetPosition(self._head, angle)

        # sets values for wait_until_completed()
        self._active_joint = self._head

        return self

    def set_lift_height(self, height: float):
        sim = RemoteAPIClient().require('sim')

        sim.setJointTargetPosition(self._lift, height)

        # sets active joint for wait_until_completed()
        self._active_joint = self._lift

        return self

    def drive_straight(self, distance: float, speed: float):
        sim = RemoteAPIClient().require('sim')

        sim.setJointTargetVelocity(self._left_motor, speed)
        sim.setJointTargetVelocity(self._right_motor, speed)
        sim.setJointTargetPosition(self._left_motor, distance)
        sim.setJointTargetPosition(self._right_motor, distance)

        # sets active joint for wait_until_completed()
        self._active_joint = self._right_motor

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
                # self.process_iu(iu)
                self.queue.append(iu)
        self.process_iu()

    def process_iu(self):
        if len(self.queue) == 0: return

        iu = self.queue.pop(0)
        command = iu.payload
        if "turn left" in command:
            self.robot.turn_in_place(radians=np.pi-(np.pi/180), speed=np.pi/180).wait_until_completed()
        if "turn right" in command:
            self.robot.turn_in_place(radians=(-np.pi)+(np.pi/180), speed=np.pi/180).wait_until_completed()
        if "look up" in command:
            self.robot.set_head_angle(angle=self.max_head_angle).wait_until_completed()
        if "look down" in command:
            self.robot.set_head_angle(angle=0).wait_until_completed()
        if "lift up" in command:
            self.robot.set_lift_height(height=self.max_lift_height).wait_until_completed()
        if "lift down" in command:
            self.robot.set_lift_height(height=0).wait_until_completed()
        if "drive forward" in command:
            self.shutdown()
            self.robot.drive_straight(distance=3.14, speed=0.5).wait_until_completed()
        if "drive backward" in command:
            self.shutdown()
            self.robot.drive_straight(distance=-3.14, speed=-0.5).wait_until_completed()

    def shutdown(self):
        self.robot.shutdown()