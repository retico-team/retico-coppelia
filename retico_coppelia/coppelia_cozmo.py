import threading
import time
import numpy as np
import retico_core
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from retico_core.text import TextIU

from retico_coppelia.coppelia_cozmo_util import (
    AngleDistance, Speed, Degrees, DegreesPerSecond, Radians, RadiansPerSecond
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


    FRONT_WHEEL_RADIUS = 0.165  # 0.165
    BACK_WHEEL_RADIUS = 0.125  # 0.125
    WHEELBASE_RADIUS = 0.247  # 0.243
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

        # Initialize joint velocities
        self._sim.setJointTargetVelocity(self._joint_handles['left_front'], 0)
        self._sim.setJointTargetVelocity(self._joint_handles['left_back'], 0)
        self._sim.setJointTargetVelocity(self._joint_handles['right_front'], 0)
        self._sim.setJointTargetVelocity(self._joint_handles['right_back'], 0)

        if self.start_scene:
            print("Starting simulation...")
            self._sim.startSimulation()

    def shutdown(self):
        if self.start_scene:
            print("Stopping simulation...")
            self._sim.stopSimulation()

    def wait_until_completed(self):
        while self._sim.callScriptFunction("is_moving", self._script_handle):
            print("Waiting...")
            time.sleep(0.1)
        print("Completed.")

    def turn_in_place(self, angle: AngleDistance, speed: Speed):
        while self._simlock: continue
        self._sim.callScriptFunction("turn_in_place", self._script_handle, angle.to_radians(), speed.to_rads())

        return self


    @staticmethod
    def _turn_in_place(sim, joints, angle: AngleDistance, speed: Speed, radi: list):
        if angle.magnitude == 0: return

        # # Calculate velocity for front and rear wheels
        angular_vel_front = 2 * (speed.to_rads() * radi[0]) / radi[1]
        angular_vel_back = 2 * (speed.to_rads() * radi[0]) / radi[2]

        # # Calculate time until joint has fully rotated
        t = angle.get_wait_time(speed)
        # print(f"Wait time: {t}")

        end_pos_left = sim.getJointPosition(joints[0]) + angle.to_radians()
        if end_pos_left > np.pi:
            end_pos_left -= 2 * np.pi
        elif end_pos_left < -np.pi:
            end_pos_left += 2 * np.pi

        end_pos_right = sim.getJointPosition(joints[2]) - angle.to_radians()
        if end_pos_right > np.pi:
            end_pos_right -= 2 * np.pi
        elif end_pos_right < -np.pi:
            end_pos_right += 2 * np.pi

        # # Calculate velocity for front and rear wheels
        angular_vel_front = 2 * (speed.to_rads() * radi[0]) / radi[1]
        angular_vel_back = 2 * (speed.to_rads() * radi[0]) / radi[2]

        # # Calculate time until joint has fully rotated
        t = angle.get_wait_time(speed)
        # print(f"Wait time: {t}")

        if angle.magnitude > 0:
            sim.setJointTargetVelocity(joints[0], -angular_vel_front)
            sim.setJointTargetVelocity(joints[1], -angular_vel_back)
            sim.setJointTargetVelocity(joints[2], angular_vel_front)
            sim.setJointTargetVelocity(joints[3], angular_vel_back)
        else:
            sim.setJointTargetVelocity(joints[0], angular_vel_front)
            sim.setJointTargetVelocity(joints[1], angular_vel_back)
            sim.setJointTargetVelocity(joints[2], -angular_vel_front)
            sim.setJointTargetVelocity(joints[3], -angular_vel_back)

        # # Wait for joint to rotate
        time.sleep(t)

        sim.setJointTargetVelocity(joints[0], 0)
        sim.setJointTargetVelocity(joints[1], 0)
        sim.setJointTargetVelocity(joints[2], 0)
        sim.setJointTargetVelocity(joints[3], 0)

        # Correct the position
        sim.setJointPosition(joints[0], end_pos_left)
        sim.setJointPosition(joints[1], end_pos_left)
        sim.setJointPosition(joints[2], end_pos_right)
        sim.setJointPosition(joints[3], end_pos_right)

    def set_head_angle(self, angle: AngleDistance, speed: Speed):
        while self._simlock: continue

        thread = threading.Thread(
            target=self._set_head_angle,
            args=[
                self._sims['head'],
                self._head,
                angle,
                speed,
                self.FRONT_LEFT_RADIUS
            ]
        )
        thread.daemon = True
        thread = self.CozmoThread(self, thread)
        thread.start()

        return thread

    @staticmethod
    def _set_head_angle(sim, joint, angle: AngleDistance, speed: Speed, radius: float):
        sim.setJointTargetPosition(joint, angle.magnitude)  # TODO utilize speed for movement

        # Calculate time for joint to move and wait before terminating
        time.sleep(angle.get_wait_time(speed, radius))

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
        thread.daemon = True
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
                [
                    self._left_front_motor,
                    self._left_back_motor,
                    self._right_front_motor,
                    self._right_back_motor
                ],
                distance,
                speed,
                self.FRONT_LEFT_RADIUS
            ]
        )
        thread.daemon = True
        thread = self.CozmoThread(self, thread)
        thread.start()

        return thread

    @staticmethod
    def _drive_straight(sim, joints, distance: AngleDistance, speed: Speed, radius: float):
        if distance.magnitude > 0:
            # trigger front wheel velocities before back wheel velocities
            sim.setJointTargetVelocity(joints[0], speed.to_rads() / radius)
            sim.setJointTargetVelocity(joints[1], speed.to_rads() / radius)
            sim.setJointTargetVelocity(joints[2], speed.to_rads() / radius)
            sim.setJointTargetVelocity(joints[3], speed.to_rads() / radius)
        else:
            # trigger back wheel velocities before front wheel velocities
            sim.setJointTargetVelocity(joints[3], -speed.to_rads() / radius)
            sim.setJointTargetVelocity(joints[2], -speed.to_rads() / radius)
            sim.setJointTargetVelocity(joints[1], -speed.to_rads() / radius)
            sim.setJointTargetVelocity(joints[0], -speed.to_rads() / radius)

        # Calculate how long to turn joints...
        time.sleep(distance.get_wait_time(speed))

        sim.setJointTargetVelocity(joints[0], 0)
        sim.setJointTargetVelocity(joints[1], 0)
        sim.setJointTargetVelocity(joints[2], 0)
        sim.setJointTargetVelocity(joints[3], 0)


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
            self.robot.turn_in_place(angle=Radians(3 * np.pi), speed=RadiansPerSecond(np.pi)).wait_until_completed()
            # self.robot.turn_in_place(angle=Radians(2 * np.pi), speed=RadiansPerSecond(np.pi/2))
        if "turn right" in command:
            self.robot.turn_in_place(angle=Radians(3 * -np.pi), speed=RadiansPerSecond(np.pi)).wait_until_completed()
            # self.robot.turn_in_place(angle=Radians(2 * -np.pi), speed=RadiansPerSecond(np.pi/2))
        if "look up" in command:
            self.robot.set_head_angle(angle=Radians(self.max_head_angle), speed=RadiansPerSecond(1)).wait_until_completed()
        if "look down" in command:
            self.robot.set_head_angle(angle=Radians(0), speed=RadiansPerSecond(1)).wait_until_completed()
        if "lift up" in command:
            self.robot.set_lift_height(height=self.max_lift_height, speed=RadiansPerSecond(1)).wait_until_completed()
        if "lift down" in command:
            self.robot.set_lift_height(height=0, speed=RadiansPerSecond(1)).wait_until_completed()
        if "drive forward" in command:
            self.robot.drive_straight(distance=Degrees(90), speed=DegreesPerSecond(90)).wait_until_completed()
        if "drive backward" in command:
            self.robot.drive_straight(distance=Degrees(-90), speed=DegreesPerSecond(90)).wait_until_completed()

    def shutdown(self):
        self.robot.shutdown()