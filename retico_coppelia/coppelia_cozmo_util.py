from abc import ABC, abstractmethod
import numpy as np


class Speed:
    def __init__(self, rate):
        if rate < 0:
            raise Exception("Can't pass negative value for speed.")

        self.rate = rate


class DegreesPerSecond(Speed):
    def __init__(self, degs):
        super().__init__(degs)

    def to_rads(self):
        if __name__ == '__main__':
            return (np.pi / 180) * self.rate

    def from_rads(self, rads):
        raise NotImplementedError  # TODO


class RadiansPerSecond(Speed):
    def __init__(self, rads):
        super().__init__(rads)

    def from_degs(self, degs):
        raise NotImplementedError  # TODO

    def from_mms(self, mms):
        raise NotImplementedError  # TODO


class MillimetersPerSecond(Speed):
    def __init__(self, mms):
        super().__init__(mms)

    def from_degs(self, degs):
        raise NotImplementedError  # TODO

    def from_rads(self, rads):
        raise NotImplementedError  # TODO


class AngleDistance(ABC):
    def __init__(self, magnitude):
        self.magnitude = magnitude

    @abstractmethod
    def get_wait_time(self, angular_speed, joint_radius):
        raise NotImplementedError  # TODO


class Degrees(AngleDistance):
    def __init__(self, magnitude):
        super().__init__(magnitude)

    def from_radians(self, radians):
        self.magnitude = (180 / np.pi) * radians

    def to_radians(self):
        return (np.pi / 180) * self.magnitude

    def get_wait_time(self, angular_speed: DegreesPerSecond, joint_radius: float):
        return abs(self.magnitude / (joint_radius * angular_speed.rate))


class Radians(AngleDistance):
    def __init__(self, magnitude):
        super().__init__(magnitude)

    def from_degrees(self, degrees):
        self.magnitude = (np.pi / 180) * degrees

    def get_wait_time(self, angular_speed: RadiansPerSecond, joint_radius: float):
        return abs(self.magnitude / (joint_radius * angular_speed.rate))


class Millimeters(AngleDistance):
    def __init__(self, magnitude):
        super().__init__(magnitude)

    def get_wait_time(self, angular_speed: MillimetersPerSecond, joint_radius: float):
        raise NotImplementedError  # TODO
