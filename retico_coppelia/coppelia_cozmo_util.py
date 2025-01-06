from abc import ABC, abstractmethod
import numpy as np


class Speed(ABC):
    def __init__(self, rate):
        if rate < 0:
            raise Exception("Can't pass negative value for speed.")

        self.rate = rate

    @abstractmethod
    def to_rads(self):
        raise NotImplementedError


class DegreesPerSecond(Speed):
    def __init__(self, degs):
        super().__init__(degs)

    def to_rads(self):
        # return (np.pi / 180) * self.rate
        return np.radians(self.rate)


class RadiansPerSecond(Speed):
    def __init__(self, rads):
        super().__init__(rads)

    def to_rads(self):
        return self.rate


class MillimetersPerSecond(Speed):
    def __init__(self, mms):
        super().__init__(mms)

    def to_rads(self):
        raise NotImplementedError  # TODO


class AngleDistance(ABC):
    def __init__(self, magnitude):
        self.magnitude = magnitude

    @abstractmethod
    def get_wait_time(self, speed: Speed):
        raise NotImplementedError

    @abstractmethod
    def to_radians(self):
        raise NotImplementedError


class Degrees(AngleDistance):
    def __init__(self, magnitude):
        super().__init__(magnitude)

    def get_wait_time(self, speed: Speed):
        return abs(self.to_radians() / speed.to_rads())

    def to_radians(self):
        # return (np.pi / 180) * self.magnitude
        return np.degrees(self.magnitude)


class Radians(AngleDistance):
    def __init__(self, magnitude):
        super().__init__(magnitude)

    def get_wait_time(self, speed: Speed):
        return abs(self.magnitude / speed.to_rads())

    def to_radians(self):
        return self.magnitude


class Millimeters(AngleDistance):
    def __init__(self, magnitude):
        super().__init__(magnitude)

    def get_wait_time(self, speed: Speed):
        raise NotImplementedError  # TODO
