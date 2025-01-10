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

    @abstractmethod
    def to_dps(self):
        raise NotImplementedError

    @abstractmethod
    def to_mmps(self):
        raise NotImplementedError


class DegreesPerSecond(Speed):
    def __init__(self, degs):
        super().__init__(degs)

    def to_rads(self):
        # return (np.pi / 180) * self.rate
        return np.radians(self.rate)

    def to_dps(self):
        raise NotImplementedError

    def to_mmps(self):
        raise NotImplementedError


class RadiansPerSecond(Speed):
    def __init__(self, rads):
        super().__init__(rads)

    def to_rads(self):
        return self.rate

    def to_dps(self):
        raise NotImplementedError

    def to_mmps(self):
        raise NotImplementedError


class MillimetersPerSecond(Speed):
    def __init__(self, mms):
        super().__init__(mms)

    def to_rads(self):
        raise NotImplementedError

    def to_dps(self):
        raise NotImplementedError

    def to_mmps(self):
        return self.rate


class Angle(ABC):
    def __init__(self, magnitude):
        self.magnitude = magnitude

    @abstractmethod
    def to_radians(self):
        raise NotImplementedError


class Degrees(Angle):
    def __init__(self, magnitude):
        super().__init__(magnitude)

    def to_radians(self):
        # return (np.pi / 180) * self.magnitude
        return np.degrees(self.magnitude)


class Radians(Angle):
    def __init__(self, magnitude):
        super().__init__(magnitude)

    def to_radians(self):
        return self.magnitude


class Distance(ABC):
    def __init__(self, magnitude):
        self.magnitude = magnitude

    @abstractmethod
    def to_millimeters(self):
        raise NotImplementedError


class Millimeters(Distance):
    def __init__(self, magnitude):
        super().__init__(magnitude)

    def to_millimeters(self):
        return self.magnitude
