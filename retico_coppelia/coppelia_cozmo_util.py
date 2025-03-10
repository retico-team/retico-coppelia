from abc import ABC, abstractmethod
import numpy as np
import retico_core


class AngularSpeed(ABC):
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


class DPS(AngularSpeed):
    def __init__(self, rate):
        super().__init__(rate)

    def to_rads(self):
        return np.radians(self.rate)

    def to_dps(self):
        return self.rate


class Rads(AngularSpeed):
    def __init__(self, rate):
        super().__init__(rate)

    def to_rads(self):
        return self.rate

    def to_dps(self):
        return np.degrees(self.rate)


class MMPS:
    def __init__(self, rate):
        self.rate = rate

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
        return np.radians(self.magnitude)

    def to_degrees(self):
        return self.magnitude


class Radians(Angle):
    def __init__(self, magnitude):
        super().__init__(magnitude)

    def to_radians(self):
        return self.magnitude

    def to_degrees(self):
        return np.degrees(self.magnitude)


class Distance(ABC):
    def __init__(self, magnitude):
        self.magnitude = magnitude

    @abstractmethod
    def to_mm(self):
        raise NotImplementedError


class Millimeters(Distance):
    def __init__(self, magnitude):
        super().__init__(magnitude)

    def to_mm(self):
        return self.magnitude
