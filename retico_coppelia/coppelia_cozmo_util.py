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


class CozmoStateIU(retico_core.abstract.IncrementalUnit):
    """Attributes:

    :param creator (AbstractModule): The module that created this IU
    :param previous_iu (IncrementalUnit): A link to the IU created before the
        current one.
    :param grounded_in (IncrementalUnit): A link to the IU this IU is based on.
    :param created_at (float): The UNIX timestamp of the moment the IU is created.
    :param state (dict): The state of the robot
    """

    @staticmethod
    def type():
        return "Cozmo State IU"

    def __init__(self, creator=None, iuid=0, previous_iu=None, grounded_in=None, state=None,
                 **kwargs):
        super().__init__(creator=creator, iuid=iuid, previous_iu=previous_iu,
                         grounded_in=grounded_in, payload=state)
        self.payload = state

    def set_state(self, state):
        """Sets the state of the robot"""
        self.payload = state