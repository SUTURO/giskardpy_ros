from enum import unique, Enum


@unique
class GripperTypes(Enum):
    OPEN = 'open'
    CLOSE = 'close'
    NEUTRAL = 'neutral'


# TODO: Create Enums for every Force and/or Torque sensitive action, to be used in force_monitor.py
class ForceThresholds(Enum):
    NONE = 'none'


class TorqueThresholds(Enum):
    NONE = 'none'
