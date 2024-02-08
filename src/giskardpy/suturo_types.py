from enum import unique, Enum


@unique
class GripperTypes(Enum):
    OPEN = 'open'
    CLOSE = 'close'
    NEUTRAL = 'neutral'


# List of Methods that need Thresholds for force_monitor: GraspObjectCarefully, Placing,
# TODO: Create Enums for every Force and/or Torque sensitive action, to be used in force_monitor.py
class ForceTorqueThresholds(Enum):
    FT_GraspWithCare = 'FTGraspC'
    FT_Placing = 'FTPlace'
    FT_Door = 'FTDoor'

