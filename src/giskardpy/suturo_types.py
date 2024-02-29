from enum import unique, Enum

# States for the HSRs gripper, is being used only in old_python_interface as of now
@unique
class GripperTypes(Enum):
    OPEN = 'open'
    CLOSE = 'close'
    NEUTRAL = 'neutral'


# List of Methods that need Thresholds for force_monitor: GraspObjectCarefully(might include Doors?), Placing,
# TODO: Create Enums for every Force and/or Torque sensitive action, to be used in force_monitor.py
@unique
class ForceTorqueThresholds(Enum):
    FT_GraspWithCare = 'FTGraspC'
    FT_Placing = 'FTPlace'
    FT_Door = 'FTDoor'

