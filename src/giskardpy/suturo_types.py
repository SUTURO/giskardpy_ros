from enum import StrEnum, unique


@unique
class gripper_types(StrEnum):
    OPEN = 'open'
    CLOSE = 'close'
    NEUTRAL = 'neutral'
