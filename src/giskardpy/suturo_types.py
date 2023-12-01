from enum import unique, Enum


@unique
class gripper_types(Enum):
    OPEN = 'open'
    CLOSE = 'close'
    NEUTRAL = 'neutral'
