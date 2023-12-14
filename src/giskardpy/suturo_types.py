from enum import unique, Enum


@unique
class GripperTypes(Enum):
    OPEN = 'open'
    CLOSE = 'close'
    NEUTRAL = 'neutral'
