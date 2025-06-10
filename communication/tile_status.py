from enum import Enum, auto


class TileStatus(Enum):
    UNKNOWN = auto()
    FREE = auto()
    OBSTACLE = auto()
