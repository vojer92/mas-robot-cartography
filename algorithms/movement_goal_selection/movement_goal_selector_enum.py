from enum import Enum, auto

class MovementGoalSelectorEnum(Enum):
    NEAREST_BIGGEST_FRONTIER = auto()
    # more movement-goal-finding-algorithms