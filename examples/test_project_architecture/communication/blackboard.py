from dataclasses import dataclass, field

from .tile_status import TileStatus


@dataclass
class Blackboard:
    known_tiles: dict[tuple[int, int], TileStatus] = field(default_factory=dict)
    agent_positions: dict[str, tuple[int, int]] = field(default_factory=dict)
