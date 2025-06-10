from .blackboard import Blackboard
from .oaa_broker import OAABroker
from .tile_status import TileStatus


class KnowledgeBase:
    """"""

    def __init__(self):
        self._broker = OAABroker()
        self._blackboard = Blackboard()

    def update_agent_position(self, agent_id: str, pos: tuple[int, int]):
        self._blackboard.agent_positions[agent_id] = pos
        print(f"[Blackboard] Position von {agent_id} ist jetzt {pos}")

    def update_known_tiles(self, location: tuple[int, int], tile_status: TileStatus):
        self._blackboard.known_tiles[location] = tile_status
        print(f"[Blackboard] Known Tile von {location} geändert zu {tile_status}")
