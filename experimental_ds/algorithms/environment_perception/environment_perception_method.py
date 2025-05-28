from abc import ABC, abstractmethod
from environment.multi_grid_with_properties import MultiGridWithProperties

class EnvironmentPerceptionMethod(ABC):
    """
    Interface: Procedure for the environment_perception of the environment through the Agents.
    Return: None. Information from world are transferred to agent_local_memory
    """

    @abstractmethod
    def scan_environment(self,
                         world: MultiGridWithProperties,
                         current_time: int,
                         agent_pos: tuple[int, int],
                         agent_orientation: int,
                         agent_local_memory: MultiGridWithProperties,
                         blocked_by_objects: bool = True
                         ) -> None:
        pass