# TODO:
#  Add method for turning and integrate it into movement
import random
from abc import ABC, abstractmethod
from mesa.discrete_space import DiscreteSpace, CellAgent



class BaseAgent(CellAgent, ABC):
    """
    Base class for all agents.
    Expects initialized activities in constructor (from factory)
    """
    def __init__(self,
        local_map_cell_nr
        local_grid_moore: bool, # Moore property to define possible movement and turning
        model_random_generator: random.Random , #
        *args,
        **kwargs
    ) -> None:
        super().__init__(*args, **kwargs)
        # Local memory of the environment
        self.agent_local_map = DiscreteSpace(
            shape=local_map_cell_nr,
            cell_klass = None, # Default -> Cell
            random = model_random_generator,
        )
        self.local_grid.add_property("perception_time")
        # ...
        self.orientation = 270
        self.step_counter = 0

    @abstractmethod
    def step(self) -> None:
        """
        Actions of the Agent in a single simulation step.
        """
        pass

    def move(self,
    ) -> bool:


    def turn(self,
    ) -> bool :