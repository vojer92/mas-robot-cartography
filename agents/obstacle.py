from mesa import Model
from mesa.discrete_space import Cell, FixedAgent


class Obstacle(FixedAgent):
    def __init__(self, model: Model, cell: Cell) -> None:
        super().__init__(model)
        self.cell = cell
        self.blocking = True