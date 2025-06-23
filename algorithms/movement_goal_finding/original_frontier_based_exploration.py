from mesa.discrete_space import Cell, CellAgent
from agents.obstacle import Obstacle
from agents.ground import Ground
from algorithms.movement_goal_finding.movement_goal_finder import MovementGoalFinder

class OriginalFBE(MovementGoalFinder):
    """
    Finding of movement goals using the Frontier Based Exploration idea from Yamauchi (1997).
    :return: List of frontier positions.
    """
    def __init__(self,
        agent: CellAgent,
    ):
        self.agent = agent

    # TODO:
    #  Change to local knowledge base!
    #  Depending on its format, its necessary to change parameter of _is_frontier()

    def find_goals(self,
    ) -> list[tuple[int, int]]:
        frontier_cells = []
        for x in range(self.agent.model.grid.width):
            for y in range(self.agent.model.grid.height):
                if self._is_frontier(self.agent.model.grid.get_cell(x, y)):
                    frontier_cells.append((x, y))
        return frontier_cells

    @staticmethod
    def _is_frontier(cell: Cell) -> bool:
        """
        Uses the following frontier definition:
        Frontier = A cell which is known and free with at least one unknown neighbor cell.
        """
        if cell is None:
            return False

        # Occupation by other robot, etc. is irrelevant for Frontier determination,
        #  so no check if cell is blocked by other agent via cell_blocking-property (e.g. AStar)
        # Check if cell is blocked by (not moving) Obstacle
        if any(isinstance(agent, Obstacle) for agent in cell.agents):
            return False
        # Check if cell is already known
        ground_agents = [agent for agent in cell.agents if isinstance(agent, Ground)]
        if not ground_agents:
            raise RuntimeError(f"Environment Error: Cell {cell.coordinate} has no Obstacle and no Ground agent.")
        if not ground_agents[0].explored:
            return False
        # Check for unknown neighbor cells
        neighbor_cells = cell.get_neighborhood(radius=1, include_center=False)
        for neighbor_cell in neighbor_cells:
            neighbor_ground_agents = [agent for agent in neighbor_cell.agents if isinstance(agent, Ground)]
            if not neighbor_ground_agents:
                continue # Neighbors with obstacles have no ground agent
            if neighbor_ground_agents[0].explored:
                return True
        return False