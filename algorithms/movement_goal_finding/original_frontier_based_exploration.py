from agents.explorer_robot import ExplorerRobot, CellInfo

from algorithms.movement_goal_finding.movement_goal_finder import MovementGoalFinder

class OriginalFBE(MovementGoalFinder):
    """
    Finding of movement goals using the Frontier Based Exploration idea from Yamauchi (1997).
    :return: List of frontier positions.
    """
    def __init__(self,
        agent: ExplorerRobot,
    ):
        self.agent = agent

    def find_goals(self,
    ) -> list[tuple[int, int]]:
        frontier_cells = []
        for pos, cell_info in self.agent.local_memory.grid_info.items():
            if self._is_frontier(pos, cell_info):
                frontier_cells.append(pos)
        return frontier_cells

    def _is_frontier(self,
            pos: tuple[int, int],
            cell_info: CellInfo
    ) -> bool:
        """
        Uses the following frontier definition:
        Frontier = A cell which is known and free with at least one unknown neighbor cell.
        """
        # No check for exploration, because only already explored cells are given

        # Check if cell is blocked by not moving blocking agent.
        # Occupation by moving agents (e.g., other robots) is irrelevant for frontier determination,
        # because it is expected that they will not block the cell permanently.
        # Therefore, only check for agents that are both non-moving and blocking!
        if any(not agent.moving and agent.cell_blocking for agent in cell_info.agents):
            return False

        # Check for at least one unknown neighbor
        for neighbor_pos in self.agent.local_memory.get_all_neighbor_positions(pos):
            if neighbor_pos not in self.agent.local_memory.grid_info:
                return True

        return False