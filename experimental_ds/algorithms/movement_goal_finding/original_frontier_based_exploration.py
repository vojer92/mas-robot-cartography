# TODO: Change to mesa 3.2.0!
#  MultiGridWithProperties -> DiscreteSpace + CellObjects + PropertyLayer

# TODO:
#  If charging stations (or other objects which dont block cells) are introduced
#  is_cell_empty cannot be used anymore!

from algorithms.movement_goal_finding.movement_goal_finding_method import MovementGoalFindingMethod

from environment.multi_grid_with_properties import MultiGridWithProperties

class OriginalFrontierBasedExploration(MovementGoalFindingMethod):
    def find_goals(self,
        local_grid: MultiGridWithProperties,
        moore: bool,
    ) -> list[tuple[int, int]]:
        all_frontier_cells = self._find_frontier_cells(local_grid, moore)
        if not all_frontier_cells:
            return []
        else:
            # Return all single frontier-cells
            return all_frontier_cells


    def _find_frontier_cells(self,
        local_grid: MultiGridWithProperties,
        moore: bool
    ) -> list[tuple[int, int]]:
        frontier_cells = []
        for x in range(local_grid.width):
            for y in range(local_grid.height):
                pos = (x, y)
                if self._is_frontier(local_grid, pos, moore):
                    frontier_cells.append(pos)
        return frontier_cells


    def _is_frontier(self,
        local_grid: MultiGridWithProperties,
        pos: tuple[int, int],
        moore: bool,
    ) -> bool:
        """
        Uses the following frontier definition:
        Frontier = A cell which is known and free with at least one unknown neighbor cell.
        """
        # Check grid borders
        if local_grid.out_of_bounds(pos):
            return False
        # Check if known and free
        if local_grid.get_property(pos, "unknown") or not local_grid.is_cell_empty(pos):
            return False
        # Check for unknown neighbor cells
        for neighbor_pos in local_grid.iter_neighborhood(pos, moore, include_center=False, radius=1):
            if local_grid.get_property(neighbor_pos, "unknown"):
                return True
        return False