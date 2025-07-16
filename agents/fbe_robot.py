import math

from mesa import Model
from mesa.discrete_space import Cell

from agents.explorer_robot import (
    AgentInfo,
    CellInfo,
    ExplorerRobot,
    FrontierInfo,
    FrontierStatus,
)
from algorithms.movement_goal_finding.movement_goal_finder_enum import (
    MovementGoalFinderEnum,
)
from algorithms.movement_goal_finding.movement_goal_finder_factory import (
    MovementGoalFinderFactory,
)
from algorithms.movement_goal_selection.movement_goal_selector_enum import (
    MovementGoalSelectorEnum,
)
from algorithms.movement_goal_selection.movement_goal_selector_factory import (
    MovementGoalSelectorFactory,
)
from algorithms.pathfinding.pathfinder_enum import PathfinderEnum
from algorithms.pathfinding.pathfinder_factory import PathfinderFactory
from communication.pubSubBroker import PubSubBroker


class FBERobot(ExplorerRobot):
    """
    An explorer robot that uses the FBE method.
    """

    def __init__(
        self,
        model: Model,
        cell: Cell,
        pubSubBroker: PubSubBroker,
        view_radius: int = 1,
        view_angle: int = 90,
        view_resolution: int = 5,
        orientation: int = -90,
        factor_distance: float = 1.0,
        factor_size: float = 0.1,
    ):
        super().__init__(
            model, cell, view_radius, view_angle, view_resolution, orientation
        )
        self.pubSubBroker = pubSubBroker
        self.pubSubBroker.subscribe(
            "new_grid_info", self._new_gird_info_callback, self.unique_id
        )
        self.pubSubBroker.subscribe(
            "new_frontier_into", self._new_frontier_info_callback, self.unique_id
        )
        self.pathfinder = PathfinderFactory.give_pathfinder(self, PathfinderEnum.ASTAR)
        self.goal_finder = MovementGoalFinderFactory.give_movement_goal_finder(
            self, MovementGoalFinderEnum.ORIGINAL_FBE
        )
        self.goal_selector = MovementGoalSelectorFactory.give_movement_goal_selector(
            self,
            MovementGoalSelectorEnum.NEAREST_BIGGEST_FRONTIER,
            factor_distance=factor_distance,
            factor_size=factor_size,
        )
        self.goal = None
        self.path = None
        self.path_index = 0
        self.blocked_counter = 0
        self.blocked_counter_max = (
            1  # Number of rounds the agent waits when its route is blocked
        )

    def step(self):
        # Environment perception
        self.viewport = self.scan_environment()
        self.pubSubBroker.publish(
            "new_grid_info", self.local_memory.grid_info, self.unique_id
        )

        # Determine Frontiers
        current_frontiers = set(self.goal_finder.find_goals())
        memory_frontiers = set(self.local_memory.frontier_info.keys())
        # Remove outdated Frontiers
        to_remove = memory_frontiers - current_frontiers
        for pos in to_remove:
            del self.local_memory.frontier_info[pos]
        # Add new Frontiers
        to_add = current_frontiers - memory_frontiers
        for pos in to_add:
            self.local_memory.frontier_info[pos] = FrontierInfo(
                status=FrontierStatus.OPEN, agent_id=None
            )

        # Loop for new attempt in same step, if no path was found, to avoid waiting when reachable goals exist
        attempts = 0
        max_attempts = len(
            self.local_memory.frontier_info
        )  # Necessary to avoid endless loop. Tries every existing goal once.
        blacklist = (
            set()
        )  # Necessary to avoid the reselection of the same unreachable goal max_attempts-times.
        while attempts < max_attempts:
            # Check if current goal is still relevant. If not select new goal.
            if (
                self.goal is None
                or self.goal not in self.local_memory.frontier_info.keys()
            ):
                possible_goals = {
                    pos
                    for pos, frontier in self.local_memory.frontier_info.items()
                    if frontier.status == FrontierStatus.OPEN
                } - blacklist
                if not possible_goals:
                    self.goal = None
                    self.path = None
                    break
                else:
                    new_goal = self.goal_selector.select_goal(possible_goals)

                # Check if new goal was selected.
                if new_goal is not None:
                    self.goal = new_goal
                    self.local_memory.frontier_info[self.goal].status = (
                        FrontierStatus.IN_WORK
                    )
                    self.local_memory.frontier_info[self.goal].agent_id = self.unique_id
                    self.path = None

                else:
                    # If all Frontiers are explored, no new goal is left.
                    self.goal = None
                    self.path = None

                    break

            # Calculate path to goal
            if self.path is None and self.goal is not None:
                self.path = self.pathfinder.find_path(self.goal)
                self.path_index = 0
            if self.path is None and self.goal is not None:
                # No path to goal could be calculated!
                # Add goal to blacklist
                blacklist.add(self.goal)
                # Start new selection and calculation
                self.local_memory.frontier_info[self.goal].status = FrontierStatus.OPEN
                self.local_memory.frontier_info[self.goal].agent_id = None
                self.goal = None
                attempts += 1
                continue
            else:
                break

        # Broadcast new frontier and goal selection information to all robots
        self.pubSubBroker.publish(
            "new_frontier_info", self.local_memory.frontier_info, self.unique_id
        )


        # Determine current position
        current_pos = self.cell.coordinate

        # Check for unexplored neighbors before moving
        unexplored_neighbors = [
            pos
            for pos in self.local_memory.get_all_neighbor_positions(current_pos)
            if pos not in self.local_memory.grid_info
        ]
        if unexplored_neighbors:
            # Look towards unexplored neighbors
            target_pos = unexplored_neighbors[0]
            self.orientation = self.normalize_round45_angle(
                (
                    math.degrees(
                        math.atan2(
                            target_pos[1] - current_pos[1],
                            target_pos[0] - current_pos[0],
                        )
                    )
                )
            )
        else:
            # Move along path
            if self.path is not None:

                # If path points on current_pos (usually at the beginning of the path) -> continue with next position
                while (
                    self.path_index < len(self.path)
                    and self.path[self.path_index] == current_pos
                ):
                    self.path_index += 1

                if self.path_index < len(self.path):
                    # Get next (new) position on path
                    next_pos = self.path[self.path_index]
                    # Check if next_pos is blocked
                    if any(
                        agent.cell_blocking
                        for agent in self.local_memory.grid_info[next_pos].agents
                    ):
                        # Blocked -> Wait till blocked_counter_max is reached, then resets path for recalculation in next step
                        self.blocked_counter += 1

                        if self.blocked_counter >= self.blocked_counter_max:
                            self.path = None  # Reset path for recalculation in next step
                            self.blocked_counter = 0
                    else:
                        # Not blocked -> Move agent to next position on path
                        next_cell = [
                            cell
                            for cell in self.model.grid.all_cells
                            if cell.coordinate == next_pos
                        ][0]
                        self.local_memory.grid_info[current_pos].agents = [
                            agent
                            for agent in self.local_memory.grid_info[current_pos].agents
                            if agent.unique_id != self.unique_id
                        ]
                        self.orientation = self.normalize_round45_angle(
                            math.degrees(
                                math.atan2(
                                    next_pos[1] - current_pos[1],
                                    next_pos[0] - current_pos[0],
                                )
                            )
                        )
                        self.cell = next_cell
                        self.local_memory.grid_info[next_pos].agents.append(
                            AgentInfo(
                                unique_id=self.unique_id,
                                agent_type=type(self).__name__,
                                cell_blocking=self.cell_blocking,
                                moving=self.moving,
                            )
                        )
                        self.blocked_counter = 0
                        self.path_index += 1

            # Check if end of the path is reached
            if self.path is not None and self.path_index == len(self.path):
                self.path = None
                self.goal = None

    def _new_gird_info_callback(self, data: dict[tuple[int, int], CellInfo]):
        self.local_memory.grid_info = data.copy()

    def _new_frontier_info_callback(self, data: dict[tuple[int, int], FrontierInfo]):
        self.local_memory.frontier_info = data.copy()
