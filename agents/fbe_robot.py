import logging
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

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


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
            self, MovementGoalSelectorEnum.NEAREST_BIGGEST_FRONTIER
        )
        self.goal = None
        self.path = None
        self.path_index = 0
        self.blocked_counter = 0
        self.blocked_counter_max = (
            1  # Number of rounds the agent waits when its route is blocked
        )

    def step(self):



        logger.info(f"[{self.unique_id}] Position begin step {self.cell.coordinate}")



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
        max_attempts = 3 # Necessary to avoid endless loop, when actually no path exist
        # NOTE: With max_attempts = len(self.local_memory.frontier_info) is every frontier tried once.
        blacklist = set() # Necessary to avoid the reselection of the same unreachable goal max_attempts-times.
        while attempts < max_attempts:
            # Check if current goal is still relevant. If not select new goal.
            if (self.goal is None or
                    self.goal not in self.local_memory.frontier_info.keys()):
                possible_goals = set(self.local_memory.frontier_info.keys()) - blacklist
                if not possible_goals:



                    logger.info(f"[{self.unique_id}] No possible goals left to try in this step.")



                    self.goal = None
                    self.path = None
                    break
                else:
                    new_goal = self.goal_selector.select_goal(possible_goals)
                # Check if new goal was selected. If all Frontiers are explored, no new goal is left.
                if new_goal is not None:
                    self.goal = new_goal
                    self.local_memory.frontier_info[self.goal].status = (
                        FrontierStatus.IN_WORK
                    )
                    self.local_memory.frontier_info[self.goal].agent_id = self.unique_id
                    self.path = None



                    logger.info(f"[{self.unique_id}] Goal: {self.goal}")



                else:
                    self.goal = None
                    self.path = None



                    logger.info(
                        f"[{self.unique_id}] No goal available (all frontiers explored?)"
                    )



                    break

            # Calculate path to goal
            if self.path is None and self.goal is not None:
                self.path = self.pathfinder.find_path(self.goal)



                logger.info(f"[{self.unique_id}] {self.path} => {self.goal}")



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



        if attempts > 1:
            logger.info(
                "[%s] Needed %d attempts to find a reachable goal in this step.",
                self.unique_id, attempts
            )
        elif self.path is None and self.goal is None:
            logger.info(
                "[%s] No reachable goal found after %d attempts.", self.unique_id, attempts
            )



        # Broadcast new frontier and goal selection information to all robots
        self.pubSubBroker.publish(
            "new_frontier_info", self.local_memory.frontier_info, self.unique_id
        )

        # Move along path
        current_pos = self.cell.coordinate
        moved = False

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
                    moved = True

        # Check if end of the path is reached
        if self.path is not None and self.path_index == len(self.path):

            self.path = None
            self.goal = None

        # If no movement was possible
        if not moved:



            logger.info(
                f"[{self.unique_id}] No movement. Looking for unexplored neighbors."
            )



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



        logger.info(f"[{self.unique_id}] Position end step {self.cell.coordinate}")



    def _new_gird_info_callback(self, data: dict[tuple[int, int], CellInfo]):
        self.local_memory.grid_info = data.copy()

    def _new_frontier_info_callback(self, data: dict[tuple[int, int], FrontierInfo]):
        self.local_memory.frontier_info = data.copy()
