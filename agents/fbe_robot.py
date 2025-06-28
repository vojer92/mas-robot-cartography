import math

from mesa import Model
from mesa.discrete_space import Cell

from agents.explorer_robot import ExplorerRobot, FrontierInfo, FrontierStatus

from algorithms.pathfinding.pathfinder_enum import PathfinderEnum
from algorithms.pathfinding.pathfinder_factory import PathfinderFactory

from algorithms.movement_goal_finding.movement_goal_finder_enum import MovementGoalFinderEnum
from algorithms.movement_goal_finding.movement_goal_finder_factory import MovementGoalFinderFactory

from algorithms.movement_goal_selection.movement_goal_selector_enum import MovementGoalSelectorEnum
from algorithms.movement_goal_selection.movement_goal_selector_factory import MovementGoalSelectorFactory


class FBERobot(ExplorerRobot):
    """
    An explorer robot that uses the FBE method.
    """

    def __init__(
            self,
            model: Model,
            cell: Cell,
            view_radius: int = 1,
            view_angle: int = 90,
            view_resolution: int = 5,
            orientation: int = -90,
    ):
        super().__init__(model,
            cell,
            view_radius,
            view_angle,
            view_resolution,
            orientation
        )
        self.pathfinder = PathfinderFactory.give_pathfinder(self, PathfinderEnum.ASTAR)
        self.goal_finder = MovementGoalFinderFactory.give_movement_goal_finder(self, MovementGoalFinderEnum.ORIGINAL_FBE)
        self.goal_selector = MovementGoalSelectorFactory.give_movement_goal_selector(self, MovementGoalSelectorEnum.NEAREST_BIGGEST_FRONTIER)
        self.goal = None
        self.path = None
        self.path_index = 0
        self.blocked_counter = 0
        self.blocked_counter_max = 1 # Number of rounds the agent waits when its route is blocked

     def step(self):
        #Environment perception
        self.viewport = self.scan_environment()


        #Broadcast new environment information to all robots
        for position in self.viewport:
            ....local_memory[position] = self.local_memory[position]
            #TODO:
            # Communication needs to be implemented!


        #Pulling new environment information is unnecessary due broadcast


        #Determine Frontiers
        current_frontiers = set(self.goal_finder.find_goals())
        memory_frontiers = set(self.local_memory.frontier_info.keys())
        #Remove outdated Frontiers
        to_remove = memory_frontiers - current_frontiers
        for pos in to_remove:
            del self.local_memory.frontier_info[pos]
        #Add new Frontiers
        to_add = current_frontiers - memory_frontiers
        for pos in to_add:
            self.local_memory.frontier_info[pos] = FrontierInfo(
                status=FrontierStatus.OPEN,
                agent_id = None
            )


        #Check if current goal is still relevant. If not select new goal.
        new_goal = None
        if (
            self.goal is None or
            self.goal not in self.local_memory.frontier_info.keys()
        ):
            new_goal = self.goal_selector.select_goal()
            self.goal = new_goal
            self.path = None


        #Broadcast new frontier information to all robots
        for frontier in to_remove:
            ....local_memory.frontier_info.pop(frontier)
            #TODO:
            # Communication needs to be implemented!
        for frontier in to_add:
            ....local_memory.frontier_info[frontier] = FrontierInfo(
                status=FrontierStatus.OPEN,
                agent_id = None
            )
            #TODO:
            # Communication needs to be implemented!
        if new_goal:
            ....local_memory.fronter_info[new_goal].agent_id = self.unique_id
            #TODO:
            # Communication needs to be implemented!

        #Calculate path to goal
        if self.path is None:
            self.path = self.pathfinder.find_path(self.goal)
            self.path_index = 0
        if self.path is None: #Fallback if no path was found
            ...
        #TODO:
        # If there is no connection of already explored free cells from agent position to goal position, it is unable to
        # calculate a path towards it!
        # Kind of like alternative strategy is necessary... but i have no idea which one... evtl. random movement...

        #Get next position on path
        current_pos = self.cell.coordinate
        if self.path_index < len(self.path) - 1:
            self.path_index += 1
            next_pos = self.path[self.path_index]
        else:
            #TODO: Possible???

        #Current Position Check, Collision-Check and movement
        if current_pos == next_pos:
            self.orientation += self.view_angle
            #TODO:
            # Improvement: Turning directly towards unknown neighbor cells!


        elif next_pos not in self.local_memory.grid_info.keys():
            #Unknown -> don't move there, but look in this direction
            self.orientation = self.normalize_round45_angle(
                (
                    math.degrees(
                        math.atan2(
                            next_pos[1] - current_pos[1],
                            next_pos[0] - current_pos[0],
                        )
                    )
                )
            )
            self.blocked_counter = 0
        elif any(agent.cell_blocking for agent in self.local_memory.grid_info[next_pos].agents):
            #Blocked -> Wait till blocked_counter_max is reached, then resets path for recalculation in next step
            self.blocked_counter += 1
            if self.blocked_counter >= self.blocked_counter_max:
                self.path = None
        else:
            #Free -> move there
            self.model.grid.move_agent(self, next_pos)
            self.orientation = self.normalize_round45_angle(
                (
                    math.degrees(
                        math.atan2(
                            next_pos[1] - current_pos[1],
                            next_pos[0] - current_pos[0],
                        )
                    )
                )
            )
            self.blocked_counter = 0


    #TODO:
    # Receiving the broadcasted information needs to be implemented!