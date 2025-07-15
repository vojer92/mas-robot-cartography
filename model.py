import math

import numpy as np
from mesa import Model
from mesa.datacollection import DataCollector
from mesa.discrete_space import OrthogonalMooreGrid
from mesa.experimental.devs import ABMSimulator

from agents.explorer_robot import ExplorerRobot
from agents.fbe_robot import FBERobot
from agents.ground import Ground
from agents.obstacle import Obstacle
from agents.random_walk_robot import RandomWalkRobot
from algorithms.explorability_analysis import (flood_fill, load_no_unexplorable,
                                               save_no_unexplorable)
from communication.pubSubBroker import PubSubBroker

#OBSTACLES = [
#    [(-2, 0), (-1, 0), (0, 0)],
#    [(-3, 0), (-2, 0), (-1, 0), (0, 0)],
#    [(-4, 0), (-3, 0), (-2, 0), (-1, 0), (0, 0)],
#    [(0, -2), (0, -1), (0, 0)],
#    [(0, -3), (0, -2), (0, -1), (0, 0)],
#    [(0, -4), (0, -3), (0, -2), (0, -1), (0, 0)],
#]


class Exploration(Model):
    def __init__(
        self,
        width=20,
        height=20,
        obstacle_density=0.3,
        view_radius=1,
        view_angle=90,
        view_resolution=5,
        initial_no_robots=1,
        robot_type_str: str = "FBERobot",
        seed=None,
        simulator: ABMSimulator = ABMSimulator(),
    ):
        super().__init__(seed=seed)
        self.seed = seed
        self.simulator = simulator
        self.simulator.setup(self)

        self.height = height
        self.width = width
        self.obstacle_density = obstacle_density
        self.no_robots = None

        self.initial_no_robots = initial_no_robots

        self.grid = OrthogonalMooreGrid(
            [self.height, self.width],
            torus=False,
            capacity=math.inf,
            random=self.random,
        )

        self.pubSubBroker = PubSubBroker()

        model_reporter = {
            "Explored": lambda m: len(
                [
                    agent
                    for agent in self.grid.all_cells.agents
                    if isinstance(agent, Ground) and agent.explored
                ]
            )
            # TODO: Add all metrics, e.g. exploration_progress
        }


        self.datacollector = DataCollector(model_reporter)

        # Place obstacles
        self._place_obstacles()

        # Place (unexplored) Ground-agents in all cells
        for cell in self.grid.all_cells:
            Ground(self, cell=cell)

        # Place k robots random on free cells
        free_cells = [
            cell
            for cell in self.grid.all_cells
            if not any(isinstance(agent, Obstacle) for agent in cell.agents)
        ]

                  

        if self.initial_no_robots:
            match robot_type_str:
                case "FBERobot":
                    self.robot_type = FBERobot
                    self.robot_type.create_agents(
                        self,
                        self.initial_no_robots,
                        pubSubBroker=self.pubSubBroker,
                        cell=self.random.sample(free_cells, k=self.initial_no_robots),
                        # Sample instead of choice to avoid duplicates
                        # k have to be >= ground_cells.len! If not sample throws error.
                        view_radius=view_radius,
                        view_angle=view_angle,
                        orientation=self.random.choices(
                            range(0, 315, 45), k=self.initial_no_robots
                        ),
                        view_resolution=view_resolution,
                    )
                case "RandomWalkRobot": 
                    self.robot_type = RandomWalkRobot
                    self.robot_type.create_agents(
                        self,
                        self.initial_no_robots,
                        cell=self.random.sample(free_cells, k=self.initial_no_robots),
                        # Sample instead of choice to avoid duplicates
                        # k have to be >= ground_cells.len! If not sample throws error.
                        view_radius=view_radius,
                        view_angle=view_angle,
                        orientation=self.random.choices(
                            range(0, 315, 45), k=self.initial_no_robots
                        ),
                        view_resolution=view_resolution,
                    )


        # Determine unexplorable positions
        # Check if already calculated
        no_unexplorable = load_no_unexplorable(
            seed = self.seed,
            grid_width=self.width,
            grid_height=self.height,
            no_agents=self.initial_no_robots,
        )
        if no_unexplorable is None:
            # Determine all agent positions as start positions for flood-fill
            agent_positions = [
                (agent.cell.coordinate[0], agent.cell.coordinate[1])
                for agent in self.agents_by_type[self.robot_type]
            ]
            # Create obstacle grid as base structure for flood-fill
            obstacle_grid = np.zeros((self.height, self.width), dtype=int)
            for y in range(self.height):
                for x in range(self.width):
                    cell = self.grid[x, y] # mesa: at first x (width), then y (height)
                    if any(isinstance(agent,Obstacle) for agent in cell.agents):
                        obstacle_grid[y,x] = 1 # NumPy: at first y (height), then x (width)
            # Create reachability-mask for all robot-start-positions and unite all explorable positions
            explorable = np.zeros_like(obstacle_grid, dtype=bool)
            for pos in agent_positions:
                explorable |= flood_fill(obstacle_grid, pos) #In-place bitwise or-operation
            # Calculate number of unexplorable positions
            no_unexplorable = np.size(explorable) - np.count_nonzero(explorable)
            # NOTE: Optional save mask of unexplorable positions for highlighting them in visualization
            # Save no_unexplorable
            save_no_unexplorable(
                no_unexplorable=no_unexplorable,
                seed=self.seed,
                grid_width=self.width,
                grid_height=self.height,
                no_agents=self.initial_no_robots,
            )

        self.no_unexplorable = no_unexplorable


        self.running = True
        self.datacollector.collect(self)

    def step(self):
        if self.agents:
            self.agents_by_type[self.robot_type].shuffle_do("step")

        self.datacollector.collect(self)

    def _place_obstacles(self):
        free_cells = [
            cell
            for cell in self.grid.all_cells
            if not any(
                isinstance(agent, Obstacle) or not isinstance(agent, ExplorerRobot)
                for agent in cell.agents
            )
        ]

        num = int((self.width * self.height) * self.obstacle_density)

        Obstacle.create_agents(self, num, cell=self.random.sample(free_cells, k=num))
        # for obstacle in OBSTACLES:
        #     random_cell = self.grid.select_random_empty_cell()
        #     neighborhood = random_cell.get_neighborhood(
        #         len(obstacle), include_center=True
        #     )
        #     for coordinate in obstacle:
        #         target_cells = neighborhood.select(
        #             lambda cell: cell.coordinate
        #             == tuple(
        #                 abs(a + b) for a, b in zip(random_cell.coordinate, coordinate)
        #             )
        #         )
        #         # Transfer list (usually 1, but also 0 or >1 possible) to coordinate of 1 cell
        #         if target_cells.cells:
        #             Obstacle(self, cell=target_cells.cells[0])
