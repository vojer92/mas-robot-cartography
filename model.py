import math
from functools import reduce

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
from algorithms.explorability_analysis import (flood_fill,
                                               load_no_unexplorable,
                                               save_no_unexplorable)
from communication.pubSubBroker import PubSubBroker

#NOTE: If you want to use given shapes or place all obstacles adjust here and change
# method below.
OBSTACLE_SHAPES = [
    [(-2, 0), (-1, 0), (0, 0)],

    [(-4, 0), (-3, 0), (-2, 0), (-1, 0), (0, 0)],
    [(0, -2), (0, -1), (0, 0)],
    [(0, -3), (0, -2), (0, -1), (0, 0)],
    [(0, -4), (0, -3), (0, -2), (0, -1), (0, 0)],
]

# Up to 40x40-grid
OBSTACLES = []

OBSTACLES.extend([(x, 0) for x in range(0, 33)])
OBSTACLES.extend([(x, 0) for x in range(36, 40)])

OBSTACLES.extend([(x, 10) for x in range(0, 6)])
OBSTACLES.extend([(x, 10) for x in range(8, 15)])
OBSTACLES.extend([(x, 10) for x in range(17, 31)])
OBSTACLES.extend([(x, 10) for x in range(37, 40)])

OBSTACLES.extend([(x, 17) for x in range(0, 6)])
OBSTACLES.extend([(x, 17) for x in range(8, 10)])

OBSTACLES.extend([(x, 23) for x in range(2, 16)])
OBSTACLES.extend([(x, 23) for x in range(18, 31)])
OBSTACLES.extend([(x, 23) for x in range(37, 40)])

OBSTACLES.extend([(x, 29) for x in range(2, 11)])
OBSTACLES.extend([(x, 29) for x in range(13, 21)])
OBSTACLES.extend([(x, 29) for x in range(23, 40)])

OBSTACLES.extend([(x, 38) for x in range(2, 11)])
OBSTACLES.extend([(x, 38) for x in range(13, 16)])

OBSTACLES.extend([(2, y) for y in range(17, 25)])
OBSTACLES.extend([(2, y) for y in range(27, 40)])

OBSTACLES.extend([(4, y) for y in [11,14,15,16]])

OBSTACLES.extend([(9, y) for y in range(11, 12)])
OBSTACLES.extend([(9, y) for y in range(14, 24)])
OBSTACLES.extend([(9, y) for y in range(29, 33)])
OBSTACLES.extend([(9, y) for y in range(35, 39)])

OBSTACLES.extend([(13, y) for y in range(0, 10)])

OBSTACLES.extend([(15, y) for y in range(29, 40)])

OBSTACLES.extend([(19, y) for y in range(29, 33)])
OBSTACLES.extend([(19, y) for y in range(35, 40)])

OBSTACLES.extend([(31, y) for y in range(0, 24)])

OBSTACLES.extend([(37, y) for y in range(0, 7)])
OBSTACLES.extend([(37, y) for y in range(9, 12)])
OBSTACLES.extend([(37, y) for y in range(14, 24)])

# Remove duplicates and sort
OBSTACLES = list(set(OBSTACLES))
OBSTACLES.sort(key=lambda c: (c[0], c[1]))



class Exploration(Model):
    def __init__(
        self,
        grid_size=20,
        obstacle_density=0.3,
        view_radius=1,
        view_angle=90,
        view_resolution=5,
        initial_no_robots=1,
        factor_distance = 1.0,
        factor_size = 0.1,
        robot_type_str: str = "FBERobot",
        seed=42,
        simulator: ABMSimulator = ABMSimulator(),
    ):
        super().__init__(seed=seed)
        self.seed = seed
        self.simulator = simulator
        self.simulator.setup(self)

        self.height = grid_size
        self.width = grid_size
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

        # Place obstacles
        #NOTE: Select the placement method you want.
        self._place_obstacles_random_shattered()
        #self._place_obstacles_random_given_shapes()
        #self._place_obstacles_given()

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
                        factor_size=factor_size, 
                        factor_distance=factor_distance,
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

        model_reporter = {
            "Explored": lambda m: len(
                [
                    agent
                    for agent in self.grid.all_cells.agents
                    if isinstance(agent, Ground) and agent.explored
                ]
            ) / ((self.width * self.height) - self.no_unexplorable) * 100.0,
            "Explored_fields": lambda m: len(
                [
                    agent
                    for agent in self.grid.all_cells.agents
                    if isinstance(agent, Ground) and agent.explored
                ]
            ),
            "Step_Count_All_Agents": lambda m: reduce(lambda a, b: a + b, [
                agent.step_count 
                for agent in self.grid.all_cells.agents
                if isinstance(agent, ExplorerRobot)
            ])
        }

        agenttype_reporter = {
            ExplorerRobot: {"Step_Count": lambda a: a.step_count},
        }

        self.datacollector = DataCollector(model_reporters=model_reporter, agenttype_reporters=agenttype_reporter)

        self.running = True
        self.datacollector.collect(self)

    def step(self):
        if self.agents:
            self.agents_by_type[self.robot_type].shuffle_do("step")

        self.datacollector.collect(self)

        if self.steps == 999:
            self.simulator.reset()

    def _place_obstacles_random_shattered(self):
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

    def _place_obstacles_random_given_shapes(self):
        for obstacle in OBSTACLE_SHAPES:
             random_cell = self.grid.select_random_empty_cell()
             neighborhood = random_cell.get_neighborhood(
                 len(obstacle), include_center=True
             )
             for coordinate in obstacle:
                 target_cells = neighborhood.select(
                     lambda cell: cell.coordinate
                     == tuple(
                         abs(a + b) for a, b in zip(random_cell.coordinate, coordinate)
                     )
                 )
                 # Transfer list (usually 1, but also 0 or >1 possible) to coordinate of 1 cell
                 if target_cells.cells:
                     Obstacle(self, cell=target_cells.cells[0])

    def _place_obstacles_given(self):
        for coordinate in OBSTACLES:
            if coordinate[0]< self.width and coordinate[1]< self.height:
                Obstacle.create_agents(self, 1, cell=self.grid[coordinate])
            else:
                continue