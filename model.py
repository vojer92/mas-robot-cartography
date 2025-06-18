import math

from mesa import Model, Agent
from mesa.datacollection import DataCollector
from mesa.discrete_space import OrthogonalMooreGrid
from mesa.experimental.devs import ABMSimulator

from agents.explorer_robot import ExplorerRobot
from agents.ground import Ground
from agents.obstacle import Obstacle
from agents.random_walk_robot import RandomWalkRobot

OBSTACLES = [
    [(-2, 0), (-1, 0), (0, 0)],
    [(-3, 0), (-2, 0), (-1, 0), (0, 0)],
    [(-4, 0), (-3, 0), (-2, 0), (-1, 0), (0, 0)],
    [(0, -2), (0, -1), (0, 0)],
    [(0, -3), (0, -2), (0, -1), (0, 0)],
    [(0, -4), (0, -3), (0, -2), (0, -1), (0, 0)],
]

class Exploration(Model):
    def __init__(
        self,
        width=20,
        height=20,
        view_radius=1,
        view_angle=90,
        view_resolution=32,
        initial_no_robots=1,
        robot_type: ExplorerRobot = RandomWalkRobot,
        seed=None,
        simulator: ABMSimulator = ABMSimulator(),
    ):
        super().__init__(seed=seed)
        self.simulator = simulator
        self.simulator.setup(self)

        self.height = height
        self.width = width

        self.initial_no_robots = initial_no_robots
        self.robot_type = robot_type

        self.grid = OrthogonalMooreGrid(
            [self.height, self.width],
            torus=False,
            capacity=math.inf,
            random=self.random,
        )

        model_reporter = {
            "Explored": lambda m: len(
                [
                    agent
                    for agent in self.grid.all_cells.agents
                    if isinstance(agent, Ground) and agent.explored
                ]
            )
        }

        self.datacollector = DataCollector(model_reporter)

        self._place_obstacles()

        free_cells = [
            cell for cell in self.grid.all_cells if not any(isinstance(agent, Obstacle) for agent in cell.agents)
        ]

        # Place Ground-agents in all cells without unmobile obstacles
        # Needs to be reviewed and manybe changed for mobile obstacles
        for cell in free_cells:
            Ground(self, cell=cell)

        # Place k robots random on free cells
        if self.initial_no_robots:
            robot_type.create_agents(
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

        self.running = True
        self.datacollector.collect(self)

    def step(self):
        if self.agents:
            self.agents_by_type[self.robot_type].shuffle_do("step")

        self.datacollector.collect(self)

    def _place_obstacles(self):
        for obstacle in OBSTACLES:
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