import math

from mesa import Model
from mesa.datacollection import DataCollector
from mesa.discrete_space import OrthogonalMooreGrid
from mesa.experimental.devs import ABMSimulator

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
        view_resulution=32,
        initial_random_walk_robot=1,
        seed=None,
        simulator: ABMSimulator = ABMSimulator(),
    ):
        super().__init__(seed=seed)
        self.simulator = simulator
        self.simulator.setup(self)

        self.height = height
        self.width = width

        self.initial_random_walk_robot = initial_random_walk_robot

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

        ground_cells = self.grid.all_cells.select(
            lambda cell: not any(isinstance(agent, Obstacle) for agent in cell.agents)
        ).cells

        for cell in ground_cells:
            Ground(self, cell=cell)

        if self.initial_random_walk_robot:
            RandomWalkRobot.create_agents(
                self,
                initial_random_walk_robot,
                cell=self.random.choices(ground_cells, k=initial_random_walk_robot),
                view_radius=view_radius,
                view_angle=view_angle,
                orientation=self.random.choices(
                    range(0, 359, 45), k=initial_random_walk_robot
                ),
                view_resulution=view_resulution,
            )

        self.running = True
        self.datacollector.collect(self)

    def step(self):
        if self.initial_random_walk_robot:
            self.agents_by_type[RandomWalkRobot].shuffle_do("step")

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

                if target_cells.cells:
                    Obstacle(self, cell=target_cells.cells[0])
