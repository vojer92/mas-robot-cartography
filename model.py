import math

from mesa import Model, Agent
from mesa.discrete_space import PropertyLayer
from mesa.datacollection import DataCollector
from mesa.discrete_space import OrthogonalMooreGrid
from mesa.experimental.devs import ABMSimulator

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
        initial_no_robots=1,
        robot_type: Agent = RandomWalkRobot,
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

        # Property Layer if cells are already known or still unknown
        known_layer = PropertyLayer(
            name = "known",
            dimensions = self.grid.dimensions,
            default_value = False,
        )
        self.grid.add_property(known_layer)

        # Property Layer for obstacles
        obstacle_layer = PropertyLayer(
            name = "obstacle",
            dimensions = self.grid.dimensions,
            default_value = False,
        )
        self.grid.add_property(obstacle_layer)

        # TODO: Auf Property Layer anpassen
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

        self._place_agents()

        self.running = True
        self.datacollector.collect(self)

    def step(self):
        # TODO: Adjust to different agent types
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
                    self.grid.obstacle[target_cells.cells[0].coordinate] = True

    def _place_agents(self):
        free_cells = self.grid.all_cells.select(
            lambda cell: not self.grid.obstacle[cell.coordinate]
        ).cells

        # TODO: Hard copy for every agent type
        if self.initial_random_walk_robot:


            start_cells = self.random.sample(free_cells, k=initial_random_walk_robot)
            # Sample instead of choice to avoid duplicates
            # k have to be >= ground_cells.len! If not sample throws error.

            RandomWalkRobot.create_agents(
                self,
                initial_random_walk_robot,
                cell=start_cells,
                view_radius=view_radius,
                view_angle=view_angle,
                orientation=self.random.choices(
                    range(0, 315, 45), k=initial_random_walk_robot
                ),
                view_resulution=view_resulution,
            )

            for cell in start_cells:
                self.grid.known[cell.coordinate] = True