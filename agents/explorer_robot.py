import math
from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum, auto
from typing import Generator, Optional

from mesa import Model
from mesa.discrete_space import Cell, CellAgent

from agents.ground import Ground


@dataclass
class AgentInfo:
    unique_id: int
    agent_type: str
    cell_blocking: bool
    moving: bool


@dataclass
class CellInfo:
    agents: list[AgentInfo]


class FrontierStatus(Enum):
    IN_WORK = auto()
    OPEN = auto()


@dataclass
class FrontierInfo:
    status: FrontierStatus
    agent_id: Optional[int] = None


class LocalMemory:
    MOORE_NEIGHBORS = [
        (-1, 0),
        (-1, 1),
        (0, 1),
        (1, 1),
        (1, 0),
        (1, -1),
        (0, -1),
        (-1, -1),
    ]

    def __init__(self):
        self.grid_info: dict[tuple[int, int], CellInfo] = {}  # {(x,y): CellInfo}
        self.frontier_info: dict[tuple[int, int], FrontierInfo] = (
            {}
        )  # {(x,y): FrontierInfo}

    def get_known_neighbor_positions(
        self,
        pos: tuple[int, int],
    ) -> list[tuple[int, int]]:
        """
        :Return: Returns all neighboring cell positions, which are in the local memory (= already explored).
        """
        offsets = self.MOORE_NEIGHBORS
        return [
            (pos[0] + dx, pos[1] + dy)
            for dx, dy in offsets
            if (pos[0] + dx, pos[1] + dy) in self.grid_info
        ]

    def get_all_neighbor_positions(self, pos: tuple[int, int]) -> list[tuple[int, int]]:
        """
        :Return: Returns all neighboring cell positions.
        """
        offsets = self.MOORE_NEIGHBORS
        return [(pos[0] + dx, pos[1] + dy) for dx, dy in offsets]


class ExplorerRobot(ABC, CellAgent):
    """
    Base class for all exploring robots.
    Includes scan_environment-function, which implements Raycasting and Bresenham-logic for environment perception.
    """

    # NOTE:
    # If other robot types without environment perception are introduced, a base base class with some of the properties
    # is necessary. AStar & OriginalFrontierBasedExploration & NearestBiggestFrontier has to be changed to this new base base class.

    # NOTE:
    # Although RandomWalkRobot doesn't require local memory, providing a separate scan_environment implementation
    # would lead to significant code duplication and poorer maintainability, so we intentionally avoid it.

    def __init__(
        self,
        model: Model,
        cell: Cell,
        view_radius: int = 1,
        view_angle: int = 90,
        view_resolution: int = 5,
        orientation: int = -90,
    ):
        super().__init__(model)

        self.cell = cell
        self.view_radius = view_radius
        if view_angle == 360:
            self.view_angle = 359
        else:
            self.view_angle = view_angle
        self.view_resolution = view_resolution
        self.orientation = orientation

        self.cell_blocking = True  # Blocking of cell for moving agents
        self.scan_blocking = (
            False  # Blocking of environment perception for other agents
        )
        self.moving = True

        self.viewport: list[tuple[int, int]] = []  # Current view

        self.local_memory = LocalMemory()  # Local memory

    @abstractmethod
    def step(self):
        pass

    @staticmethod
    def normalize_round45_angle(angle):
        """
        Normalize any angle to the nearest multiple of 45 degrees in [0, 360).
        This is necessary to match the available markers for agent orientation visualization.
        """
        return int(45 * round((angle + 360) % 360 / 45)) % 360 - 90

    def scan_environment(self) -> list[tuple[int, int]]:
        """
        Uses Raycasts with Bresenham's line algorithm to scan the environment in a given area.
        Scanning means transferring properties and objects from the environment to the agents local memory.
        :Return: Current viewport as list of position-tuples. The viewport doesn't include cells with scan_blocking agents on it.
        """
        viewport = []

        neighbor_cells = self.cell.get_neighborhood(
            radius=self.view_radius, include_center=True
        )
        allowed_coordinates = {cell.coordinate for cell in neighbor_cells}

        for angle in self._angle_generator(self.view_angle, self.view_resolution):
            # Calculate Raycasting end-positions
            x0, y0 = self.cell.coordinate
            dx = self.view_radius * math.cos(
                math.radians(self.orientation + angle + 90)
            )
            dy = self.view_radius * math.sin(
                math.radians(self.orientation + angle + 90)
            )
            end_pos = (round(x0 + dx), round(y0 + dy))

            for pos in self._bresenham_line(self.cell.coordinate, end_pos):
                # Check for grid borders
                if pos not in allowed_coordinates:
                    break

                # Get cell for pos
                current_cells = list(
                    neighbor_cells.select(lambda cell: cell.coordinate == pos)
                )
                if len(current_cells) != 1:
                    raise RuntimeError(
                        f"Cell selection error. Expected 1 cell at position {pos}, found {len(current_cells)}"
                    )
                current_cell = current_cells[0]

                # Check if pos is already explored. If not add to local memory.
                if not pos in self.local_memory.grid_info:
                    self.local_memory.grid_info[pos] = CellInfo(agents=[])

                # Transfer all agents to local memory
                # Except ground agents, because explored property is met by existing of an entry to a position
                cell_info = self.local_memory.grid_info[pos]
                cell_info.agents = [
                    AgentInfo(
                        unique_id=agent.unique_id,
                        agent_type=type(agent).__name__,
                        cell_blocking=getattr(agent, "cell_blocking", False),
                        moving=getattr(agent, "moving", False),
                    )
                    for agent in current_cell.agents
                    if not isinstance(agent, Ground)
                ]

                # Mark cells as explored via Ground-agents explored-property
                ground_agents = [
                    agent for agent in current_cell.agents if isinstance(agent, Ground)
                ]
                if not ground_agents:
                    raise RuntimeError(
                        f"Environment Error: Cell {pos} has no Ground agent."
                    )
                ground_agents[0].explored = True

                # Scan of the subsequent cells is blocked by some agents
                if any(
                    getattr(agent, "view_blocking", False) is True
                    for agent in current_cell.agents
                ):
                    break

                # Add position to viewport (only non-blocked)
                viewport.append(pos)

        return viewport

    @staticmethod
    def _bresenham_line(
        start_pos: tuple[int, int], end_pos: tuple[int, int]
    ) -> list[tuple[int, int]]:
        """
        Using Bresenham's line algorithm to list all grid cell positions (x, y) between start_pos and end_pos (incl.).
        Those can be outside the grid environment. This is handled in higher-level steps.
        """
        x0, y0 = start_pos
        x1, y1 = end_pos

        positions = []

        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0

        sx = 1 if x1 > x0 else -1
        sy = 1 if y1 > y0 else -1

        if dx > dy:
            err = dx / 2.0
            while x != x1:
                positions.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
            positions.append((x, y))  # Add endpoint
        else:
            err = dy / 2.0
            while y != y1:
                positions.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
            positions.append((x, y))  # Add endpoint

        return positions

    @staticmethod
    def _angle_generator(
        view_angle: int,  # View angle in degrees
        view_resolution: int = 5,  # Angle resolution in degrees
    ) -> Generator[float, None, None]:
        """
        Generator
        Generate angles in the given angle (symmetric) with the given resolution.
        """
        # Check for valid parameters
        if not 0 < view_angle <= 360:
            raise TypeError(
                f"view_angle hat to be in between 1 and 360 degrees: {view_angle}"
            )

        # If necessary differ a little from the exact view_resolution to include start_angle and end_angle
        n_steps = int(round(view_angle / view_resolution))
        if n_steps < 1:
            n_steps = 1
        for i in range(n_steps + 1):
            angle = -view_angle / 2 + i * (view_angle / n_steps)
            yield angle
