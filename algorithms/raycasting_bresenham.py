# TODO: Change to mesa 3.2.0!
#  MultiGridWithProperties -> DiscreteSpace + CellObjects + PropertyLayer

# TODO:
#  If some objects dont block view and some do is_cell_empty cannot be used anymore!

# ------------------------------------------------------------------------------------------
# ???
from mesa.space import MultiGrid
import numpy as np

class MultiGridWithProperties(MultiGrid):
    """
        Representation of the environment. Local knowledge of the environment and the environment itself.
    """
    def __init__(self, width, height, torus):
        super().__init__(width, height, torus)
        self.properties = {
            "unknown": np.full((width, height), True)
        }

    def add_property(self, name: str, default_value=None):
        self.properties[name] = np.full((self.width, self.height), default_value)

    def set_property(self, pos: tuple[int, int], name: str, value):
        self.properties[name][pos[0], pos[1]] = value

    def get_property(self, pos: tuple[int, int], name: str):
        if name not in self.properties:
            raise KeyError(f"Property '{name}' does not exist.")
        return self.properties[name][pos[0], pos[1]]

#------------------------------------------------------------------------------------------
# agent.py

...
    #Initialisierung
    ...
    self.local_environment_memory = MultiGridWithProperties(width, height, torus)
    # Latest perception time for aging of the memory and knowing the newest scan
    self.local_environment_memory.add_property("perception_time")
    ...

# ------------------------------------------------------------------------------------------
# repository.py
from algorithms.raycasting_bresenham import RaycastingBresenham

...
    "raycasting_bresenham": RaycastingBresenham
...

# ------------------------------------------------------------------------------------------
# perception.py
from abc import ABC, abstractmethod
from ??? import MultiGridWithProperties

class Perception(ABC):
    """
    Interface: Procedure for the perception of the environment through the Agents.
    Return: None. Information from world are transferred to agent_local_memory
    """
    @abstractmethod
    def scan_environment(self,
        world: MultiGridWithProperties,
        current_time: int,
        agent_pos: tuple[int, int],
        agent_orientation: int,
        max_range: int, #Evtl. auch nur als Attribut von RaycastingBresenham
        start_angle: int, #Evtl. auch nur als Attribut von RaycastingBresenham
        end_angle: int, #Evtl. auch nur als Attribut von RaycastingBresenham
        resolution: int, #Evtl. auch nur als Attribut von RaycastingBresenham
        agent_local_memory: MultiGridWithProperties,
        blocked_by_objects: bool = True
    ) -> None:
        pass

# ------------------------------------------------------------------------------------------
# raycasting_bresenham.py
from .perception import Perception
from typing import Generator
from ??? import MultiGridWithProperties
import math
import copy

class RaycastingBresenham(Perception):

    def scan_environment(self,
        world: MultiGridWithProperties,
        current_time: int,
        agent_pos: tuple[int, int],
        agent_orientation: int, # 0 = east, 90/-270 = south, +/-180 = west, 270/-90 = north
        max_range: int, # perception range in fields
        start_angle: int,
        end_angle: int,
        resolution: int,
        agent_local_memory: MultiGridWithProperties,
        blocked_by_objects: bool = True
    ) -> None:
        """
        Uses Raycasts with Bresenham's line algorithm to scan the environment in a given area.
        Scanning means transferring properties and objects from the environment to the agents local memory.
        """
        # Generate angles
        for angle in self._angle_generator(start_angle, end_angle, resolution):
            # Send raycasts
            end_pos = self._get_raycast_endpoint(agent_pos, agent_orientation, angle, max_range)
            # Generate bresenham line per raycast and iterate through the cells
            for pos in self._bresenham_line(agent_pos, end_pos):
                # Check grid borders
                if world.out_of_bounds(pos):
                    break
                # Transfer information from world to agent_local_memory
                # - Properties
                for prop_name in world.properties:
                    value = world.get_property(pos, prop_name)
                    agent_local_memory.set_property(pos, prop_name, value)
                # - Objects
                global_objects = world.get_cell_list_contents([pos])
                for obj in global_objects:
                    agent_local_memory.place_agent(copy.deepcopy(obj), pos) #deepcopy for strict separation
                # Set cell als known
                agent_local_memory.set_property(pos, "unknown", False)
                # Set perception_time as now
                agent_local_memory.set_property(pos, "perception_time", current_time)
                # Break at objects (if activated)
                if blocked_by_objects and not world.is_cell_empty(pos):
                    break

    @staticmethod
    def _bresenham_line(
        start_pos: tuple[int, int],
        end_pos: tuple[int, int],
    )-> list[tuple[int, int]]:
        """
        Using Bresenham's line algorithm to list all grid cells (x, y) between start_pos and end_pos (incl.).
        Those can be outside the grid environment. This is handled in further steps.
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
            positions.append((x, y)) # Add endpoint
        else:
            err = dy / 2.0
            while y != y1:
                positions.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
            positions.append((x, y)) # Add endpoint
        return positions

    def _get_raycast_endpoint(self,
        agent_pos: tuple[int, int],
        agent_orientation: int,
        angle: int,
        max_range: int
    ) -> tuple[int, int]:
        """
        Calculates the endpoints of the raycasts as preparation for bresenham-algorithm.
        Those can be outside the grid environment. This is handled in further steps.
        """
        x0, y0 = agent_pos
        dx = max_range * math.cos(math.radians(agent_orientation + angle))
        dy = max_range * math.sin(math.radians(agent_orientation + angle))
        return (int(round(x0 + dx)), int(round(y0 + dy)))


    def _angle_generator(self,
            start_angle: int,
            end_angle: int,
            resolution: int
    ) -> Generator[int, None, None]:
        """
        Generator:
        Generate angles in the given range with the given resolution.
        """
        start_angle = (start_angle + 360) % 360
        end_angle = (end_angle + 360) % 360
        current_angle = start_angle

        if start_angle > end_angle:
            while current_angle < 360:
                yield current_angle
                current_angle += resolution
            current_angle = 0
            while current_angle <= end_angle:
                yield current_angle
                current_angle += resolution
        else:
            while current_angle <= end_angle:
                yield current_angle
                current_angle += resolution