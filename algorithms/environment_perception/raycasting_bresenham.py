import math
from typing import Generator

from agents.ground import Ground
from agents.explorer_robot import ExplorerRobot


class RaycastingBresenham():

    @staticmethod
    def scan_environment(
        agent: ExplorerRobot,
        view_radius: int = 1,
        view_angle: int = 180, # View angle in degrees
        view_resolution: int = 5 # Angle resolution in degrees
    ) -> list[tuple[int, int]]:
        """
        Uses Raycasts with Bresenham's line algorithm to scan the environment in a given area.
        Scanning means transferring properties and objects from the environment to the agents local memory.
        """
        viewport = []

        neighbor_cells = agent.cell.get_neighborhood(
            radius=view_radius, include_center=True
        )
        allowed_coordinates = {cell.coordinate for cell in neighbor_cells}

        for angle in RaycastingBresenham._angle_generator(
                view_angle, view_resolution
        ):
            # Calculate Raycasting end-positions
            x0, y0 = agent.cell.coordinate
            dx = view_radius * math.cos(math.radians(agent.orientation + angle))
            dy = view_radius * math.sin(math.radians(agent.orientation + angle))
            end_pos = (round(x0 + dx), round(y0 + dy))

            for pos in RaycastingBresenham._bresenham_line(agent.cell.coordinate, end_pos):
                # Check for grid borders
                if pos not in allowed_coordinates:
                    break

                current_cells = list(neighbor_cells.select(lambda cell: cell.coordinate == pos))
                if len(current_cells) !=1:
                    raise RuntimeError(f"Cell selection error. Expected 1 cell at position {pos}, found {len(current_cells)}")
                current_cell = current_cells[0]

                # Scan is blocked by some agents
                if any(getattr(agent, "view_blocking", False) is True for agent in current_cell.agents):
                    continue

                # Add position to viewport
                viewport.append(pos)

                # Mark cells as explored via Ground-agents explored-property
                ground_agents = [agent for agent in current_cell.agents if isinstance(agent, Ground)]
                if not ground_agents:
                    raise RuntimeError(
                        f"Environment Error: Cell {pos} has no Obstacle and no Ground agent.")
                ground_agents[0].explored = True

        return viewport

    @staticmethod
    def _bresenham_line(
                        start_pos: tuple[int, int],
                        end_pos: tuple[int, int]
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
                         view_angle: int, # View angle in degrees
                         view_resolution: int = 5 # Angle resolution in degrees
                         ) -> Generator[float, None, None]:
        """
        Generator
        Generate angles in the given angle (symmetric) with the given resolution.
        """
        # Check for valid parameters
        if not 0 < view_angle <= 360 :
            raise TypeError(f"view_angle hat to be in between 1 and 360 degrees: {view_angle}")

        # If necessary differ a little from the exact view_resolution to include start_angle and end_angle
        n_steps = int(round(view_angle / view_resolution))
        if n_steps < 1:
            n_steps = 1
        for i in range(n_steps + 1):
            angle = -view_angle / 2 + i * (view_angle / n_steps)
            yield angle



#        start_angle = -(view_angle / 2)
#        end_angle = (view_angle / 2)
#
#        # Handle view_angle with negative sign
#        if start_angle > end_angle:
#            helper = start_angle
#            start_angle = end_angle
#            end_angle = helper
#
#        current_angle = start_angle
#        while current_angle < end_angle:
#            yield current_angle
#            current_angle += view_resolution