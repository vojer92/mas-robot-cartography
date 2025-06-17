import math
from typing import Generator
from agents.explorer_robot import ExplorerRobot


class RaycastingBresenham():

    @staticmethod
    def scan_environment(
        agent: ExplorerRobot,
        view_radius: int = 1,
        view_angle: int = 180,
        view_resolution: int = 4
    ) -> list[tuple[int, int]]:
        """
        Uses Raycasts with Bresenham's line algorithm to scan the environment in a given area.
        Scanning means transferring properties and objects from the environment to the agents local memory.
        """
        viewport = []

        neighborhood = agent.cell.get_neighborhood(
            radius=view_radius, include_center=True
        )

        for angle in RaycastingBresenham._angle_generator(
                view_angle, agent.orientation, view_resolution
        ):
            x0, y0 = agent.cell.coordinate
            dx = view_angle * math.cos(math.radians(agent.orientation + angle))
            dy = view_angle * math.sin(math.radians(agent.orientation + angle))
            end_pos = (round(x0 + dx), round(y0 + dy))

            for pos in RaycastingBresenham._bresenham_line(agent.cell.coordinate, end_pos):
                if pos not in [neighbor.coordinate for neighbor in neighborhood]:
                    break
                # TODO: Umstellung auf Property Layer
                if any(
                        isinstance(agent, Obstacle)
                        for agent in neighborhood.select(
                            lambda cell: cell.coordinate == pos
                        ).agents
                ):
                    break

                viewport.append(pos)

                #TODO: Umstellung auf Property Layer
                if any(
                        isinstance(agent, Ground)
                        for agent in neighborhood.select(
                            lambda cell: cell.coordinate == pos
                        ).agents
                ):
                    for agent in neighborhood.select(
                            lambda cell: cell.coordinate == pos
                    ).agents:
                        agent.explored = True
        return viewport

    @staticmethod
    def _bresenham_line(self,
                        start_pos: tuple[int, int],
                        end_pos: tuple[int, int]
                        ) -> list[tuple[int, int]]:
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
    def _angle_generator(self,
                         view_angle: int,
                         orientation: int,
                         view_resolution: int
                         ) -> Generator[int, None, None]:
        """
        Generator
        Generate angles in the given range with the given resolution.
        """
        start_angle = round((orientation - (view_angle / 2) + 360) % 360)
        end_angle = round((orientation + (view_angle / 2) + 360) % 360)
        current_angle = start_angle
        resolution = round(view_angle / view_resolution)

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