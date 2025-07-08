import numpy as np
import os
from collections import deque #double-ended queue

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

def flood_fill(
    obstacle_grid: np.ndarray,
    start_pos: tuple[int, int],
) -> np.ndarray:
    """
    Does a flood fill / broad first search via Moore-neighborhood on a 2D-grid,
    to determine all from the start position reachable positions.
    :grid: 2D numpy array with obstacle-information (obstacle==1, else 0)
    :start_pos: start position

    """
    reachable = np.zeros_like(obstacle_grid, dtype=bool) #Initialize grid-like array
    queue = deque([start_pos]) #Initialize a double-ended queue and add start_pos

    while queue:
        x,y = queue.popleft() #Takes first element from the queue

        if not (0 <= x < obstacle_grid.shape[0] and 0 <= y < obstacle_grid.shape[1]): #Check grid borders
            continue
        if reachable[x,y]: #Check if already visited
            continue
        if obstacle_grid[x,y] ==1: #Check for obstacle
            continue
        reachable[x,y] = True #Mark position as reachable

        for dx, dy in MOORE_NEIGHBORS: #Add neighbors to queue
            queue.append((x+dx, y+dy))

    return reachable

def _mask_filename(
    seed: int,
    grid_width: int,
    grid_height: int,
    directory: str
) -> str:
    """
    Creates the filename (incl. path) for the mask of a given seed-grid-combination.
    """
    os.makedirs(directory, exist_ok=True) #Check for repository, create if not already existing
    return f"{directory}/mask_seed{seed}_size{grid_width}x{grid_height}.npy"

def save_mask(
    mask: np.ndarray,
    seed: int,
    grid_width: int,
    grid_height: int,
    directory: str = "masks"
) -> None:
    """
    Saves the given mask as .npy file in the given directory
    """
    np.save(_mask_filename(seed, grid_width, grid_height, directory), mask)

def load_mask(
    seed: int,
    grid_width: int,
    grid_height: int,
    directory: str = "masks"
) -> np.ndarray | None:
    """
    Loads a saved mask from .npy file in the given directory"
    """
    filename = _mask_filename(seed, grid_width, grid_height, directory)
    if os.path.exists(filename):
        return np.load(filename)
    return None