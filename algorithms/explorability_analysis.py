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
    to determine all from the start position explorable positions.
    :grid: 2D numpy array with obstacle-information (obstacle==1, else 0)
    :start_pos: start position

    """
    explorable = np.zeros_like(obstacle_grid, dtype=bool) #Initialize grid-like array
    queue = deque([start_pos]) #Initialize a double-ended queue and add start_pos

    while queue:
        x,y = queue.popleft() #Takes first element from the queue

        if not (0 <= x < obstacle_grid.shape[0] and 0 <= y < obstacle_grid.shape[1]): #Check grid borders
            continue
        if explorable[x,y]: #Check if already visited
            continue
        explorable[x,y] = True #Mark position as explorable
        if obstacle_grid[x,y] ==1: #Check for obstacle: Cell is explorable, but the current flood-fill-path ends here
            continue
        for dx, dy in MOORE_NEIGHBORS: #Add neighbors to queue
            queue.append((x+dx, y+dy))

    return explorable



def _mask_unexplorable_filename(
    seed: int,
    grid_width: int,
    grid_height: int,
    no_agents: int,
    directory: str
) -> str:
    """
    Creates the filename (incl. path) for the mask of a given seed-grid-combination.
    """
    os.makedirs(directory, exist_ok=True) #Check for repository, create if not already existing
    return f"{directory}/mask_seed{seed}_size{grid_width}x{grid_height}_no_agents{no_agents}.npy"

def save_unexplorable_mask(
    mask: np.ndarray,
    seed: int,
    grid_width: int,
    grid_height: int,
    no_agents: int,
    directory: str = "masks"
) -> None:
    """
    Saves the given mask as .npy file in the given directory
    """
    np.save(_mask_unexplorable_filename(seed, grid_width, grid_height, no_agents, directory), mask)

def load_unexplorable_mask(
    seed: int,
    grid_width: int,
    grid_height: int,
    no_agents: int,
    directory: str = "masks"
) -> np.ndarray | None:
    """
    Loads a saved mask from .npy file in the given directory"
    """
    filename = _mask_unexplorable_filename(seed, grid_width, grid_height, no_agents, directory)
    if os.path.exists(filename):
        return np.load(filename)
    return None



def _no_unexplorable_filename(
        seed: int,
        grid_width: int,
        grid_height: int,
        no_agents: int,
        directory: str
) -> str:
    os.makedirs(directory, exist_ok=True) #Check for repository, create if not already existing
    return f"{directory}/no_unexplorable_seed{seed}_size{grid_width}x{grid_height}_no_agents{no_agents}.txt"

def save_no_unexplorable(
    no_unexplorable: int,
    seed: int,
    grid_width: int,
    grid_height: int,
    no_agents: int,
    directory: str = "masks"
) -> None:
    filename = _no_unexplorable_filename(seed, grid_width, grid_height, no_agents, directory)
    with open(filename, "w") as f:
        f.write(str(no_unexplorable))

def load_no_unexplorable(
    seed: int,
    grid_width: int,
    grid_height: int,
    no_agents: int,
    directory: str = "masks"
) -> int | None:
    filename = _no_unexplorable_filename(seed, grid_width, grid_height, no_agents, directory)
    if os.path.exists(filename):
        with open(filename, "r") as f:
            return int(f.read())
    return None