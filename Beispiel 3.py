# Import necessary libraries
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
# from IPython.display import HTML # Use this for Colab/Jupyter display
import random
import time # To control speed if not using FuncAnimation interval effectively

# --- Simulation Parameters ---
GRID_WIDTH = 30       # Width of the grid world
GRID_HEIGHT = 30      # Height of the grid world
NUM_ROBOTS = 5        # Number of robots
SIMULATION_STEPS = 300 # Number of steps to simulate
OBSTACLE_PERCENT = 0.15 # Percentage of the grid covered by obstacles

# --- Robot Parameters ---
SENSOR_RANGE = 3      # How far a robot can 'see' (radius in grid cells)
COMM_RANGE = 5        # How far a robot can communicate (radius in grid cells)
ROBOT_SPEED = 1       # How many cells a robot moves per step

# --- Grid Cell States ---
UNKNOWN = 0
FREE_EXPLORED = 1
OBSTACLE_EXPLORED = 2
OBSTACLE_UNEXPLORED = 3 # Internal state for grid generation

# --- Visualization Parameters ---
# For non-interactive backends like saving to file, interval is less critical
# Use plt.pause() for simple delay in a standard script
FRAME_DELAY_SECONDS = 0.1 # Seconds to pause between frames

class GridWorld:
    """Represents the simulation environment."""
    def __init__(self, width, height, obstacle_percent):
        self.width = width
        self.height = height
        self.grid = np.full((height, width), UNKNOWN) # Global truth grid
        self._add_obstacles(obstacle_percent)

    def _add_obstacles(self, obstacle_percent):
        """Randomly places obstacles in the grid."""
        num_obstacles = int(self.width * self.height * obstacle_percent)
        for _ in range(num_obstacles):
            while True:
                x = random.randint(0, self.width - 1)
                y = random.randint(0, self.height - 1)
                if self.grid[y, x] == UNKNOWN:
                    self.grid[y, x] = OBSTACLE_UNEXPLORED
                    break

    def is_valid(self, x, y):
        """Checks if coordinates are within grid boundaries."""
        return 0 <= x < self.width and 0 <= y < self.height

    def is_obstacle(self, x, y):
        """Checks if a cell contains an obstacle."""
        if not self.is_valid(x, y):
            return True # Treat out-of-bounds as obstacle
        return self.grid[y, x] == OBSTACLE_UNEXPLORED or self.grid[y, x] == OBSTACLE_EXPLORED

    def get_state(self, x, y):
        """Gets the true state of a cell."""
        if not self.is_valid(x, y):
            return OBSTACLE_EXPLORED # Treat out-of-bounds as known obstacle
        if self.grid[y, x] == OBSTACLE_UNEXPLORED:
             return OBSTACLE_EXPLORED # Return it as explored obstacle if sensed
        elif self.grid[y,x] == UNKNOWN:
             return FREE_EXPLORED # Return unknown as explored free if sensed
        return self.grid[y, x]


class Robot:
    """Represents a single robot agent."""
    def __init__(self, robot_id, grid_world, start_pos=None):
        self.id = robot_id
        self.world_width = grid_world.width
        self.world_height = grid_world.height

        # Find a valid starting position if none provided
        if start_pos and grid_world.is_valid(start_pos[0], start_pos[1]) and not grid_world.is_obstacle(start_pos[0], start_pos[1]):
             self.x, self.y = start_pos
        else:
            while True:
                self.x = random.randint(0, self.world_width - 1)
                self.y = random.randint(0, self.world_height - 1)
                if not grid_world.is_obstacle(self.x, self.y):
                     # Ensure start position is marked as free in the true grid
                     if grid_world.grid[self.y, self.x] == UNKNOWN:
                         grid_world.grid[self.y, self.x] = FREE_EXPLORED
                     break # Found a valid spot

        # Robot's internal knowledge map
        self.known_grid = np.full((self.world_height, self.world_width), UNKNOWN)
        self.next_move = (self.x, self.y) # Planned move for the next step


    def sense(self, grid_world):
        """Update the robot's known_grid based on sensor range."""
        for dy in range(-SENSOR_RANGE, SENSOR_RANGE + 1):
            for dx in range(-SENSOR_RANGE, SENSOR_RANGE + 1):
                # Check Euclidean distance for circular range (optional, square range is simpler)
                # if dx**2 + dy**2 > SENSOR_RANGE**2:
                #     continue

                observe_x, observe_y = self.x + dx, self.y + dy
                if grid_world.is_valid(observe_x, observe_y):
                     # Get the true state from the world and update local map
                    true_state = grid_world.get_state(observe_x, observe_y)
                    # Only update if the new state provides more info (Obstacle > Free > Unknown)
                    if true_state > self.known_grid[observe_y, observe_x]:
                        self.known_grid[observe_y, observe_x] = true_state


    def communicate(self, robots):
        """Exchange map information with nearby robots."""
        # This simple version merges maps directly. A real scenario might exchange deltas.
        for other_robot in robots:
            if self.id == other_robot.id:
                continue # Don't communicate with self

            # Check distance (Manhattan distance for simplicity)
            dist = abs(self.x - other_robot.x) + abs(self.y - other_robot.y)
            if dist <= COMM_RANGE:
                # Merge maps: Assume higher number means more certain information
                # (OBSTACLE > FREE > UNKNOWN)
                # Update both robots' maps with the combined knowledge
                combined_map_part = np.maximum(self.known_grid, other_robot.known_grid)
                self.known_grid = np.maximum(self.known_grid, combined_map_part)
                other_robot.known_grid = np.maximum(other_robot.known_grid, combined_map_part)


    def decide_move(self, grid_world, robots):
        """Decide the next move based on exploration and avoidance."""
        possible_moves = []
        # Consider 8 neighboring cells + staying put
        for dy in [-1, 0, 1]:
            for dx in [-1, 0, 1]:
                target_x, target_y = self.x + dx * ROBOT_SPEED, self.y + dy * ROBOT_SPEED

                # Basic checks: within bounds and not a *known* obstacle
                if not grid_world.is_valid(target_x, target_y):
                    continue
                # Use robot's knowledge, not global truth for decision
                if self.known_grid[target_y, target_x] == OBSTACLE_EXPLORED:
                    continue

                # Check for collision with other robots' *current* positions
                occupied = False
                for r in robots:
                    if r.id != self.id and r.x == target_x and r.y == target_y:
                        occupied = True
                        break
                if occupied:
                    # Allow moving to the same cell IF the other robot is also moving away
                    # This check is still basic and prone to head-on issues if not careful
                    other_robot_is_moving_away = False
                    for r in robots:
                         if r.id != self.id and r.x == target_x and r.y == target_y:
                              if r.next_move != (r.x, r.y): # Check if the other robot has planned a move
                                   other_robot_is_moving_away = True
                              break # Found the occupant
                    if not other_robot_is_moving_away:
                        continue # Collision risk, don't consider this move


                possible_moves.append({'pos': (target_x, target_y), 'is_unknown_adjacent': False, 'is_current_pos': (dx==0 and dy==0)})

        if not possible_moves:
            self.next_move = (self.x, self.y) # Stay put if no valid move
            return

        # --- Prioritize Exploration ---
        unknown_adjacent_moves = []
        stay_put_option = None

        for move_info in possible_moves:
            mx, my = move_info['pos']
            if move_info['is_current_pos']:
                stay_put_option = move_info # Keep track of the 'stay put' option
                continue # Don't evaluate exploration potential for staying put

            # Check immediate vicinity of the target cell for unknown areas in *local* map
            is_near_unknown = False
            for ndy in range(-1, 2):
                 for ndx in range(-1, 2):
                      check_x, check_y = mx + ndx, my + ndy
                      if grid_world.is_valid(check_x, check_y) and self.known_grid[check_y, check_x] == UNKNOWN:
                           is_near_unknown = True
                           move_info['is_unknown_adjacent'] = True
                           break
                 if is_near_unknown:
                      break
            if is_near_unknown:
                unknown_adjacent_moves.append(move_info)

        # Prefer moves leading towards unknown areas
        if unknown_adjacent_moves:
            chosen_move_info = random.choice(unknown_adjacent_moves)
            self.next_move = chosen_move_info['pos']
        elif stay_put_option and len(possible_moves) == 1 : # Only option is to stay put
             self.next_move = stay_put_option['pos']
        else:
            # If no unknown neighbors nearby, move randomly among valid options (excluding stay_put unless it's the only one)
            valid_random_moves = [m['pos'] for m in possible_moves if not m['is_current_pos']]
            if valid_random_moves:
                self.next_move = random.choice(valid_random_moves)
            elif stay_put_option: # Should only happen if blocked on all sides
                 self.next_move = stay_put_option['pos']
            else: # Should not happen if possible_moves was not empty
                 self.next_move = (self.x, self.y)


    def move(self, target_cells_this_step):
        """Execute the planned move, checking for immediate conflicts."""
        target = self.next_move

        # Final check: Is another robot *also* moving to the *exact same* target cell in this step?
        robots_targeting_this_cell = target_cells_this_step.get(target, [])

        # Also check if target is currently occupied by a robot NOT moving away
        currently_occupied_by_stationary = False
        # We need the full robot list here - this suggests maybe move logic belongs outside the class
        # or the robot needs more context. Let's assume this check happened in decide_move
        # for r_check in robots_list_external: # Assume passed in somehow
        #     if r_check.id != self.id and r_check.x == target[0] and r_check.y == target[1] and r_check.next_move == (r_check.x, r_check.y):
        #         currently_occupied_by_stationary = True
        #         break

        if len(robots_targeting_this_cell) > 1 : # or currently_occupied_by_stationary:
            # Conflict! Stay put.
            self.next_move = (self.x, self.y) # Reset plan for next cycle
            # Do not update self.x, self.y
        else:
            # Move is valid
            self.x, self.y = target


# --- Simulation Setup ---
print("Setting up simulation...")
grid_world = GridWorld(GRID_WIDTH, GRID_HEIGHT, OBSTACLE_PERCENT)
robots = [Robot(i, grid_world) for i in range(NUM_ROBOTS)]

# Initial sensing for all robots
for robot in robots:
    robot.sense(grid_world)
print(f"Created {NUM_ROBOTS} robots.")

# --- Visualization Setup ---
plt.ion() # Turn on interactive mode
fig, ax = plt.subplots(figsize=(8, 8))

# Create the initial plot elements (will be updated)
cmap = plt.cm.colors.ListedColormap(['black', 'white', 'red']) # 0=Unk, 1=Free, 2=Obst
bounds = [UNKNOWN, FREE_EXPLORED, OBSTACLE_EXPLORED, OBSTACLE_EXPLORED+1]
norm = plt.cm.colors.BoundaryNorm(bounds, cmap.N)

# Initial combined knowledge display
combined_knowledge = np.copy(robots[0].known_grid)
for r in robots[1:]:
    combined_knowledge = np.maximum(combined_knowledge, r.known_grid)
known_obstacles_mask = (grid_world.grid == OBSTACLE_UNEXPLORED) | (grid_world.grid == OBSTACLE_EXPLORED)
combined_knowledge[known_obstacles_mask] = OBSTACLE_EXPLORED

im = ax.imshow(combined_knowledge, cmap=cmap, norm=norm, origin='lower', interpolation='nearest', animated=True)

# Use scatter for potentially better performance with many points
robot_coords = np.array([[r.x, r.y] for r in robots])
robot_colors = [plt.cm.viridis(r.id / NUM_ROBOTS) for r in robots] # Use viridis, distinct colors
scatter = ax.scatter(robot_coords[:, 0], robot_coords[:, 1], c=robot_colors, s=40, edgecolors='black', zorder=5)

title = ax.set_title("Multi-Robot Exploration - Step 0")
ax.set_xticks(np.arange(-.5, GRID_WIDTH, 1), minor=True)
ax.set_yticks(np.arange(-.5, GRID_HEIGHT, 1), minor=True)
ax.grid(which='minor', color='grey', linestyle='-', linewidth=0.5)
ax.set_xlim(-0.5, GRID_WIDTH - 0.5)
ax.set_ylim(-0.5, GRID_HEIGHT - 0.5)
ax.set_aspect('equal', adjustable='box')
ax.set_xticklabels([])
ax.set_yticklabels([])
ax.tick_params(length=0)

# --- Main Simulation Loop ---
print("Starting simulation loop...")
for step in range(SIMULATION_STEPS):
    # 1. Sensing Phase
    for robot in robots:
        robot.sense(grid_world)

    # 2. Communication Phase
    for i in range(NUM_ROBOTS):
        for j in range(i + 1, NUM_ROBOTS):
            dist = abs(robots[i].x - robots[j].x) + abs(robots[i].y - robots[j].y)
            if dist <= COMM_RANGE:
                # Perform merge communication (already implemented in Robot class method)
                 robots[i].communicate(robots) # This might be inefficient, better pairwise
                 # Pairwise merge:
                 # combined_map_part = np.maximum(robots[i].known_grid, robots[j].known_grid)
                 # robots[i].known_grid = np.maximum(robots[i].known_grid, combined_map_part)
                 # robots[j].known_grid = np.maximum(robots[j].known_grid, combined_map_part)


    # 3. Decision Phase
    for robot in robots:
        robot.decide_move(grid_world, robots) # Pass other robots for collision check

    # 4. Movement Phase (with conflict check)
    planned_targets = {} # target -> list of robot ids
    for r in robots:
        target = r.next_move
        if target not in planned_targets:
            planned_targets[target] = []
        planned_targets[target].append(r.id)

    for robot in robots:
         robot.move(planned_targets) # Pass planned targets for conflict resolution


    # --- Update Visualization ---
    combined_knowledge = np.copy(robots[0].known_grid)
    for r in robots[1:]:
        combined_knowledge = np.maximum(combined_knowledge, r.known_grid)
    known_obstacles_mask = (grid_world.grid == OBSTACLE_UNEXPLORED) | (grid_world.grid == OBSTACLE_EXPLORED)
    combined_knowledge[known_obstacles_mask] = OBSTACLE_EXPLORED

    im.set_data(combined_knowledge)

    # Update scatter plot data
    robot_coords = np.array([[r.x + 0.5, r.y + 0.5] for r in robots]) # Center points in cells
    scatter.set_offsets(robot_coords)


    explored_count = np.sum((combined_knowledge == FREE_EXPLORED) | (combined_knowledge == OBSTACLE_EXPLORED))
    # Calculate total explorable cells correctly (total minus actual obstacles)
    total_actual_obstacles = np.sum(known_obstacles_mask)
    total_explorable_cells = GRID_WIDTH * GRID_HEIGHT - total_actual_obstacles
    explored_percent = (explored_count - total_actual_obstacles) / total_explorable_cells * 100 if total_explorable_cells > 0 else 100
    title.set_text(f"Multi-Robot Exploration - Step {step + 1} ({explored_percent:.1f}% explored)")

    # Redraw the plot and pause
    fig.canvas.draw_idle()
    plt.pause(FRAME_DELAY_SECONDS) # Use pause for delay in standard scripts

    # Optional: Stop if exploration threshold reached
    if explored_percent >= 98.0:
        print(f"Exploration target reached at step {step + 1}")
        break


print("Simulation finished.")
plt.ioff() # Turn off interactive mode
plt.show() # Keep the final plot window open

# --- Optional: Save the final state ---
# final_combined_knowledge = np.copy(robots[0].known_grid)
# for r in robots[1:]:
#     final_combined_knowledge = np.maximum(final_combined_knowledge, r.known_grid)
# known_obstacles_mask = (grid_world.grid == OBSTACLE_UNEXPLORED) | (grid_world.grid == OBSTACLE_EXPLORED)
# final_combined_knowledge[known_obstacles_mask] = OBSTACLE_EXPLORED
# plt.figure(figsize=(8, 8))
# plt.imshow(final_combined_knowledge, cmap=cmap, norm=norm, origin='lower', interpolation='nearest')
# plt.scatter(robot_coords[:, 0], robot_coords[:, 1], c=robot_colors, s=40, edgecolors='black', zorder=5)
# plt.title(f"Final Map State - Step {step + 1} ({explored_percent:.1f}% explored)")
# plt.xticks(np.arange(-.5, GRID_WIDTH, 1), minor=True)
# plt.yticks(np.arange(-.5, GRID_HEIGHT, 1), minor=True)
# plt.grid(which='minor', color='grey', linestyle='-', linewidth=0.5)
# plt.xlim(-0.5, GRID_WIDTH - 0.5)
# plt.ylim(-0.5, GRID_HEIGHT - 0.5)
# plt.aspect('equal', adjustable='box')
# plt.savefig("final_exploration_map.png")
# print("Saved final map state to final_exploration_map.png")