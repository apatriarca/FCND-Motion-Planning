from enum import Enum
from queue import PriorityQueue
import numpy as np
import re


def read_obstacles(filename):
    """
    Returns the global home position (latitude and longitude) and an array representing each obstacles in the format
    (north, east, altitude, delta_north, delta_east, delta_altitude).
    """
    with open(filename, 'r') as f:
        # Read lat0 and lon0 from first row
        first_row = f.readline()
        match = re.search(r"^lat0 (?P<lat>[-0-9.]+), lon0 (?P<lon>[-0-9.]+)\n$", first_row)
        assert match, "Invalid format of 'colliders.csv' first row: {0}".format(first_row)
        lat0 = float(match.group("lat"))
        lon0 = float(match.group("lon"))
        print("lat0 {0}, lon0 {1}".format(lat0, lon0))

        # Skip second row
        f.readline()

        # read the rest in a numpy array
        obstacles = np.loadtxt(f, delimiter=',', dtype='Float64')
        return (lat0, lon0), obstacles


def create_grid(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))

    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min, 0, north_size-1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size-1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size-1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size-1)),
            ]
            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1

    return grid, int(north_min), int(east_min)


# Assume all actions cost the same.
class Action(Enum):
    """
    An action is represented by a 3 element tuple.

    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """

    WEST = (0, -1, 1)
    EAST = (0, 1, 1)
    NORTH = (-1, 0, 1)
    SOUTH = (1, 0, 1)
    NORTH_WEST = (-1, -1, np.sqrt(2))
    NORTH_EAST = (-1, 1, np.sqrt(2))
    SOUTH_WEST = (1, -1, np.sqrt(2))
    SOUTH_EAST = (1, 1, np.sqrt(2))

    @property
    def cost(self):
        return self.value[2]

    @property
    def delta(self):
        return self.value[0], self.value[1]


def bres(p1, p2):
    """
    Computes the bresenham line tracing.
    """
    x1, y1 = p1
    x2, y2 = p2
    dx = x2 - x1
    dy = y2 - y1

    # Negative dx or negative dy vertical line (flip vertices)
    if dx < 0 or (dx == 0 and dy < 0):
        return bres(p2, p1)

    # Vertical line
    if dx == 0:
        return [(x1, y) for y in range(y1, y2)]

    # Horizontal line
    if dy == 0:
        return [(x, y1) for x in range(x1, x2)]

    # abs(dy/dx) > 1 (flip coordinates)
    if abs(dy) > abs(dx):
        cells = bres((y1, x1), (y2, x2))
        return [(y, x) for x, y in cells]

    cells = []
    eps = 0
    y = y1

    if dy > 0:
        for x in range(x1, x2):
            cells.append((x, y))
            v = 2 * (eps + dy)
            if v >= dx:
                f = dy * (x + 1 - x1) - dx * (y + 1 - y1)
                if f > 0:
                    cells.append((x, min(y + 1, y2 - 1)))
                elif f < 0:
                    cells.append((min(x + 1, x2 - 1), y))
                y = min(y + 1, y2 - 1)
                eps -= dx
            eps += dy
    else:
        for x in range(x1, x2):
            cells.append((x, y - 1))
            v = 2 * (eps + dy)
            if v <= - dx:
                f = dy * (x + 1 - x1) - dx * (y - 1 - y1)
                if f > 0:
                    cells.append((min(x + 1, x2 - 1), y - 1))
                elif f < 0:
                    cells.append((x, max(y - 2, y2)))
                y = max(y - 1, y2 - 1)
                eps += dx
            eps += dy

    return np.array(cells)


def reduce_path(path, grid):
    """
    Returns a path without collinear waypoints and waypoints with a free line path between them
    """
    if len(path) > 2:
        new_path = [path[0], path[1]]
        for k in range(2, len(path)):
            p1 = new_path[-2]
            p2 = new_path[-1]
            p3 = path[k]
            area = p1[0] * (p2[1] - p3[1]) + p2[0] * (p3[1] - p1[1]) + p3[0] * (p1[1] - p2[1])
            if area == 0:  # collinear
                new_path[-1] = path[k]
            else:
                cells = bres(p1, p3)
                free_line = True
                for c in cells:
                    if grid[c[0], c[1]] == 1:
                        free_line = False

                if free_line:
                    new_path[-1] = path[k]
                else:
                    new_path.append(path[k])

        return new_path
    else:
        return path


def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid_actions = list(Action)
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node

    # check if the node is off the grid or
    # it's an obstacle

    if x - 1 < 0 or grid[x - 1, y] == 1:
        valid_actions.remove(Action.NORTH)
    if x + 1 > n or grid[x + 1, y] == 1:
        valid_actions.remove(Action.SOUTH)
    if y - 1 < 0 or grid[x, y - 1] == 1:
        valid_actions.remove(Action.WEST)
    if y + 1 > m or grid[x, y + 1] == 1:
        valid_actions.remove(Action.EAST)

    if x - 1 < 0 or y - 1 < 0 or grid[x - 1, y - 1] == 1:
        valid_actions.remove(Action.NORTH_WEST)
    if x - 1 < 0 or y + 1 > m or grid[x - 1, y + 1] == 1:
        valid_actions.remove(Action.NORTH_EAST)
    if x + 1 > n or y - 1 < 0 or grid[x + 1, y - 1] == 1:
        valid_actions.remove(Action.SOUTH_WEST)
    if x + 1 > n or y + 1 > m or grid[x + 1, y + 1] == 1:
        valid_actions.remove(Action.SOUTH_EAST)

    return valid_actions


def a_star(grid, h, start, goal):

    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False
    
    while not queue.empty():
        item = queue.get()
        current_node = item[1]
        if current_node == start:
            current_cost = 0.0
        else:              
            current_cost = branch[current_node][0]
            
        if current_node == goal:        
            print('Found a path.')
            found = True
            break
        else:
            for action in valid_actions(grid, current_node):
                # get the tuple representation
                da = action.delta
                next_node = (current_node[0] + da[0], current_node[1] + da[1])
                branch_cost = current_cost + action.cost
                queue_cost = branch_cost + h(next_node, goal)
                
                if next_node not in visited:                
                    visited.add(next_node)               
                    branch[next_node] = (branch_cost, current_node, action)
                    queue.put((queue_cost, next_node))
             
    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************') 
    return path[::-1], path_cost



def heuristic(position, goal_position):
    return np.linalg.norm(np.array(position) - np.array(goal_position))

