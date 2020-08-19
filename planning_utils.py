from enum import Enum
from queue import PriorityQueue
import numpy as np

class Action(Enum):
    """
    An action is represented by a 3 element tuple.
    The first 2 values are the delta of the action relative to the current grid position. The third and final value
    is the cost of performing the action.
    """
    WEST = (0, -1, 1)
    EAST = (0, 1, 1)
    NORTH = (-1, 0, 1)
    SOUTH = (1, 0, 1)
    NORTH_EAST = (-1, 1, np.sqrt(2))
    NORTH_WEST = (-1, -1, np.sqrt(2))
    SOUTH_WEST = (1, -1, np.sqrt(2))
    SOUTH_EAST = (1, 1, np.sqrt(2))
    def __str__(self):
        if self == self.WEST: return '<'
        elif self == self.EAST: return '>'
        elif self == self.NORTH: return '^'
        elif self == self.SOUTH: return 'v'
        elif self == self.NORTH_EAST: return '^>'
        elif self == self.NORTH_WEST: return '^<'
        elif self == self.SOUTH_WEST: return 'v<'
        elif self == self.SOUTH_EAST: return 'v>'
    
    @property
    def cost(self):
        return self.value[2]
    @property
    def delta(self):
        return (self.value[0], self.value[1])

def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid = list(Action)
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node
    
    # check if the node is off the grid or
    # it's an obstacle
    
    if x - 1 < 0 or grid[int(x-1), int(y)] == 1:
        valid.remove(Action.NORTH)
    if x + 1 > n or grid[int(x+1), int(y)] == 1:
        valid.remove(Action.SOUTH)
    if y - 1 < 0 or grid[int(x), int(y-1)] == 1:
        valid.remove(Action.WEST)
    if y + 1 > m or grid[int(x), int(y+1)] == 1:
        valid.remove(Action.EAST)
    if ((x-1) < 0 and (y+1) > m) or grid[int(x-1), int(y+1)] == 1:
        valid.remove(Action.NORTH_EAST)
    if ((x-1) < 0 and (y-1) < 0) or grid[int(x-1),int(y-1)] == 1:
        valid.remove(Action.NORTH_WEST)
    if ((x+1) > n and (y-1) < 0) or grid[int(x+1), int(y-1)] == 1:
        valid.remove(Action.SOUTH_WEST)
    if ((x+1) > n and (y+1) > m) or grid[int(x+1), int(y+1)] == 1:
        valid.remove(Action.SOUTH_EAST)
    return valid

def visualize_path(grid, path, start):
    sgrid = np.zeros(np.shape(grid), dtype=np.str)
    sgrid[:] = ' '
    sgrid[grid[:] == 1] = 'O'
    
    pos = start
    
    for a in path:
        da = a.value
        sgrid[pos[0], pos[1]] = str(a)
        pos = (pos[0] + da[0], pos[1] + da[1])
    sgrid[pos[0], pos[1]] = 'G'
    sgrid[start[0], start[1]] = 'S'  
    return sgrid

def heuristic(position, goal_position):
    h = np.linalg.norm(np.array(goal_position) - np.array(position))
    return h

def a_star(grid, start, goal, h=heuristic):
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

def actual_path(path, grid_start):
    waypoint = grid_start
    waypoints = []
    for action in path:
        waypoint = (waypoint[0] + action[0], waypoint[1] + action[1])
        waypoints.append(waypoint)
    return waypoints
