import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button

NUM_THETA_CELLS = 45
GRID_SIZE = 300
lats = np.linspace(38.1598, 38.163, num=GRID_SIZE, endpoint=True)
longs = np.linspace(-122.457, -122.451, num=GRID_SIZE, endpoint=True)

def theta_to_stack_number(theta):
    '''
    Takes an angle in radians and returns which stack in the 3D config. space
    this angle corresponds to.
    Angles near 0 go in the lower stack while angles near 2*pi go to the higher stack.
    '''
    new_theta = (theta+2*np.pi) % (2*np.pi)
    stack_number = int(round(new_theta * NUM_THETA_CELLS / (2*np.pi))) % NUM_THETA_CELLS
    return stack_number

def idx(lat_long):
    '''
    lat_long is a list: [lat, long]
    Returns the x,y location/index in the grid
    x is long & y is lat.
    '''
    try:
        x, y = np.nonzero(longs>=lat_long[1])[0][0], np.nonzero(lats>=lat_long[0])[0][0]
        return [x, y]
    except:
        # lats and longs are outside the discretized grid
        return None

class State:
    def __init__(self, long=0, lat=0, yaw=0, g=0, f=0, parent=None):
        self.long, self.lat, self.yaw, self.g, self.f, self.parent = long, lat, yaw, g, f, parent
        self.x, self.y = idx([self.lat, self.long])
        self.theta = theta_to_stack_number(self.yaw)
        # self.theta = self.yaw

    def is_inside(self, grid):
        '''
        True if state lies inside the grid boundries.
        '''
        return not ((self.x < 0 or self.x >= grid.shape[0]) or (self.y < 0 or self.y >= grid.shape[1]))

    def not_obstacle(self, grid):
        '''
        True if state is not inside an obstacle
        '''
        return grid[self.y,self.x] == 0

x_old, y_old = None, None
def onclick(event):
    global x_old, y_old, grid, home_grid, goal_grid
    xp, yp = event.xdata, event.ydata
    if x_old is None and int(xp) != 0 and int(yp) != 0:       # First Click
        x_old, y_old = xp, yp
        print(f'p1: {int(x_old)}, {int(y_old)}')
    elif int(xp) != 0 and int(yp) != 0:                   # Draw rectangle
        if y_old > yp and x_old > xp:
            grid[int(yp):int(y_old), int(xp):int(x_old)] = 1
        elif y_old > yp and x_old < xp:
            grid[int(yp):int(y_old), int(x_old):int(xp)] = 1
        elif y_old < yp and x_old > xp:
            grid[int(y_old):int(yp), int(xp):int(x_old)] = 1
        elif y_old < yp and x_old < xp:
            grid[int(y_old):int(yp), int(x_old):int(xp)] = 1
        ax.cla()    # Clear axis
        # Redrawing
        ax.imshow(grid, cmap='binary', origin='lower')
        ax.plot(home_grid[1], home_grid[0], 'x')
        ax.plot(goal_grid[1], goal_grid[0], 'x')
        ax.grid(b=True, which='major', color='#666666', linestyle='-')
        ax.minorticks_on()
        ax.grid(b=True, which='minor', color='#999999', linestyle='-', alpha=0.2)
        plt.draw()
        x_old, y_old = None, None       # setting for next time
        print(f'p2: {int(xp)}, {int(yp)}')
        print('Free')

pp = []
def button_callback(event):
    global pp, path_c
    start = State(long=home_loc[1], lat=home_loc[0], yaw=start_yaw)
    goal = State(long=goal_loc[1], lat=goal_loc[0], yaw=0)
    path, path_c = search(grid=grid, start=start, goal=goal)
    pp = path
    # ax.imshow(grid, cmap='Greys')
    ax.plot(pp[:,0], pp[:,1], linewidth=3)
    ax.scatter(pp[:,0], pp[:,1])
    plt.draw()

def bicycle(state, delta_rad, speed=2*1e-5, length=10*1e-5):
    next_yaw = state.yaw + speed/length * np.tan(delta_rad)
    if next_yaw < 0 or next_yaw > 2*np.pi: next_yaw = (next_yaw + 2*np.pi) % (2*np.pi)
    next_long = state.long + speed*np.cos(state.yaw)
    next_lat = state.lat + speed*np.sin(state.yaw)
    return next_long, next_lat, next_yaw

def heuristic(node, goal):
    '''
    resembles the distance to the goal location
    '''
    return (abs(node.x - goal.x) + abs(node.y - goal.y))
    # return np.sqrt((y-goal.y)**2 + (x-goal.x)**2)

def expand(state, goal):
    '''
    Returns a list of possible next states for a range of steering angles.
    '''
    next_states = []
    for delta in range(35,-35+5,-5):
        next_long, next_lat, next_yaw = bicycle(state, delta*np.pi/180)
        next_g = state.g + 1
        next_f = next_g + heuristic(State(long=next_long, lat=next_lat, yaw=next_yaw, g=next_g), goal)
        if (next_lat >= 38.1598 and next_lat <= 38.163) and (next_long >= -122.457 and next_long <= -122.451):
            next_states.append(State(next_long, next_lat, next_yaw, next_g, next_f, parent=state))
    
    return next_states

def path_reconstruction(expand_node):
    path = []
    path_c = []
    current = expand_node

    while current.parent:
        elmt = [current.x, current.y, current.yaw]
        elmt_c = [current.lat, current.long, current.yaw]
        path.append(elmt)
        path_c.append(elmt_c)
        current = current.parent

    elmt = [current.x, current.y, current.yaw]
    elmt_c = [current.lat, current.long, current.yaw]
    path.append(elmt)
    path_c.append(elmt_c)

    return np.array(path[::-1]), np.array(path_c[::-1])

def search(grid, start, goal, visualize=False):
    opened = []
    closed = np.zeros((NUM_THETA_CELLS, grid.shape[0], grid.shape[1]))

    state = State(start.long, start.lat, start.yaw, 0, heuristic(start, goal))
    opened.append(state)
    stack_number = state.theta
    closed[stack_number][state.x][state.y] = 1

    if visualize:
        plt.show()
        plt.grid(b=True, which='major', color='#666666', linestyle='-')
        plt.minorticks_on()
        plt.grid(b=True, which='minor', color='#999999', linestyle='-', alpha=0.2)
        ax = plt.gca()
        ax.set_xlim(0, grid.shape[0])
        ax.set_ylim(0, grid.shape[1])

    iteration = 0
    while len(opened) != 0:
        iteration += 1
        opened.sort(key=lambda state:state.f)   # sorting the opened states to start with the lowest f value.
        current = opened.pop(0)                 # returns the first state and removes it from opened list
        # Plotting
        if visualize and iteration%500 == 0:
            plt.imshow(grid, cmap="binary", origin='lower')
            plt.plot(start.x, start.y, 'x')
            plt.plot(goal.x, goal.y, 'x')
            plt.scatter(current.x, current.y, s=7, color='black')
            plt.quiver(current.x, current.y, np.cos(current.yaw), np.sin(current.yaw), units='inches', 
                        scale=5, zorder=3, color='green', width=0.007, headwidth=3, headlength=4)
            path, _ = path_reconstruction(current)
            plt.plot(path[:,0], path[:,1], '--')
            plt.draw()
            plt.pause(0.0001)
        # 
        if (current.x == goal.x) and (current.y == goal.y):   # goal is reached
            print(f'iterations: {iteration}')
            path, path_c = path_reconstruction(current)
            return path, path_c
        
        next_states = expand(current, goal)
        for next_state in next_states:
            stack_number = next_state.theta
            # if not visited before nor an obstacle
            if closed[stack_number][next_state.x][next_state.y] == 0 and next_state.not_obstacle(grid):
                opened.append(next_state)
                closed[stack_number][next_state.x][next_state.y] = 1    # mark as closed/visited
    print("Couldn't Find Path")
    return None

def discretize_world(home_loc, goal_loc):
    '''
    Note that home and goal grid locations returned are ordered as (lang, lat)
    instead of (lat, long) to match the x,y convention of the hybrid A* search
    and to be more intuitive when indexing
    '''
    grid = np.zeros((GRID_SIZE,GRID_SIZE))
    home_grid = np.nonzero(lats>=home_loc[0])[0][0], np.nonzero(longs>=home_loc[1])[0][0]
    goal_grid = np.nonzero(lats>=goal_loc[0])[0][0], np.nonzero(longs>=goal_loc[1])[0][0]
    return grid, home_grid, goal_grid

def planned_path(home_loc_, goal_loc_, home_yaw=0):
    global pp, grid, home_grid, goal_grid, home_loc, goal_loc, ax, start_yaw, path_c
    start_yaw = home_yaw
    home_loc = home_loc_
    goal_loc = goal_loc_
    grid, home_grid, goal_grid = discretize_world(home_loc, goal_loc)

    print(home_grid, '>>>', goal_grid)

    fig, ax = plt.subplots()
    plt.subplots_adjust(bottom=0.3)

    ax.imshow(grid, cmap='binary', origin='lower')
    ax.plot(home_grid[1], home_grid[0], 'x')
    ax.plot(goal_grid[1], goal_grid[0], 'x')
    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    plt.tight_layout()

    ax_button = plt.axes([0.85, 0.05, 0.1, 0.075])
    button = Button(ax_button, 'Find')
    button.on_clicked(button_callback)

    fig.canvas.mpl_connect('button_press_event', onclick)
    ax.grid(b=True, which='major', color='#666666', linestyle='-')
    ax.minorticks_on()
    ax.grid(b=True, which='minor', color='#999999', linestyle='-', alpha=0.2)
    plt.show()
    
    return path_c

if __name__ == "__main__":
    home_loc = [38.161437, -122.454534]
    home_yaw = np.deg2rad(270)
    goal_loc = [38.16210, -122.45653]

    path = planned_path(home_loc, goal_loc, home_yaw)
    plt.plot(path[:, 1], path[:, 0], '--g')
    plt.scatter(path[:, 1], path[:, 0])
    # plt.tick_params('both', length=2, width=1, which='major')
    # plt.tick_params('both', length=2, width=1, which='minor')
    plt.show()
    for waypoint in path:
        print(waypoint)