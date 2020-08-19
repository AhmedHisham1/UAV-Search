import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
from planning_utils import a_star

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
    global pp
    path, cost = a_star(grid=grid, start=home_grid, goal=goal_grid)
    pp = np.array(path)
    # ax.imshow(grid, cmap='Greys')
    ax.plot(pp[:, 1], pp[:, 0], linewidth=3)
    plt.draw()

def planned_path(home_loc, goal_loc, GRID_SIZE=200):
    global pp, grid, home_grid, goal_grid, ax
    grid = np.zeros((GRID_SIZE,GRID_SIZE))

    lats = np.linspace(38.1598, 38.163, num=GRID_SIZE, endpoint=True)
    longs = np.linspace(-122.457, -122.451, num=GRID_SIZE, endpoint=True)

    home_grid = np.nonzero(lats>=home_loc[0])[0][0], np.nonzero(longs>=home_loc[1])[0][0]
    goal_grid = np.nonzero(lats>=goal_loc[0])[0][0], np.nonzero(longs>=goal_loc[1])[0][0]

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
    
    path = np.empty(pp.shape)
    path[:,0] = lats[pp[:,0]]
    path[:,1] = longs[pp[:,1]]
    return path

if __name__ == "__main__":
    home_loc = [38.161437, -122.454534]
    goal_loc = [38.16210, -122.45653]

    path = planned_path(home_loc, goal_loc, GRID_SIZE=200)
    # print(path)
    for waypoint in path:
        print(waypoint)