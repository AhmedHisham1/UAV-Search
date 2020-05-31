import matplotlib.pyplot as plt
import tilemapbase
import numpy as np

margins = [38.1598, 38.163, -122.457, -122.451]     # lats, lons
degree_range = 0.0015

extent = tilemapbase.Extent.from_lonlat(latitude_min=margins[0]-degree_range, latitude_max=margins[1]+degree_range,
                                        longitude_min=margins[2]-degree_range, longitude_max=margins[3]+degree_range)
extent = extent.to_aspect(1.0)

def osm_plot(data, mode='path', marker='.', color='black', linewidth=2, label=None):
    '''
    uses the tilemapbase library to plot waypoints onto the sonoma raceway map from OSM.
    modes are:
    -'path': to plot a path. expects a list of [lat,lon] waypoints, the first is the home waypoint
             and the last is the target/goal waypoint.
             uses plt.show(), so expect execution to stop when called.
    -'points': plotting points 'scatter' in [lat,lon] list form. 
               You can then choose the marker, the color, and the linewidth of each.
    '''
    fig, ax = plt.subplots(figsize=(6,6), dpi=100)
    plotter = tilemapbase.Plotter(extent, tile_provider=tilemapbase.tiles.build_OSM(), width=600)
    plotter.plot(ax)
    if mode == 'path':
        path = np.array(data)
        path = path[:,0:2]

        points = np.array([tilemapbase.project(*waypoint[::-1]) for waypoint in path])
        x, y = points[:,0], points[:,1]
        # x, y = tilemapbase.project(*goal_loc[::-1])
        
        if len(data) <= 2:
            ax.scatter(x[0],y[0], marker=".", color="black", linewidth=10)
            ax.scatter(x[-1],y[-1], marker="x", color="red", linewidth=2)
        else:
            ax.scatter(x[0],y[0], marker=".", color="black", linewidth=10)
            ax.scatter(x[-1],y[-1], marker="x", color="red", linewidth=2)
            ax.plot(x,y, linewidth=2)
            ax.scatter(x,y, marker=".", color="green", linewidth=1)

        plt.xlabel('Longitude')
        plt.ylabel('Latitude')
        plt.xticks([]),plt.yticks([])
        plt.tight_layout()

        ax.grid(b=True, which='major', color='#666666', linestyle='-')
        ax.minorticks_on()
        ax.grid(b=True, which='minor', color='#999999', linestyle='-', alpha=0.2)
        # plt.draw()
        # plt.pause(0.0001)
        plt.show(block=True)
    elif mode == 'points':
        x, y = tilemapbase.project(*data[::-1])
        ax.scatter(x,y, marker=marker, color=color, linewidth=linewidth, label=label)
        plt.xlabel('Longitude')
        plt.ylabel('Latitude')
        plt.xticks([]),plt.yticks([])
        plt.tight_layout()
        if label:
            plt.legend(loc='upper left')
        ax.grid(b=True, which='major', color='#666666', linestyle='-')
        ax.minorticks_on()
        ax.grid(b=True, which='minor', color='#999999', linestyle='-', alpha=0.2)
        plt.show(block=True)
        

if __name__ == "__main__":
    path = [[  38.1614402,  -122.45452764], [  38.16177789, -122.4551608 ], [  38.16219598, -122.4551608 ],
            [  38.16221206, -122.45519095], [  38.16221206, -122.45612563], [  38.16219598, -122.45615578],
            [  38.16219598, -122.45621608], [  38.1621799,  -122.45624623], [  38.1621799,  -122.45627638],
            [  38.16216382, -122.45630653], [  38.16216382, -122.45633668], [  38.16214774, -122.45636683],
            [  38.16214774, -122.45639698], [  38.16213166, -122.45642714], [  38.16213166, -122.45645729],
            [  38.16211558, -122.45648744], [  38.16211558, -122.45651759]]
    
    # osm_plot(path)
    for point in path[0:2]:
        osm_plot(point, mode='points', marker="$\star$", linewidth=2, color='black', label='Person Location')
