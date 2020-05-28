import numpy as np
import json
from matplotlib import pyplot

"""
Python version: 3.7
Created by: Tim Stahl
Created on: 21.05.2019

Script providing methods for track extraction (bounds) from a scenario file as well as the conversion to an occupancy
grid.
"""


def get_scene_track(file_path: str) -> tuple:
    """
    Method extracting the track bounds from a scene file.

    :param file_path:    string holding the path to the scene data file
    :returns (bound_l,   coordinates of the tracks bounds left
              bound_r)   coordinates of the tracks bounds right
    """

    # -- read relevant lines from file ---------------------------------------------------------------------------------
    bound_l = None
    bound_r = None
    with open(file_path) as fp:
        for i, line in enumerate(fp):
            if 'bound_l' in line:
                line = line.replace("# bound_l:", "")
                bound_l = np.array(json.loads(line))
            elif 'bound_r' in line:
                line = line.replace("# bound_r:", "")
                bound_r = np.array(json.loads(line))
            else:
                break

    if bound_l is None or bound_r is None:
        raise ValueError("Something went wrong, while extracting the bound data from the provided file! Check if the"
                         "first two lines of the file hold boundary information.")

    return bound_l, bound_r


def get_scene_occupancy(bound_l: np.ndarray,
                        bound_r: np.ndarray,
                        grid_res: float = 0.1,
                        x_range: tuple = (0.0, 100.0),
                        y_range: tuple = (0.0, 100.0),
                        plot_occupancy=False) -> dict:
    """
    Method generating an occupancy grid from provided bound coordinate arrays.

    :param bound_l:         coordinates of the left bound
    :param bound_r:         coordinates of the right bound
    :param grid_res:        resolution of the occupancy grid (cell width / height in meters)
    :param x_range:         x-coordinate range of the occ. grid to be generated (start and end coordinate, respectively)
    :param y_range:         y-coordinate range of the occ. grid to be generated (start and end coordinate, respectively)
    :param plot_occupancy:  flag, determining whether or not to visualize the occupancy grid
    :returns occ_map [DICT]
      * grid:         occupancy grid (numpy array) holding ones for occupied cells and zeros for unoccupied cells
      * origin:       x, y coordinates of the origin (0, 0) in the occupancy grid
      * resolution:   resolution of the occupancy grid (cell width / height in meters)


    NOTE: The returned occupancy grid holds the standard specs of an image! Therefore, the origin (0, 0) is in the upper
          left corner. Furthermore, the y-axis is given first!
    """

    # initialize matrix of occupancy grid (use predefined min and max bounds)
    origin = [x_range[0], y_range[1]]
    occ_grid = np.zeros((int((y_range[1] - y_range[0]) / grid_res), int((x_range[1] - x_range[0]) / grid_res)))

    for bound in [bound_l, bound_r]:
        for i in range(bound.shape[0] - 1):
            bound_s_x = bound[i][0]
            bound_e_x = bound[i + 1][0]
            bound_s_y = bound[i][1]
            bound_e_y = bound[i + 1][1]

            # get number of samples (maximum of steps required for each of the coordinates)
            n_steps = int(np.ceil(max(np.abs(bound_e_x - bound_s_x) / grid_res,
                                      np.abs(bound_e_y - bound_s_y) / grid_res))) * 2

            # linear interpolation for each coordinate (finer than grid resolution)
            x_steps = np.linspace(bound_s_x, bound_e_x, n_steps)
            y_steps = np.linspace(bound_s_y, bound_e_y, n_steps)

            # loop through all coordinates and set the corresponding bins to one
            for j in range(n_steps):
                x_idx = int((origin[1] - y_steps[j]) / grid_res)
                y_idx = int((x_steps[j] - origin[0]) / grid_res)

                # if index in bounds of occupancy grid
                if 0 <= x_idx < occ_grid.shape[0] and 0 <= y_idx < occ_grid.shape[1]:
                    occ_grid[x_idx, y_idx] = 1.0

    # plot occupancy, if flag is set
    if plot_occupancy:
        pyplot.imshow(occ_grid, aspect='equal', cmap='Greys')
        pyplot.show()

    # assemble output dict
    occ_map = {'grid': occ_grid,
               'origin': origin,
               'resolution': grid_res}

    return occ_map


# -- main --------------------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    pass
    # z = get_scene_occupancy(bound_l=,
    #                         bound_r=,
    #                         x_range=(0.0, 100.0),
    #                         y_range=(0.0, 100.0),
    #                         plot_occupancy=True)
    # print(z)
