import numpy as np
import os

"""
Python version: 3.7
Created by: Tim Stahl
Created on: 12.05.2020

Sample script extracting the ego-trajectory for a given scenario file (whole scenario duration).
"""


def get_scene_ego_traj(file_path: str) -> tuple:
    """
    Method extracting the ego-trajectory for a given scenario file (whole duration).

    :param file_path:   string holding the path to the scene data file
    :returns (time,     time stamps along the trajectory
              x,        x-coordinates along the time-stamps of the ego vehicle
              y,        y-coordinates along the time-stamps of the ego vehicle
              heading,  heading of the ego vehicle along the time-stamps
              curv,     curvature of the path at the position of each time-stamp
              vel,      velocity of the ego vehicle at the position of each time-stamp
              acc)      acceleration of the ego vehicle at the position of each time-stamp
    """

    data = np.genfromtxt(fname=file_path,
                         delimiter=";",
                         names=True,
                         skip_header=2,
                         usecols=(0, 1, 2, 3, 4, 5, 6))

    return data['time'], data['x'], data['y'], data['heading'], data['curv'], data['vel'], data['acc']


# -- main --------------------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    scenario_path = (os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
                     + "/sample_files/scenario_n_vehicle/modena_overtake_tight.scn")
    z = get_scene_ego_traj(file_path=scenario_path)
    print(z)
