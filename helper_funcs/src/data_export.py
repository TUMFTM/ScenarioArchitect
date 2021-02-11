import json
import numpy as np

"""
Python version: 3.7
Created by: Tim Stahl
Created on: 27.04.2019

Functions used to write a given data structure to a file.
"""

# header string - string to be written to the header (titles for the stored columns)
HEADER_STR = "time;x;y;heading;curv;vel;acc;ego_traj;ego_traj_em;object_array;safety_dyn;safety_stat"


def init_exportfile(file_path: str,
                    bound_l: list,
                    bound_r: list) -> None:
    """
    Inputs:
    - file_path:    string holding the path to the scene data file to be generated
    - bound_l/r:    bound coordinates of the track (left and right respectively)
    """
    # write header to logging file
    with open(file_path, "w+") as fh:
        fh.write("# bound_l:" + json.dumps(bound_l, default=default) + "\n")
        fh.write("# bound_r:" + json.dumps(bound_r, default=default) + "\n")
        fh.write(HEADER_STR)


def write_timestamp(file_path: str,
                    time: float,
                    pos: np.ndarray,
                    heading: float,
                    curv: float,
                    vel: float,
                    acc: float,
                    ego_traj: np.ndarray,
                    ego_traj_em: np.ndarray,
                    object_array: list,
                    safety_dyn: bool,
                    safety_stat: bool) -> None:
    """
    Inputs:
    - file_path:    string holding the path to the scene data file to be generated
    - time:         time stamp in the considered time series
    - time:         time stamp of the returned sample
    - pos:          position of the ego vehicle (list holding x and y)
    - heading:      heading of the ego vehicle (in the global frame)
    - curv:         curvature of the path at the position of the ego vehicle
    - vel:          velocity of the ego vehicle (in the direction of the heading)
    - acc:          acceleration of the ego vehicle (in the direction of the heading)
    - ego_traj:     planned ego-trajectory starting at the current position (x, y, heading, curv, vel, acc)
    - ego_traj_em:  planned emergency-ego-trajectory starting at the current position (x, y, heading, curv, vel, acc)
    - object_array: information about the vehicles in the scene (list of lists, each object holding an ID-string and a
                    list holding [x, y, heading, vel, length, width])
    - safety_dyn:   ground truth for safety in a dynamic environment (i.e. w.r.t. other vehicles)
    - safety_stat:  ground truth for safety in a static environment (i.e. w.r.t. bounds, acceleration limits, ...)
    """

    # write line to file (order should match with the "HEADER_STR")
    with open(file_path, "a") as fh:
        fh.write("\n"
                 + str(time) + ";"
                 + str(pos[0]) + ";"
                 + str(pos[1]) + ";"
                 + str(heading) + ";"
                 + str(curv) + ";"
                 + str(vel) + ";"
                 + str(acc) + ";"
                 + json.dumps(ego_traj, default=default) + ";"
                 + json.dumps(ego_traj_em, default=default) + ";"
                 + json.dumps(object_array, default=default) + ";"
                 + json.dumps(safety_dyn) + ";"
                 + json.dumps(safety_stat))


def default(obj):
    # handle numpy arrays when converting to json
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    if isinstance(obj, np.integer):
        return int(obj)
    raise TypeError('Not serializable (type: ' + str(type(obj)) + ')')
