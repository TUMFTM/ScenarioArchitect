import numpy as np
import os
import json
import scenario_testing_tools
import warnings

"""
Python version: 3.7
Created by: Tim Stahl
Created on: 27.04.2019

Sample script extracting scenario data for a given time instance 't_in'. If the given time step is not present in the
data file, linear interpolation is used to generate the desired instance between the neighboring time instances.
"""


def get_scene_timesample(file_path: str,
                         t_in: float,
                         time_f: np.ndarray = None) -> tuple:
    """
    Method extracting scenario data for a given time instance. If the given time step is not present in the data file,
    linear interpolation is used to generate the desired instance between the neighboring time instances.

    :param file_path:       string holding the path to the scene data file
    :param t_in:            two options:
                            * float number holding the time-stamp to be extracted [linear interp. between neighb. pts]
                            * int number holding the number of the data reading to be extracted from the file
    :param time_f:          time-stamps from file (for faster exec.: load once and hand to function on next iter)
    :returns (time,         time stamp of the returned sample
              pos,          position of the ego vehicle (list holding x and y)
              heading,      heading of the ego vehicle (in the global frame)
              curv,         curvature of the path at the position of the ego vehicle
              vel,          velocity of the ego vehicle (in the direction of the heading)
              acc,          acceleration of the ego vehicle (in the direction of the heading)
              ego_traj,     planned ego trajectory starting at the current position (x, y, heading, curv., vel, acc)
              ego_traj_em,  (if available, else ego_traj) planned emergency ego traj (x, y, heading, curv., vel, acc)
              object_array, information about the vehicles in the scene (dict of dicts, each key being the object id and
                            every value holding the following keys ['X', 'Y', 'psi', 'vel', 'length', 'width'])
              time_f)       time-stamps from file (for faster exec.: store this value and hand to function on next iter)
    """
    # -- get timestamps ------------------------------------------------------------------------------------------------
    if time_f is None:
        # load time-stamps from file, if not provided
        time_f = np.genfromtxt(file_path, delimiter=';', skip_header=2, names=True)['time']

    if type(t_in) is int:
        idx = t_in
        if idx >= len(time_f):
            raise ValueError("Provided integer time variable is out of the range of the provided file!")
    else:
        # get timestamp before or at position of provided time step
        if time_f[0] <= t_in <= time_f[-1]:
            idx = next((x[0] for x in enumerate(time_f) if x[1] > t_in), time_f.shape[0]) - 1
        else:
            warnings.warn("Provided time value is out of range in provided data file! Returned last time entry.")

            idx = time_f.shape[0] - 1
            t_in = time_f[idx]

    # -- read relevant lines from file ---------------------------------------------------------------------------------
    header = None
    line_prev = None
    line_next = None
    with open(file_path) as fp:
        for i, line in enumerate(fp):
            if i == 2:
                line = line.replace("\n", "")
                header = line.split(";")
            elif i == idx + 3:
                line = line.replace("\n", "")
                line_prev = line.split(";")
            elif i == idx + 4:
                line = line.replace("\n", "")
                line_next = line.split(";")
                break

    # check object_array for propper format
    object_array_prev = json.loads(line_prev[header.index("object_array")])

    if len(object_array_prev[0][1]) != 6:
        raise ValueError("Provided scenario file does not hold the expected amount of object parameters. Check for "
                         "version compliance (update scenario_testing_tools and scenario-architect to newest version) "
                         "and load + export the relevant scenarios again.")

    # retrieve or interpolate data from file
    if type(t_in) is int or time_f[idx] == t_in or line_next is None:
        # -- extract values at current position ------------------------------------------------------------------------
        time = time_f[idx]
        pos = [float(line_prev[header.index("x")]), float(line_prev[header.index("y")])]
        heading = line_prev[header.index("heading")]
        curv = line_prev[header.index("curv")]
        vel = line_prev[header.index("vel")]
        acc = line_prev[header.index("acc")]
        ego_traj = np.array(json.loads(line_prev[header.index("ego_traj")]))
        if 'ego_traj_em' in header:
            ego_traj_em = np.array(json.loads(line_prev[header.index("ego_traj_em")]))
        else:
            ego_traj_em = ego_traj

        object_list = dict()
        for veh in object_array_prev:
            object_list[veh[0]] = {'X': veh[1][0],
                                   'Y': veh[1][1],
                                   'psi': veh[1][2],
                                   'vel': veh[1][3],
                                   'length': veh[1][4],
                                   'width': veh[1][5]}

    else:
        # -- interpolate between timestamps ----------------------------------------------------------------------------
        time = np.interp(t_in, time_f[idx:idx + 2], time_f[idx:idx + 2])
        pos = [np.interp(t_in, time_f[idx:idx + 2], [float(line_prev[header.index("x")]),
                                                     float(line_next[header.index("x")])]),
               np.interp(t_in, time_f[idx:idx + 2], [float(line_prev[header.index("y")]),
                                                     float(line_next[header.index("y")])])]

        # convert to positive values and back in order to avoid linear interpolation issues
        psi_range = np.array([float(line_prev[header.index("heading")]), float(line_next[header.index("heading")])])
        heading = scenario_testing_tools.interp_heading.interp_heading(heading=psi_range,
                                                                       t_series=time_f[idx:idx + 2],
                                                                       t_in=t_in)

        curv = np.interp(t_in, time_f[idx:idx + 2], [float(line_prev[header.index("curv")]),
                                                     float(line_next[header.index("curv")])])
        vel = np.interp(t_in, time_f[idx:idx + 2], [float(line_prev[header.index("vel")]),
                                                    float(line_next[header.index("vel")])])
        acc = np.interp(t_in, time_f[idx:idx + 2], [float(line_prev[header.index("acc")]),
                                                    float(line_next[header.index("acc")])])

        # get ego-trajectory (interpolate first point for requested t_in)
        ego_traj = np.vstack(
            (interp_1d(x=t_in,
                       xp=time_f[idx:idx + 2],
                       fp_array=np.array([np.array(json.loads(line_prev[header.index("ego_traj")]))[0, :],
                                          np.array(json.loads(line_next[header.index("ego_traj")]))[0, :]]),
                       idx_col_heading=[2]),
             np.array(json.loads(line_next[header.index("ego_traj")])))
        )

        # Check if imported trajectory holds all entries as stated above
        if ego_traj.shape[1] != 6:
            raise ValueError("Could not retrieve all expected trajectory entities (x, y, heading, curv, vel, acc)!")

        # if available, get emergency ego-trajectory (interpolate first point for requested t_in)
        if 'ego_traj_em' in header:
            ego_traj_em = np.vstack(
                (interp_1d(x=t_in,
                           xp=time_f[idx:idx + 2],
                           fp_array=np.array([np.array(json.loads(line_prev[header.index("ego_traj_em")]))[0, :],
                                              np.array(json.loads(line_next[header.index("ego_traj_em")]))[0, :]]),
                           idx_col_heading=[2]),
                 np.array(json.loads(line_next[header.index("ego_traj_em")])))
            )
        else:
            ego_traj_em = ego_traj

        object_array_next = json.loads(line_next[header.index("object_array")])

        object_list = dict()
        for veh_prev, veh_next in zip(object_array_prev, object_array_next):
            interp_obj = interp_1d(x=t_in,
                                   xp=time_f[idx:idx + 2],
                                   fp_array=np.array([veh_prev[1], veh_next[1]]),
                                   idx_col_heading=[2])

            object_list[veh_prev[0]] = {'X': interp_obj[0],
                                        'Y': interp_obj[1],
                                        'psi': interp_obj[2],
                                        'vel': interp_obj[3],
                                        'length': interp_obj[4],
                                        'width': interp_obj[5]}

    return time, pos, heading, curv, vel, acc, ego_traj, ego_traj_em, object_list, time_f


def interp_1d(x: float,
              xp: np.ndarray,
              fp_array: np.ndarray,
              idx_col_heading: list = None) -> np.ndarray:
    """
    Extends the functionality of numpy's 'interp' to an array like structure

    :param x:                desired index for new value
    :param xp:               existing series of indexes
    :param fp_array:         array of existing function return values (matching xp)
    :param idx_col_heading:  list of ints holding columns to be treated as headings (avoid jumps when crossing pi/-pi)
    :returns y_array:        array of interpolated values
    """
    y_array = np.zeros(fp_array.shape[1])
    for col in range(fp_array.shape[1]):
        if idx_col_heading is not None and col in idx_col_heading:
            y_array[col] = scenario_testing_tools.interp_heading.interp_heading(heading=fp_array[:, col],
                                                                                t_series=xp,
                                                                                t_in=x)
        else:
            y_array[col] = np.interp(x, xp, fp_array[:, col])

    return y_array


# -- main --------------------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    scenario_path = (os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
                     + "/sample_files/scenario_n_vehicle/modena_T3_T4_overtake_opp.scn")
    z = get_scene_timesample(file_path=scenario_path,
                             t_in=4.0)
    print(z)
