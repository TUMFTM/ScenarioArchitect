import numpy as np
import shutil
import zipfile
import json
import os

"""
Python version: 3.7
Created by: Tim Stahl
Created on: 12.05.2020

Sample script extracting the ego-trajectory for a given scenario file (whole scenario duration).
"""


def get_scene_ego_traj(file_path: str,
                       append_plan: bool = True,
                       rtrn_safety: bool = False) -> tuple:
    """
    Method extracting the ego-trajectory for a given scenario file (whole duration).

    :param file_path:   string holding the path to the scene data file ('*.scn') or Scenario-Architect archive ('*.saa')
    :param append_plan: if 'True': return not only passed poses, but also append planned ego-traj. from last time-stamp
    :param rtrn_safety: if 'True': return a safety rating for each time-stamps, if present in provided file else "None"
    :returns (time,     time stamps along the trajectory
              x,        x-coordinates along the time-stamps of the ego vehicle
              y,        y-coordinates along the time-stamps of the ego vehicle
              heading,  heading of the ego vehicle along the time-stamps
              curv,     curvature of the path at the position of each time-stamp
              vel,      velocity of the ego vehicle at the position of each time-stamp
              acc)      acceleration of the ego vehicle at the position of each time-stamp
              sfty_dyn, (if enabled) safety with respect to other dynamic vehicles for each t [True, False, None]
              sfty_stat)(if enabled) safety with respect to a static environment for each t [True, False, None]
    """

    if not (".saa" in file_path or ".scn" in file_path):
        raise ValueError("Unsupported file! Make sure to provide a Scenario-Architect archive ('*.saa') or a scene data"
                         " file ('*.scn').")

    # if archive, extract relevant file
    if ".saa" in file_path:
        with zipfile.ZipFile(file_path) as zipObj:
            tmp_file = next((x for x in zipObj.namelist() if '.scn' in x), None)

            if tmp_file is not None:
                with zipObj.open(tmp_file) as zf, open(file_path.replace('.saa', '.scn'), 'wb') as f:
                    shutil.copyfileobj(zf, f)
            else:
                raise ValueError("Could not find *.scn file in the provided Scenario-Architect archive!")

    # retrieve data from file
    data = np.genfromtxt(fname=file_path.replace('.saa', '.scn'),
                         delimiter=";",
                         names=True,
                         skip_header=2,
                         usecols=(0, 1, 2, 3, 4, 5, 6))

    if append_plan:
        # get ego-trajectory from last time stamp (since 'x' and 'y' only holds last poses)
        with open(file_path.replace('.saa', '.scn')) as file:
            # get to top of file (1st line)
            file.seek(0)

            file.readline()
            file.readline()
            header = file.readline()[:-1]

            # extract last line
            line = ""
            for line in file:
                pass

            # parse the data objects we want to retrieve from that line
            ego_traj = np.array(json.loads(dict(zip(header.split(";"), line.split(";")))['ego_traj']))

        # calculate distances along ego-trajectory (in order to determine time-stamps)
        distances = np.sqrt(np.sum(np.power(np.diff(ego_traj[:, 0:2], axis=0), 2), axis=1))

        # calculate time-stamps for ego-trajectory
        t = np.concatenate(([0], np.cumsum(np.divide(distances, ego_traj[:-1, 4],
                                                     out=np.full(ego_traj[:-1, 4].shape[0], np.inf),
                                                     where=ego_traj[:-1, 5] != 0))))

        # fuse file data and last trajectory information
        time = np.concatenate((data['time'], t[1:] + data['time'][-1]))
        x = np.concatenate((data['x'], ego_traj[1:, 0]))
        y = np.concatenate((data['y'], ego_traj[1:, 1]))
        heading = np.concatenate((data['heading'], ego_traj[1:, 2]))
        curv = np.concatenate((data['curv'], ego_traj[1:, 3]))
        vel = np.concatenate((data['vel'], ego_traj[1:, 4]))
        acc = np.concatenate((data['acc'], ego_traj[1:, 5]))

    else:
        time = data['time']
        x = data['x']
        y = data['y']
        heading = data['heading']
        curv = data['curv']
        vel = data['vel']
        acc = data['acc']

    # extract safety data (if requested)
    safety_dyn = None
    safety_stat = None
    if rtrn_safety:
        with open(file_path.replace('.saa', '.scn')) as file:
            # get to top of file (1st line)
            file.seek(0)
            file.readline()
            file.readline()
            header = file.readline()[:-1]

            if "safety_dyn" in header and "safety_stat" in header:
                safety_dyn = [None] * len(time)
                safety_stat = [None] * len(time)

                # extract last line
                for i, line in enumerate(file):
                    safety_dyn[i] = json.loads(line.split(";")[header.split(";").index("safety_dyn")])
                    safety_stat[i] = json.loads(line.split(";")[header.split(";").index("safety_stat")])

    # if it was in archive, remove extracted file after import
    if ".saa" in file_path:
        os.remove(file_path.replace('.saa', '.scn'))

    if not rtrn_safety:
        return time, x, y, heading, curv, vel, acc
    else:
        return time, x, y, heading, curv, vel, acc, safety_dyn, safety_stat


# -- main --------------------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    scenario_path = (os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
                     + "/sample_files/scenario_n_vehicle/modena_T3_T4_overtake_opp.saa")
    z = get_scene_ego_traj(file_path=scenario_path)
    print(z)
