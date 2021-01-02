import shutil
import zipfile
import os

import trajectory_planning_helpers as tph

"""
Python version: 3.7
Created by: Tim Stahl
Created on: 13.08.2020

Script providing a method to extract vehicle parameter matrices from a Scenario-Architect archive ('*.saa').
"""


def get_scene_veh_param(file_path: str) -> tuple:
    """
    Method to extract vehicle parameter matrices from a Scenario-Architect archive ('*.saa').

    :param file_path:               string holding path to a Scenario-Architect archive ('*.saa')
    :returns (ggv,                  numpy array holding ggv diagram (first column: velocities, second and third column:
                                    matching allowed acceleration in longitudinal and lateral direction respectively)
              ax_max_machines)      numpy array holding max. possible acceleration from machine at different velocity
                                    levels (first column: velocities, second column: maximal possible acceleration)
    """

    # -- check for valid file type -------------------------------------------------------------------------------------
    if ".saa" not in file_path:
        raise ValueError("Unsupported file! Make sure to provide a Scenario-Architect archive ('*.saa').")

    # -- unzip relevant files ------------------------------------------------------------------------------------------
    with zipfile.ZipFile(file_path) as zipObj:
        zf1_name = next((x for x in zipObj.namelist() if '_ggv.csv' in x), None)
        zf2_name = next((x for x in zipObj.namelist() if '_ax_max_machines.csv' in x), None)

        if zf1_name is None or zf2_name is None:
            raise ValueError("Could not find all vehicle parameters in the Scenario-Architect archive!")

        with zipObj.open(zf1_name) as zf1, zipObj.open(zf2_name) as zf2,\
                open(file_path.replace('.saa', '_ggv.csv'), 'wb') as f1,\
                open(file_path.replace('.saa', '_ax_max_machines.csv'), 'wb') as f2:
            shutil.copyfileobj(zf1, f1)
            shutil.copyfileobj(zf2, f2)

    # -- read relevant lines from file ---------------------------------------------------------------------------------
    ggv, ax_max_machines = tph.import_veh_dyn_info.\
        import_veh_dyn_info(ggv_import_path=file_path.replace('.saa', '_ggv.csv'),
                            ax_max_machines_import_path=file_path.replace('.saa', '_ax_max_machines.csv'))

    # -- cleanup temp-files --------------------------------------------------------------------------------------------
    os.remove(file_path.replace('.saa', '_ggv.csv'))
    os.remove(file_path.replace('.saa', '_ax_max_machines.csv'))

    return ggv, ax_max_machines


# -- main --------------------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    scenario_path = (os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
                     + "/sample_files/scenario_n_vehicle/modena_T3_T4_overtake_opp.saa")
    z = get_scene_veh_param(file_path=scenario_path)
    print(z)
