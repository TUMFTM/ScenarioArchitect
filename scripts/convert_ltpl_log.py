import json
import pickle
import sys
import os
import io
import numpy as np
import warnings

# custom modules
import helper_funcs
import trajectory_planning_helpers as tph

# reference to trajectory planning module (in order to un-pickle the logged graph-base) -> search all parallel folders
module_found = False
for root, dirs, filenames in os.walk(os.path.dirname(os.path.realpath(__file__ + "/../../../"))):
    for dir in dirs:
        if dir == 'mod_local_trajectory':
            sys.path.append(os.path.join(root, dir))
            import ltpl.data_objects   # noqa: F401

            module_found = True
            break

if not module_found:
    warnings.warn('Could not find "mod_local_trajectory", which is required for full functionality!')


"""
Python version: 3.7
Created by: Tim Stahl
Created on: 08.08.2019

Documentation:  Convert a LTPL-log (recorded Roborace log-file of the trajectory planner) to a scenario file (can not be
imported / edited by the Scenario Architect [yet] but replayed).

"""

# the LTPL log only hosts a radius for each object vehicle, by default the length and width is estimated by:
#  * length = radius * 2
#  * width = radius

# in order to enforce a length and width set float values for the variables below, else set them to 'None'
ENFORCED_LENGTH = 4.7
ENFORCED_WIDTH = 2.8


# -- MODIFIED UNPICKLE -------------------------------------------------------------------------------------------------
# provide support for old log files by converting to the new structure
class RenameUnpickler(pickle.Unpickler):
    def find_class(self, module, name):
        renamed_module = module
        if "data_objects" in module and "ltpl" not in module:
            renamed_module = module.replace("data_objects", "ltpl.data_objects")

        return super(RenameUnpickler, self).find_class(renamed_module, name)


def renamed_load(file_obj):
    return RenameUnpickler(file_obj).load()


def renamed_loads(pickled_bytes):
    file_obj = io.BytesIO(pickled_bytes)
    return renamed_load(file_obj)


# -- CONVERSION FUNCTION -----------------------------------------------------------------------------------------------
def convert_ltpl_log(file_in: str,
                     file_out: str,
                     t_start: float = None,
                     t_end: float = None) -> None:
    """
    Convert a LTPL-log (recorded Roborace log-file of the trajectory planner) to a scenario file.

    :param file_in:     string holding the path to the log-file to be converted.
    :param file_out:    string holding the path to the scenario-file to be generated.
    :param t_start:     (optional) trim the returned scenario file to this time-stamp
    :param t_end:       (optional) trim the returned scenario file to this time-stamp
    :return:
    """

    # -- get relevant files and try to read data (graph base and log file) ---------------------------------------------

    # get data file
    file_path_data = file_in[:file_in.rfind("_")] + "_data.csv"

    with open(file_path_data) as file:
        # get to top of file
        file.seek(0)

        # get header (":-1" in order to remove tailing newline character)
        graph_file = file.readline()[1:-1]
        header = file.readline()[:-1]
        content = file.readlines()

    # get graph object
    graph_str_path = os.path.expanduser(os.path.dirname(file_path_data) + '/../Graph_Objects/') + graph_file + '.pckl'

    if module_found and os.path.isfile(graph_str_path):
        try:
            # unpickle graph object
            f = open(graph_str_path, 'rb')
            graph_base = renamed_load(f)
            f.close()

            # get bound coordinates from graph base
            bound_l = graph_base.refline - graph_base.normvec_normalized * np.expand_dims(graph_base.track_width_left,
                                                                                          1)
            bound_r = graph_base.refline + graph_base.normvec_normalized * np.expand_dims(graph_base.track_width_right,
                                                                                          1)

        except ImportError as e:
            warnings.warn("Error while loading graph-base object! Exported file will not host track bound information!"
                          "\nFor a working import, the local trajectory repository must be available on your machine."
                          "\nDetails: " + str(e))

            graph_base = None

            # set empty bounds
            bound_l = []
            bound_r = []
    else:
        warnings.warn("Could not load graph-base object! Please double-check the provided path ("
                      + graph_str_path + ").\nExported file will not host track bound information!")

        graph_base = None

        # set empty bounds
        bound_l = []
        bound_r = []

    # -- extract maximum lat / long acceleration -----------------------------------------------------------------------
    # init ggv [vx, ax_max, ay_max]
    ggv = np.atleast_2d(np.array([100.0, 0.0, 0.0]))

    # loop through all lines
    for count, (line, next_line) in enumerate(zip(content[:-1], content[1:])):
        # progress bar
        tph.progressbar.progressbar(i=count,
                                    i_total=len(content) - 2,
                                    prefix='Scanning vor a_max ')

        # extract action id from next line
        action_id = json.loads(dict(zip(header.split(";"), next_line.split(";")))['action_id_prev'])

        # setup dict based on header and line information
        data = dict(zip(header.split(";"), line.split(";")))

        if action_id not in json.loads(data['kappa_list']).keys():
            action_id = 'straight'
            if 'straight' not in json.loads(data['kappa_list']).keys():
                action_id = 'follow'

        # update ax
        ggv[0, 1] = max(ggv[0, 1], max(np.array(json.loads(data['a_list'])[action_id][0])))

        # update ay
        ggv[0, 2] = max(ggv[0, 2], max(np.power(np.array(json.loads(data['vel_list'])[action_id][0]), 2)
                                       * np.array(json.loads(data['kappa_list'])[action_id][0])))

    # -- init export file ----------------------------------------------------------------------------------------------
    helper_funcs.src.data_export.init_exportfile(file_path=file_out,
                                                 bound_l=bound_l,
                                                 bound_r=bound_r)

    # -- loop through all lines ----------------------------------------------------------------------------------------
    for count, (line, next_line) in enumerate(zip(content[:-1], content[1:])):
        # progress bar
        tph.progressbar.progressbar(i=count,
                                    i_total=len(content) - 2,
                                    prefix='Converting log file')

        # extract action id from next line
        action_id = json.loads(dict(zip(header.split(";"), next_line.split(";")))['action_id_prev'])

        # setup dict based on header and line information
        data = dict(zip(header.split(";"), line.split(";")))

        # -- check if in rage of trimmed are (if provided) -------------------------------------------------------------
        if ((t_start is not None and json.loads(data['time']) < t_start)
                or (t_end is not None and json.loads(data['time']) > t_end)):
            continue

        # -- extract relevant data -------------------------------------------------------------------------------------
        if action_id not in json.loads(data['kappa_list']).keys():
            action_id = 'straight'
            if 'straight' not in json.loads(data['kappa_list']).keys():
                action_id = 'follow'

        # Check if "pos_list" exists in logs (older logs - before 08/2019 - do not hold this information)
        if 'pos_list' in data.keys():
            pos_list = np.array(json.loads(data['pos_list'])[action_id][0])

        else:
            # If 'pos_list' not existent in log, generate based on nodes, const path seg and spline calculation

            if graph_base is None:
                raise ValueError("Could not load graph-base object! \nSince the provided trajectory-log does not hold "
                                 "positions, the graph-base object is mandatory!")

            # const path seg
            const_path_seg = np.array(json.loads(data['const_path_seg']))

            # extract node-coordinates and calculate corresponding spline
            nodes_list = json.loads(data['nodes_list'])[action_id][0]
            psi_list = np.array(json.loads(data['psi_list'])[action_id][0])

            # find start node in nodes_list and cut leading nodes (already represented in const_path_seg)
            for i, node in enumerate(nodes_list):
                if node == json.loads(data['start_node']):
                    nodes_list = nodes_list[i:]
                    break

            # Identify parameters sitting on the nodes (based on number of points in edges of graph_base)
            node_idx = [0]
            spline_param_fuse = np.empty((0, 5))
            dists = np.empty(0)

            # get coordinate of first node
            last_node = None

            for i, node in enumerate(nodes_list):
                if last_node is not None:
                    spline_coeff, spline_param, _, spline_length = graph_base.get_edge(start_layer=last_node[0],
                                                                                       start_node=last_node[1],
                                                                                       end_layer=node[0],
                                                                                       end_node=node[1])

                    # Remove last value of coordinate, psi and curvature, since they appear twice at transitions
                    # between two spline segments -> indicator that is one except in the last iteration
                    if i != len(nodes_list) - 1:
                        rmv_el = -1
                    else:
                        rmv_el = None

                    # Add the coordinates to a single array
                    spline_param_fuse = np.append(spline_param_fuse, spline_param[:rmv_el, :], axis=0)
                    dists = np.append(dists, spline_length)

                    # Provide indexes of loc_path_nodes in loc_path_coords
                    node_idx.append(np.size(spline_param_fuse, axis=0) - 1 * (rmv_el is None))

                last_node = node

            # calculate curvature continous spline
            if psi_list.shape[0] - const_path_seg.shape[0] >= 2:
                psi_s = psi_list[const_path_seg.shape[0]]

                spline_coeff_mat = np.column_stack(tph.calc_splines.
                                                   calc_splines(path=spline_param_fuse[node_idx, 0:2],
                                                                psi_s=psi_s,
                                                                psi_e=spline_param_fuse[-1, 2],
                                                                el_lengths=dists)[0:2])

                # Calculate new curvature, coordinates and headings based on splines
                pos_list = tph.interp_splines.interp_splines(coeffs_x=spline_coeff_mat[:, :4],
                                                             coeffs_y=spline_coeff_mat[:, 4:],
                                                             incl_last_point=True,
                                                             stepnum_fixed=(np.diff(node_idx) + 1).tolist())[0]

                # prepend const_path_seg (if existing)
                if const_path_seg.shape[0] > 1:
                    pos_list = np.vstack((const_path_seg[:-1], pos_list))
            else:
                pos_list = const_path_seg

        # get length of exported trajectory (old logs: coordinates and acceleration profile might differ by one digit)
        n_pts_traj = min(pos_list.shape[0], np.array(json.loads(data['psi_list'])[action_id][0]).shape[0])

        # assemble trajectory (x, y, heading, curvature, vel, acc)
        ego_trajectory = np.column_stack((pos_list[:n_pts_traj, 0],
                                          pos_list[:n_pts_traj, 1],
                                          np.array(json.loads(data['psi_list'])[action_id][0])[:n_pts_traj],
                                          np.array(json.loads(data['kappa_list'])[action_id][0])[:n_pts_traj],
                                          np.array(json.loads(data['vel_list'])[action_id][0])[:n_pts_traj],
                                          np.array(json.loads(data['a_list'])[action_id][0]))[:n_pts_traj])

        # assemble object array
        # information in scene-file:    [..., [id, [x, y, psi, vel, length, width]], ...]
        # information in ltpl-log:      [..., [id, [pos], psi, obj_radius, vel, [prediction]], ...]
        object_array = []
        for obj in json.loads(data['obj_veh']):
            if ENFORCED_LENGTH is None or ENFORCED_WIDTH is None:
                object_array.append([obj[0], [obj[1][0], obj[1][1], obj[2], obj[4], obj[3], obj[3] / 2]])

            else:
                object_array.append([obj[0], [obj[1][0], obj[1][1], obj[2], obj[4], ENFORCED_LENGTH, ENFORCED_WIDTH]])

        # calculate (simple) emergency trajectory
        ego_trajectory_em = helper_funcs.src.calc_brake_emergency.calc_brake_emergency(traj=ego_trajectory,
                                                                                       ggv=ggv)

        # -- store line in scene format --------------------------------------------------------------------------------
        helper_funcs.src.data_export.write_timestamp(file_path=file_out,
                                                     time=json.loads(data['time']),
                                                     pos=pos_list[0, :],
                                                     heading=json.loads(data['psi_list'])[action_id][0][0],
                                                     curv=json.loads(data['kappa_list'])[action_id][0][0],
                                                     vel=json.loads(data['vel_list'])[action_id][0][0],
                                                     acc=json.loads(data['a_list'])[action_id][0][0],
                                                     ego_traj=ego_trajectory,
                                                     ego_traj_em=ego_trajectory_em,
                                                     object_array=object_array)


# ----------------------------------------------------------------------------------------------------------------------
# MAIN SCRIPT ----------------------------------------------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    top_path = os.path.dirname(os.path.realpath(__file__ + "/../"))
    sys.path.append(top_path)

    # -- get log-file name (via arguments or most recent one) ----------------------------------------------------------
    if len(sys.argv) == 2:
        # if one argument provided, assume provided file name
        file_path = sys.argv[1]  # First argument
    else:
        # specific file
        file_path = os.path.expanduser("C:/Users/ga79jix/Downloads/logs_modena/2019_05_17/Run3_DB23_12_05_40_data.csv")

    # -- call file conversion ------------------------------------------------------------------------------------------
    convert_ltpl_log(file_in=file_path,
                     file_out=file_path[:file_path.rfind(".")] + "_converted.scn",
                     t_start=1558095600.0,
                     t_end=1558096579.0)
