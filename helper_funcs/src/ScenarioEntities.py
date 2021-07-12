import numpy as np
from sklearn.metrics.pairwise import euclidean_distances
from numpy import matlib
import matplotlib.pyplot as plt
import matplotlib.patches as ptch
import matplotlib.patheffects as patheffcts

import scenario_testing_tools

"""
Python version: 3.7
Created by: Tim Stahl
Created on: 27.05.2020

Class structure used to define and interact with all possible entities in a scenario.
"""


class Entity:
    """
    Entity class beeing the parent of all entities in a scenario (bounds and vehicles).
    Common variables and methods are defined and initialized here.
    """

    TYPE_BOUND = False
    TYPE_VEHICLE = False
    TYPE_EGO = False

    # toggle mode (temporary mode during edit)
    tmp_mode = False

    # coordinate sequence of the entity (numpy array with columns x, y)
    data_coord = None

    # temporary sequence of coordinates (while editing - not stored)
    data_coord_tmp = None

    # unique ID
    id = None

    def get_mode_coord(self) -> np.ndarray:
        """
        Returns coordinates depending on internal mode (temporary or normal).
        :return data_coord: numpy array holding the path coordinates (columns: x, y)
        """

        if self.tmp_mode:
            return self.data_coord_tmp
        else:
            return self.data_coord


class Bound(Entity):
    """
    Bound entity holding all variables (memory) and functions to describe and visualize a single boundary (x, y-coord.).
    """

    def __init__(self,
                 id_in: str,
                 plane_axis: plt.axes,
                 color_str: str = 'black') -> None:
        """
        Initializes a bound instance (stores relevant data and plot elements)

        :param id_in:       unique id-string specifing the bound
        :param plane_axis:  axes handle for the main plane
        :param color_str:   matplotlib color string (either hash or matplotlib names color)
        """

        self.TYPE_BOUND = True

        # - INIT VARIABLES ---------------------------------------------------------------------------------------------
        # unique ID
        self.id = id_in

        # - INIT PLOT HANDLES ------------------------------------------------------------------------------------------
        # bound handle
        self.__bound_handle, = plane_axis.plot([], [], lw=1, color=color_str, zorder=50)

    def update_xy_plot(self,
                       reset: bool = False) -> None:
        """
        Update the internal storage of the path in the xy-plane.

        :param reset:     if 'True' any present trajectory is removed
        :return:
        """

        if reset:
            self.__bound_handle.set_data([[], []])

        else:
            self.__bound_handle.set_data([self.get_mode_coord()[:, 0], self.get_mode_coord()[:, 1]])

    def __del__(self) -> None:
        """
        Destructor - cleanup of all plotting handles.
        :return:
        """

        # remove plots
        self.__bound_handle.remove()


class Vehicle(Entity):
    """
    Vehicle entity holding all variables (memory) and functions to describe and visualize a single vehicle.
    This information includes not only the path (x, y-coordinate), but also the temporal information (v, a).
    """

    def __init__(self,
                 i_in: int,
                 plane_axis: plt.axes,
                 a_t_axis: plt.axes,
                 v_t_axis: plt.axes,
                 v_s_axis: plt.axes,
                 ssm_t_axis: plt.axes,
                 veh_length: float = 4.7,
                 veh_width: float = 2.0,
                 color_str: str = 'blue',
                 plan_hor: float = 2.0,
                 type_ego: bool = False) -> None:
        """
        Initializes a vehicle instance (stores relevant data and plot handles).

        :param i_in:        number of vehicle (for generation of id)
        :param plane_axis:  axes handle for the main plane
        :param a_t_axis:    axes handle for the acceleration over time plot
        :param v_t_axis:    axes handle for the velocity over time plot
        :param v_s_axis:    axes handle for the velocity over path plot
        :param ssm_t_axis:  axes handle for the SSM over time plot
        :param veh_length:  length of vehicle in m
        :param veh_width:   width of vehicle in m
        :param color_str:   matplotlib color string (either hash or matplotlib names color)
        :param plan_hor;    planning horizon in seconds for visualized trajectory (only relevant for ego-vehicle)
        :param type_ego:    'True' for the ego-vehicle
        """

        self.TYPE_VEHICLE = True
        self.TYPE_EGO = type_ego

        # - INIT VARIABLES ---------------------------------------------------------------------------------------------
        # unique ID
        self.id = 'veh_' + str(i_in)

        # coordinate sequence of the entity used to calculate latest vx and ax profile (numpy array with columns: x, y)
        # NOTE: e.g. used to trigger vel-(re)calculation, when data_coord is not same as data_vel_coord
        self.data_vel_coord = None

        # export data (numpy array with columns: s, x, y, psi, kappa, vx, ax)
        self.data_exp = None

        # surrogate safety metric data
        self.data_ssm = None

        # temporary data container (used to store any data with this object)
        self.temp_container = dict()

        # vehicle dimensions
        self.__length = veh_length
        self.__width = veh_width
        self.__plan_horizon = plan_hor

        # - INIT PLOT HANDLES ------------------------------------------------------------------------------------------
        self.__plane_axis = plane_axis
        self.__color_str = color_str
        self.__veh_plane_handle, = plane_axis.plot([], [], lw=1, color=color_str, zorder=99)

        # acceleration plot handles (combined, ax, ay)
        self.__acomb_t_handle, = a_t_axis.plot([], [], lw=1, color=color_str, zorder=99, label='$a_{cum}$')
        self.__ax_t_handle, = a_t_axis.plot([], [], '-.', lw=1, color=color_str, zorder=99, label='$a_x$')
        self.__ay_t_handle, = a_t_axis.plot([], [], ':', lw=1, color=color_str, zorder=99, label='$a_y$')

        if type_ego:
            a_t_axis.legend(bbox_to_anchor=(1, 1.02), loc="lower right", borderaxespad=0, ncol=3, columnspacing=0.6,
                            handlelength=1, handletextpad=0.1)

        # velocity plot handles (over time and path)
        self.__vx_t_handle, = v_t_axis.plot([], [], lw=1, color=color_str, zorder=99)
        self.__vx_s_handle, = v_s_axis.plot([], [], lw=1, color=color_str, zorder=99)

        # ttc plot handles
        self.__ttc_t_handle, = ssm_t_axis.plot([], [], ':', lw=1, color=color_str, zorder=99, label='TTC')
        self.__dss_t_handle, = ssm_t_axis.plot([], [], lw=1, color=color_str, zorder=99, label='DSS')

        if type_ego:
            ssm_t_axis.legend(bbox_to_anchor=(1, 1.02), loc="lower right", borderaxespad=0, ncol=4, columnspacing=0.6,
                              handlelength=1, handletextpad=0.1)

        # initialize patch handle for vehicle plots and corresponding text labels
        self.__veh_ptch_handle = {}
        self.__veh_text_handle = {}

        # if ego, initialize trajectory visualization handle
        self.__traj_ptch_hdl = None
        if type_ego:
            self.__traj_ptch_hdl, = plane_axis.plot([], [], lw=3, color='red', zorder=98)

    def get_timestamp_info(self,
                           t_in: float) -> tuple or None:
        """
        Extract all available information about a given time stamp (interpolate between stored data points).

        CAUTION: all entities are interpolated linearly - this may cause inaccuracies, especially for the acceleration

        :param t_in:        time-stamp to be returned (interpolated linearly between stored values)
        :returns (pos,      position [x, y]
                  heading,  heading angle
                  curv,     curvature of path at pos
                  vel,      velocity along heading
                  acc)      acceleration
        """

        if self.data_coord is not None and self.data_exp is not None:
            # calculate time values for velocity profile
            t = np.concatenate(([0], np.cumsum(np.divide(np.diff(self.data_exp[:, 0]), self.data_exp[:-1, 5],
                                                         out=np.full(self.data_exp[:-1, 5].shape[0], np.inf),
                                                         where=self.data_exp[:-1, 5] != 0))))

            if t_in > max(t):
                pos_idx = len(t) - 1
                pos = self.data_exp[pos_idx, 1:3]
                heading = self.data_exp[pos_idx, 3]
                curv = self.data_exp[pos_idx, 4]
                return pos, heading, curv, None, None
            else:
                pos = [np.interp(t_in, t, self.data_exp[:, 1]), np.interp(t_in, t, self.data_exp[:, 2])]
                heading = scenario_testing_tools.interp_heading.interp_heading(heading=self.data_exp[:, 3],
                                                                               t_series=t,
                                                                               t_in=t_in)
                curv = np.interp(t_in, t, self.data_exp[:, 4])
                vel = np.interp(t_in, t, self.data_exp[:, 5])
                acc = np.interp(t_in, t, self.data_exp[:, 6])
                return pos, heading, curv, vel, acc
        else:
            return None

    def get_timestamp_info_lanebased(self,
                                     t_in: float) -> tuple or None:
        """
        Extract all available information about a given time stamp (interpolate between stored data points). The
        information is lane-based. (For example: calculation of the lane-based values of the ego vehicle, needed for
        ttc evaluation)

        :param t_in:        time-stamp to be returned (interpolated linearly between stored values)
        :returns (pos,      position [x, y]
                  heading,  heading angle
                  vel)      velocity along heading
        """

        if self.data_coord is not None and self.data_exp is not None:
            # calculate time values for velocity profile
            t = np.concatenate(([0], np.cumsum(np.divide(np.diff(self.data_ssm[:, 3]), self.data_ssm[:-1, 6],
                                                         out=np.full(self.data_ssm[:-1, 6].shape[0], np.inf),
                                                         where=self.data_ssm[:-1, 6] != 0))))

            if t_in > max(t):
                pos_idx = len(t) - 1
                pos = self.data_ssm[pos_idx, 3:5]
                heading = self.data_ssm[pos_idx, 5]
                return pos, heading, None
            else:
                pos = [np.interp(t_in, t, self.data_ssm[:, 3]), np.interp(t_in, t, self.data_ssm[:, 4])]
                heading = scenario_testing_tools.interp_heading.interp_heading(heading=self.data_ssm[:, 5],
                                                                               t_series=t,
                                                                               t_in=t_in)
                vel = np.interp(t_in, t, self.data_ssm[:, 6])
                return pos, heading, vel
        else:
            return None

    def highlight_pose(self,
                       t_in: float or None) -> None:
        """
        Highlight a vehicle pose with a bounding-box (if possible).

        :param t_in:        time-stamp to be highlighted (if 'None', the plot handle will be removed)
        :return:
        """

        t_info = None
        if t_in is not None:
            t_info = self.get_timestamp_info(t_in=t_in)

            # if requested t exceeds time-series (t_info[4] = None), do not plot vehicle
            if t_info is not None and t_info[4] is None:
                t_info = None

        if t_info is not None:
            self.plot_vehicle(pos=t_info[0],
                              heading=[t_info[1]],
                              zorder=102,
                              color_str=self.__color_str)

            # additionally highlight trajectory for ego-vehicle
            if self.TYPE_EGO:
                self.plot_traj(t_in=t_in,
                               t_horizon=self.__plan_horizon)
        else:
            if 'default' in self.__veh_ptch_handle.keys():
                self.__veh_ptch_handle['default'].remove()
                del self.__veh_ptch_handle['default']

            # if ego, reset trajectory
            if self.TYPE_EGO:
                self.plot_traj(t_in=None)

        # update plot
        self.__plane_axis.figure.canvas.draw_idle()

    def plot_vehicle(self,
                     pos: np.ndarray,
                     heading: list,
                     zorder: int = 10,
                     color_str: str = 'blue',
                     id_in: str = 'default') -> None:
        """
        Plot the vehicle-bounding-box for one or multiple vehicle poses.

        :param pos:        numpy array holding one or multiple positions (columns x, y; each row a pose)
        :param heading:    list of floats holding the heading for each position (if only one pose, list with one float)
        :param zorder:     (optional) integer defining z-order of the vehicle
        :param color_str:  (optional) string specifying the color of the plotted vehicle
        :param id_in:      (optional) string specifying id (used to delete the same type of vehicle plotted previously)
        :return:
        """

        # force position to be 2-dimensional
        pos = np.atleast_2d(pos)

        # delete highlighted positions with handle
        if id_in in self.__veh_ptch_handle.keys():
            self.__veh_ptch_handle[id_in].remove()
            del self.__veh_ptch_handle[id_in]

            # check for further time instances from the last time-step
            counter = 1
            while id_in + "_i" + str(counter) in self.__veh_ptch_handle.keys():
                self.__veh_ptch_handle[id_in + "_i" + str(counter)].remove()
                del self.__veh_ptch_handle[id_in + "_i" + str(counter)]
                counter += 1

        counter = 0
        # for every position to be plotted
        for head in heading:
            theta = head - np.pi / 2
            pos_i = pos[counter, :]

            bbox = (matlib.repmat([[pos_i[0]], [pos_i[1]]], 1, 4)
                    + np.matmul([[np.cos(theta), -np.sin(theta)],
                                 [np.sin(theta), np.cos(theta)]],
                                [[-self.__length / 2, self.__length / 2, self.__length / 2, -self.__length / 2],
                                 [-self.__width / 2, -self.__width / 2, self.__width / 2, self.__width / 2]]))

            patch = np.array(bbox).transpose()
            patch = np.vstack((patch, patch[0, :]))

            # for counter >0 generate further ids:
            handle_id = id_in
            if counter > 0:
                handle_id += "_i" + str(counter)

            plt_patch = ptch.Polygon(patch, facecolor=color_str, linewidth=1, edgecolor='k', zorder=zorder)
            self.__veh_ptch_handle[handle_id] = self.__plane_axis.add_artist(plt_patch)

            counter += 1

    def plot_text(self,
                  pos: np.ndarray,
                  heading: list,
                  text_list: list,
                  text_dist: float = 10.0,
                  plot_ith: int = 2,
                  avoid_pos: np.ndarray = None,
                  zorder: int = 100,
                  id_in: str = 'default') -> None:
        """
        Plot text (perpendicular) next to poses. The heading is used to determine in a perpendicular direction the
        offset of the text to the pose (allows to plot text next to certain poses on a spline). If empty arrays are
        provided, all previous instances with the specified 'id_in' will be removed.

        :param pos:        numpy array holding one or multiple positions  (columns x, y; each row a pose)
        :param heading:    list of floats holding the heading for each position (if only one pose, list with one float)
        :param text_list:  list of texts to be printed next to each pose (if only one pose, list with one text)
        :param text_dist:  (optional) distance the text should be away from 'pos' in m
        :param plot_ith:   (optional) plot only the text for every i-th pose (e.g. if '1' every pose, if '2' every 2nd)
        :param avoid_pos:  (optional) numpy array of positions to be avoided by text
                                      -> Select favored side (left/right) of heading
        :param zorder:     (optional) z-order of the vehicle
        :param id_in:      (optional) string specifying the id (used to delete same type of vehicle plotted previously)
        :return:
        """

        # force position to be 2-dimensional
        pos = np.atleast_2d(pos)

        # delete highlighted positions with handle
        if id_in in self.__veh_text_handle.keys():
            self.__veh_text_handle[id_in].remove()
            del self.__veh_text_handle[id_in]

            # check for further time instances from the last time-step
            counter = 0 + plot_ith
            while id_in + "_i" + str(counter) in self.__veh_text_handle.keys():
                self.__veh_text_handle[id_in + "_i" + str(counter)].remove()
                del self.__veh_text_handle[id_in + "_i" + str(counter)]
                counter += plot_ith

        # return, if no data provided
        if not heading:
            return

        # check which side of heading occupies less instances of "avoid_pos"
        theta_l = np.array(heading) + np.pi / 2
        theta_r = np.array(heading) - np.pi / 2

        pos_l = np.column_stack((pos[:, 0] - np.sin(theta_l) * text_dist, pos[:, 1] + np.cos(theta_l) * text_dist))
        pos_r = np.column_stack((pos[:, 0] - np.sin(theta_r) * text_dist, pos[:, 1] + np.cos(theta_r) * text_dist))

        if avoid_pos is not None:
            # force position to be 2-dimensional
            avoid_pos = np.atleast_2d(avoid_pos)

            # calculate distances between all combinations of pos and avoid pos
            dist_l = euclidean_distances(pos_l, avoid_pos)
            dist_r = euclidean_distances(pos_r, avoid_pos)

            # get number of points that are smaller than the plot text distance
            num_dist_l = sum(i < text_dist for i in dist_l.min(axis=1))
            num_dist_r = sum(i < text_dist for i in dist_r.min(axis=1))

            if num_dist_l < num_dist_r:
                pos_text = pos_l
            else:
                pos_text = pos_r
        else:
            pos_text = pos_l

        counter = 0
        # for every position to be plotted
        for head, text in zip(heading, text_list):
            if counter % plot_ith != 0:
                counter += 1
                continue

            pos_i = pos_text[counter, :]

            # for counter >0 generate further ids:
            handle_id = id_in
            if counter > 0:
                handle_id += "_i" + str(counter)

            self.__veh_text_handle[handle_id] = self.__plane_axis.text(pos_i[0],
                                                                       pos_i[1],
                                                                       text,
                                                                       rotation=0.0,
                                                                       verticalalignment="center",
                                                                       horizontalalignment="center",
                                                                       clip_on=True,
                                                                       zorder=zorder)
            self.__veh_text_handle[handle_id].set_path_effects([patheffcts.withStroke(linewidth=2, foreground='w')])

            counter += 1

    def plot_traj(self,
                  t_in: float or None,
                  t_horizon: float = 2.0) -> None:
        """
        Highlight the trajectory of the vehicle at a defined time-stamp.

        :param t_in:        time-stamp to highlight; if set to "None", the trajectory is removed from the plane
        :param t_horizon:   time-horizon of the trajectory to be visualized (starting at t_in)
        :return:
        """
        if t_in is not None:
            t_in = max(t_in, 0.0)

            # get data points
            x = []
            y = []
            for t in np.linspace(t_in, t_in + t_horizon, int(t_horizon / 0.1)):
                val = self.get_timestamp_info(t_in=t)
                if val is not None:
                    x.append(val[0][0])
                    y.append(val[0][1])

            self.__traj_ptch_hdl.set_data([x, y])
        else:
            self.__traj_ptch_hdl.set_data([[], []])

    def update_xy_plot(self,
                       reset: bool = False) -> None:
        """
        Update the path plot in the xy-plane.

        :param reset:     if 'True' any present trajectory is removed
        :return:
        """

        if reset:
            self.__veh_plane_handle.set_data([[], []])

        else:
            self.__veh_plane_handle.set_data([self.data_exp[:, 1], self.data_exp[:, 2]])

    def update_temporal_plot(self,
                             reset: bool = False) -> None:
        """
        Update plots of temporal information.

        :param reset:   if 'True' any present trajectory is removed
        :return:
        """

        if reset:
            # reset all temporal information plots
            self.__acomb_t_handle.set_data([[], []])
            self.__ax_t_handle.set_data([[], []])
            self.__ay_t_handle.set_data([[], []])
            self.__vx_t_handle.set_data([[], []])
            self.__vx_s_handle.set_data([[], []])
            self.__ttc_t_handle.set_data([], [])
            self.__dss_t_handle.set_data([], [])

        else:
            # calculate temporal coordinate (for temporal plots)
            t = np.concatenate(([0], np.cumsum(np.divide(np.diff(self.data_exp[:, 0]), self.data_exp[:-1, 5],
                                                         out=np.full(self.data_exp[:-1, 5].shape[0], np.inf),
                                                         where=self.data_exp[:-1, 5] != 0))))

            # transform curvature kappa into corresponding radii
            radii = np.abs(np.divide(1, self.data_exp[:, 4],
                                     out=np.full(self.data_exp[:, 4].shape[0], np.inf),
                                     where=self.data_exp[:, 4] != 0))

            # calculate lateral acceleration
            ay = np.divide(np.power(self.data_exp[:, 5], 2), radii)

            # calculate combined acceleration
            acomb = np.sqrt(self.data_exp[:, 6] * self.data_exp[:, 6] + ay * ay)

            # update plots
            self.__acomb_t_handle.set_data([t, acomb])
            self.__ax_t_handle.set_data([t[:-1], self.data_exp[:-1, 6]])
            self.__ay_t_handle.set_data([t, ay])
            self.__vx_t_handle.set_data([t, self.data_exp[:, 5]])
            self.__vx_s_handle.set_data([self.data_exp[:, 0], self.data_exp[:, 5]])
            self.__ttc_t_handle.set_data([self.data_ssm[:, 0], self.data_ssm[:, 1]])
            self.__dss_t_handle.set_data([self.data_ssm[:, 0], self.data_ssm[:, 2] / 10.0])

    def change_temporal_plot_lw(self,
                                lw: float = 1.0) -> None:
        """
        Update line width in temporal information plots (e.g. to highlight a selected entity).

        :param lw:      set line-width (default is 1.0).
        :return:
        """

        # set line-width
        self.__acomb_t_handle.set_lw(lw)
        self.__ax_t_handle.set_lw(lw)
        self.__ay_t_handle.set_lw(lw)
        self.__vx_t_handle.set_lw(lw)
        self.__vx_s_handle.set_lw(lw)
        self.__ttc_t_handle.set_lw(lw)
        self.__dss_t_handle.set_lw(lw)

    def __del__(self) -> None:
        """
        Destructor - cleanup of all plotting handles.
        :return:
        """

        # delete plot handles
        self.__acomb_t_handle.remove()
        self.__ax_t_handle.remove()
        self.__ay_t_handle.remove()
        self.__vx_t_handle.remove()
        self.__vx_s_handle.remove()
        self.__veh_plane_handle.remove()
        self.__ttc_t_handle.remove()

        for handle in self.__veh_ptch_handle.values():
            handle.remove()
