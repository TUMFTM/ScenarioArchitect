import os
import io
import sys
import json
import copy
import time
import pickle
import pkg_resources
import configparser
import numpy as np
import tkinter as tk
from tkinter import messagebox, filedialog, simpledialog

# matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as ptch
from matplotlib.widgets import Slider, Button, RadioButtons, CheckButtons

# Show plot in serif font (e.g. for publications)
# import matplotlib
# matplotlib.rcParams['mathtext.fontset'] = 'stix'
# matplotlib.rcParams['font.family'] = 'STIXGeneral'

# custom packages
import trajectory_planning_helpers as tph
import scenario_testing_tools

# custom modules
import helper_funcs

"""
Python version: 3.7
Created by: Tim Stahl
Created on: 17.04.2019

Graphic user interface allowing to generate, load and export a scenario with one or multiple vehicles in a scene.
The bounds, as well as the path and velocity profile of a vehicle are all drawn and manipulated by using the pointing
device only.
"""

# path pointing to config-file
CONFIG_FILE = os.path.dirname(os.path.realpath(__file__)) + '/params/config.ini'


def check_py_dep() -> None:
    """
    Checks if all dependencies listed in the 'requirements.txt' are installed.
    """

    # get current path
    module_path = os.path.dirname(os.path.abspath(__file__))

    # read dependencies from requirements.txt
    requirements_path = os.path.join(module_path, 'requirements.txt')
    dependencies = []

    with open(requirements_path, 'r') as fh:
        line = fh.readline()

        while line:
            dependencies.append(line.rstrip())
            line = fh.readline()

    # check dependencies
    pkg_resources.require(dependencies)


class ScenarioArchitect:
    """
    Main class for the GUI.
    """

    def __init__(self,
                 load_file: str = None) -> None:
        """
        Initializes the Scenario Architect. All internal variables and scenario entities are created. The graphical
        interface (e.g. buttons and switches) as well as plot-areas are initialized.

        :param load_file:   (optional) existing scenario file to be loaded on startup.
        :return
        """

        # get config-file
        self.__config = configparser.ConfigParser()
        if not self.__config.read(CONFIG_FILE):
            raise ValueError('Specified config-file does not exist or is empty!')

        self.__color_dict = json.loads(self.__config.get('VISUAL', 'color_dict'))
        self.__keep_veh_plot = self.__config.getboolean('VISUAL', 'keep_veh_plot')

        # --------------------------------------------------------------------------------------------------------------
        # - INIT PLOTS -------------------------------------------------------------------------------------------------
        # --------------------------------------------------------------------------------------------------------------

        # -- main plot -------------------------------------------------------------------------------------------------
        self.__fig_main, ax = plt.subplots()
        self.__fig_main.canvas.set_window_title("Scenario GUI")
        plt.subplots_adjust(right=0.8, bottom=0.25)
        self.__plot_all_vehicles = False
        self.__plot_all_vehicles_text = True

        # init data rows
        self.__x_marks, = plt.plot([], [], 'xk', zorder=98)
        self.__x_mark_highlight, = plt.plot([], [], 'ok', zorder=97, alpha=0.5)
        self.__bound_connectors, = plt.plot([], [], '--', color=self.__color_dict['TUM_grey_dark'], zorder=5)

        self.__track_ptch_hdl = None
        self.__text_hdl = {}

        # set plot layout
        plt.axis([0, self.__config.getfloat('VISUAL', 'default_x_scale'),
                  0, self.__config.getfloat('VISUAL', 'default_x_scale') * 0.8])
        plt.tight_layout(rect=(0.05, 0.03, 0.8, 1.0))
        self.__axes_plane = self.__fig_main.gca()
        self.__axes_plane.grid()
        self.__axes_plane.set_aspect('equal', 'box')
        self.__axes_plane.set_xlabel('x in m')
        self.__axes_plane.set_ylabel('y in m')

        # register mouse events
        self.__fig_main.canvas.mpl_connect('button_press_event', self.onclick_main)
        self.__fig_main.canvas.mpl_connect('motion_notify_event', self.onmove_main)
        self.__fig_main.canvas.mpl_connect('button_release_event', self.onrelease_main)

        # -- velocity plot ---------------------------------------------------------------------------------------------
        self.__fig_time, axes = plt.subplots(3, 1)
        self.__fig_time.canvas.set_window_title("Temporal Plot")

        self.__axes_a_t = axes[0]
        self.__axes_a_t.grid()
        self.__axes_a_t.set_ylabel("a in m/s²")
        self.__axes_a_t.xaxis.set_ticks_position('none')              # x-ticks invisible
        plt.setp(self.__axes_a_t.get_xticklabels(), visible=False)    # set x-ticks invisible
        # self.__axes_a_t.set_xlim(left=0.0)                            # force 0.0 to be lower bound of x-axis
        # NOTE: fixing lower bound of x-axis here brakes auto-scaling --> enforced on every plot update

        plt.subplot(312, sharex=self.__axes_a_t)
        self.__axes_v_t = plt.gca()
        self.__axes_v_t.grid()
        self.__axes_v_t.set_xlabel("t in s", labelpad=0)
        self.__axes_v_t.set_ylabel("v in m/s")
        self.__axes_v_t.xaxis.set_ticks_position('none')              # x-ticks invisible
        self.__axes_v_t.tick_params(axis='x', which='major', pad=0)   # x-tick labels closer to axis
        # self.__axes_v_t.set_ylim(bottom=0.0)                          # force 0.0 to be lower bound of y-axis

        self.__axes_v_s = axes[2]
        self.__axes_v_s.grid()
        self.__axes_v_s.set_xlabel("s in m", labelpad=0)
        self.__axes_v_s.set_ylabel("v in m/s")
        self.__axes_v_s.xaxis.set_ticks_position('none')             # x-ticks invisible
        self.__axes_v_s.tick_params(axis='x', which='major', pad=0)  # x-tick labels closer to axis
        # self.__axes_v_s.set_ylim(bottom=0.0)                         # force 0.0 to be lower bound of y-axis
        # self.__axes_v_s.set_xlim(left=0.0)                           # force 0.0 to be lower bound of x-axis

        # -- change axis location of __axes_v_t --
        pos_acc_ax = self.__axes_a_t.get_position()
        pos_vel_ax = self.__axes_v_t.get_position()

        # get limits of plot boxes (in percent of the plot window) [[left, low], [right, up]]
        points_acc_ax = pos_acc_ax.get_points()
        points_vel_ax = pos_vel_ax.get_points()

        # set new frame of the velocity plot
        points_vel_ax[0][1] += points_acc_ax[0][1] - 0.01 - points_vel_ax[1][1]  # move up to reveal description
        points_vel_ax[1][1] = points_acc_ax[0][1] - 0.01        # move closer to bottom of acceleration plot (1% offset)

        pos_vel_ax.set_points(points_vel_ax)

        self.__axes_v_t.set_position(pos_vel_ax)

        # Member variables for manipulation of the velocity profile
        self.__vel_manip_handle = None
        self.__vel_manip_handle_tmp = None
        self.__vel_mod_range = None
        self.__vel_mod_y = None
        self.__vel_mod_protect = {}

        # event handles for velocity plot (moving mouse or pressing mouse buttons)
        self.__fig_time.canvas.mpl_connect('button_press_event', self.onclick_vel)
        self.__fig_time.canvas.mpl_connect('button_release_event', self.onrelease_vel)
        self.__fig_time.canvas.mpl_connect('motion_notify_event', self.onmove_vel)

        # add patch for limits (red shade for velocity bounds not to be crossed)
        max_comb_acc = self.__config.getfloat('VISUAL', 'vis_max_comb_acc')
        patch_xy = np.array([[-1000, -1000, 1000, 1000], [max_comb_acc, 1000, 1000, max_comb_acc]]).T
        track_ptch = ptch.Polygon(patch_xy, facecolor="red", alpha=0.3, zorder=1)
        self.__axes_a_t.add_artist(track_ptch)
        patch_xy = np.array([[-1000, -1000, 1000, 1000], [-max_comb_acc, -1000, -1000, -max_comb_acc]]).T
        track_ptch = ptch.Polygon(patch_xy, facecolor="red", alpha=0.3, zorder=1)
        self.__axes_a_t.add_artist(track_ptch)

        # define cursor highlighting selected position in the temporal plot (red vertical line)
        self.__vel_ax_cursor, = self.__axes_v_t.plot([0.0], [0.0], lw=1, color='r', zorder=999)
        self.__acc_ax_cursor, = self.__axes_a_t.plot([0.0], [0.0], lw=1, color='r', zorder=999)
        self.__cursor_last_t = 0.0

        # --------------------------------------------------------------------------------------------------------------
        # - DEFINE GUI ELEMENTS (BUTTONS, SLIDERS, ...) ----------------------------------------------------------------
        # --------------------------------------------------------------------------------------------------------------

        # Set time window as active figure
        plt.figure(self.__fig_time.number)

        # Open plot window button
        plybck_ax = self.__fig_time.add_axes([0.02, 0.95, 0.15, 0.04])
        button_plybck = Button(plybck_ax, 'Playback', color=self.__config.get('VISUAL', 'btn_color'),
                               hovercolor='0.975')
        button_plybck.on_clicked(self.on_playback_click)

        # Set main window as active figure
        plt.figure(self.__fig_main.number)

        # Text
        text_ax = self.__fig_main.add_axes([0.8, 0.82, 0.15, 0.2])
        text_ax.axis('off')
        text_ax.set_navigate(False)
        self.__entity_descr = text_ax.text(0.0, 0.0, "Select entity\nto be drawn:")

        # Radio buttons for entity selection
        rax = self.__fig_main.add_axes([0.8, 0.6, 0.15, 0.2], facecolor=self.__config.get('VISUAL', 'btn_color'))
        self.__radio = RadioButtons(rax, ('None', 'bound_l', 'bound_r', 'veh_1', 'veh_2', 'veh_3'), active=0)
        self.__radio.on_clicked(self.toggled_radio)

        # Reset button
        button_reset_ent = Button(self.__fig_main.add_axes([0.8, 0.5, 0.15, 0.04]), 'Reset Entity',
                                  color=self.__config.get('VISUAL', 'btn_color'),
                                  hovercolor='0.975')
        button_reset_ent.on_clicked(self.reset_ent)

        # Specify import button
        button_import = Button(self.__fig_main.add_axes([0.8, 0.45, 0.15, 0.04]), 'Import Scen.',
                               color=self.__config.get('VISUAL', 'btn_color'),
                               hovercolor='0.975')
        button_import.on_clicked(self.load_pckl)

        # Specify export button
        button_export = Button(self.__fig_main.add_axes([0.8, 0.4, 0.15, 0.04]), 'Export Scen.',
                               color=self.__config.get('VISUAL', 'btn_color'),
                               hovercolor='0.975')
        button_export.on_clicked(self.export_to_file)

        # Specify import track button
        button_import_track = Button(self.__fig_main.add_axes([0.8, 0.35, 0.15, 0.04]), 'Import Track',
                                     color=self.__config.get('VISUAL', 'btn_color'),
                                     hovercolor='0.975')
        button_import_track.on_clicked(self.load_track)

        # Plot all vehicles checkbox
        check_all_veh = CheckButtons(self.__fig_main.add_axes([0.8, 0.24, 0.15, 0.1]), ['All Poses', 'Add Text'],
                                     [self.__plot_all_vehicles, self.__plot_all_vehicles_text])
        check_all_veh.on_clicked(self.toggled_veh)

        # Open plot window button
        button_plt_wndw = Button(self.__fig_main.add_axes([0.8, 0.09, 0.15, 0.04]), 'Open Plot',
                                 color=self.__config.get('VISUAL', 'btn_color'),
                                 hovercolor='0.975')
        button_plt_wndw.on_clicked(self.open_plot_window)

        # Scaling reset button
        button = Button(self.__fig_main.add_axes([0.8, 0.02, 0.1, 0.04]), 'Reset',
                        color=self.__config.get('VISUAL', 'btn_color'),
                        hovercolor='0.975')
        button.on_clicked(self.reset)

        # Scaling slider
        self.__sld_x_axis = Slider(self.__fig_main.add_axes([0.1, 0.02, 0.6, 0.03],
                                                            facecolor=self.__config.get('VISUAL', 'btn_color')),
                                   label='scale',
                                   valmin=self.__config.getfloat('VISUAL', 'delta_xy'),
                                   valmax=self.__config.getfloat('VISUAL', 'max_x_scale'),
                                   valinit=self.__config.getfloat('VISUAL', 'default_x_scale'),
                                   valstep=self.__config.getfloat('VISUAL', 'delta_xy'))
        self.__sld_x_axis.on_changed(self.change_ratio)

        # --------------------------------------------------------------------------------------------------------------
        # - INIT VARIABLES AND ENTITIES --------------------------------------------------------------------------------
        # --------------------------------------------------------------------------------------------------------------
        # entity container (holding instances of all entities in the scenario) - initialize with two bound entities
        self.__ent_cont = [helper_funcs.src.ScenarioEntities.Bound(id_in='bound_l', plane_axis=self.__axes_plane),
                           helper_funcs.src.ScenarioEntities.Bound(id_in='bound_r', plane_axis=self.__axes_plane)]

        # add ego-vehicle
        self.__ent_cont.append(
            helper_funcs.src.ScenarioEntities.Vehicle(i_in=1,
                                                      plane_axis=self.__axes_plane,
                                                      a_t_axis=self.__axes_a_t,
                                                      v_t_axis=self.__axes_v_t,
                                                      v_s_axis=self.__axes_v_s, type_ego=True,
                                                      color_str=self.get_veh_color(1),
                                                      plan_hor=self.__config.getfloat('FILE_EXPORT',
                                                                                      'export_time_traj_horizon')),
        )

        # add two other vehicles
        for i in range(2, 4):
            self.__ent_cont.append(
                helper_funcs.src.ScenarioEntities.Vehicle(i_in=i,
                                                          plane_axis=self.__axes_plane,
                                                          a_t_axis=self.__axes_a_t,
                                                          v_t_axis=self.__axes_v_t,
                                                          v_s_axis=self.__axes_v_s,
                                                          color_str=self.get_veh_color(i)),
            )

        # currently selected entity (via radio-buttons - one element in the list of '__ent_cont')
        self.__sel_ent = None

        self.__pidx_vel = None
        self.__pidx_main = None

        # import ggv from file
        top_path = os.path.dirname(os.path.abspath(__file__))
        ggv_path = top_path + "/params/veh_dyn_info/ggv.csv"
        ax_max_machines_path = top_path + "/params/veh_dyn_info/ax_max_machines.csv"
        self.__ggv, self.__ax_max_machines = tph.import_veh_dyn_info. \
            import_veh_dyn_info(ggv_import_path=ggv_path,
                                ax_max_machines_import_path=ax_max_machines_path)

        # load scenario, if provided
        if load_file is not None:
            self.load_pckl(_=None,
                           file_path=load_file)

        plt.show()

    # ------------------------------------------------------------------------------------------------------------------
    # - INTERACTION WITH GUI ELEMENTS ----------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    def toggled_radio(self,
                      _: str = None) -> None:
        """
        Called whenever the radio button selection changes.

        :param _:   selected value (string of description), not used since accessable via radio.value_selected
        :return
        """

        # load latest stored information in temporary (live-visualized container)
        for obj in self.__ent_cont:
            obj.data_coord_tmp = copy.deepcopy(obj.data_coord)

        # find selected entity in container
        self.__sel_ent = None
        for obj in self.__ent_cont:
            if obj.id == self.__radio.value_selected:
                self.__sel_ent = obj
                break

        self.update_plot()

    def toggled_veh(self,
                    text_label) -> None:
        """
        Called whenever the checkbox 'All Poses' was clicked.

        :param text_label:  clicked text-label besides checkbox (used to discriminate checked boxes).
        :return
        """

        if text_label == 'All Poses':
            self.__plot_all_vehicles = not self.__plot_all_vehicles
        else:
            self.__plot_all_vehicles_text = not self.__plot_all_vehicles_text
        self.update_plot()

    def change_ratio(self,
                     _) -> None:
        """
        Called when scaling slider was moved. Manipulate plot scaling accordingly.
        """

        self.__axes_plane.set_ylim(0.0, self.__sld_x_axis.val * 0.8)
        self.__axes_plane.set_xlim(0.0, self.__sld_x_axis.val)
        self.__fig_main.canvas.draw_idle()

    def reset(self,
              _):
        """
        Called when the scaling's reset-button was clicked.
        """

        self.__sld_x_axis.reset()

    def reset_ent(self,
                  _) -> None:
        """
        Called when 'Reset Entity'-button was clicked. Resets the corresponding entity.
        """

        # reset container
        if self.__radio.value_selected != 'None':
            # remove entity from list of protected velocity profiles
            self.__vel_mod_protect.pop(self.__radio.value_selected, None)

            for obj in self.__ent_cont:
                if obj.id == self.__radio.value_selected:
                    obj.data_coord = None
                    obj.data_exp = None
                    obj.data_vel_coord = None

                    # remove vehicle visualization
                    if obj.TYPE_VEHICLE:
                        obj.highlight_pose(t_in=None)

                # update temporary data
                obj.data_coord_tmp = copy.deepcopy(obj.data_coord)

        # update plot
        self.update_plot()

    def on_playback_click(self,
                          _) -> None:
        """
        Called when the 'payback'-button is clicked.
        """

        # playback the scenario in real time (with updates as fast as possible)
        # stop edit mode in main axis
        if self.__radio.value_selected != 'None' and self.__sel_ent.data_coord_tmp is not None and \
                self.__sel_ent.data_coord_tmp.shape > self.__sel_ent.data_coord.shape:
            self.__sel_ent.data_coord_tmp = self.__sel_ent.data_coord
            self.update_plot(hover_mode=True)

        # get range of x axis
        t_range = self.__axes_a_t.get_xlim()
        t = t_range[0]

        # store axis limits in order to avoid readout every iteration
        vel_ylim = self.__axes_v_t.get_ylim()
        acc_ylim = self.__axes_a_t.get_ylim()

        while t < t_range[1]:
            tic = time.time()

            # update cursor in velocity window
            self.__vel_ax_cursor.set_data([t, t], vel_ylim)
            self.__acc_ax_cursor.set_data([t, t], acc_ylim)
            self.__fig_time.canvas.draw_idle()

            # for all trajectories holding temporal information, calculate index
            for obj in self.__ent_cont:
                if obj.TYPE_VEHICLE:
                    obj.highlight_pose(t_in=t)

            plt.pause(0.001)
            t += time.time() - tic

    def open_plot_window(self,
                         _) -> None:
        """
        Called when the 'Open Plot'-button is clicked.
        """

        # dump the whole plot in a pickle
        inx = list(self.__fig_main.axes).index(self.__axes_plane)
        buf = io.BytesIO()
        pickle.dump(self.__fig_main, buf)
        buf.seek(0)

        # load pickle in new plot figure (without buttons)
        fig_plot = pickle.load(buf)
        fig_plot.set_tight_layout(True)

        # delete everything except main axes
        for i, ax in enumerate(fig_plot.axes):
            if i != inx:
                fig_plot.delaxes(ax)

        fig_plot.show()

    # ------------------------------------------------------------------------------------------------------------------
    # - PATH AND VELOCITY CALCULATIONS (AND ACCORDING PLOT UPDATES) ----------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    def update_plot(self,
                    hover_mode=False,
                    force_update=False) -> None:
        """
        Update both - path and velocity plot (including relevant calculations).

        :param hover_mode:      if set to 'True' temporary values (e.g. currently manipulated trajectory without fixed
                                last coordinate will be live-visualized).
        :param force_update:    if set to 'True' all plots will be forced to show updates (not only obvious changes).
        :return
        """

        if not hover_mode:
            for obj in self.__ent_cont:
                obj.tmp_mode = False
        else:
            for obj in self.__ent_cont:
                obj.tmp_mode = True

        self.update_path_plot()
        self.update_vel_plot(force_update=force_update)

        for obj in self.__ent_cont:
            obj.tmp_mode = False

    def update_path_plot(self) -> None:
        """
        Update information relevant for the path plot (bound- and path-handling) and trigger plot updates.
        """

        x_marks = [[], []]

        for obj in self.__ent_cont:
            if obj.get_mode_coord() is not None and obj.get_mode_coord().size > 2:
                # -- calculate new information -------------------------------------------------------------------------
                if self.__radio.value_selected == obj.id:
                    # xmarks for active entry
                    x_marks[0].extend(obj.get_mode_coord()[:, 0])
                    x_marks[1].extend(obj.get_mode_coord()[:, 1])

                if obj.TYPE_VEHICLE:
                    # calculate heading
                    tangvecs = np.stack((obj.get_mode_coord()[1:, 0] - obj.get_mode_coord()[:-1, 0],
                                         obj.get_mode_coord()[1:, 1] - obj.get_mode_coord()[:-1, 1]), axis=1)
                    psi_vel = np.arctan2(tangvecs[:, 1], tangvecs[:, 0]) - np.pi / 2
                    psi_vel[psi_vel < -np.pi] += 2 * np.pi

                    # calculate spline
                    x_coeff, y_coeff, _, _ = tph.calc_splines.calc_splines(path=obj.get_mode_coord(),
                                                                           psi_s=psi_vel[0],
                                                                           psi_e=psi_vel[-1])

                    dpoints, spln_inds, t_val, dists_int = tph.interp_splines.interp_splines(coeffs_x=x_coeff,
                                                                                             coeffs_y=y_coeff,
                                                                                             stepsize_approx=2.0,
                                                                                             incl_last_point=True)

                    psi, kappa = tph.calc_head_curv_an.calc_head_curv_an(coeffs_x=x_coeff,
                                                                         coeffs_y=y_coeff,
                                                                         ind_spls=spln_inds,
                                                                         t_spls=t_val)

                    s = np.concatenate(([0], np.cumsum(np.sqrt(np.sum(np.power(np.diff(dpoints, axis=0), 2), axis=1)))))

                    # export s, x, y, psi, kappa, vx, ax (if data available and changed)
                    if (obj.data_vel_coord is None or self.__pidx_main is not None
                            or not np.array_equal(obj.get_mode_coord(), obj.data_vel_coord)):
                        obj.data_exp = np.zeros((psi.shape[0], 7))
                        obj.data_exp[:, 0:5] = np.column_stack((s, dpoints, psi, kappa))

                # -- update plots --------------------------------------------------------------------------------------
                obj.update_xy_plot()
            else:
                obj.update_xy_plot(reset=True)

        self.__x_marks.set_data(x_marks)

        # plot track
        if self.__track_ptch_hdl is not None:
            self.__track_ptch_hdl.remove()
            self.__track_ptch_hdl = None

        # expect bounds on first two entities
        if self.__ent_cont[0].TYPE_BOUND and self.__ent_cont[1].TYPE_BOUND:
            if self.__ent_cont[0].get_mode_coord() is not None and self.__ent_cont[1].get_mode_coord() is not None:
                # track patch
                patch_xy = np.vstack((self.__ent_cont[0].get_mode_coord(),
                                      np.flipud(self.__ent_cont[1].get_mode_coord())))
                track_ptch = ptch.Polygon(patch_xy, facecolor="black", alpha=0.2, zorder=1)
                self.__track_ptch_hdl = self.__axes_plane.add_artist(track_ptch)

                # connecting lines
                n_shared_el = int(min(self.__ent_cont[0].get_mode_coord().size / 2,
                                      self.__ent_cont[1].get_mode_coord().size / 2))
                data_bound_lines = [[], []]
                if n_shared_el > 1:
                    for i in range(n_shared_el):
                        data_bound_lines[0].extend([self.__ent_cont[0].get_mode_coord()[i, 0],
                                                    self.__ent_cont[1].get_mode_coord()[i, 0], None])
                        data_bound_lines[1].extend([self.__ent_cont[0].get_mode_coord()[i, 1],
                                                    self.__ent_cont[1].get_mode_coord()[i, 1], None])
                self.__bound_connectors.set_data(data_bound_lines)
            else:
                self.__bound_connectors.set_data([[], []])
        else:
            raise ValueError("Something went wrong! Assumed bound data on first two container entries!")

        self.__fig_main.canvas.draw_idle()

    def update_vel_plot(self,
                        force_update=False) -> None:
        """
        Update information relevant for the path plot (bound- and path-handling) and trigger plot updates.

        :param force_update:    (optional) if set to 'True', all entities are forced to be updated
                                           if set to 'False', only obviously changed entities are updated
        :return
        """

        # plot track
        if self.__vel_manip_handle is not None:
            self.__vel_manip_handle.remove()
            self.__vel_manip_handle = None

        # check if any of the text next to the vehicle poses is initially plotted (then all need an update due to
        # repositioning)
        all_veh_text_need_update = False
        if self.__plot_all_vehicles and self.__plot_all_vehicles_text and 'veh' not in self.__radio.value_selected:
            for obj in self.__ent_cont:
                if obj.TYPE_VEHICLE and obj.get_mode_coord() is not None and obj.data_exp is not None \
                        and not obj.id + "_all" in self.__text_hdl:
                    all_veh_text_need_update = True
                    break

        # for all entities
        for obj in self.__ent_cont:

            # if entity is a vehicle
            if obj.TYPE_VEHICLE:
                if obj.get_mode_coord() is not None and obj.data_exp is not None:

                    # skip plot update, if data did not change
                    if obj.data_vel_coord is None or max(obj.data_exp[:, 5]) == 0.0 \
                            or self.__radio.value_selected == obj.id or self.__plot_all_vehicles \
                            or all_veh_text_need_update or force_update:

                        # if more than two coordinates and velocity profile unset or changed (set to zeros by path plan)
                        if obj.get_mode_coord().size > 2 and (max(obj.data_exp[:, 5]) == 0.0
                                                              or obj.data_vel_coord is None):
                            obj.data_vel_coord = np.copy(obj.get_mode_coord())

                            # calculate velocity profile
                            vx = tph.calc_vel_profile.calc_vel_profile(ggv=self.__ggv,
                                                                       ax_max_machines=self.__ax_max_machines,
                                                                       kappa=obj.data_exp[:, 4],
                                                                       el_lengths=np.diff(obj.data_exp[:, 0]),
                                                                       mu=np.ones(obj.data_exp.shape[0]),
                                                                       v_start=500.0,
                                                                       drag_coeff=self.__config.getfloat('VEHICLE',
                                                                                                         'drag_coeff'),
                                                                       m_veh=self.__config.getfloat('VEHICLE', 'm_veh'),
                                                                       closed=False)
                            # smoothen velocity profile
                            vx = tph.conv_filt.conv_filt(signal=vx,
                                                         filt_window=3,
                                                         closed=False)

                            # calculate acceleration profile
                            ax = tph.calc_ax_profile.calc_ax_profile(vx_profile=vx,
                                                                     el_lengths=np.diff(obj.data_exp[:, 0]))

                            # store vx and ax to array
                            obj.data_exp[:, 5] = vx
                            obj.data_exp[:, 6] = np.concatenate((ax, [0]))
                        else:
                            # recalculate ax for given vx profile
                            ax = tph.calc_ax_profile.calc_ax_profile(vx_profile=obj.data_exp[:, 5],
                                                                     el_lengths=np.diff(obj.data_exp[:, 0]))

                            obj.data_exp[:, 6] = np.concatenate((ax, [0]))

                        # update temporal plots
                        obj.update_temporal_plot()

                        # plot temporal selector
                        if self.__radio.value_selected in obj.id:
                            self.__vel_manip_handle, = self.__axes_v_s.plot(obj.data_exp[:, 0], obj.data_exp[:, 5],
                                                                            'ok', alpha=0.5, zorder=100)
                        # check if "plot all vehicles" is activated
                        if self.__plot_all_vehicles:
                            # generate time stamps for stored vehicle data (if v=0, do not move forward instead of inf)
                            s_past = np.diff(obj.data_exp[:, 0])
                            v_past = obj.data_exp[:-1, 5]
                            t_stamps = np.cumsum(np.concatenate(([0.0], np.divide(s_past, v_past,
                                                                                  out=np.full(v_past.shape[0], 0.0),
                                                                                  where=v_past != 0))))

                            # extract relevant poses
                            num_steps = int(t_stamps[-1]
                                            / self.__config.getfloat('VISUAL', 'plot_temporal_increment')) + 1
                            pos = np.zeros((num_steps, 2))
                            psi = []
                            text_list = []
                            for i in range(num_steps):
                                # get time stamp
                                t = i * self.__config.getfloat('VISUAL', 'plot_temporal_increment')

                                pos[i, :] = [np.interp(t, t_stamps, obj.data_exp[:, 1]),
                                             np.interp(t, t_stamps, obj.data_exp[:, 2])]

                                psi.append(scenario_testing_tools.interp_heading.
                                           interp_heading(heading=obj.data_exp[:, 3],
                                                          t_series=t_stamps,
                                                          t_in=t))

                                text_list.append("$t^{" + obj.id.replace("_", "") + "}_{" + str(i) + "}$")

                            # plot all vehicles
                            obj.plot_vehicle(pos=pos,
                                             heading=psi,
                                             color_str=self.get_veh_color(int(obj.id[-1])),
                                             zorder=101,
                                             id_in=obj.id + "_all")

                            # print text if enabled and not currently selected (performance issues)
                            if self.__plot_all_vehicles_text and self.__radio.value_selected != obj.id:
                                # gather positions of all other vehicles
                                avoid_pos = None
                                for tmp_obj in self.__ent_cont:
                                    if tmp_obj.TYPE_VEHICLE and tmp_obj.id != obj.id and tmp_obj.data_exp is not None:
                                        if avoid_pos is None:
                                            avoid_pos = tmp_obj.data_exp[:, 1:3]
                                        else:
                                            avoid_pos = np.vstack((avoid_pos, tmp_obj.data_exp[:, 1:3]))

                                # print text next to each pose
                                obj.plot_text(pos=pos,
                                              heading=psi,
                                              text_list=text_list,
                                              text_dist=self.__config.getfloat('VISUAL', 'plot_text_distance'),
                                              plot_ith=self.__config.getint('VISUAL', 'plot_text_every_ith_element'),
                                              avoid_pos=avoid_pos,
                                              id_in=obj.id + "_all")

                        # for scaling, remove cursor
                        temp_va = self.__vel_ax_cursor.get_xdata()
                        temp_aa = self.__acc_ax_cursor.get_xdata()
                        self.__vel_ax_cursor.set_data([[], []])
                        self.__acc_ax_cursor.set_data([[], []])

                        # auto-scale axes (without cursor)
                        self.__axes_v_t.relim()
                        self.__axes_v_t.autoscale_view(True, True, True)
                        self.__axes_v_s.relim()
                        self.__axes_v_s.autoscale_view(True, True, True)
                        self.__axes_a_t.relim()
                        self.__axes_a_t.autoscale_view(True, True, True)

                        # force lower limits on axes (has to be done in every iteration, since autoscale is borken else)
                        self.__axes_a_t.set_xlim(left=0.0, auto=True)
                        self.__axes_v_t.set_ylim(bottom=0.0, auto=True)
                        self.__axes_v_s.set_xlim(left=0.0, auto=True)
                        self.__axes_v_s.set_ylim(bottom=0.0, auto=True)

                        # restore cursors
                        self.__vel_ax_cursor.set_data(temp_va, self.__axes_v_t.get_ylim())
                        self.__acc_ax_cursor.set_data(temp_aa, self.__axes_a_t.get_ylim())

                    # highlight selected data row (thicker line)
                    if self.__radio.value_selected in obj.id:
                        obj.change_temporal_plot_lw(lw=2)
                    else:
                        obj.change_temporal_plot_lw(lw=1)
                else:
                    # reset temporal plots
                    obj.update_temporal_plot(reset=True)

                # handle removal of vehicle plots (when data was removed or configuration changed)
                if not self.__plot_all_vehicles or obj.get_mode_coord() is None or obj.data_exp is None:
                    # remove all vehicle plots
                    obj.plot_vehicle(pos=np.array([]),
                                     heading=[],
                                     id_in=obj.id + "_all")

                if not self.__plot_all_vehicles or not self.__plot_all_vehicles_text or obj.get_mode_coord() is None \
                        or obj.data_exp is None or self.__radio.value_selected == obj.id:
                    # remove text
                    obj.plot_text(pos=np.array([]),
                                  heading=[],
                                  text_list=[],
                                  id_in=obj.id + "_all")

        self.__fig_time.canvas.draw_idle()
        self.__fig_main.canvas.draw_idle()

    # ------------------------------------------------------------------------------------------------------------------
    # - SUPPORTING FUNCTIONS -------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    def get_veh_color(self,
                      i: int) -> str:
        """
        Return vehicle color according to list specified in the config-file.

        :param i:               return color for i-th vehicle, starting with index "1"
        :returns color_str:     color string (hash or matplotlib color name)
        """

        # read requested color string (loop if index exceeded)
        veh_color_list = json.loads(self.__config.get('VISUAL', 'veh_colors'))
        req_color = veh_color_list[max((i - 1), 0) % len(veh_color_list)]

        # if in color dict, return entry, else return 'req_color' directly (assume matplotlib color name)
        return self.__color_dict.get(req_color, req_color)

    # ------------------------------------------------------------------------------------------------------------------
    # - DATA EXPORT / IMPORT -------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------
    def export_to_file(self,
                       _) -> None:
        """
        Called when the export button is pressed. Handles export of the constructed scene to a CSV-file representation
        of the scene with certain temporal increment (suited for input to other functions). The file path is defined by
        and prompt dialog (GUI).
        """
        # -- calculate trajectory information for every vehicle (will be used later for export) --
        veh_data = {}
        for obj in self.__ent_cont:
            if obj.TYPE_VEHICLE:
                t = 0.0
                while True:
                    t_data = obj.get_timestamp_info(t_in=t)

                    # When no further data points are present, quit
                    if t_data is not None and t_data[3] is not None:
                        if obj.id not in veh_data.keys():
                            veh_data[obj.id] = np.hstack(t_data)
                        else:
                            veh_data[obj.id] = np.vstack((veh_data[obj.id], np.hstack(t_data)))
                    else:
                        break

                    # increment time
                    t += self.__config.getfloat('FILE_EXPORT', 'export_time_increment')

        # -- check if export is valid --
        if self.__ent_cont[0].data_coord is None or self.__ent_cont[1].data_coord is None:
            messagebox.showerror("Error", "Could not export data (yet), since the bounds are not specified!")
            return
        if self.__ent_cont[0].data_coord.size != self.__ent_cont[1].data_coord.size:
            messagebox.showerror("Error", "Could not export data, "
                                          "since the bounds don't hold the same amount of points!")
            return
        if self.__ent_cont[2].data_coord is None:
            messagebox.showerror("Error", "Could not export data, since the ego-vehicle (veh_1) is not specified!")
            return
        if veh_data['veh_1'].shape[0] <= (self.__config.getfloat('FILE_EXPORT', 'export_time_traj_horizon')
                                          / self.__config.getfloat('FILE_EXPORT', 'export_time_increment')):
            messagebox.showerror("Error", "Could not export data, since the ego-vehicle trajectory is too short!\n"
                                          "Keep in mind, that the ego-vehicle's trajectory must additionally serve for"
                                          " an trajectory preview horizon (%.2fs)" % self.__config.
                                 getfloat('FILE_EXPORT', 'export_time_traj_horizon'))
            return

        # -- get filename --
        root = tk.Tk()
        root.withdraw()
        file_path = filedialog.asksaveasfile(defaultextension=".scn",
                                             filetypes=(("Scene-File", "*.scn"), ("All Files", "*.*")))
        if file_path is None:
            raise Warning('File export was aborted!')

        # -- init export-file --
        helper_funcs.src.data_export.init_exportfile(file_path=file_path.name,
                                                     bound_l=self.__ent_cont[0].data_coord,
                                                     bound_r=self.__ent_cont[1].data_coord)

        # -- assemble export-file --
        n_traj_export = int(self.__config.getfloat('FILE_EXPORT', 'export_time_traj_horizon')
                            / self.__config.getfloat('FILE_EXPORT', 'export_time_increment'))
        for i in range(veh_data['veh_1'].shape[0] - n_traj_export):
            # extract ego trajectory
            ego_traj = veh_data['veh_1'][i:(i + n_traj_export), :]

            # generate object array (information about all objects in the scene)
            # each object holds: [id, [x, y, heading, obj_radius, vel]]
            obj_array = []
            for obj in veh_data.keys():
                if obj != 'veh_1' and veh_data[obj].shape[0] > i:
                    obj_array.append([obj, [veh_data[obj][i, 0],
                                            veh_data[obj][i, 1],
                                            veh_data[obj][i, 2],
                                            max(self.__config.getfloat('VEHICLE', 'veh_length'),
                                                self.__config.getfloat('VEHICLE', 'veh_width')) / 2,
                                            veh_data[obj][i, 4]]])

            # calculate (simple) emergency trajectory
            ego_traj_em = helper_funcs.src.calc_brake_emergency.calc_brake_emergency(traj=ego_traj,
                                                                                     ggv=self.__ggv)

            # write to file
            helper_funcs.src.data_export.write_timestamp(file_path=file_path.name,
                                                         time=i * self.__config.getfloat('FILE_EXPORT',
                                                                                         'export_time_increment'),
                                                         pos=veh_data['veh_1'][i, 0:2],
                                                         heading=veh_data['veh_1'][i, 2],
                                                         curv=veh_data['veh_1'][i, 3],
                                                         vel=veh_data['veh_1'][i, 4],
                                                         acc=veh_data['veh_1'][i, 5],
                                                         ego_traj=ego_traj,
                                                         ego_traj_em=ego_traj_em,
                                                         object_array=obj_array)

        # -- store pickle file in order to load the scenario for further edits --
        self.save_pckl(file_path=file_path.name.replace('.scn', '.sas'))

    def load_pckl(self,
                  _,
                  file_path=None) -> None:
        """
        Load an existing scenario file (of type 'pickle') that was generated previously with the Scenario Architect.

        :param file_path:   (optional) file-path to be loaded, if not provided / 'None' a GUI file-selector is shown.
        """

        if file_path is None:
            root = tk.Tk()
            root.withdraw()
            file_path = filedialog.askopenfilename(defaultextension=".sas",
                                                   filetypes=(("Scene-File", "*.sas"), ("All Files", "*.*")))

        if ".sas" not in file_path:
            simpledialog.messagebox.showinfo(title="Unsupported file type",
                                             message="Please make sure to provide a scene file of type '*.sas'!")

        elif file_path != '':
            f = open(file_path, 'rb')
            imp_container = pickle.load(f)
            f.close()

            # sort imported data into corresponding class variables (if existing update, else create)
            for key in imp_container[0].keys():

                obj = None
                for obj in self.__ent_cont:
                    if obj.id == key:
                        break

                if obj is None:
                    # if vehicle
                    if key in imp_container[1].keys():
                        self.__ent_cont.append(helper_funcs.src.ScenarioEntities.
                                               Vehicle(i_in=int(key[-1]),
                                                       plane_axis=self.__axes_plane,
                                                       a_t_axis=self.__axes_a_t,
                                                       v_t_axis=self.__axes_v_t,
                                                       v_s_axis=self.__axes_v_s,
                                                       color_str=self.get_veh_color(int(key[-1]))))

                    # if bound
                    else:
                        self.__ent_cont.append(helper_funcs.src.ScenarioEntities.
                                               Bound(id_in=key,
                                                     plane_axis=self.__axes_plane))

                    obj = self.__ent_cont[-1]

                obj.data_coord = imp_container[0][key]
                obj.data_coord_tmp = obj.data_coord

                # if vehicle
                if key in imp_container[1].keys():
                    obj.data_vel_coord = imp_container[1][key]
                    obj.data_exp = imp_container[2][key]

            # protect all velocity profiles
            for obj in self.__ent_cont:
                if obj.TYPE_VEHICLE:
                    self.__vel_mod_protect[obj.id] = True

            self.update_plot(force_update=True)

    def save_pckl(self,
                  file_path: str) -> None:
        """
        Store the current scenario to a file (of type 'pickle'), which allows to reopen and edit the scenario with the
        Scenario Architect at an point later in time.

        :param file_path:   (optional) file-path to be loaded, if not provided / 'None' a GUI file-selector is shown.
        """

        data_coord_tmp = dict()
        data_vel_coord_tmp = dict()
        data_exp_tmp = dict()

        # retrieve data from objects
        for obj in self.__ent_cont:
            data_coord_tmp[obj.id] = obj.data_coord

            if obj.TYPE_VEHICLE:
                data_vel_coord_tmp[obj.id] = obj.data_vel_coord
                data_exp_tmp[obj.id] = obj.data_exp

        exp_container = [data_coord_tmp, data_vel_coord_tmp, data_exp_tmp]
        f = open(file_path, 'wb')
        pickle.dump(exp_container, f, 2)
        f.close()

    def load_track(self,
                   _) -> None:
        """
        Load a section of an existing track layout to initialize the bounds and the trajectory of the ego-vehicle.
        Currently the track must be provided in an CSV-flavoured file with data headers in the first row.
        Currently the following two types are supported (headers must hold exactly same names):
         - [x_m;y_m;w_tr_right_m;w_tr_left_m]
         - [x_ref_m;y_ref_m;width_right_m;width_left_m;x_normvec_m;y_normvec_m;alpha_mincurv_m;s_raceline_m]

        The start and end of the section to be used from the track is specified by the s-coordinate measured from the
        start-finish line (GUI-promts will request start and end s-coordinate).
        """

        FILE_TYPE1_HEAD = ['x_m', 'y_m', 'w_tr_right_m', 'w_tr_left_m']
        FILE_TYPE2_HEAD = ['x_ref_m', 'y_ref_m', 'width_right_m', 'width_left_m', 'x_normvec_m', 'y_normvec_m',
                           'alpha_m', 's_racetraj_m']

        root = tk.Tk()
        root.withdraw()
        file_path = filedialog.askopenfilename(defaultextension=".csv",
                                               filetypes=(("CSV-File", "*.csv*"), ("All Files", "*.*")),)

        if file_path != '':
            # detect used delimiter and skip comment lines (without any of the delimiters)
            with open(file_path) as f:
                first_line = ''
                skip_head = -1
                while first_line.count(';') < 1 and first_line.count(',') < 1:
                    first_line = f.readline()
                    skip_head += 1

                if first_line.count(';') > first_line.count(','):
                    delimiter = ';'
                else:
                    delimiter = ','

            csv_data_temp = np.genfromtxt(file_path, delimiter=delimiter, dtype=None, encoding=None,
                                          skip_header=skip_head, names=True)

            # check if supported type of file is provided
            if all(elem in csv_data_temp.dtype.names for elem in FILE_TYPE1_HEAD):
                # get ref.-line
                refline = np.column_stack((csv_data_temp['x_m'], csv_data_temp['y_m']))

                # get widths right/left
                width_right = csv_data_temp['w_tr_right_m']
                width_left = csv_data_temp['w_tr_left_m']

                # get normalized normal vectors
                el_lengths = np.sqrt(np.sum(np.diff(refline, axis=0) ** 2, axis=1))
                psi = tph.calc_head_curv_num.calc_head_curv_num(path=refline,
                                                                el_lengths=el_lengths,
                                                                is_closed=False,
                                                                calc_curv=False)[0]
                normvec_normalized = tph.calc_normal_vectors.calc_normal_vectors(psi=psi)

                # get race line s-coords
                s = [0]
                s.extend(list(np.cumsum(el_lengths)))

                # Calculate bounds
                ego_path = None
                bound_l = refline - normvec_normalized * np.expand_dims(width_left, 1)
                bound_r = refline + normvec_normalized * np.expand_dims(width_right, 1)

            elif all(elem in csv_data_temp.dtype.names for elem in FILE_TYPE2_HEAD):
                # get ref.-line
                refline = np.column_stack((csv_data_temp['x_ref_m'], csv_data_temp['y_ref_m']))

                # get widths right/left
                width_right = csv_data_temp['width_right_m']
                width_left = csv_data_temp['width_left_m']

                # get normalized normal vectors
                normvec_normalized = np.column_stack((csv_data_temp['x_normvec_m'], csv_data_temp['y_normvec_m']))

                # get ego-path alpha
                alpha_mincurv = csv_data_temp['alpha_m']

                # get race line s-coords
                s = csv_data_temp['s_racetraj_m']

                # Calculate bounds and ego-path
                ego_path = refline + normvec_normalized * alpha_mincurv[:, np.newaxis]
                bound_l = refline - normvec_normalized * np.expand_dims(width_left, 1)
                bound_r = refline + normvec_normalized * np.expand_dims(width_right, 1)

            else:
                messagebox.showerror("Error", "Provided track file is not supported! First line must hold CSV data-row"
                                              "headers which match with the expected ones (see function description).")
                return

            # Offer to cut according to s-coordinate
            s_min = simpledialog.askfloat("Start s-coordinate",
                                          "Choose a start s-coordinate for the track-data to be extracted:",
                                          parent=root,
                                          minvalue=0.0, maxvalue=s[-1])

            if s_min is not None:
                s_max = simpledialog.askfloat("End s-coordinate",
                                              "Choose an end s-coordinate for the track-data to be extracted:",
                                              parent=root,
                                              minvalue=0.0, maxvalue=s[-1])
            else:
                s_max = None

            # Determine start and end index
            if s_min is not None and s_max is not None:
                idx_s = 0
                while s[idx_s] < s_min:
                    idx_s += 1

                idx_e = 0
                while s[idx_e] < s_max:
                    idx_e += 1
            else:
                idx_s = 0
                idx_e = -1

            # Cut bounds and race line
            if idx_e > idx_s:
                if ego_path is not None:
                    ego_path = ego_path[idx_s:idx_e, :]
                bound_l = bound_l[idx_s:idx_e, :]
                bound_r = bound_r[idx_s:idx_e, :]
            else:
                if ego_path is not None:
                    ego_path = np.vstack((ego_path[idx_s:-1, :], ego_path[0:idx_e, :]))
                bound_l = np.vstack((bound_l[idx_s:-1, :], bound_l[0:idx_e, :]))
                bound_r = np.vstack((bound_r[idx_s:-1, :], bound_r[0:idx_e, :]))

            # calculate shift (base scenario around origin)
            x_shift = min(min(bound_l[:, 0]), min(bound_r[:, 0])) - 1.0
            y_shift = min(min(bound_l[:, 1]), min(bound_r[:, 1])) - 1.0

            # sort imported data into class variables
            self.__ent_cont[0].data_coord = bound_l - np.array([x_shift, y_shift])
            self.__ent_cont[1].data_coord = bound_r - np.array([x_shift, y_shift])

            if ego_path is not None:
                self.__ent_cont[2].data_coord = ego_path - np.array([x_shift, y_shift])

            # update temp-data with stored data
            for obj in self.__ent_cont:
                obj.data_coord_tmp = obj.data_coord

            self.update_plot(force_update=True)

    # ------------------------------------------------------------------------------------------------------------------
    # - MOUSE EVENTS ---------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------
    def onclick_main(self,
                     event) -> None:
        """
        Called whenever a mouse-click is registered on the main-window.

        :param event:       MouseEvent (holds current position of the mouse cursor)
        :return
        """

        if event.inaxes != self.__axes_plane or self.__radio.value_selected == 'None':
            return

        # calculate distance to all existing data points of selected data row
        if self.__sel_ent.data_coord is not None and self.__sel_ent.data_coord.shape[0] > 2:
            dist2 = (np.power(self.__sel_ent.data_coord[:, 0] - event.xdata, 2)
                     + np.power(self.__sel_ent.data_coord[:, 1] - event.ydata, 2))
        else:
            dist2 = None

        # if double click, deselect current data row
        if event.dblclick and event.button == 1:
            self.__radio.set_active(0)
        else:
            # if close to existing point
            if dist2 is not None and min(dist2) < 1.0:
                # if left-clicked, set selection marker
                if event.button == 1:
                    self.__pidx_main = np.argmin(dist2)

                # if right-clicked, remove point
                elif event.button == 3:
                    self.__sel_ent.data_coord = np.delete(self.__sel_ent.data_coord, (np.argmin(dist2)), axis=0)

            # if not close to existing point
            else:
                # if left-clicked, add point at current mouse position
                if event.button == 1:
                    if self.__sel_ent.data_coord is None:
                        self.__sel_ent.data_coord = np.array((event.xdata, event.ydata))
                    else:
                        self.__sel_ent.data_coord = np.array(np.vstack((self.__sel_ent.data_coord,
                                                                        np.array((event.xdata, event.ydata)))))

        # update temp-data with stored data
        if self.__sel_ent is not None:
            self.__sel_ent.data_coord_tmp = self.__sel_ent.data_coord

        # update plot
        self.update_plot()

    def onmove_main(self,
                    event) -> None:
        """
        Called whenever a mouse-movement is registered in the main-window.

        :param event:       MouseEvent (holds current position of the mouse cursor)
        :return
        """

        # stop edit mode, when out of axis
        if event.inaxes != self.__axes_plane:
            if self.__radio.value_selected != 'None' and self.__sel_ent.data_coord_tmp is not None and \
                    self.__sel_ent.data_coord_tmp.shape > self.__sel_ent.data_coord.shape:
                self.__sel_ent.data_coord_tmp = self.__sel_ent.data_coord
                self.update_plot(hover_mode=True)
            return

        # when in main axis, update vehicle-visualization of active vehicle
        if self.__keep_veh_plot:
            if self.__radio.value_selected != 'None' and self.__sel_ent.TYPE_VEHICLE:
                self.__sel_ent.highlight_pose(t_in=self.__cursor_last_t)
        else:
            if self.__radio.value_selected != 'None':
                for obj in self.__ent_cont:
                    if obj.TYPE_VEHICLE:
                        obj.highlight_pose(t_in=None)

        # check if velocity profile was edited manually
        if self.__vel_mod_protect.get(self.__radio.value_selected, False):
            answer = messagebox.askyesno("Warning", "The velocity profile of entity " + self.__radio.value_selected
                                         + " has been changed manually! If you move the mouse in the main axis, a new"
                                         " velocity profile is calculated. All manual data will be lost!\n\n"
                                         "Do you want to keep the manual velocity profile?")
            if not answer:
                self.__vel_mod_protect.pop(self.__radio.value_selected, None)
            else:
                return

        if self.__radio.value_selected != 'None':
            # check if in main-axis and mouse button is held down
            if self.__pidx_main is not None and event.button == 1:
                # update highlight
                self.__x_mark_highlight.set_data([event.xdata, event.ydata])

                self.__sel_ent.data_coord[self.__pidx_main, 0] = event.xdata
                self.__sel_ent.data_coord[self.__pidx_main, 1] = event.ydata
                self.__sel_ent.data_coord_tmp = self.__sel_ent.data_coord

            elif self.__sel_ent.data_coord_tmp is not None:
                # calculate distance to all existing data points of selected data row
                if self.__sel_ent.data_coord.shape[0] > 2:
                    dist2 = (np.power(self.__sel_ent.data_coord[:, 0] - event.xdata, 2)
                             + np.power(self.__sel_ent.data_coord[:, 1] - event.ydata, 2))
                else:
                    dist2 = None

                if dist2 is not None and min(dist2) < 1.0:
                    self.__sel_ent.data_coord_tmp = self.__sel_ent.data_coord

                    # update highlight
                    self.__x_mark_highlight.set_data(self.__sel_ent.data_coord[np.argmin(dist2), :])
                else:
                    self.__sel_ent.data_coord_tmp = np.array(np.vstack((self.__sel_ent.data_coord,
                                                                        np.array((event.xdata, event.ydata)))))

                    # update highlight
                    self.__x_mark_highlight.set_data([[], []])

        # update plot
        self.update_plot(hover_mode=True)

    def onrelease_main(self,
                       event) -> None:
        """
        Called whenever a mouse-click release is registered on the main-window.

        :param event:       MouseEvent (holds current position of the mouse cursor)
        :return
        """

        if event.button != 1:
            return
        self.__pidx_main = None

    def onclick_vel(self,
                    event) -> None:
        """
        Called whenever a mouse-click is registered on the temporal-window.

        :param event:       MouseEvent (holds current position of the mouse cursor)
        :return
        """

        if event.inaxes != self.__axes_v_s or event.button != 1:
            return

        if 'veh_' in self.__radio.value_selected and self.__sel_ent.data_exp is not None:
            self.__pidx_vel = self.get_ind_under_point(event,
                                                       x_coords=self.__sel_ent.data_exp[:, 0],
                                                       y_coords=self.__sel_ent.data_exp[:, 5])

    def onmove_vel(self,
                   event) -> None:
        """
        Called whenever a mouse-movement is registered in the temporal-window.

        :param event:       MouseEvent (holds current position of the mouse cursor)
        :return
        """

        # stop edit mode in main axis
        if self.__radio.value_selected != 'None' and self.__sel_ent.data_coord_tmp is not None and \
                self.__sel_ent.data_coord_tmp.shape > self.__sel_ent.data_coord.shape:
            self.__sel_ent.data_coord_tmp = self.__sel_ent.data_coord
            self.update_plot(hover_mode=True)

        # check if in any of the axes
        if event.inaxes == self.__axes_v_t or event.inaxes == self.__axes_a_t:
            # update cursor in velocity window
            self.__vel_ax_cursor.set_data([event.xdata, event.xdata], self.__axes_v_t.get_ylim())
            self.__acc_ax_cursor.set_data([event.xdata, event.xdata], self.__axes_a_t.get_ylim())
            self.__fig_time.canvas.draw_idle()

            # update cursor data in main window
            self.__cursor_last_t = event.xdata

            # for all trajectories holding temporal information, calculate index
            for obj in self.__ent_cont:
                if obj.TYPE_VEHICLE:
                    obj.highlight_pose(t_in=self.__cursor_last_t)

        # check if in vel2 axis and mouse button is held down
        elif self.__pidx_vel is not None and event.inaxes == self.__axes_v_s and event.button == 1:
            # clear latest points
            if self.__vel_manip_handle_tmp is not None:
                self.__vel_manip_handle_tmp.remove()
                self.__vel_manip_handle_tmp = None

            # get index for current x-value
            idx_cur = int(np.argmin(np.abs(self.__sel_ent.data_exp[:, 0] - event.xdata)))

            # get range of selected values
            if idx_cur < int(self.__pidx_vel):
                self.__vel_mod_range = range(idx_cur, int(self.__pidx_vel) + 1)
                self.__vel_mod_y = np.interp(
                    x=self.__sel_ent.data_exp[idx_cur:int(self.__pidx_vel) + 1, 0],
                    xp=self.__sel_ent.data_exp[[idx_cur, int(self.__pidx_vel)], 0],
                    fp=[event.ydata, self.__sel_ent.data_exp[int(self.__pidx_vel), 5]])
            elif idx_cur == int(self.__pidx_vel):
                self.__vel_mod_range = np.array([int(self.__pidx_vel)])
                self.__vel_mod_y = event.ydata
            else:
                self.__vel_mod_range = range(int(self.__pidx_vel), idx_cur + 1)
                self.__vel_mod_y = np.interp(
                    x=self.__sel_ent.data_exp[int(self.__pidx_vel):idx_cur + 1, 0],
                    xp=self.__sel_ent.data_exp[[int(self.__pidx_vel), idx_cur], 0],
                    fp=[self.__sel_ent.data_exp[int(self.__pidx_vel), 5], event.ydata])

            # plot circles with lin-space from first to last point in y-coord
            self.__vel_manip_handle_tmp, = self.__axes_v_s.plot(self.__sel_ent.data_exp[self.__vel_mod_range, 0],
                                                                self.__vel_mod_y,
                                                                'ok', alpha=0.5, zorder=100)

            self.__axes_v_s.set_xlim(left=0.0, auto=True)
            self.__axes_v_s.set_ylim(bottom=0.0, auto=True)
            self.__fig_time.canvas.draw_idle()

    def onrelease_vel(self,
                      event) -> None:
        """
        Called whenever a mouse-click release is registered on the temporal-window.

        :param event:       MouseEvent (holds current position of the mouse cursor)
        :return
        """

        if event.button != 1 or self.__pidx_vel is None:
            return

        self.__pidx_vel = None

        # update points
        self.__sel_ent.data_exp[self.__vel_mod_range, 5] = self.__vel_mod_y
        self.update_vel_plot()

        # store coordinates of manipulated indexes
        self.__vel_mod_protect[self.__radio.value_selected] = True

        # clear plotted points
        if self.__vel_manip_handle_tmp is not None:
            self.__vel_manip_handle_tmp.remove()
            self.__vel_manip_handle_tmp = None

    @staticmethod
    def get_ind_under_point(event,
                            x_coords: np.ndarray,
                            y_coords: np.ndarray,
                            epsilon: float = 1.0) -> int or None:
        """
        Get the index of the vertex closest to mouse cursor if within epsilon tolerance (used in the velocity plot).

        :param event:       MouseEvent (holds current position of the mouse cursor)
        :param x_coords:    x-coordinates of the data-line to be analyzed (e.g. s-value in velocity plot)
        :param y_coords:    y-coordinates of the data-line to be analyzed (e.g. v-value in velocity plot)
        :param epsilon:     allowed offset from mouse cursor to points
        :returns idx:       index of coordinate in provided datarow that is closest to mouse cursor
                            'None' if further away than specified epsilon
        """

        idx = None

        # calculate euclidean distances to all points
        dist2 = np.power(x_coords - event.xdata, 2) + np.power(y_coords - event.ydata, 2)

        if min(dist2) < epsilon:
            idx = np.argpartition(dist2, 1)[:1]

        # print(ind)
        return idx


# main --------------------------------------------------------------------------------------------------------------
if __name__ == "__main__":

    # check if package dependencies are met
    check_py_dep()

    # ----- check if scenario file is provided via parameters -----
    if len(sys.argv) == 2:
        # if one argument provided, assume provided file name
        arg_file_path = sys.argv[1]  # First argument
    else:
        arg_file_path = None

    # launch the Scenario Architect
    ScenarioArchitect(load_file=arg_file_path)
