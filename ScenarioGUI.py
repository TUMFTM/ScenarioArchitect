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
from numpy import matlib
import zipfile
import tkinter as tk
import shapely.geometry
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
                 load_file: str = None,
                 data_mode: bool = False) -> None:
        """
        Initializes the Scenario Architect. All internal variables and scenario entities are created. The graphical
        interface (e.g. buttons and switches) as well as plot-areas are initialized.

        :param load_file:   (optional) existing scenario file to be loaded on startup.
        :param data_mode:   (optional) if set to "True", no GUI will be displayed --> used for script interfacing
        :return
        """

        # get config-file
        self.__config = configparser.ConfigParser()
        if not self.__config.read(CONFIG_FILE):
            raise ValueError('Specified config-file does not exist or is empty!')

        self.__color_dict = json.loads(self.__config.get('VISUAL', 'color_dict'))
        self.__keep_veh_plot = self.__config.getboolean('VISUAL', 'keep_veh_plot')
        self.__import_track_shift = self.__config.getboolean('TRACK_IMPORT', 'shift_track')

        # import ggv from file
        top_path = os.path.dirname(os.path.abspath(__file__))
        ggv_path = top_path + "/params/veh_dyn_info/ggv.csv"
        ax_max_machines_path = top_path + "/params/veh_dyn_info/ax_max_machines.csv"
        self.__ggv, self.__ax_max_machines = tph.import_veh_dyn_info. \
            import_veh_dyn_info(ggv_import_path=ggv_path,
                                ax_max_machines_import_path=ax_max_machines_path)

        # --------------------------------------------------------------------------------------------------------------
        # - INIT PLOTS -------------------------------------------------------------------------------------------------
        # --------------------------------------------------------------------------------------------------------------

        # -- main plot -------------------------------------------------------------------------------------------------
        self.__fig_main, ax = plt.subplots(figsize=(10, 6))
        self.__fig_main.canvas.set_window_title("Scenario GUI")
        self.__plot_all_vehicles = False
        self.__plot_all_vehicles_text = False

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

        # change axis of __axes_plane (should be on the left)
        pos_fig_plane = self.__axes_plane.get_position()
        points_fig_plane = pos_fig_plane.get_points()
        points_fig_plane[1][0] -= 0.05
        points_fig_plane[1][1] += 0.05
        pos_fig_plane.set_points(points_fig_plane)
        self.__axes_plane.set_position(pos_fig_plane)

        # -- velocity plot ---------------------------------------------------------------------------------------------
        self.__fig_time, axes = plt.subplots(4, 1, figsize=(8, 9))  # create a window (8x9) with 4 plots
        self.__fig_time.canvas.set_window_title("Temporal Plot")

        self.__axes_a_t = axes[0]
        self.__axes_a_t.grid()
        self.__axes_a_t.set_ylabel("a in m/s²")
        self.__axes_a_t.xaxis.set_ticks_position('none')  # x-ticks invisible
        plt.setp(self.__axes_a_t.get_xticklabels(), visible=False)  # set x-ticks invisible
        # self.__axes_a_t.set_xlim(left=0.0)                            # force 0.0 to be lower bound of x-axis
        # NOTE: fixing lower bound of x-axis here brakes auto-scaling --> enforced on every plot update

        self.__axes_v_t = axes[1]
        plt.subplot(412, sharex=self.__axes_a_t)  # 4 lines, 1 row, number 2 of the plots
        self.__axes_v_t = plt.gca()
        self.__axes_v_t.grid()
        self.__axes_v_t.set_xlabel("t in s", labelpad=0)
        self.__axes_v_t.set_ylabel("v in m/s")
        self.__axes_v_t.xaxis.set_ticks_position('none')  # x-ticks invisible
        self.__axes_v_t.tick_params(axis='x', which='major', pad=0)  # x-tick labels closer to axis
        # self.__axes_v_t.set_ylim(bottom=0.0)                          # force 0.0 to be lower bound of y-axis

        self.__axes_v_s = axes[2]
        self.__axes_v_s.grid()
        self.__axes_v_s.set_xlabel("s in m", labelpad=0)
        self.__axes_v_s.set_ylabel("v in m/s")
        self.__axes_v_s.xaxis.set_ticks_position('none')  # x-ticks invisible
        self.__axes_v_s.tick_params(axis='x', which='major', pad=0)  # x-tick labels closer to axis
        # self.__axes_v_s.set_ylim(bottom=0.0)                         # force 0.0 to be lower bound of y-axis
        # self.__axes_v_s.set_xlim(left=0.0)                           # force 0.0 to be lower bound of x-axis

        self.__axes_ssm = axes[3]
        plt.subplot(414, sharex=self.__axes_a_t)  # 4 lines, 1 row, number 4 of the plots
        self.__axes_ssm = plt.gca()  # shows the plot if the axe is shared
        self.__axes_ssm.grid()
        self.__axes_ssm.set_xlabel("t in s", labelpad=0)
        self.__axes_ssm.set_ylabel("TTC in s  |  DSS/10 in m")
        self.__axes_ssm.xaxis.set_ticks_position('none')  # x-ticks invisible
        self.__axes_ssm.tick_params(axis='x', which='major', pad=0)  # x-tick labels closer to axis
        self.__axes_ssm.set_ylim(bottom=-5.0, top=15.0)

        # instantiate a second axes that shares the same x-axis
        self.__axes_ssm2 = self.__axes_ssm.twinx()
        self.__axes_ssm.set_zorder(self.__axes_ssm.get_zorder() + 1)  # set parent axis zorder higher for event handler
        self.__axes_ssm2.set_ylim(bottom=-5.0, top=15.0)
        self.__axes_ssm2.set_yticks([7.5, 12.5])
        self.__axes_ssm2.set_yticklabels(["Static\nsafety", "Dynamic\nsafety"])

        # -- change axis location of __axes_v_t and __axes_ssm--
        pos_acc_ax = self.__axes_a_t.get_position()
        pos_vel_ax = self.__axes_v_t.get_position()
        pos_sp_ttc = self.__axes_ssm.get_position()

        # get limits of plot boxes (in percent of the plot window) [[left, low], [right, up]]
        points_acc_ax = pos_acc_ax.get_points()
        points_vel_ax = pos_vel_ax.get_points()
        points_sp_ttc = pos_sp_ttc.get_points()

        # set new frame of the velocity plot
        points_vel_ax[0][1] += points_acc_ax[0][1] - 0.01 - points_vel_ax[1][1]  # move up to reveal description
        points_vel_ax[1][1] = points_acc_ax[0][1] - 0.01  # move closer to bottom of acceleration plot (1% offset)

        pos_vel_ax.set_points(points_vel_ax)

        # changes the position of the TTC-x plot (should be lower)
        points_sp_ttc[0][1] -= 0.02
        points_sp_ttc[1][1] -= 0.02

        pos_sp_ttc.set_points(points_sp_ttc)

        # update the new plot setups
        self.__axes_v_t.set_position(pos_vel_ax)
        self.__axes_ssm.set_position(pos_sp_ttc)

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

        np.sqrt(max(self.__ggv[:, 1]) ** 2 + max(self.__ggv[:, 2]) ** 2)
        max_comb_acc = np.max(self.__ggv[:, 1:]) * self.__config.getfloat('SAFETY', 'comb_acc_factor')
        patch_xy = np.array([[-1000, -1000, 1000, 1000], [max_comb_acc, 1000, 1000, max_comb_acc]]).T
        track_ptch = ptch.Polygon(patch_xy, facecolor="red", alpha=0.3, zorder=1)
        self.__axes_a_t.add_artist(track_ptch)
        patch_xy = np.array([[-1000, -1000, 1000, 1000], [-max_comb_acc, -1000, -1000, -max_comb_acc]]).T
        track_ptch = ptch.Polygon(patch_xy, facecolor="red", alpha=0.3, zorder=1)
        self.__axes_a_t.add_artist(track_ptch)

        # add patch limits for SSM plot
        if self.__config.get('SAFETY', 'ssm_safety') == 'ttc':
            min_ssm = self.__config.getfloat('SAFETY', 'ttc_crit')
            mid_ssm = self.__config.getfloat('SAFETY', 'ttc_undefined')
        elif self.__config.get('SAFETY', 'ssm_safety') == 'dss':
            min_ssm = self.__config.getfloat('SAFETY', 'dss_crit') / 10.0
            mid_ssm = self.__config.getfloat('SAFETY', 'dss_undefined') / 10.0
        else:
            min_ssm = 0.0
            mid_ssm = 0.0
        patch_xy = np.array([[-1000, -1000, 1000, 1000], [min_ssm, -1000, -1000, min_ssm]]).T
        limit_ptch = ptch.Polygon(patch_xy, facecolor="red", alpha=0.3, zorder=1)
        self.__axes_ssm.add_artist(limit_ptch)
        patch_xy = np.array([[-1000, -1000, 1000, 1000], [mid_ssm, min_ssm, min_ssm, mid_ssm]]).T
        limit_ptch = ptch.Polygon(patch_xy, facecolor="orange", alpha=0.3, zorder=1)
        self.__axes_ssm.add_artist(limit_ptch)

        # define cursor highlighting selected position in the temporal plot (red vertical line)
        self.__vel_ax_cursor, = self.__axes_v_t.plot([0.0], [0.0], lw=1, color='r', zorder=999)
        self.__acc_ax_cursor, = self.__axes_a_t.plot([0.0], [0.0], lw=1, color='r', zorder=999)
        self.__ttc_ax_cursor, = self.__axes_ssm.plot([0.0], [0.0], lw=1, color='r', zorder=999)
        self.__cursor_last_t = 0.0

        # handles for automatic safety rating in scenario
        self.__safety_dyn_safe, = self.__axes_ssm.plot([], [], lw=10, color=self.__color_dict["TUM_green"], alpha=0.5,
                                                       zorder=999, label="  Safe ")
        self.__safety_dyn_unsafe, = self.__axes_ssm.plot([], [], lw=10, color=self.__color_dict["TUM_orange"],
                                                         alpha=0.5, zorder=999, label="  Unsafe")
        self.__safety_stat_safe, = self.__axes_ssm.plot([], [], lw=10, color=self.__color_dict["TUM_green"], alpha=0.5,
                                                        zorder=999)
        self.__safety_stat_unsafe, = self.__axes_ssm.plot([], [], lw=10, color=self.__color_dict["TUM_orange"],
                                                          alpha=0.5, zorder=999)

        # store file-path (if saving again, suggest same name)
        self.__load_file = load_file

        # --------------------------------------------------------------------------------------------------------------
        # - DEFINE GUI ELEMENTS (BUTTONS, SLIDERS, ...) ----------------------------------------------------------------
        # --------------------------------------------------------------------------------------------------------------

        # Set time window as active figure
        plt.figure(self.__fig_time.number)

        # Playback button
        plybck_ax = self.__fig_time.add_axes([0.02, 0.95, 0.15, 0.04])
        button_plybck = Button(plybck_ax, 'Playback', color=self.__config.get('VISUAL', 'btn_color'),
                               hovercolor='0.975')
        button_plybck.on_clicked(self.on_playback_click)

        # Open window with SSM info
        ssm = self.__fig_time.add_axes([0.22, 0.95, 0.15, 0.04])
        button_ssm = Button(ssm, 'SSM Information', color=self.__config.get('VISUAL', 'btn_color'),
                            hovercolor='0.975')
        button_ssm.on_clicked(self.on_ssm_click)

        # Set main window as active figure
        plt.figure(self.__fig_main.number)

        # Text
        text_ax = self.__fig_main.add_axes([0.8, 0.82, 0.15, 0.2])
        text_ax.axis('off')
        text_ax.set_navigate(False)
        self.__entity_descr = text_ax.text(0.0, 0.0, "Select entity\nto be drawn:")

        # Radio buttons for entity selection
        rax = self.__fig_main.add_axes([0.8, 0.55, 0.15, 0.25], facecolor=self.__config.get('VISUAL', 'btn_color'))
        self.__radio = RadioButtons(rax, ('None', 'bound_l', 'bound_r', 'veh_1', 'veh_2', 'veh_3', 'veh_4', 'veh_5'),
                                    active=0)
        self.__radio.on_clicked(self.toggled_radio)

        # Reset all button
        button_reset = Button(self.__fig_main.add_axes([0.8, 0.50, 0.15, 0.04]), 'Reset all Entities',
                              color=self.__config.get('VISUAL', 'btn_color'),
                              hovercolor='0.975')
        button_reset.on_clicked(self.reset_all)

        # Reset button
        button_reset_ent = Button(self.__fig_main.add_axes([0.8, 0.45, 0.15, 0.04]), 'Reset Entity',
                                  color=self.__config.get('VISUAL', 'btn_color'),
                                  hovercolor='0.975')
        button_reset_ent.on_clicked(self.reset_ent)

        # Specify import button
        button_import = Button(self.__fig_main.add_axes([0.8, 0.40, 0.15, 0.04]), 'Import Scen.',
                               color=self.__config.get('VISUAL', 'btn_color'),
                               hovercolor='0.975')
        button_import.on_clicked(self.load_pckl)

        # Specify export button
        button_export = Button(self.__fig_main.add_axes([0.8, 0.35, 0.15, 0.04]), 'Export Scen.',
                               color=self.__config.get('VISUAL', 'btn_color'),
                               hovercolor='0.975')
        button_export.on_clicked(self.export_to_file)

        # Specify import track button
        button_import_track = Button(self.__fig_main.add_axes([0.8, 0.30, 0.15, 0.04]), 'Import Track',
                                     color=self.__config.get('VISUAL', 'btn_color'),
                                     hovercolor='0.975')
        button_import_track.on_clicked(self.load_track)

        # Plot all vehicles checkbox
        check_all_veh = CheckButtons(self.__fig_main.add_axes([0.8, 0.19, 0.15, 0.1]), ['All Poses', 'Add Text'],
                                     [self.__plot_all_vehicles, self.__plot_all_vehicles_text])
        check_all_veh.on_clicked(self.toggled_veh)

        # Open plot window button
        button_plt_wndw = Button(self.__fig_main.add_axes([0.8, 0.09, 0.15, 0.04]), 'Open Plot',
                                 color=self.__config.get('VISUAL', 'btn_color'),
                                 hovercolor='0.975')
        button_plt_wndw.on_clicked(self.open_plot_window)

        # Scaling reset button
        button = Button(self.__fig_main.add_axes([0.8, 0.02, 0.15, 0.04]), 'Reset Scale',
                        color=self.__config.get('VISUAL', 'btn_color'),
                        hovercolor='0.975')
        button.on_clicked(self.reset)

        # Scaling slider
        self.__sld_x_axis = Slider(self.__fig_main.add_axes([0.1, 0.02, 0.6, 0.03],
                                                            facecolor=self.__config.get('VISUAL', 'btn_color')),
                                   label='Scale',
                                   valmin=self.__config.getfloat('VISUAL', 'delta_xy'),
                                   valmax=self.__config.getfloat('VISUAL', 'max_x_scale'),
                                   valinit=self.__config.getfloat('VISUAL', 'default_x_scale'),
                                   valstep=self.__config.getfloat('VISUAL', 'delta_xy'))
        self.__sld_x_axis_cid = self.__sld_x_axis.on_changed(self.change_ratio)

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
                                                      ssm_t_axis=self.__axes_ssm,
                                                      color_str=self.get_veh_color(1),
                                                      veh_width=self.__config.getfloat('VEHICLE', 'veh_width'),
                                                      veh_length=self.__config.getfloat('VEHICLE', 'veh_length'),
                                                      plan_hor=self.__config.getfloat('FILE_EXPORT',
                                                                                      'export_time_traj_horizon')),
        )

        # add 4 other vehicles
        for i in range(2, 6):
            self.__ent_cont.append(
                helper_funcs.src.ScenarioEntities.Vehicle(i_in=i,
                                                          plane_axis=self.__axes_plane,
                                                          a_t_axis=self.__axes_a_t,
                                                          v_t_axis=self.__axes_v_t,
                                                          v_s_axis=self.__axes_v_s,
                                                          ssm_t_axis=self.__axes_ssm,
                                                          color_str=self.get_veh_color(i),
                                                          veh_width=self.__config.getfloat('VEHICLE', 'veh_width'),
                                                          veh_length=self.__config.getfloat('VEHICLE', 'veh_length'))
            )

        # currently selected entity (via radio-buttons - one element in the list of '__ent_cont')
        self.__sel_ent = None

        self.__pidx_vel = None
        self.__pidx_main = None

        # load scenario, if provided
        if load_file is not None:
            self.load_pckl(_=None,
                           file_path=load_file)

        if not data_mode:
            plt.show()

    def __del__(self):
        plt.close(self.__fig_main)
        plt.close(self.__fig_time)

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

    def reset_all(self,
                  _) -> None:

        # reset container
        if self.__radio.value_selected != 'None':
            # remove entity from list of protected velocity profiles
            self.__vel_mod_protect.pop(self.__radio.value_selected, None)

        for obj in self.__ent_cont:
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
        ttc_ylim = self.__axes_ssm.get_ylim()

        while t < t_range[1]:
            tic = time.time()

            # update cursor in velocity window
            self.__vel_ax_cursor.set_data([t, t], vel_ylim)
            self.__acc_ax_cursor.set_data([t, t], acc_ylim)
            self.__ttc_ax_cursor.set_data([t, t], ttc_ylim)
            self.__fig_time.canvas.draw_idle()

            # for all trajectories holding temporal information, calculate index
            for obj in self.__ent_cont:
                if obj.TYPE_VEHICLE:
                    obj.highlight_pose(t_in=t)

            plt.pause(0.001)
            t += time.time() - tic

    def on_ssm_click(self,
                     _) -> None:

        """
        Called when the 'SSM information'-button is clicked.
        """

        # definition of a function to destroy and refresh / reload the window
        def button_ok_click():
            window.destroy()

        # definition of a function, which opens a new window with information about the SSM calculation
        def button_information_calculation():
            messagebox.showinfo('SSM Information',
                                'Key facts for the calculation of the SSM:\n'
                                '- The lane-based coordinates can only be calculated for paths after the first point '
                                'and before the last point of the lane-based coordinate system.\n'
                                '- Both bounds must be specified for the calculation of the SSMs.\n'
                                '- The SSM scores are only calculated for the ego vehicle in relation to each other '
                                'vehicle.')

        def calculate_info():
            # init variables
            # Variable that stores whether an ego vehicle is present or not (False = no ego vehicle)
            ego_veh_used = False

            # variables to calculate the lateral distance between teh vehicles
            pos_ego_stamps = None

            # counter for bounds
            counter_bound = 0

            veh_width = self.__config.getfloat('VEHICLE', 'veh_width')
            veh_length = self.__config.getfloat('VEHICLE', 'veh_length')

            for bound in self.__ent_cont:
                if bound.TYPE_BOUND and bound.get_mode_coord() is not None:
                    counter_bound += 1

            # generate information about the SSMs
            for obj in self.__ent_cont:

                if obj.TYPE_VEHICLE:
                    info_txt = ""

                    # SSM information for veh_1 (ego-vehicle)
                    if obj.id == 'veh_1':
                        # if the ego-vehicle is used
                        if obj.get_mode_coord() is not None and counter_bound == 2:
                            info_txt = 'There is no SSM calculation for the ego-vehicle.'
                            ego_veh_used = True
                            pos_ego_stamps = obj.data_ssm[:, 3:5]

                        # if the Ego-Vehicle is not specified
                        elif obj.get_mode_coord() is None and counter_bound == 2:
                            info_txt = 'Ego-vehicle is not specified yet!'
                        else:
                            info_txt = 'SSM calculation is not possible since one or both bounds are not specified.'

                    # SSM information for the other vehicles
                    else:

                        # check if preconditions hold
                        if obj.get_mode_coord() is not None and max(obj.data_ssm[:, 1]) != 0 and ego_veh_used is True \
                                and counter_bound == 2 and obj.data_ssm is not None:

                            # check if vehicles drive in same direction
                            driving_same_direction = ((pos_ego_stamps[0, 0] < pos_ego_stamps[-1, 0]
                                                       and obj.data_ssm[0, 3] < obj.data_ssm[-1, 3])
                                                      or (pos_ego_stamps[0, 0] > pos_ego_stamps[-1, 0]
                                                          and obj.data_ssm[0, 3] > obj.data_ssm[-1, 3]))

                            if driving_same_direction:

                                # get minimal values
                                i_ttc = np.nanargmin(obj.data_ssm[:, 1])
                                i_dss = np.nanargmin(obj.data_ssm[:, 2])
                                print(obj.data_ssm[:, 2])

                                # check for collision at given points
                                if not (abs(pos_ego_stamps[i_ttc, 0] - obj.data_ssm[i_ttc, 3]) < veh_length * 1.2
                                        and abs(pos_ego_stamps[i_ttc, 1] - obj.data_ssm[i_ttc, 4]) < veh_width * 1.2):
                                    ttc_col_pre = 'no '
                                else:
                                    ttc_col_pre = 'possible '

                                if not (abs(pos_ego_stamps[i_dss, 0] - obj.data_ssm[i_dss, 3]) < veh_length * 1.2
                                        and abs(pos_ego_stamps[i_dss, 1] - obj.data_ssm[i_dss, 4]) < veh_width * 1.2):
                                    dss_col_pre = 'no '
                                else:
                                    dss_col_pre = 'possible '

                                # generate info text
                                info_txt = ('The minimal TTC is ' + format(obj.data_ssm[i_ttc, 1], '.2f')
                                            + ' s at t=' + format(obj.data_ssm[i_ttc, 0], '.2f') + ' s. '
                                            + '(' + ttc_col_pre + 'collision)\n'
                                            + 'The minimal DSS is ' + format(obj.data_ssm[i_dss, 2], '.2f')
                                            + ' m at t=' + format(obj.data_ssm[i_dss, 0], '.2f') + ' s. '
                                            + '(' + dss_col_pre + 'collision)')

                            else:
                                info_txt = 'Vehicle is driving in opposing direction!'

                        # if not all bounds specified
                        elif counter_bound < 2:
                            print("updated")
                            info_txt = 'SSM calculation is not possible since one or both bounds are not specified.'

                        # if the ego-vehicle is not used
                        elif ego_veh_used is False and counter_bound == 2:
                            info_txt = 'SSM calculation is not possible since the ego-vehicle (veh_1) is not specified!'

                        # if the vehicle is not specified
                        elif obj.get_mode_coord() is None and obj.data_ssm is None:
                            info_txt = 'Vehicle "' + str(obj.id) + '" is not specified!'

                    # set label text
                    label_info_dict[obj.id].configure(text=info_txt)

        # -- init window elements (labels and buttons) --
        # open window
        window = tk.Tk()
        window.title('TTC information')
        window.geometry('600x300')

        # text at the top of the new window
        label = tk.Label(window, text='This window provides detailed information about the SMMs:')
        label.place(x=10, y=5)

        # init vehicle labels
        y = 50
        label_info_dict = dict()
        for obj_ in self.__ent_cont:
            if obj_.TYPE_VEHICLE:
                label_descr = tk.Label(window, text=obj_.id + ":")
                label_descr.place(x=10, y=y)

                label_info_dict[obj_.id] = tk.Label(window, text='--', anchor="e", justify=tk.LEFT)
                label_info_dict[obj_.id].place(x=100, y=y)

                y += 40

        # update data
        calculate_info()

        # button to refresh the window
        button = tk.Button(master=window, text='Refresh', command=calculate_info)
        button.place(x=10, y=275, width=150, height=20)

        # button to close the window
        button_ok = tk.Button(master=window, text='Close', command=button_ok_click)
        button_ok.place(x=210, y=275, width=150, height=20)

        # button to open a window with information about the ttc calculation the window
        button_info = tk.Button(master=window, text='SSM calculation info', command=button_information_calculation)
        button_info.place(x=410, y=275, width=150, height=20)

        tk.mainloop()

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
                    if (obj.data_vel_coord is None
                            or (self.__pidx_main is not None and obj == self.__radio.value_selected)
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

        # -------------------------------------- calculate center line -------------------------------------------------
        calc_center_line = None

        # get the center line and the distance s
        if self.__ent_cont[0].TYPE_BOUND and self.__ent_cont[1].TYPE_BOUND:
            if self.__ent_cont[0].get_mode_coord() is not None and \
                    self.__ent_cont[1].get_mode_coord() is not None:

                # calculate number of shared elements of the left and right bound
                n_shared_el = min(self.__ent_cont[0].get_mode_coord().shape[0],
                                  self.__ent_cont[1].get_mode_coord().shape[0])

                # for the number of not shared elements
                not_shared_el = abs(self.__ent_cont[0].get_mode_coord().shape[0]
                                    - self.__ent_cont[1].get_mode_coord().shape[0])

                # define the left and right bound
                bound_left = np.zeros(shape=(n_shared_el + not_shared_el, 2))
                bound_right = np.zeros(shape=(n_shared_el + not_shared_el, 2))

                # Calculate the values of the bounds for the same number of bound points on the left and right
                # side
                if n_shared_el > 1:
                    # init bounds by filling common places
                    bound_left[:self.__ent_cont[0].get_mode_coord().shape[0]] = self.__ent_cont[0].get_mode_coord()
                    bound_right[:self.__ent_cont[1].get_mode_coord().shape[0]] = self.__ent_cont[1].get_mode_coord()

                    # if the right bound [1] has more elements
                    if self.__ent_cont[0].get_mode_coord().shape[0] < self.__ent_cont[1].get_mode_coord().shape[0]:
                        bound_left[n_shared_el:, :] = bound_left[n_shared_el - 1, :]

                    # if the left bound [0] has more elements
                    elif self.__ent_cont[0].get_mode_coord().shape[0] > self.__ent_cont[1].get_mode_coord().shape[0]:
                        bound_right[n_shared_el:, :] = bound_right[n_shared_el - 1, :]

                    # Calculate the center line and the distance s
                    calc_center_line = helper_funcs.src.ssm.calc_center_line(bound_l=bound_left,
                                                                             bound_r=bound_right)

        # -------------------------------- calculate lane based coordinates for ego (for TTC) --------------------------
        # calculate the lane based coordinates for all vehicles expect the ego vehicle
        vehicle_ego_info = None

        # calculate the lane based coordinates for the ego vehicle
        if calc_center_line is not None and self.__ent_cont[2].TYPE_VEHICLE and self.__ent_cont[2].TYPE_EGO \
                and self.__ent_cont[2].data_exp is not None:
            # init lane based position for each point in data_exp
            ego_pos_lane = np.zeros((self.__ent_cont[2].data_exp.shape[0], 3))

            # for each point in data_exp calculate the lane based coordinates (lane_pos holds n, s, track angle)
            for i in range(int(self.__ent_cont[2].data_exp.shape[0])):
                ego_pos_lane[i, 0:3] = helper_funcs.src.ssm.global_2_lane_based(pos=self.__ent_cont[2].data_exp[i][1:3],
                                                                                center_line=calc_center_line[0],
                                                                                s_course=calc_center_line[1])

            # calculate the value of position and speed for defined increment
            vehicle_ego_info = helper_funcs.src.ssm.timestamp_info(lane_based_poses=ego_pos_lane,
                                                                   covered_distance=self.__ent_cont[2].data_exp[:, 0],
                                                                   velocity=self.__ent_cont[2].data_exp[:, 5],
                                                                   t_increment=self.__config.getfloat('SAFETY',
                                                                                                      'ssm_increment'),
                                                                   t_horizon=self.__config.getfloat('SAFETY',
                                                                                                    'ssm_horizon'))

        # for all entities
        for obj in self.__ent_cont:

            # if entity is a vehicle
            if obj.TYPE_VEHICLE:
                # -------------------------------- update (if) or reset (else) plot ------------------------------------
                if obj.get_mode_coord() is not None and obj.data_exp is not None:

                    # skip plot update, if data did not change; continue if the plot has changed or is forced to update
                    if obj.data_vel_coord is None or max(obj.data_exp[:, 5]) == 0.0 \
                            or self.__radio.value_selected == obj.id or self.__plot_all_vehicles \
                            or all_veh_text_need_update or force_update:
                        # ------------------------------ calculate velocity and acceleration ---------------------------
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

                        # -------------------------------------- calculate TTC -----------------------------------------
                        # if velocity has been calculated
                        if max(obj.data_exp[:, 5]) != 0.0:

                            # calculate the passed time for each position
                            t = abs(np.concatenate(([0], np.cumsum(np.divide(np.diff(obj.data_exp[:, 0]),
                                                                             obj.data_exp[:-1, 5],
                                                                             out=np.full(obj.data_exp[:-1, 5].shape[0],
                                                                                         np.inf),
                                                                             where=obj.data_exp[:-1, 5] != 0)))))

                            # get number of discretized steps based on time-increment
                            n_steps = int((min(max(t), self.__config.getfloat('SAFETY', 'ssm_horizon'))
                                           / self.__config.getfloat('SAFETY', 'ssm_increment')) + 1)

                            # init variable data_ssm (time, TTC, DSS, s_pos, n_pos, heading, velocity)
                            obj.data_ssm = np.zeros((n_steps, 7))

                            # if ego vehicle, calculate braking dist for every point (used for safety metrics)
                            if obj.TYPE_EGO and obj.data_exp.shape[0] > 5:
                                # init brake storage
                                obj.temp_container['brake_dist'] = np.column_stack([[0.0] * obj.data_exp.shape[0],
                                                                                    [None] * obj.data_exp.shape[0]])

                                for i in range(obj.data_exp.shape[0] - 5):
                                    obj.temp_container['brake_dist'][i, 0] = t[i]
                                    obj.temp_container['brake_dist'][i, 1] = helper_funcs.src.calc_brake_emergency \
                                        .calc_brake_dist(traj=obj.data_exp[i:, 1:],
                                                         ggv=self.__ggv)

                        else:
                            # (re)init variables if no velocity profile exists
                            obj.data_ssm = np.zeros((obj.data_exp.shape[0], 7))

                        # -- calculate the surrogate safety metric(s) --
                        if calc_center_line is not None and vehicle_ego_info is not None \
                                and max(obj.data_exp[:, 5]) != 0.0:

                            # calculate the lane based coordinates
                            # init variable to save lane-based coordinates (n-coordinate, s-coordinate, track heading)
                            veh_pose_lane = np.zeros((obj.data_exp.shape[0], 3))

                            # calculate and save lane-based values
                            for i in range(int(obj.data_exp.shape[0])):
                                veh_pose_lane[i, 0:3] = helper_funcs.src.ssm.\
                                    global_2_lane_based(pos=obj.data_exp[i][1:3],
                                                        center_line=calc_center_line[0],
                                                        s_course=calc_center_line[1])

                            # calculate the position and speed for defined increment
                            vehicle_info = helper_funcs.src.ssm.timestamp_info(lane_based_poses=veh_pose_lane,
                                                                               covered_distance=obj.data_exp[:, 0],
                                                                               velocity=obj.data_exp[:, 5],
                                                                               t_increment=self.__config.getfloat
                                                                               ('SAFETY', 'ssm_increment'),
                                                                               t_horizon=self.__config.getfloat
                                                                               ('SAFETY', 'ssm_horizon'))

                            # store data to object
                            obj.data_ssm[:, 0] = vehicle_info[:, 0]
                            obj.data_ssm[:, 3:] = vehicle_info[:, 1:]

                            # calculate the time to collision when the ego_vehicle and another vehicle exists
                            if vehicle_info is not None and vehicle_ego_info is not None and not obj.TYPE_EGO and \
                                    obj.data_ssm.shape[0] > 1:

                                # get ego stopping dist, if available
                                ego_stopping_dist = None
                                if self.__ent_cont[2].TYPE_EGO:
                                    ego_stopping_dist = self.__ent_cont[2].temp_container.get('brake_dist', None)

                                # calculate the ttc for every timestamp
                                ttc, dss = helper_funcs.src.ssm.calc_ssm(
                                    pos_ego_stamps=vehicle_ego_info,
                                    pos_vehicle_stamps=vehicle_info,
                                    veh_length=self.__config.getfloat('VEHICLE', 'veh_length'),
                                    reaction_time=self.__config.getfloat('SAFETY', 'dds_reaction_t'),
                                    maximum_acc=np.max(self.__ggv[:, 1:]),
                                    ego_stopping_dist=ego_stopping_dist
                                )

                                # save tht ttc value in data_exp
                                obj.data_ssm[:, 1] = ttc
                                obj.data_ssm[:, 2] = dss

                            # if only the ego vehicle is used
                            elif vehicle_info is not None and obj.TYPE_EGO:

                                obj.data_ssm[:, 1] = None
                                obj.data_ssm[:, 2] = None

                        # ------------------------------------ update temporal plots -----------------------------------
                        obj.update_temporal_plot()

                        # ------------------------------------ plot temporal selector ----------------------------------
                        if self.__radio.value_selected in obj.id:
                            self.__vel_manip_handle, = self.__axes_v_s.plot(obj.data_exp[:, 0], obj.data_exp[:, 5],
                                                                            'ok', alpha=0.5, zorder=100)

                        # ---------------------------- plot all vehicles along trajectories ----------------------------
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
                        temp_ttc = self.__ttc_ax_cursor.get_xdata()
                        self.__vel_ax_cursor.set_data([[], []])
                        self.__acc_ax_cursor.set_data([[], []])
                        self.__ttc_ax_cursor.set_data([[], []])

                        # auto-scale axes (without cursor)
                        self.__axes_v_t.relim()
                        self.__axes_v_t.autoscale_view(True, True, True)
                        self.__axes_v_s.relim()
                        self.__axes_v_s.autoscale_view(True, True, True)
                        self.__axes_a_t.relim()
                        self.__axes_a_t.autoscale_view(True, True, True)
                        self.__axes_ssm.relim()
                        self.__axes_ssm.autoscale_view(True, True, True)

                        # force lower limits on axes (has to be done in every iteration, since autoscale is broken else)
                        self.__axes_a_t.set_xlim(left=0.0, auto=True)
                        self.__axes_v_t.set_ylim(bottom=0.0, auto=True)
                        self.__axes_v_s.set_xlim(left=0.0, auto=True)
                        self.__axes_v_s.set_ylim(bottom=0.0, auto=True)
                        self.__axes_ssm.set_xlim(left=0.0, auto=True)

                        # restore cursors
                        self.__vel_ax_cursor.set_data(temp_va, self.__axes_v_t.get_ylim())
                        self.__acc_ax_cursor.set_data(temp_aa, self.__axes_a_t.get_ylim())
                        self.__ttc_ax_cursor.set_data(temp_ttc, self.__axes_ssm.get_ylim())

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

        # ---------------------------- generate safety evaluation for scene --------------------------------------------
        time_stamps = None
        safety_dynamic = None
        safety_static = None
        if self.__ent_cont[2].TYPE_VEHICLE and self.__ent_cont[2].TYPE_EGO and self.__ent_cont[2].data_exp is not None:
            # get maximum time present in ego-trajectory
            # calculate temporal coordinate (for temporal plots)
            t = np.concatenate(([0], np.cumsum(np.divide(np.diff(self.__ent_cont[2].data_exp[:, 0]),
                                                         self.__ent_cont[2].data_exp[:-1, 5],
                                                         out=np.full(self.__ent_cont[2].data_exp[:-1, 5].shape[0],
                                                                     np.inf),
                                                         where=self.__ent_cont[2].data_exp[:-1, 5] != 0))))

            time_stamps = np.arange(0.0, max(t), self.__config.getfloat('SAFETY', 'ssm_increment'))

            safety_dynamic, safety_static = self.get_safety(time_stamps=time_stamps)

        # plot safety, if not none
        if safety_dynamic is not None and safety_static is not None:
            self.__safety_dyn_safe.set_data([time_stamps,
                                             [12.5 if s is not None and s else None for s in safety_dynamic]])
            self.__safety_dyn_unsafe.set_data([time_stamps,
                                               [12.5 if s is not None and not s else None for s in safety_dynamic]])
            self.__safety_stat_safe.set_data([time_stamps,
                                              [7.5 if s is not None and s else None for s in safety_static]])
            self.__safety_stat_unsafe.set_data([time_stamps,
                                                [7.5 if s is not None and not s else None for s in safety_static]])

        else:
            self.__safety_dyn_safe.set_data([[], []])
            self.__safety_stat_safe.set_data([[], []])

        # update plots
        self.__fig_time.canvas.draw_idle()
        self.__fig_main.canvas.draw_idle()

    # ------------------------------------------------------------------------------------------------------------------
    # - SAFETY GROUND TRUTH --------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    def get_safety(self,
                   time_stamps: np.ndarray):
        """
        Calculate a safety rating (dynamic and static environment) at the given time-stamps. Each time-stamp is mapped
        to one of the following ratings:
            - True:     associated time-stamp is safe
            - False:    associated time-stamp is unsafe
            - None:     associated time-stamp does not hold any rating (can be both)

        :param time_stamps:
        :return:
        """

        # load relevant data
        veh_width = self.__config.getfloat('VEHICLE', 'veh_width')
        veh_length = self.__config.getfloat('VEHICLE', 'veh_length')
        drag_coeff = self.__config.getfloat('VEHICLE', 'drag_coeff')
        m_veh = self.__config.getfloat('VEHICLE', 'm_veh')
        ttc_crit = self.__config.getfloat('SAFETY', 'ttc_crit')
        dss_crit = self.__config.getfloat('SAFETY', 'dss_crit')
        ttc_undefined = self.__config.getfloat('SAFETY', 'ttc_undefined')
        dss_undefined = self.__config.getfloat('SAFETY', 'dss_undefined')
        ssm_safety = self.__config.get('SAFETY', 'ssm_safety')
        si = self.__config.getfloat('SAFETY', 'shape_inflation')
        acc_factor = self.__config.getfloat('SAFETY', 'comb_acc_factor')
        turn_radius_unsafe = self.__config.getfloat('SAFETY', 'turn_radius_unsafe')
        turn_radius_undefined = self.__config.getfloat('SAFETY', 'turn_radius_undefined')
        acc_m_factor = self.__config.getfloat('SAFETY', 'mach_acc_factor')
        ax_max = max(self.__ggv[:, 1])
        ay_max = max(self.__ggv[:, 2])

        # check if all required parameters have been set (bounds, ego vehicle)
        if not ((self.__ent_cont[0].TYPE_BOUND and self.__ent_cont[0].get_mode_coord() is not None
                 and len(self.__ent_cont[0].get_mode_coord()) >= 3)
                and (self.__ent_cont[1].TYPE_BOUND and self.__ent_cont[1].get_mode_coord() is not None
                     and len(self.__ent_cont[1].get_mode_coord()) >= 3)
                and (self.__ent_cont[2].TYPE_VEHICLE and self.__ent_cont[2].TYPE_EGO
                     and self.__ent_cont[2].data_exp is not None)):
            return None, None

        # init safety arrays
        safety_dynamic = [None] * len(time_stamps)
        safety_static = [True] * len(time_stamps)

        # setup shapely geometry for track (extend at start and end)
        ds1 = np.diff(self.__ent_cont[0].get_mode_coord()[:2, :], axis=0)[0]
        ds2 = np.diff(self.__ent_cont[0].get_mode_coord()[-2:, :], axis=0)[0]
        ds3 = np.diff(self.__ent_cont[1].get_mode_coord()[:2, :], axis=0)[0]
        ds4 = np.diff(self.__ent_cont[1].get_mode_coord()[-2:, :], axis=0)[0]
        bound_outline = np.vstack((self.__ent_cont[0].get_mode_coord()[0, :] - ds1 * 5.0 / np.hypot(ds1[0], ds1[1]),
                                   self.__ent_cont[0].get_mode_coord(),
                                   self.__ent_cont[0].get_mode_coord()[-1, :] + ds2 * 5.0 / np.hypot(ds2[0], ds2[1]),
                                   self.__ent_cont[1].get_mode_coord()[-1, :] + ds4 * 5.0 / np.hypot(ds4[0], ds4[1]),
                                   np.flipud(self.__ent_cont[1].get_mode_coord()),
                                   self.__ent_cont[1].get_mode_coord()[0, :] - ds3 * 5.0 / np.hypot(ds3[0], ds3[1])))

        track_polygon = shapely.geometry.Polygon([list(bound_outline[i, :]) for i in range(bound_outline.shape[0])])

        for i, t_in in enumerate(time_stamps):
            # -- get ego data for desired time-stamp -------------------------------------------------------------------
            # get global data for ego-vehicle (pos, heading, curv, vel, acc)
            data_ego_global = self.__ent_cont[2].get_timestamp_info(t_in=t_in)
            pos, heading, curv, vel, a_lon_used = data_ego_global

            # get lane based data for ego-vehicle (pos_lane, heading, vel)
            data_ego_lane = self.__ent_cont[2].get_timestamp_info_lanebased(t_in=t_in)

            # -- safety in dynamic environment (other vehicles, via TTC) -----------------------------------------------
            # for all entities
            tmp_safety_dyn = True
            for obj in self.__ent_cont:
                # if entity is a vehicle
                if obj.TYPE_VEHICLE and not obj.TYPE_EGO:
                    if obj.get_mode_coord() is not None and obj.data_ssm is not None:
                        # interpolate lane pos for object vehicle
                        pos_lane, _, vel_tmp = obj.get_timestamp_info_lanebased(t_in=t_in)

                        # if time-track of object vehicle exceeded, set "safe" rating
                        if vel_tmp is None:
                            safety = True

                        else:
                            # get safety evaluation
                            if ssm_safety == 'ttc':
                                # interpolate TTC for current time-stamp
                                ttc = np.interp(t_in, obj.data_ssm[:, 0], obj.data_ssm[:, 1])
                                safety = helper_funcs.src.ssm.evaluation_ttc(ttc=ttc,
                                                                             lane_pos_obj=pos_lane,
                                                                             lane_pos_ego=data_ego_lane[0],
                                                                             veh_width=veh_width,
                                                                             undefined_ttc=ttc_undefined,
                                                                             crit_ttc=ttc_crit)
                            elif ssm_safety == 'dss':
                                # interpolate DSS for current time-stamp
                                dss = np.interp(t_in, obj.data_ssm[:, 0], obj.data_ssm[:, 2])
                                safety = helper_funcs.src.ssm.evaluation_dss(dss=dss,
                                                                             lane_pos_obj=pos_lane,
                                                                             lane_pos_ego=data_ego_lane[0],
                                                                             veh_width=veh_width,
                                                                             undefined_dss=dss_undefined,
                                                                             crit_dss=dss_crit)

                            else:
                                raise ValueError('Specified "ssm_safety" parameter (' + str(ssm_safety)
                                                 + ') is not supported!')

                        # update safety rating of current step
                        if safety is not None and not safety:
                            # if unsafe for this vehicle, set globally as unsafe
                            tmp_safety_dyn = False
                            break
                        elif safety is None and tmp_safety_dyn:
                            # if not set to 'False' for any object and undefined for this vehicle
                            tmp_safety_dyn = None

            safety_dynamic[i] = tmp_safety_dyn

            # -- safety in static environment --------------------------------------------------------------------------
            tmp_safety_stat = True

            # -- collisions with wall --
            # check for collisions with wall (one larger --> any rating allowed, one actual size --> must be unsafe)
            veh_polygon = (matlib.repmat([[pos[0]], [pos[1]]], 1, 4)
                           + np.matmul([[np.cos(heading + np.pi / 2), -np.sin(heading + np.pi / 2)],
                                        [np.sin(heading + np.pi / 2), np.cos(heading + np.pi / 2)]],
                                       [[-veh_length / 2, veh_length / 2, veh_length / 2, -veh_length / 2],
                                        [-veh_width / 2, -veh_width / 2, veh_width / 2, veh_width / 2]]))

            veh_polygon = shapely.geometry.Polygon([list(veh_polygon[:, i]) for i in range(veh_polygon.shape[1])])

            # if vehicle is not contained in track, rate unsafe and skip rest of evaluations
            if not track_polygon.contains(veh_polygon):
                safety_static[i] = False
                continue

            else:
                # if actual vehicle shape is within track, check if larger shape exceeds track
                veh_polygon = (matlib.repmat([[pos[0]], [pos[1]]], 1, 4)
                               + np.matmul([[np.cos(heading + np.pi / 2) * si, -np.sin(heading + np.pi / 2) * si],
                                            [np.sin(heading + np.pi / 2) * si, np.cos(heading + np.pi / 2) * si]],
                                           [[-veh_length / 2, veh_length / 2, veh_length / 2, -veh_length / 2],
                                            [-veh_width / 2, -veh_width / 2, veh_width / 2, veh_width / 2]]))
                veh_polygon = shapely.geometry.Polygon([list(veh_polygon[:, i]) for i in range(veh_polygon.shape[1])])

                if not track_polygon.contains(veh_polygon):
                    tmp_safety_stat = None

            # -- friction exceeded --
            # lateral acceleration based on curvature and velocity
            a_lat_used = np.power(vel, 2) * curv

            # drag reduces requested deceleration but increases requested acceleration at the tire
            a_lon_used += np.power(vel, 2) * drag_coeff / m_veh

            # get used percentage of allowed tire acceleration including safety factor
            a_comb_used_perc = np.sqrt((np.power(np.abs(a_lon_used) / (ax_max * acc_factor), 2)
                                        + np.power(np.abs(a_lat_used) / (ay_max * acc_factor), 2)))

            # if acceleration with safety factor is exceeded, rate unsafe  and skip rest of evaluations
            if a_comb_used_perc > 1.0:
                safety_static[i] = False
                continue

            elif tmp_safety_stat is not None:
                # get used percentage of allowed tire acceleration
                a_comb_used_perc = np.sqrt((np.power(np.abs(a_lon_used) / ax_max, 2)
                                            + np.power(np.abs(a_lat_used) / ay_max, 2)))

                if a_comb_used_perc > 1.0:
                    tmp_safety_stat = None

            # -- kinematic limits --
            if abs(curv) > (1 / turn_radius_unsafe):
                safety_static[i] = False
                continue

            elif abs(curv) > (1 / turn_radius_undefined):
                tmp_safety_stat = None

            # -- motor limits --
            ax_max_lim = np.interp(vel, self.__ax_max_machines[:, 0], self.__ax_max_machines[:, 1])

            # if violated, rate unsafe and skip rest of evaluations
            if a_lon_used > ax_max_lim * acc_m_factor:
                safety_static[i] = False
                continue

            elif a_lon_used > ax_max_lim:
                tmp_safety_stat = None

            safety_static[i] = tmp_safety_stat

        return safety_dynamic, safety_static

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
                       _,
                       file_path=None) -> None:
        """
        Called when the export button is pressed. Handles export of the constructed scene to a CSV-file representation
        of the scene with certain temporal increment (suited for input to other functions). The file path is defined by
        and prompt dialog (GUI).

        :param file_path:       (optional) provide file_path and do not use path selection GUI
        """
        # -- calculate trajectory information for every vehicle (will be used later for export) --
        veh_data = {}
        for obj in self.__ent_cont:
            # calculate trajectory information
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

                if obj.id in veh_data:
                    # recalculate acceleration profiles since linear interpolations are used when extracting the values
                    el_lengths = np.sqrt(np.sum(np.diff(veh_data[obj.id][:, 0:2], axis=0) ** 2, axis=1))
                    veh_data[obj.id][:-1, 5] = tph.calc_ax_profile.calc_ax_profile(vx_profile=veh_data[obj.id][:, 4],
                                                                                   el_lengths=el_lengths)

        # -- calculate safety ground truth --
        # get safety rating
        t_safe = np.arange(0, list(veh_data.values())[0].shape[0]) * self.__config.getfloat('SAFETY', 'ssm_increment')
        safety_dynamic, safety_static = self.get_safety(time_stamps=t_safe)

        # modify safety score in order to represent trajectory planning horizon
        t_plan = self.__config.getfloat('FILE_EXPORT', 'export_time_traj_horizon')

        # loop backwards, holding "False" and "None" for duration of planning horizon (False dominates None)
        t_false = np.inf
        t_none = np.inf
        for i in reversed(range(len(t_safe))):
            if safety_static[i] is not None and not safety_static[i]:
                # if contains False value, store time-stamp where this segment is not part of trajectory anymore
                t_false = t_safe[i] - t_plan

            elif safety_static[i] is None:
                # if None, store last None value
                t_none = t_safe[i] - t_plan

            if t_safe[i] >= t_false:
                safety_static[i] = False

            elif t_safe[i] >= t_none:
                safety_static[i] = None

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

        if file_path is None:
            if self.__load_file is not None:
                file_path = filedialog.asksaveasfile(defaultextension=".saa",
                                                     initialdir=os.path.dirname(self.__load_file),
                                                     initialfile=os.path.splitext(
                                                         os.path.basename(self.__load_file))[0],
                                                     filetypes=(("Scenario-Architect Archive", "*.saa"),
                                                                ("All Files", "*.*")))
            else:
                file_path = filedialog.asksaveasfile(defaultextension=".saa",
                                                     filetypes=(("Scenario-Architect Archive", "*.saa"),
                                                                ("All Files", "*.*")))
            if file_path is None:
                raise Warning('File export was aborted!')
            else:
                file_path = file_path.name

        # -- init export-file --
        helper_funcs.src.data_export.init_exportfile(file_path=file_path.replace('.saa', '.scn'),
                                                     bound_l=self.__ent_cont[0].data_coord,
                                                     bound_r=self.__ent_cont[1].data_coord)

        # -- assemble export-file --
        n_traj_export = int(self.__config.getfloat('FILE_EXPORT', 'export_time_traj_horizon')
                            / self.__config.getfloat('FILE_EXPORT', 'export_time_increment'))
        for i in range(veh_data['veh_1'].shape[0] - n_traj_export):
            # extract ego trajectory
            ego_traj = veh_data['veh_1'][i:(i + n_traj_export), :]

            # generate object array (information about all objects in the scene)
            # each object holds: [id, [x, y, heading, vel, length, width]]
            obj_array = []
            for obj in veh_data.keys():
                if obj != 'veh_1' and veh_data[obj].shape[0] > i:
                    obj_array.append([obj, [veh_data[obj][i, 0],
                                            veh_data[obj][i, 1],
                                            veh_data[obj][i, 2],
                                            veh_data[obj][i, 4],
                                            self.__config.getfloat('VEHICLE', 'veh_length'),
                                            self.__config.getfloat('VEHICLE', 'veh_width')]])

            # calculate (simple) emergency trajectory
            ego_traj_em = helper_funcs.src.calc_brake_emergency.calc_brake_emergency(traj=ego_traj,
                                                                                     ggv=self.__ggv)

            # check for safe end state (if activated in config)
            if self.__config.getboolean('SAFETY', 'check_safe_end_state') and ego_traj_em[-1, 4] > 1.0:
                safety_static[i] = False

            # write to file
            helper_funcs.src.data_export.write_timestamp(file_path=file_path.replace('.saa', '.scn'),
                                                         time=i * self.__config.getfloat('FILE_EXPORT',
                                                                                         'export_time_increment'),
                                                         pos=veh_data['veh_1'][i, 0:2],
                                                         heading=veh_data['veh_1'][i, 2],
                                                         curv=veh_data['veh_1'][i, 3],
                                                         vel=veh_data['veh_1'][i, 4],
                                                         acc=veh_data['veh_1'][i, 5],
                                                         ego_traj=ego_traj,
                                                         ego_traj_em=ego_traj_em,
                                                         object_array=obj_array,
                                                         safety_dyn=safety_dynamic[i],
                                                         safety_stat=safety_static[i])

        # -- store pickle file in order to load the scenario for further edits --
        self.save_pckl(file_path=file_path.replace('.saa', '.sas'))

        # -- store vehicle parameters --
        np.savetxt(fname=file_path.replace('.saa', '_ggv.csv'),
                   X=self.__ggv,
                   delimiter=', ',
                   fmt='%.1f',
                   header="v_mps, ax_max_mps2, ay_max_mps2")
        np.savetxt(fname=file_path.replace('.saa', '_ax_max_machines.csv'),
                   X=self.__ax_max_machines,
                   delimiter=', ',
                   fmt='%.1f',
                   header="v_mps, ax_max_machines_mps2")

        # -- zip all files to one container --
        with zipfile.ZipFile(file_path, 'w') as zipObj:
            # Add multiple files to the zip
            zipObj.write(file_path.replace('.saa', '.scn'),
                         os.path.basename(file_path.replace('.saa', '.scn')))
            zipObj.write(file_path.replace('.saa', '.sas'),
                         os.path.basename(file_path.replace('.saa', '.sas')))
            zipObj.write(file_path.replace('.saa', '_ggv.csv'),
                         os.path.basename(file_path.replace('.saa', '_ggv.csv')))
            zipObj.write(file_path.replace('.saa', '_ax_max_machines.csv'),
                         os.path.basename(file_path.replace('.saa', '_ax_max_machines.csv')))

        # -- cleanup remaining (source) files --
        os.remove(file_path.replace('.saa', '.scn'))
        os.remove(file_path.replace('.saa', '.sas'))
        os.remove(file_path.replace('.saa', '_ggv.csv'))
        os.remove(file_path.replace('.saa', '_ax_max_machines.csv'))

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
            file_path = filedialog.askopenfilename(defaultextension=".saa",
                                                   filetypes=(("Scenario-Architect Archive", "*.saa"),
                                                              ("Scenario-Architect Source", "*.sas"),
                                                              ("All Files", "*.*")))

        if file_path != '':
            if not (".saa" in file_path or ".sas" in file_path):
                simpledialog.messagebox.showinfo(title="Unsupported file type",
                                                 message="Please make sure to provide a Scenario-Architect archive "
                                                         "('*.saa') or a Scenario-Architect source file ('*.sas').")

            # store path (in case of saving, the same path is suggested)
            self.__load_file = file_path

            # load pickle
            if ".saa" in file_path:
                # from zip file -> open file with .sas extension
                with zipfile.ZipFile(file_path) as zipObj:
                    tmp_file = next((x for x in zipObj.namelist() if '.sas' in x), None)

                    if tmp_file is not None:
                        with zipObj.open(tmp_file) as f:
                            imp_container = pickle.load(f)
                    else:
                        simpledialog.messagebox.showinfo(title="Unsupported file type",
                                                         message="Could not find a '*.sas' file in the "
                                                                 "Scenario-Architect archive ('*.saa')!")
                        return
            else:
                # directly from file
                with open(file_path, 'rb') as f:
                    imp_container = pickle.load(f)

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
                                                       ssm_t_axis=self.__axes_ssm,
                                                       color_str=self.get_veh_color(int(key[-1])),
                                                       veh_width=self.__config.getfloat('VEHICLE', 'veh_width'),
                                                       veh_length=self.__config.getfloat('VEHICLE', 'veh_length')))

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
                if obj.TYPE_VEHICLE and obj.data_vel_coord is not None:
                    self.__vel_mod_protect[obj.id] = True

            self.update_plot(force_update=True)

            # load veh-params
            if ".saa" in file_path:
                # load vehicle parameters
                ggv, ax_max_mach = scenario_testing_tools.get_scene_veh_param.get_scene_veh_param(file_path=file_path)

                # compare to parameters loaded on init
                matching = np.array_equal(ggv, self.__ggv) and np.array_equal(ax_max_mach, self.__ax_max_machines)

                # if differing, ask user which ones to use
                if not matching:
                    answer = messagebox.askyesno("Load vehicle parameters",
                                                 "The loaded Scenario-Architect archive holds stored vehicle parameters"
                                                 " (ggv and machine limits) differing from the local ones (in the "
                                                 "repository files [/params/veh_dyn_info] of the Scenario GUI).\n\n"
                                                 "Do you want to use the loaded parameters instead of the local ones?")

                    if answer:
                        self.__ggv = ggv
                        self.__ax_max_machines = ax_max_mach

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
                                               filetypes=(("CSV-File", "*.csv*"), ("All Files", "*.*")), )

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
            if self.__import_track_shift:
                x_shift = min(min(bound_l[:, 0]), min(bound_r[:, 0])) - 1.0
                y_shift = min(min(bound_l[:, 1]), min(bound_r[:, 1])) - 1.0
            else:
                x_shift = 0.0
                y_shift = 0.0

            # sort imported data into class variables
            self.__ent_cont[0].data_coord = bound_l - np.array([x_shift, y_shift])
            self.__ent_cont[1].data_coord = bound_r - np.array([x_shift, y_shift])

            if ego_path is not None:
                self.__ent_cont[2].data_coord = ego_path - np.array([x_shift, y_shift])

            # update temp-data with stored data
            for obj in self.__ent_cont:
                obj.data_coord_tmp = obj.data_coord

            self.update_plot(force_update=True)

            # adapt axis limits
            range_x = [min(min(self.__ent_cont[0].data_coord[:, 0]), min(self.__ent_cont[1].data_coord[:, 0])) - 10.0,
                       max(max(self.__ent_cont[0].data_coord[:, 0]), max(self.__ent_cont[1].data_coord[:, 0])) + 10.0]
            range_y = [min(min(self.__ent_cont[0].data_coord[:, 1]), min(self.__ent_cont[1].data_coord[:, 1])) - 10.0,
                       max(max(self.__ent_cont[0].data_coord[:, 1]), max(self.__ent_cont[1].data_coord[:, 1])) + 10.0]

            max_range = max(abs(range_x[1] - range_x[0]), abs(range_y[1] - range_y[0]))

            self.__axes_plane.set_xlim(np.average(range_x) - max_range / 2, np.average(range_x) + max_range / 2)
            self.__axes_plane.set_ylim(np.average(range_y) - max_range / 2, np.average(range_y) + max_range / 2)

            # hide scaling slider
            self.__sld_x_axis.disconnect(self.__sld_x_axis_cid)
            self.__sld_x_axis.ax.set_visible(False)
            self.__button_reset.disconnect(self.__button_reset_cid)
            self.__button_reset.ax.set_visible(False)

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
                # select "none" entity
                self.__radio.set_active(0)
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
        if event.inaxes == self.__axes_v_t or event.inaxes == self.__axes_a_t or event.inaxes == self.__axes_ssm:
            # update cursor in velocity window
            self.__vel_ax_cursor.set_data([event.xdata, event.xdata], self.__axes_v_t.get_ylim())
            self.__acc_ax_cursor.set_data([event.xdata, event.xdata], self.__axes_a_t.get_ylim())
            self.__ttc_ax_cursor.set_data([event.xdata, event.xdata], self.__axes_ssm.get_ylim())
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
