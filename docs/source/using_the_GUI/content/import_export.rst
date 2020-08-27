===================================
Import and Export
===================================

Export a scenario
====================================
The drawn scenario can be exported using the button `Export Scen.`. This process will generate a Scenario Architect
archive file at the specified location. The generated file is a zip-like structure that holds the following data-files:

* Scenario dataset ('\*.scn'): csv-flavoured file, holding all relevant data to be processed by any other software.
* Scenario pickle ('\*.sas'): Pickle file, which can be directly imported by the GUI in order to modify any entity.
* Vehicle parameters ('\*_ggv.csv' and '\*_ax_max_machines.csv') holding associated vehicle parameters (when exporting,
  the currently loaded (from another '\*.saa'-file) or in the 'params'-folder defined parameters are stored)

.. hint::
    In order to export a scenario, at least the following entities have to be defined:

    * Bounds (bound_l, bound_r)
    * Trajectory of ego-vehicle (veh_1)

.. hint::
    When generating a scenario, the ego vehicles trajectory (veh_1) is the characteristic one for the data export.
    Please also consider, that the ego vehicle holds a planned trajectory, which is exported to the scenario file.
    Therefore, the  actually driven trajectory ends earlier (according to the specified trajectory preview horizon).

    Other vehicles, holding a (temporally) longer trajectory compared to the ego vehicle, are cut at the position of the
    last ego-vehicle's data point.

.. hint::
    It is possible to unzip the Scenario Architect archive ('\*.saa') to obtain the individual files listed above.
    However, we recommend using the zipped structure in order to keep all files at hand.


Import a scenario
====================================
Use the button `Import Scen.` in order to import a Scenario Architect archive ('\*.saa') or stored scene
pickle ('\*.sas').

Exemplary scenarios generated with the Scenario Architect can be found in the folder 'sample_files/scenario_1_vehicle'
for the ego vehicle only and 'sample_files/scenario_n_vehicle' for multiple vehicles.

.. hint::
    If a Scenario Architect archive ('\*.saa') with stored vehicle parameters is loaded, the parameters are compared to
    the local parameters (in the 'params'-folder). When the parameter sets differ, the user is asked to select which of
    the two sets should be used.



Importing a track
====================================
The button `Import Track` allows to import stored track information (bounds). This function
supports appropriate '\*.csv' files (delimiter ';' or ',' - detected automatically). When importing a file, the user can
specify the range of s-coordinates (counted from the first data point in the file) to be imported. If the start
s-coordinate is larger than the end s-coordinate, the area crossing the start / end of the file is imported (in this
case a closed track is assumed).

The headers in the first line of the csv-file (beginning with a '#') must at least hold one of the following list of
entries:

* ``x_m;y_m;w_tr_right_m;w_tr_left_m`` (track-bounds will be imported)
* ``x_ref_m;y_ref_m;width_right_m;width_left_m;x_normvec_m;y_normvec_m;alpha_mincurv_m;s_raceline_m`` (track-bounds and
  ego-path will be imported)

A sample track of the first type is provided in the folder 'sample_files/sample_track'. Further supported tracks -
extracted from real racetracks - can be found `here <http://github.com/TUMFTM/racetrack-database>`_ in the folder
'tracks'.
