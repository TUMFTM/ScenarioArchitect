=========================================
Using Exported Scenarios in Your Project
=========================================

The generated '\*.scn'-files in the Scenario Architect archive '\*.saa' hold a CSV-flavoured listing of all relevant
variables. The first two lines specify the surrounding boundaries of the track. The third line holds the headers for
each data column (each separated with a semicolon). All subsequent lines hold each a snapshot of the scene at a certain
time instance.

Retrieving data for a specific timestamp
========================================
An exemplary function to extract a certain time-stamp from any '\*.saa' or '\*.scn' file can be found in the
'interface_package/scenario_testing_tools'-folder.
The function '`get_scene_timestample()`' ('get_scene_timestample.py') takes as input the path to a '\*.saa' or '\*.scn'
file, paired with a timestamp and returns all relevant variables at the given timestamp. The function is set up in a
way, that the given timestamp can be given in form of an integer as well as a floating point value. When an integer is
given, the respective data recording in the file is returned. When a floating point value is given, the values at the
time stamp (usually measured in seconds relative to the start of the scenario) is returned. When the given time instance
is not present in the file directly, linear interpolation is used to generate the values between the neighboring
occurrences.

Retrieving track / map information
==================================
A sample function to extract the track bounds from any '\*.saa' or '\*.scn' file is given with the function
'`get_scene_track()`' in the 'interface_package/scenario_testing_tools/get_scene_track.py'-file (same folder as above).
It takes the path to a '\*.saa' or '\*.scn' file as input and returns two numpy arrays, holding the
coordinates of the left and right bound, respectively. The file also offers a second function '`get_scene_occupancy()`',
which generates a basic occupancy grid with parameterizable resolution and framing. For further details, refer to the
function's header documentation.

A function to generate a reference-line, matching normal-vectors and track-widths is given with the function
'`generate_refline()`' ('interface_package/scenario_testing_tools/generate_refline.py').
The function takes the coordinate-arrays of the left and right bound (plus an option resolution for the resulting
reference line).

Retrieving vehicle dynamic parameters
=====================================
A function to extract the vehicle dynamic parameters (ggv and machine limitations) is given with the function
'`get_scene_veh_param()`' in the 'interface_package/scenario_testing_tools/get_scene_veh_param.py'-file (same folder
as above). It takes the path to a '\*.saa' file as input and returns two numpy arrays, holding the
ggv-diagram and maximum machine accelerations (details about the files in the :doc:`/../using_the_GUI/content/config`).

Implementation guide
====================
In order to use these functions (functions within the 'interface_package/scenario_testing_tools'-folder) in any other
repository, simply install the package with the command:

.. code-block:: bash

    pip3 install scenario-testing-tools

In order to use the function, import the package and use the function as usual, e.g.:

.. code-block:: python

    import scenario_testing_tools as stt


In order to extract a certain timestamp, insert the following line (replace input parameters):

.. code-block:: python

    time, pos, heading, curv, vel, acc, ego_traj, object_array, time_f = \
        stt.get_scene_timesample.get_scene_timesample(file_path=YOUR_PATH, t_in=1.234)

In order to extract a certain timestamp with its expected safety rating [True, False, None], insert the following line
(replace input parameters):

.. code-block:: python

    time, pos, heading, curv, vel, acc, ego_traj, ego_traj_em, object_list, time_f, safety_dyn, safety_stat = \
        stt.get_scene_timesample.get_scene_timesample(file_path=YOUR_PATH, t_in=1.234, append_safety=True)


.. hint:: In order to emulate real-time playback on a vehcile, keep track of the passed time and call the function above
    iteratively, e.g.:

    .. code-block:: python

        import time
        tic = time.time()

        while True:
            return_tuple = stt.get_scene_timesample.get_scene_timesample(file_path=YOUR_PATH,
                                                                         t_in=time.time() - tic)


In order to extract the bound-arrays use the following function:

.. code-block:: python

    bound_l, bound_r = stt.get_scene_track.get_scene_track(file_path=YOUR_PATH)


The bound-arrays can be used to retrieve an occupancy grid and / or a reference line with matching normal-vectors (e.g.
useful for lane-based coordinate systems):

.. code-block:: python

    occ_grid, origin = stt.get_scene_track.get_scene_occupancy(bound_l=bound_l, bound_r=bound_r)

    ref_line, normal_vectots, tw_left, tw_right = \
        stt.generate_refline.generate_refline(bound_l=<TODO>, bound_r=<TODO>)

.. hint:: All functions in this package are designed in a way to cope with Scenario Architect archives ('\*.saa') as
    well as with scenario files ('\*.scn').
