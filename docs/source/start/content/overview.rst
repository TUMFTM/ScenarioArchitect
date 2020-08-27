=============================
Repository Overview
=============================

The repository is composed of different components. Each of them is contained in a sub-folder.

+-------------------+--------------------------------------------------------------------------+
| Folder            | Description                                                              |
+===================+==========================================================================+
| helper_funcs      | This python module contains various helper functions used in several     |
|                   |                                                                          |
|                   | other functions, e.g. calculation of splines or velocity profiles.       |
+-------------------+--------------------------------------------------------------------------+
| params            | This folder holds parameters configuring the Scenario Architect (e.g.    |
|                   |                                                                          |
|                   | export time increments, color styles, vehicle parameters for the         |
|                   |                                                                          |
|                   | automatic velocity-profile initialization)                               |
+-------------------+--------------------------------------------------------------------------+
| interface_package | This folder holds some python script snippets, which come in handy, when |
|                   |                                                                          |
|                   | working with the generated scenarios (e.g. extracting a specific time    |
|                   |                                                                          |
|                   | instance from the file).                                                 |
+-------------------+--------------------------------------------------------------------------+
| sample_files      | This folder holds a couple of sample scenarios generated with the        |
|                   |                                                                          |
|                   | Scenario Architect, as well as an sample track-file that can be imported |
|                   |                                                                          |
|                   | with the Scenario Architect.                                             |
+-------------------+--------------------------------------------------------------------------+


In the root folder is the `ScenarioGUI.py`-file located. This python class holds the main graphical user interface (GUI)
used for the creation and modification of scenarios.
