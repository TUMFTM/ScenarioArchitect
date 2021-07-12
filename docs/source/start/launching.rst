===================================
Launching the Software
===================================


Launching the Scenario Architect GUI
====================================
In order to launch the main graphical editor, launch the `ScenarioGUI.py` script, located in the root-folder:

.. code-block:: bash

    python3 ScenarioGUI.py



(Optional) Open a scenario file via command line
================================================
In order to launch the main graphical editor with a certain scenario file pre-loaded, launch the `ScenarioGUI.py`
script appended by the path to the Scenario Architect archive file ('\*.saa'):

.. code-block:: bash

    python3 ScenarioGUI.py "c:\path\to\your\scenario-file.saa"

.. hint:: Replace the exemplary path by the absolute path to your Scenario Architect archive file.



(Optional) Associate scenario files with the Scenario Architect
===============================================================
On Windows machines, you can associate the Scenario Architect archive files ('\*.saa') with the Scenario Architect GUI.
That way, a double left-click with the pointing device on any stored scenario will open the scenario in the Scenario
Architect.

In order to do so, create a new batch-file ('\*.bat') at your desired location on your machine. The file should host the
following content:

.. code-block:: bash
    :linenos:

    @ECHO OFF
    SET "log_path=%*"
    ECHO %log_path:\=/%
    "C:\Python\Python37\python.exe" C:\scenario-architect\ScenarioGUI.py %log_path:\=/%
    pause

.. hint::
    * Replace the first path in line 4 with the absolute path to your python installation.
    * Replace the second path in line 4 with the absolute path to the 'ScenarioGUI.py'-file on your machine.

Afterwards, double left-click any scenario-file ('\*.saa') with your mouse and select the created bash-file as default
application to open this type of files.



Common problems
=============================
When launching directly from the PyCharm IDE make sure to **disable** 'Python Scientific'. Otherwise, the GUI will be
displayed in form of a static image, not allowing for interaction.
