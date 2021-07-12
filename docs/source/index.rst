.. Agent Simulation documentation master file, created by
   sphinx-quickstart on Thu May 28 14:28:26 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Scenario Architect Documentation
==========================================================================

.. image:: /figures/Overview.png
  :width: 800
  :alt: Scenario Architect Overview Image


The Scenario Architect is a basic python tool to generate, import and export short concrete scenario intervals.
These scenarios can then be used for safety assessment metric evaluations or other benchmarks.


.. toctree::
   :maxdepth: 2
   :caption: Getting Started:

   start/overview.rst
   start/installation.rst
   start/launching.rst

.. toctree::
   :maxdepth: 2
   :caption: Using the GUI:

   using_the_GUI/basics.rst
   using_the_GUI/import_export.rst
   using_the_GUI/workspace.rst
   using_the_GUI/temporal.rst
   using_the_GUI/safety.rst
   using_the_GUI/config.rst

.. toctree::
   :maxdepth: 2
   :caption: Post Creation Usage:

   post_creation_usage/main.rst

People Involved
================

Core Developer
--------------

- `Tim Stahl <tim.stahl@tum.de>`_


Acknowledgements
----------------
Regina Harrer contributed during her Project Thesis to the calculation of the Time to Collision (TTC) score.


Contributions
=============================================================

[1] T. Stahl and J. Betz, “An Open-Source Scenario Architect for Autonomous Vehicles,”
in 2020 Fifteenth International Conference on Ecological Vehicles and Renewable Energies (EVER), 2020.
`(view pre-print) <https://arxiv.org/pdf/2006.09731>`_


If you find our work useful in your research, please consider citing:

.. code-block:: latex

   @inproceedings{stahl2020a,
     title = {An Open-Source Scenario Architect for Autonomous Vehicles},
     booktitle = {2020 Fifteenth International Conference on Ecological Vehicles and Renewable Energies (EVER)},
     author = {Stahl, Tim and Betz, Johannes},
     year = {2020}
   }
