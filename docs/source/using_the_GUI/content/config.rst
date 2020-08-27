===================================
Configuration
===================================

Vehicle dimensions, trajectory discretization and visualization cues (e.g. coloring) can be altered in the config file
'params/config.ini'. The config-file holds a detailed explanation for each of the parameters.

The dynamical behavior of the vehicle for the initially generated velocity profile can be adjusted with the files in the
'params/veh_dyn_info'-folder.

* The 'ax_max_machines.csv'-file describes the acceleration resources of the motor at certain velocity thresholds
  (values in between are interpolated linearly). The first column holds velocity values and the second column the
  possible acceleration at that velocity.
* The 'ggv.csv'-file describes the available friction based longitudinal and lateral acceleration at certain velocity
  thresholds (values in between are interpolated linearly). The first column holds velocity values, the second and third
  column the maximal longitudinal and lateral acceleration at that velocity.
