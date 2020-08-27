# List of components
* `generate_refline`: Generate a reference-line with matching normal-vectors and track-widths based on two bounds.
* `get_scene_ego_traj`: Extract the whole course of the ego-vehicle within an scenario.
* `get_scene_timesample`: Retrieve a time sample from a given (custom) csv-flavoured scenario file.
* `get_scene_track`: Retrieve a geometric or occupancy grid map from a given (custom) csv-flavoured scenario file.
* `get_scene_veh_param`: Retrieve vehicle specific parameters stored with the scenario (ggv and max. acc. of machines).
* `interp_heading`: Interpolate heading between given values, takes care of jumps near the switching point (-pi to pi).

Contact person: [Tim Stahl](mailto:stahl@ftm.mw.tum.de).