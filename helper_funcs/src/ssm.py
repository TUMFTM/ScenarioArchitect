import numpy as np
import math
import trajectory_planning_helpers as tph
import scenario_testing_tools

"""
Python version: 3.7
Created by: Tim Stahl & Regina Harrer
Created on: 27.08.2020

Calculation of surrogate safety metrics between the ego-Vehicle and another vehicle
"""


def distance(pos_1,
             pos_2) -> float:
    # calculate distance between two poses
    length = np.sqrt(pow((pos_1[0] - pos_2[0]), 2) + pow((pos_1[1] - pos_2[1]), 2))
    return length


def calc_center_line(bound_l: np.ndarray,
                     bound_r: np.ndarray) -> tuple:
    """
    Calculates the center-line as well as the s-coordinate along this line for a pair of given bounds. The bounds must
    hold the same amount of elements.

    :param bound_l:             bound of the left track
    :param bound_r:             bound of the right track
    :returns:
        * **center_line** -     the cartesian coordinates of the center line
        * **s** -               the cumulative distance of every points after interpolation (s-coordinate)

    """

    # calculate center line
    center_line = 0.5 * (bound_r + bound_l)

    # Calculating the Frenet s coordinate (along the track):
    # 1. calculate the distance between the points on the central line one by one
    # 2. Add the distance up
    s = np.cumsum(np.sqrt(np.sum(np.power(np.diff(center_line, axis=0), 2), axis=1)))
    s = np.insert(s, 0, 0.0)

    return center_line, s


def angle3pt(a: tuple, b: tuple, c: tuple) -> float:
    """
    Calculate the angle by turning from coordinate a to c around b.

    :param a:             coordinate a (x, y)
    :param b:             coordinate b (x, y)
    :param c:             coordinate c (x, y)
    :returns:
        * **ang** -       angle between a and c

    """

    ang = math.atan2(c[1] - b[1], c[0] - b[0]) - math.atan2(a[1] - b[1], a[0] - b[0])

    # if angle is bigger then 3,14 the angle is different calculated (assume it is in the other direction)
    if ang > math.pi:
        ang -= 2 * math.pi
    elif ang <= -math.pi:
        ang += 2 * math.pi

    return ang


def global_2_lane_based(pos: np.ndarray,
                        center_line: np.ndarray,
                        s_course: np.ndarray,
                        closed: bool = False) -> tuple:
    """
    This function transforms a global coordinate into the lane_based coordinate system by the following steps:

    #. find the intersection point of the vertical line of the input point and the center-line (global coordinate)
    #. calculate the distance between the intersection point and the input point ==> new x coordinate
    #. use the vector pointing from the intersection and the vector in the direction of the curve (from the intersection
       point to the next point on the central curve) to decide whether the point is on the right or left hand side
    #. correspondingly assign the proper sing (+/-) to the new x coordinate
    #. return the new coordinate in the lane_based coordinate system

    :param pos:                  the input position/coordinate(e.g the pos of ego vehicle or the objective vehicle)
    :param center_line:          the coordinate of the center_line line in global coordinate
    :param s_course:             the cumulative distance along the center_line line
    :param closed:               is the race course a closed or an open circuit. Default closed circuit
    :returns:
        * **s** -                the s coordinate in the lane_based system (longitudinal)
        * **n** -                the n coordinate in the lane_based system (lateral)
        * **angle_head_track** - the angle of the heading_track

    """
    # calculate squared distances between path array and reference poses
    distances2 = np.power(center_line[:, 0] - pos[0], 2) + np.power(center_line[:, 1] - pos[1], 2)

    # get smallest index (argmin: returns the indices of the minimum values along an axis)
    idx_nb = np.argmin(distances2)

    if closed:
        idx1 = idx_nb - 1
        idx2 = idx_nb + 1
        idx3 = idx_nb + 2

        if idx2 > (center_line.shape[0] - 1):
            idx2 = 0
            idx3 = 1
        if idx3 > (center_line.shape[0] - 1):
            idx3 = 0
    else:
        idx1 = max(idx_nb - 1, 0)
        idx2 = min(idx_nb + 1, np.size(center_line, axis=0) - 1)
        idx3 = min(idx_nb + 2, np.size(center_line, axis=0) - 2)

    # get angle between input point, closest point and next point on center line
    ang1 = abs(angle3pt(tuple(pos), center_line[idx_nb, :], center_line[idx2, :]))

    last_point = False

    # Extract neighboring points (A and B) for all points expect the last
    # if the angle between the input point, closest point and next point is smaller than 90°
    if ang1 <= math.pi / 2 and center_line[idx_nb, 0] != center_line[idx2, 0] and \
            center_line[idx_nb, 1] != center_line[idx2, 1]:
        a_pos = center_line[idx_nb, :]
        b_pos = center_line[idx2, :]
        c_pos = center_line[idx3, :]

    # if the angle between the input point, closest point and next point is bigger than 90°
    elif ang1 > math.pi / 2 and center_line[idx_nb, 0] != center_line[idx2, 0] and \
            center_line[idx_nb, 1] != center_line[idx2, 1]:
        a_pos = center_line[idx1, :]
        b_pos = center_line[idx_nb, :]
        c_pos = center_line[idx2, :]
    # for the last points (half of the distance between the last centerline point and the penultimate cerntierline
    # point)
    else:
        a_pos = center_line[idx1, :]
        b_pos = center_line[idx_nb, :]
        c_pos = center_line[idx2, :]
        last_point = True

    # calculate the heading of the track using the next center_line point (in 0 = north convention)
    heading_track = b_pos - a_pos
    angle_heading_track = tph.normalize_psi.normalize_psi(math.atan2(heading_track[1],
                                                                     heading_track[0]) - 1 / 2 * math.pi)

    # if not fist points
    if not (a_pos[0] == b_pos[0] and a_pos[1] == b_pos[1]):
        # get sign (left / right of center line) by calculating angle between point of interest (pos) and center line
        sign = -np.sign(angle3pt(b_pos, tuple(pos), a_pos))

        # get point (s_pos) perpendicular on the line between the two closest points
        # https://stackoverflow.com/questions/10301001/perpendicular-on-a-line-segment-from-a-given-point
        temp = (((pos[0] - a_pos[0]) * (b_pos[0] - a_pos[0]) + (pos[1] - a_pos[1]) * (b_pos[1] - a_pos[1]))
                / (np.power(b_pos[0] - a_pos[0], 2) + np.power(b_pos[1] - a_pos[1], 2)))
        s_pos = [a_pos[0] + temp * (b_pos[0] - a_pos[0]), a_pos[1] + temp * (b_pos[1] - a_pos[1])]

        # calculate distance between closest point on center line and s_pos
        ds = np.sqrt(np.power(a_pos[0] - s_pos[0], 2) + np.power(a_pos[1] - s_pos[1], 2))

        # calculate distance between point of interest (pos) and s_pos
        n = np.sqrt(np.power(pos[0] - s_pos[0], 2) + np.power(pos[1] - s_pos[1], 2)) * sign

        # Calculate length of line segment [a_pos, b_pos]
        len_ab = np.sqrt(np.power(b_pos[0] - a_pos[0], 2) + np.power(b_pos[1] - a_pos[1], 2))

        # calculation of the length offset
        if last_point is False:
            # Calculate length of segment [a_pos, b_pos] parallel to center line but moved by n
            len_ab_offset = len_ab - n * math.tan(angle3pt(a_pos, b_pos, c_pos))
        else:
            # Nutrition for the last points
            len_ab_offset = len_ab

        # Calculate reduced ds (ray set)
        ds_reduced = ds * len_ab / len_ab_offset

        # get total s_course
        if ang1 <= math.pi / 2 and center_line[idx_nb, 0] != center_line[idx2, 0] and \
                center_line[idx_nb, 1] != center_line[idx2, 1]:
            s = s_course[idx_nb] + ds_reduced
        else:
            s = s_course[idx1] + ds_reduced

    else:
        # get sign (left / right of center line) by calculating angle between point of interest (pos) and center line
        sign = -np.sign(angle3pt(c_pos, tuple(pos), b_pos))

        # TODO: currently assume s and n equal to zero -> interpolate negative numbers, if outside track
        s = 0.0
        n = np.sqrt(np.power(pos[0] - a_pos[0], 2) + np.power(pos[1] - a_pos[1], 2)) * sign

    return s, n, angle_heading_track


def timestamp_info(lane_based_poses: np.ndarray,
                   covered_distance: np.ndarray,
                   velocity: np.ndarray,
                   t_increment: float,
                   t_horizon: float) -> np.ndarray:
    """
    Calculation of position, heading and velocity for specific timestamps (used for calculation of
    the timestamps of the lane based system)

    :param lane_based_poses:    lane-based poses [s, n, heading]
    :param velocity:            velocity along heading
    :param covered_distance     s value for each position
    :param parm t_increment:    time-stamp to be returned (interpolated linearly between stored values)
    :param t_horizon:           maximum horizon the ttc is calculated for (i.e. limiting factor at very low speeds)
    :returns:
        * **vehicle_info** -    array with all values for the timestamps: time-stamp, s, n, heading, velocity

    """

    # initialise variables
    # variable to increment time-stamp
    t_in = 0.0

    # calculates the time needed to cover a distance at a certain speed
    t = abs(np.concatenate(([0], np.cumsum(np.divide(np.diff(covered_distance[:]), velocity[:-1],
                                                     out=np.full(velocity[:-1].shape[0], np.inf),
                                                     where=velocity[:-1] != 0)))))

    # initialise the storage to save the information (pos, heading, velocity, timestamp)
    vehicle_info = np.zeros((int(min(max(t), t_horizon) / t_increment + 1), 5))

    # repeats itself as long as the timestamp is smaller than max (t)
    for i in range(vehicle_info.shape[0]):
        # interpolation of the required information (lane_based_poses, heading, velocity, time)
        vehicle_info[i, 0] = t_in
        vehicle_info[i, 1:3] = [np.interp(t_in, t, lane_based_poses[:, 0]), np.interp(t_in, t, lane_based_poses[:, 1])]
        vehicle_info[i, 3] = scenario_testing_tools.interp_heading.interp_heading(heading=lane_based_poses[:, 2],
                                                                                  t_series=t, t_in=t_in)
        vehicle_info[i, 4] = np.interp(t_in, t, velocity[:])

        # increments the time
        t_in += t_increment

    return vehicle_info


def calc_ssm(pos_ego_stamps: np.ndarray,
             pos_vehicle_stamps: np.ndarray,
             veh_length: float = 3.0,
             reaction_time: float = 0.1,
             maximum_acc: float = 10.0,
             ego_stopping_dist: np.ndarray = None) -> tuple:
    """
    Calculate Surrogate Safety Metrics (SSM) for a given pair of trajectories (ego and object vehicle).

    :param pos_ego_stamps:       time, s, n, heading, velocity of the ego vehicle
    :param pos_vehicle_stamps:   time, s, n, heading, velocity of the other vehicles
    :param veh_length:           length of the vehicles
    :param reaction_time:        reaction time in seconds
    :param maximum_acc:          maximum (braking) acceleration assumed for both vehicles in m/s²
    :param ego_stopping_dist     (optional) ego stop dist array, columns [t, dist], (calc based on maximum_acc else)
    :returns:
        * **dss** -              Difference of Space distance and Stopping distance (DSS)

    """

    # init ttc and dss array
    ttc = [None] * pos_vehicle_stamps.shape[0]
    dss = [None] * pos_vehicle_stamps.shape[0]

    # check if vehicles drive in same direction
    driving_same_direction = ((pos_ego_stamps[0, 1] < pos_ego_stamps[-1, 1]
                               and pos_vehicle_stamps[0, 1] < pos_vehicle_stamps[-1, 1])
                              or (pos_ego_stamps[0, 1] > pos_ego_stamps[-1, 1]
                                  and pos_vehicle_stamps[0, 1] > pos_vehicle_stamps[-1, 1]))

    if driving_same_direction:

        # iterate over number of shared points
        ego_stop_dist_i = None
        for i in range(min(pos_vehicle_stamps.shape[0], pos_ego_stamps.shape[0])):
            if ego_stopping_dist is not None:
                # get closest stopping dist for given time
                ego_stop_dist_i = ego_stopping_dist[np.argmin(abs(ego_stopping_dist[:, 0] - pos_ego_stamps[i, 0])), 1]

            # check if object vehicle is in front of ego (REGular or REVerse direction)
            in_front_reg = (pos_ego_stamps[i, 1] < (pos_vehicle_stamps[i, 1] - veh_length)
                            and pos_ego_stamps[0, 1] < pos_ego_stamps[-1, 1])
            in_front_rev = (pos_ego_stamps[i, 1] > (pos_vehicle_stamps[i, 1] - veh_length)
                            and pos_ego_stamps[0, 1] > pos_ego_stamps[-1, 1])

            # object vehicle is in front
            if in_front_reg or in_front_rev:

                # distance between vehicles (bumper to bumper)
                delta_s = abs(pos_vehicle_stamps[i, 1] - pos_ego_stamps[i, 1]) - veh_length

                ttc[i] = time_to_collision(v_ego=pos_ego_stamps[i, 4],
                                           v_obj=pos_vehicle_stamps[i, 4],
                                           dist=delta_s)

                dss[i] = difference_space_stopping(v_ego=pos_ego_stamps[i, 4],
                                                   v_obj=pos_vehicle_stamps[i, 4],
                                                   dist=delta_s,
                                                   reaction_time=reaction_time,
                                                   maximum_acc=maximum_acc,
                                                   ego_stopping_dist=ego_stop_dist_i)

            else:
                # check if overlapping
                if abs(pos_ego_stamps[i, 1] - pos_vehicle_stamps[i, 1]) <= veh_length:
                    ttc[i] = 0.0
                    dss[i] = 0.0

    return ttc, dss


def time_to_collision(v_ego: float,
                      v_obj: float,
                      dist: float) -> float:
    """
    Calculation of the time to collision. The TTC is calculated for the situation, where the ego-vehicle is behind
    another vehicle.

    :param v_ego:       velocity of the ego-vehicle
    :param v_obj:       velocity of the object-vehicle
    :param dist:        distance between the two vehicles (bumper to bumper)
    :returns:
        * **ttc** -     time to collision

    """

    # if the ego vehicle is faster than the other
    if v_ego > v_obj:
        ttc = dist / (v_ego - v_obj)

    else:
        ttc = np.inf

    return ttc


def difference_space_stopping(v_ego: float,
                              v_obj: float,
                              dist: float,
                              reaction_time: float = 0.1,
                              maximum_acc: float = 10.0,
                              ego_stopping_dist: float = None) -> float:
    """
    Calculation of the Difference of Space distance and Stopping distance (DSS). This safety metric represents the
    remaining distance between two vehicles when assuming both vehicles immediately breaking hard.
    https://www.sciencedirect.com/science/article/pii/S0386111217300286 (table at end of paper)

    :param v_ego:               velocity of the ego-vehicle
    :param v_obj:               velocity of the object-vehicle
    :param dist:                distance between the two vehicles (bumper to bumper)
    :param reaction_time:       (optional) reaction time in seconds
    :param maximum_acc:         (optional) maximum (braking) acceleration assumed for both vehicles in m/s²
    :param ego_stopping_dist    (optional) custom stopping dist of ego vehicle (calculated based on maximum_acc else)
    :returns:
        * **dss** -             Difference of Space distance and Stopping distance (DSS)

    """

    if ego_stopping_dist is None:
        ego_stopping_dist = (v_ego ** 2) / (2 * maximum_acc)

    dss = ((v_obj ** 2) / (2 * maximum_acc) + dist) - (v_ego * reaction_time + ego_stopping_dist)

    return dss


def evaluation_ttc(ttc: float,
                   lane_pos_obj: tuple,
                   lane_pos_ego: tuple,
                   veh_width: float,
                   undefined_ttc: float = 1.7,
                   crit_ttc: float = 1.5) -> bool:
    """
    Determination of a safety flag depending on the situation at a given point in time. The safety flag is influenced
    by the TTC and the vehicle's lateral offset.

    Safety flag:
     - True:    TTC > crit_ttc (or TTC < 0 or None, i.e. ego is ahead)
     - False:   Collision or TTC < crit_ttc and vehicle overlap in lateral frame
     - None:    Else

    :param ttc:             time to collision [in s]
    :param lane_pos_obj:    lane-based Position for object vehicle (s, n)
    :param lane_pos_ego:    lane-based information for ego vehicle(s, n)
    :param veh_width:       the width of the vehicles [in m]
    :param undefined_ttc:   threshold of TTC being assumed undefined (any safety rating allowed) [in s]
    :param crit_ttc:        threshold of TTC being assumed critical [in s]
    :return:
        * **safety_flag** - boolean value or None (as stated in the description)

    """

    # init safety flag
    safety_flag = None

    if ttc > undefined_ttc or ttc < 0.0 or ttc is None:
        safety_flag = True

    elif 0.0 <= ttc <= crit_ttc and abs(lane_pos_obj[1] - lane_pos_ego[1]) <= veh_width:
        safety_flag = False

    return safety_flag


def evaluation_dss(dss: float,
                   lane_pos_obj: tuple,
                   lane_pos_ego: tuple,
                   veh_width: float,
                   undefined_dss: float = 15.0,
                   crit_dss: float = 0.0) -> bool:
    """
    Determination of a safety flag depending on the situation at a given point in time. The safety flag is influenced
    by the DSS and the vehicle's lateral offset.

    Safety flag:
     - True:    DSS > crit_ttc (or DSS is None, i.e. ego is ahead)
     - False:   Collision or DSS < crit_ttc and vehicle overlap in lateral frame
     - None:    Else

    :param dss:             Difference of Space distance and Stopping distance (DSS) [in m]
    :param lane_pos_obj:    lane-based Position for object vehicle (s, n)
    :param lane_pos_ego:    lane-based information for ego vehicle(s, n)
    :param veh_width:       the width of the vehicles [in m]
    :param undefined_dss:   threshold of DSS being assumed undefined (any safety rating allowed) [in m]
    :param crit_dss:        threshold of DSS being assumed critical [in m]
    :return:
        * **safety_flag** - boolean value or None (as stated in the description)

    """

    # init safety flag
    safety_flag = None

    if dss > undefined_dss or dss is None:
        safety_flag = True

    elif dss <= crit_dss and abs(lane_pos_obj[1] - lane_pos_ego[1]) <= veh_width:
        safety_flag = False

    return safety_flag
