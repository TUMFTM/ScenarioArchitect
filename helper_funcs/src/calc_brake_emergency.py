import numpy as np
import trajectory_planning_helpers as tph

"""
Python version: 3.7
Created by: Tim Stahl
Created on: 27.04.2019

Documentation:  Function used to calculate a simple emergency trajectory based on a regular trajectory.
"""

# vehicle parameters
VEH_MASS = 1160.0
VEH_DRAGCOEFF = 0.854


def calc_brake_emergency(traj: np.ndarray,
                         ggv: np.ndarray) -> np.ndarray:
    """
    Calculates a simple emergency profile (brake to stop) for a given regular path.

    :param traj:       trajectory with the columns (x, y, heading, curv, vel, acc)
    :param ggv:        ggv-diagram to be applied: [vx, ax_max, ay_max]. Velocity in m/s, accelerations in m/s2.
    :returns traj_em:  emergency trajectory with columns (x, y, heading, curv, vel, acc)
    """

    # calculate element length
    el_lengths = np.sqrt(np.power(np.diff(traj[:, 0]), 2) + np.power(np.diff(traj[:, 1]), 2))

    # calculate brake-vel-profile
    v_brake = tph.calc_vel_profile_brake.calc_vel_profile_brake(kappa=traj[:, 3],
                                                                el_lengths=el_lengths,
                                                                v_start=traj[0, 4],
                                                                drag_coeff=VEH_DRAGCOEFF,
                                                                m_veh=VEH_MASS,
                                                                ggv=ggv)

    # calculate matching acceleration profile
    idx_em = len(v_brake)
    a_brake = tph.calc_ax_profile.calc_ax_profile(vx_profile=v_brake,
                                                  el_lengths=el_lengths[:idx_em],
                                                  eq_length_output=True)

    # assemble emergency trajectory
    traj_em = np.column_stack((traj[:idx_em, 0:4], v_brake, a_brake))

    return traj_em


def calc_brake_dist(traj: np.ndarray,
                    ggv: np.ndarray) -> (float, None):
    """
    Calculates the braking distance based on a simple emergency profile (brake to stop) for a given regular path.

    :param traj:       trajectory with the columns (x, y, heading, curv, vel, acc)
    :param ggv:        ggv-diagram to be applied: [vx, ax_max, ay_max]. Velocity in m/s, accelerations in m/s2.
    :returns em_dist:  emergency brake dist in m, "None" if velocity did not reach zero
    """

    # calculate element length
    el_lengths = np.sqrt(np.power(np.diff(traj[:, 0]), 2) + np.power(np.diff(traj[:, 1]), 2))

    # calculate brake-vel-profile
    v_brake = tph.calc_vel_profile_brake.calc_vel_profile_brake(kappa=traj[:, 3],
                                                                el_lengths=el_lengths,
                                                                v_start=traj[0, 4],
                                                                drag_coeff=VEH_DRAGCOEFF,
                                                                m_veh=VEH_MASS,
                                                                ggv=ggv)

    if v_brake[-1] > 0.3:
        em_dist = None

    else:
        # calculate brake dist
        idx_em = np.argmax(abs(v_brake) < 0.3)
        em_dist = np.sum(el_lengths[:idx_em])

    return em_dist
