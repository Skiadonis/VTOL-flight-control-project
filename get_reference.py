import numpy as np

def get_reference(t):
    """
    Generates a complete reference trajectory (Position -> Snap).
    Uses global constants T_START_TRAJ and T_MOVE_DURATION.
    """
    # Case 1: Before departure (Stationary)
    if t <= T_START_TRAJ:
        return POS_INIT, np.zeros(2), np.zeros(2), np.zeros(2), np.zeros(2)

    # Case 2: After arrival (Stationary)
    elif t >= T_END_TRAJ:
        return POS_TARGET, np.zeros(2), np.zeros(2), np.zeros(2), np.zeros(2)

    # Case 3: In motion (5th-order polynomial)
    else:
        # Normalized time tau between 0 and 1
        tau = (t - T_START_TRAJ) / T_MOVE_DURATION

        # Pre-calculation of tau powers
        tau2, tau3, tau4, tau5 = tau**2, tau**3, tau**4, tau**5

        # Interpolation polynomial s(tau): 10*t^3 - 15*t^4 + 6*t^5
        s = 10*tau3 - 15*tau4 + 6*tau5

        # Derivatives with respect to tau
        ds_tau = 30*tau2 - 60*tau3 + 30*tau4
        dds_tau = 60*tau - 180*tau2 + 120*tau3
        d3s_tau = 60 - 360*tau + 360*tau2
        d4s_tau = -360 + 720*tau

        # Movement amplitude
        delta_pos = POS_TARGET - POS_INIT

        # Time scaling (Chain Rule)
        yd   = POS_INIT + s * delta_pos
        dyd  = (ds_tau  / T_MOVE_DURATION)    * delta_pos
        ddyd = (dds_tau / T_MOVE_DURATION**2) * delta_pos
        d3yd = (d3s_tau / T_MOVE_DURATION**3) * delta_pos
        d4yd = (d4s_tau / T_MOVE_DURATION**4) * delta_pos

        return yd, dyd, ddyd, d3yd, d4yd