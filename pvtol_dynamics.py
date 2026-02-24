def pvtol_dynamics(t, X, controller_func):
    """
    Real PVTOL Dynamics.
    X: State [x, dx, z, dz, theta, dtheta, (xi1, xi2...)]
    """
    # 1. Physical state extraction
    x, dx, z, dz, theta, dtheta = X[0:6]

    # 2. Controller call
    control_out = controller_func(t, X)
    u1_cmd, u2_cmd = control_out[0], control_out[1]

    # 3. Physical constraint: thrust must be positive (u1 >= 0)
    u1_eff = max(0.0, u1_cmd)

    # 4. Equations of motion (Normalized m=1, g=1)
    c, s = np.cos(theta), np.sin(theta)

    x_ddot = -(s * u1_eff) + (EPSILON * c * u2_cmd) / M
    z_ddot = (c * u1_eff) + (EPSILON * s * u2_cmd) / M - G
    theta_ddot = u2_cmd

    dX = [dx, x_ddot, dz, z_ddot, dtheta, theta_ddot]

    # Handle extended states for Controller S2
    if len(control_out) > 2:
        dX.extend(control_out[2:])

    return dX