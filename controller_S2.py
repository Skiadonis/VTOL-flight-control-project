def controller_S2(t, X):
    """
    Dynamic Extension Controller (Flatness / Feedback Linearization).
    Stabilizes the Center of Oscillation (CO) output.
    """
    # 1. State extraction (8 states: 6 physical + 2 extended)
    x, dx, z, dz, theta, dtheta = X[0:6]
    xi1, xi2 = X[6], X[7] 

    # Corrected thrust relation: u1 = xi1 + epsilon * dtheta^2
    u1_reel = xi1 + EPSILON * dtheta**2

    # 2. Reference (Desired trajectory up to 4th derivative)
    yd, dyd, ddyd, d3yd, d4yd = get_reference(t)
    s, c = np.sin(theta), np.cos(theta)

    # 3. Kinematics of the Center of Oscillation (CO)
    y_co = np.array([x - EPSILON * s, z + EPSILON * c])
    dy_co = np.array([dx - EPSILON * c * dtheta, dz - EPSILON * s * dtheta])
    
    # Acceleration and Jerk of CO (derived from theory)
    ddy_co = np.array([-s * xi1, c * xi1 - G])
    d3y_co = np.array([-c * dtheta * xi1 - s * xi2, -s * dtheta * xi1 + c * xi2])

    # 4. Pole Placement (Order 4: (s + lambda)^4)
    L = 3.0  # Lambda
    K0, K1, K2, K3 = L**4, 4*L**3, 6*L**2, 4*L

    # Errors
    e = y_co - yd
    de = dy_co - dyd
    dde = ddy_co - ddyd
    d3e = d3y_co - d3yd

    # Virtual command W (Desired Snap)
    W = d4yd - K3*d3e - K2*dde - K1*de - K0*e

    # 5. Model Inversion: Y^(4) = A_exact + B_dyn * [v1, u2]^T = W
    # B_dyn matrix (simplified from preparatory work)
    B_dyn = np.array([[-s, -EPSILON * c * xi1],
                      [ c, -EPSILON * s * xi1]])
    
    # A_exact vector (drift terms)
    A_exact = np.array([c * dtheta**2 * xi1 - 2 * c * dtheta * xi2,
                        s * dtheta**2 * xi1 - 2 * s * dtheta * xi2])

    try:
        # Solving for [v1, u2]
        cmd = np.linalg.solve(B_dyn, W - A_exact)
    except np.linalg.LinAlgError:
        cmd = np.zeros(2) # Anti-singularity protection

    v1, u2_reel = cmd[0], cmd[1]

    # Return: [u1, u2, d(xi1)/dt, d(xi2)/dt]
    return [u1_reel, u2_reel, xi2, v1]