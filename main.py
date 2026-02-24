import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

# Import your modules
from get_reference import get_reference
from pvtol_dynamics import pvtol_dynamics
from controller_S2 import controller_S2

# Global Simulation Constants
EPSILON = 0.1  # Coupling parameter
M = 1.0        # Normalized mass
G = 1.0        # Normalized gravity
T_START_TRAJ = 1.0
T_MOVE_DURATION = 8.0
T_END_TRAJ = T_START_TRAJ + T_MOVE_DURATION
T_SIMU_TOTAL = 12.0
POS_INIT = np.array([0.0, 0.0])
POS_TARGET = np.array([5.0, 5.0])

if __name__ == "__main__":
    print("Sir, launching PVTOL Dynamic Extension Simulation...")
    
    # 8-state vector: [x, dx, z, dz, theta, dtheta, xi1, xi2]
    # Initialize xi1 to M*G to prevent singularity
    X0_ext = np.zeros(8)
    X0_ext[6] = M * G 
    
    t_eval = np.linspace(0, T_SIMU_TOTAL, 300)
    
    # Run Simulation
    sol = solve_ivp(
        pvtol_dynamics, 
        [0, T_SIMU_TOTAL], 
        X0_ext, 
        args=(controller_S2,), 
        t_eval=t_eval,
        rtol=1e-6
    )
    
    # Plotting logic
    plt.figure(figsize=(10, 5))
    plt.plot(sol.y[0], sol.y[2], label='Center of Mass (CoM)')
    plt.title("PVTOL Flight Path")
    plt.xlabel("x (Ground)")
    plt.ylabel("z (Altitude)")
    plt.legend()
    plt.grid(True)
    plt.show()