# VTOL Flight Control Project

University project focused on generating feasible flight paths and designing controllers that ensure a VTOL system’s dynamics follow the desired trajectory.

## PVTOL Control: Non-Minimum Phase Analysis & Dynamic Extension

This repository studies the control of a **Planar Vertical Take-Off and Landing (PVTOL)** aircraft—a classic underactuated, unstable benchmark in aerial robotics. It highlights the limitations of standard feedback linearization and presents a more robust approach using **Dynamic Extension** and **flat-output regulation**.

---

## 1) Problem Statement

The PVTOL system has 3 degrees of freedom \((x, z, \theta)\) but only 2 control inputs:

- \(u_1\): main thrust  
- \(u_2\): rolling moment  

A key difficulty is the coupling parameter \(\epsilon \neq 0\): the rolling moment creates a parasitic lateral force, which makes the dynamics significantly harder to control.

### Mathematical Model

With normalized mass and gravity \((m=1, g=1)\), the dynamics are:

$$
\begin{cases}
\ddot{x} = -\sin(\theta)u_{1} + \epsilon \cos(\theta)u_{2} \\
\ddot{z} = \cos(\theta)u_{1} + \epsilon \sin(\theta)u_{2} - 1 \\
\ddot{\theta} = u_{2}
\end{cases}
$$

---

## 2) Control Challenges

- **Underactuation**: lateral motion requires tilting (changing \(\theta\)).
- **Non-minimum phase behavior**: directly tracking the center-of-mass position \(y=[x, z]^T\) yields **unstable internal (zero) dynamics** in \(\theta\).
- **Physical constraints**: thrust must remain nonnegative \((u_1 \ge 0)\), since propellers cannot generate “negative thrust.”

---

## 3) Proposed Solution: Dynamic Extension

To recover stable tracking, the controller regulates a virtual point called the **Center of Oscillation (CO)**, located a distance \(\epsilon\) above the CoM. This output is a **flat output** and avoids the unstable zero dynamics associated with CoM tracking.

The method uses **dynamic extension** by treating \(u_1\) (thrust) as an internal state instead of a direct input, introducing two integrators. This makes \(u_2\) appear in the **fourth derivative** of the regulated output (“snap”), enabling exact linearization and stable tracking.

---

## 4) Project Structure

The implementation is provided as a Google Colab notebook in a “literate programming” style:

- **`pvtol_dynamics`**: Nonlinear equations of motion
- **`controller_S1`**: Baseline feedback linearization (CoM tracking) — demonstrates failure via angle divergence
- **`controller_S2`**: CO-based controller with dynamic extension — ensures asymptotic convergence of tracking error
- **Trajectory generation**: smooth polynomial (“smooth step”) point-to-point trajectory (e.g., \((0,0)\rightarrow(5,5)\) in 8 s)


