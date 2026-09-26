from pathlib import Path
import numpy as np
from casadi import *
from hardware.python.interface import load, SegwayInterface
import matplotlib.pyplot as plt 
import control

from model.segway_model import nonlinear_sys, linearize, discretize
from controllers import two_dof as ctrl
from controllers import feedforward as ff

def main(sim:bool=True):
    # Time Parameters
    Trun = 30
    dt = 0.01 # Sampling Time
    Nrun = int(Trun / dt)

    # System Dimensions
    nx = 4 # States
    nu = 1 # Input

    # Load system parameters
    params_path = Path(__file__).resolve().parents[1] / 'model' / 'params.json'
    p = load(params_path)

    # Define symbolic state and input
    x = SX.sym('x', nx, 1) # x ∈ R^(4*1)
    u = SX.sym('u', nu, 1) # u ∈ R^(1*1)

    # Call the Sys_NL function that returns the symbolic system model
    f = nonlinear_sys(p=p, x=x, u=u) # x_dot = f(x,u)

    # Define operating point (equilibrium point)
    xs = np.zeros((nx, 1))
    us = np.zeros((nu, 1))

    # Linearize and discretize system
    Ac, Bc = linearize(f,x,u,xs,us,dt) # x_dot = Ac*x + Bc*u    
    Ad, Bd = discretize(Ac,Bc,dt) # x(k+1) = Ad*xk + Bd*uk

    # Check controllability for both continous and discrete systems
    ctrb_c = control.ctrb(Ac, Bc)
    ctrb_d = control.ctrb(Ad, Bd)
    rank_c = np.linalg.matrix_rank(ctrb_c)
    rank_d = np.linalg.matrix_rank(ctrb_d)
    print(f"Continuous system controllability rank: {rank_c} / {nx} -> {'CONTROLLABLE' if rank_c == nx else 'NOT CONTROLLABLE'}")
    print(f"Discrete system controllability rank: {rank_d} / {nx} -> {'CONTROLLABLE' if rank_d == nx else 'NOT CONTROLLABLE'}")

    # =============================== Feedforward Controller ==============================

    # Reference trajectory
    track_freq = 0.1 # Hz - how fast the robot oscillates
    track_amp = 0.05 # m - how far the robot moves

    # Build symbolic sin refrence for s
    #t_sym, s_ref_sym = ff.get_sinusoid_sym(track_freq, track_amp)

    phi = 0
    t_sym, s_ref_sym = ff.get_sinusoid_sym(track_freq, track_amp, phase_shift=phi)

    # Build full symbolic state reference
    x_ref_sym = ff.get_state_reference_sym(t_sym, s_ref_sym, Ac)

    # Initialize feedforward controller
    feedforward_controller = ff.Feedforward(Ac, Bc, x_ref_sym, t_sym, dt)


    _, x_ref_traj = feedforward_controller.get_ref_trajectories(Nrun-1)

    # =============================== Feedback Controller ==============================

    Q = np.diag([800.0, 600.0, 1.0, 0.01])
    R = np.array([[0.06]])

    # Initialize feedback controller
    feedback_controller = ctrl.Feedback(Ad, Bd, Q, R)


    # =============================== Two-DoF Controller ==============================
    controller = ctrl.Two_DoF(feedforward_controller, feedback_controller)


    # Define Segway Interface
    PORT = None
    # x0 = feedforward_controller.get_state_ref(0).full().flatten().reshape((nx, 1))
    x0 = np.zeros((nx, 1));
    minseg = SegwayInterface(sim=sim, x0=x0, PORT=PORT)
    
    # Declare variables to store closed loop trajectory
    x_traj = []
    u_traj = []
    t_sol = []

    # Control Loop
    print(f"START CONTROL")
    for k in range(Nrun-1):
        # 1. Measure raw state
        x_raw = minseg.get_state().flatten()
        
        

        x_k = x_raw[:nx].reshape((nx, 1))

        t_sol.append(k * dt)       
        x_traj.append(x_k.flatten()[:nx])

        # 3. Calculate current input
        u_k_array = controller.get_input(x_k, k)
        u_k = np.float64(u_k_array[0, 0])

        print(f"Iteration {k}, State: {x_k.flatten()[:nx]}, Input: {u_k}")
        
        # Send control Input to Arduino or Simulation Environment
        if sim:
            # Wrap u_k in a 2D array [[u_k]] or 1D array [u_k] so matrix multiplication works
            u_sim = np.array([[u_k]]) 
            minseg.apply_input(u_sim, Ad, Bd)
        else:
            # The physical hardware/serial wrapper expects the raw scalar float
            minseg.apply_input(u_k)

        u_traj.append([u_k]) # Store input to plot

        k += 1

    # close connection if opened
    if not sim:
        minseg.close()


    # Plots
    title = "State and Input Trajectories with 2-DoF Control"

    if sim:
        title += " (Simulation)"

    x_traj_array = np.array(x_traj)
    u_traj_array = np.vstack(u_traj)
    t_sol_array = np.array(t_sol)
    x_ref_traj_arr = np.array(x_ref_traj).squeeze()

    # ---- RMS tracking-error metric (post-hoc, not used elsewhere in the script) ----
    pos_err = x_traj_array[:, 0] - x_ref_traj_arr[:, 0]
    e_s_rms = np.sqrt(np.mean(pos_err**2))
    e_s_pct = e_s_rms / track_amp * 100
    u_rms = np.sqrt(np.mean(u_traj_array[:, 0]**2))
    print(f"Position RMS tracking error: {e_s_rms:.4f} m  ({e_s_pct:.1f}% of track_amp)")
    print(f"Control voltage RMS: {u_rms:.3f} V")
    # ----------------------------------------------------------------------------

    state_labels = ['$s$ [m]' , r'$\alpha$ [rad]' , r'$\dot{s}$ [m/s]' , r'$\dot{\alpha}$ [rad/s]']

    fig, axes = plt.subplots(nx + nu, 1, figsize=(12, 10), sharex=True)
    fig.suptitle(title)

    for i in range(nx):
        axes[i].plot(t_sol_array, x_traj_array[:, i], label='Actual', color='blue')
        axes[i].plot(t_sol_array, x_ref_traj_arr[:, i], label='Reference', color='red', linestyle='--')
        axes[i].set_ylabel(state_labels[i])
        axes[i].legend(loc='upper right')
        axes[i].grid(True)
    
    axes[nx].plot(t_sol_array, u_traj_array[:, 0], color='blue')
    axes[nx].set_ylabel('$u$ [V]')
    axes[nx].set_xlabel('Time [s]')
    axes[nx].grid(True)
    axes[nx].axhline(0, color='r', linestyle='--', linewidth=0.8)
    plt.tight_layout()


    fig2, axes2 = plt.subplots(nx, 1, figsize=(12, 10), sharex=True)
    fig2.suptitle("Tracking Error $e = x - x_{ref}$")

    for i in range(nx):
        error = x_traj_array[:, i] - x_ref_traj_arr[:, i]
        axes2[i].plot(t_sol_array, error, color='green')
        axes2[i].set_ylabel(f'$e_{{{i+1}}}$')
        axes2[i].axhline(0, color='r', linestyle='--', linewidth=0.8)
        axes2[i].grid(True)
    
    axes2[-1].set_xlabel('Time [s]')
    plt.tight_layout()

    plt.show()


if __name__ == '__main__':
    main(sim = True)