from pathlib import Path
import numpy as np
from casadi import *
from model.segway_model import nonlinear_sys, linearize, discretize
from hardware.python.interface import load, SegwayInterface
import matplotlib.pyplot as plt 
import control

from control import lqr as ctrl

def main(sim:bool=True, finite_horizon:bool=False):
    # Time Parameters
    Trun = 20 # 20 sec
    dt = 0.01 # Sampling Time 10 ms
    Nrun = int(Trun / dt) # Number of iterations 2000

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
    Ac, Bc = linearize(f,x,u,xs,us) # x_dot = Ac*x + Bc*u
    Ad, Bd = discretize(Ac,Bc,dt) # x(k+1) = Ad*xk + Bd*uk

    # Check controllability
    ctrb_matrix = control.ctrb(Ad, Bd) # Computes controllability matrix
    rank = np.linalg.matrix_rank(ctrb_matrix)
    print(f"Controllability matrix rank: {rank}")
    print(f"system order (nx): {nx}")
    if rank == nx:
        print("System is CONTROLLABLE")
    else:
        print("System is NOT controllable")

    # Define Q and R matrices  
    Q = np.diag([1.0, 100.0, 1.0, 10.0])
    R = np.array([[15]])

    # Initialize controller
    if finite_horizon:
        T_pred = 2.0
        N_pred = int(T_pred / dt)
        controller = ctrl.LQR_FH(Ad, Bd, Q, R, N_pred)
    else:
        controller = ctrl.LQR_InfH(Ad, Bd, Q, R)

    # Define Segway Interface
    PORT = None
    x0 = np.zeros((nx, 1));
    # x0 =  np.array([[0.0], [0.1], [0.0], [0.0]]) # Initial state (tilted ~6 degrees)
    minseg = SegwayInterface(sim=sim, x0=x0, PORT=PORT)
    
    # Declare variables to store closed loop trajectory
    x_traj = []
    u_traj = []
    t_sol = []

    # Control Loop
    print(f"START CONTROL")
    k = 0
    for k in range(Nrun-1):
        # Measure current state
        x = minseg.get_state()
        t_sol.append(k * dt)       # store time
        x_traj.append(x.flatten()) # Store current state to plot

        # Calculate current input 
        if finite_horizon:
            k_ctrl = min(k, N_pred - 2)
            k_idx = (N_pred - 2) - k_ctrl
            u = controller.get_input(x, k_idx)
        else:
            u = controller.get_input(x, k)

        print(f"Iteration {k}, State: {x}, Input: {u}")
        
        # Send control Input to Arduino
        if sim:
            minseg.apply_input(u, Ad, Bd) # Updates simulation dynamics
        else:
            minseg.apply_input(float(u.flatten()[0])) # Sends voltage to motor

        u_traj.append(u.flatten()) # Store input to plot

        k += 1

    # close connection if opened
    if not sim:
        minseg.close()



    # =================================================================
    
    # Plot trajectories
    title = "State and Input Trajectories"

    if finite_horizon:
        title += " with Finite-Horizon LQR"
    else:
        title += " with Infinite-Horizon LQR"

    if sim:
        title += " (Simulation)"

    x_traj_array = np.array(x_traj)
    u_traj_array = np.vstack(u_traj)
    t_sol_array = np.array(t_sol)

    state_labels = ['$s$ [m]' , r'$\alpha$ [rad]' , r'$\dot{s}$ [m/s]' , r'$\dot{\alpha}$ [rad/s]']

    fig, axes = plt.subplots(nx + nu, 1, figsize=(10, 12), sharex=True)
    fig.suptitle(title)

    for i in range(nx):
        axes[i].plot(t_sol_array, x_traj_array[:, i])
        axes[i].set_ylabel(state_labels[i])
        axes[i].grid(True)
        axes[i].axhline(0, color='r', linestyle='--',linewidth=0.8)
    
    axes[nx].plot(t_sol_array, u_traj_array[:, 0])
    axes[nx].set_ylabel('$u$ [V]')
    axes[nx].set_xlabel('Time [s]')
    axes[nx].grid(True)
    axes[nx].axhline(0, color='r', linestyle='--', linewidth=0.8)
    plt.tight_layout()


    # Plot the gains

    if finite_horizon:
        N_values = [50, 100, 200, 500]
        fig_gains, axes_gains = plt.subplots(nu, nx, figsize=(14, 4))
        fig_gains.suptitle('LQR Gains for varying horizon N')

        state_labels_short = ['s', 'alpha', 's_dot', 'alpha_dot']

        for N_val in N_values:
            K_seq = ctrl.get_FH_LQR_gain(Ad, Bd, Q, R, N_val) # Computes gain evolution

            for j in range(nx): # Show how gains change over time
                axes_gains[j].plot(K_seq[:, 0, j], label=f'N={N_val}')
                axes_gains[j].set_ylabel(f'$K_{{{state_labels_short[j]}}}$')
                axes_gains[j].set_xlabel('Time step k')
                axes_gains[j].legend()
                axes_gains[j].grid(True)
        
        plt.tight_layout()
        plt.show()

    else: 
        plt.show()





if __name__ == '__main__':
    try:
        main(sim=True, finite_horizon=False)
    except Exception as e:
        import traceback
        traceback.print_exc()
