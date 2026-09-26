from casadi import *
from scipy.signal import cont2discrete


def nonlinear_sys(p,x,u):
    """
    Function to define the nonlinear Segway model
    x = [s, alpha, s_dot, alpha_dot]
    u = motor voltage
    """

    alpha = x[1]
    s_dot = x[2]
    alpha_dot = x[3]

    # Define mass matrix
    M = vertcat(
        horzcat(p.mp + 2*p.mw + 2*p.Jw/p.rw**2, p.mp*p.l*cos(alpha)),
        horzcat(p.mp*p.l*cos(alpha),            p.Jp + p.mp*p.l**2)
    )
    

    # Define equations of motion 

    f_q = vertcat(
        (p.kt/(p.R*p.rw))*u - (p.kt*p.kb/(p.rw**2*p.R))*s_dot
        + (p.kt*p.kb/(p.R*p.rw))*alpha_dot + p.mp*p.l*sin(alpha)*alpha_dot**2,

        -(p.kt/p.R)*u + (p.kt*p.kb/(p.R*p.rw))*s_dot
        - (p.kt*p.kb/p.R)*alpha_dot + p.mp*p.g*p.l*sin(alpha)
    )

    # Solve for accelerations = M * [s_ddot, alpha_ddot]^T = f_q
    accelerations = solve(M, f_q)

    # \dot{x} = f(x,u)
    f = SX.zeros(4, 1)
    
    f[0] = x[2]             # s_dot
    f[1] = x[3]             # alpha_dot
    f[2] = accelerations[0] # s_ddot
    f[3] = accelerations[1] # alpha_ddot

    return f  


def linearize(f,x,u,xs,us):
    """
    Function to linearize nonlinear model around equilibrium 
    """

    A_fcn = Function('A', [x, u], [jacobian(f, x)])
    B_fcn = Function('B', [x, u], [jacobian(f, u)])
    
    A = A_fcn(xs, us).full()
    B = B_fcn(xs, us).full()

    return A, B


def discretize(Ac,Bc,dt):
    """
    Function to discretize linear model
    """

    nx = Ac.shape[0]

    Cc = np.eye(nx)
    Dc = np.zeros((nx, 1))

    Ad, Bd, _, _, _ = cont2discrete((Ac, Bc, Cc, Dc), dt, method='zoh')

    return Ad, Bd 