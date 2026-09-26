import numpy as np
import numpy.linalg as la
import casadi as ca
    

class Feedforward:
    def __init__(self, A, B, x_ref_sym, t_sym, dt):
        self.u_ref_func, self.x_ref_func = self.get_ref_funcs(A, B, x_ref_sym, t_sym, dt)
    
    def get_input_ref(self, k):
        return self.u_ref_func(k)
    
    def get_state_ref(self, k):
        return self.x_ref_func(k)
    
    def get_ref_trajectories(self, N):
        u_ref_traj = []
        x_ref_traj = []
        for k in range(N):
            u_ref_traj.append(self.u_ref_func(k))
            x_ref_traj.append(self.x_ref_func(k))

        return np.array(u_ref_traj), np.array(x_ref_traj)
    
    def get_ref_funcs(self, A, B, x_ref_sym, t_sym, dt):
        
        """
        Compute feedforward input using model inversion.
        u_ref = B_pinv * (x_ref_dot - A * x_ref)
        """
        x_ref_dot_sym = ca.jacobian(x_ref_sym, t_sym)

        B_pinv = la.pinv(B)


        u_ref_sym = B_pinv @ (x_ref_dot_sym - A @ x_ref_sym)

        k_sym = ca.SX.sym('k')

        x_ref_discrete = ca.substitute(x_ref_sym, t_sym, dt * k_sym)
        u_ref_discrete = ca.substitute(u_ref_sym, t_sym, dt * k_sym)

        u_ref_func = ca.Function('u_ref', [k_sym], [u_ref_discrete])
        x_ref_func = ca.Function('x_ref', [k_sym], [x_ref_discrete])

        return u_ref_func, x_ref_func



def get_state_reference_sym(t_sym, x1_ref, Ac):
    s_ref = x1_ref
    s_dot_ref = ca.jacobian(s_ref, t_sym)
    s_ddot_ref = ca.jacobian(s_dot_ref, t_sym)
    s_dddot_ref = ca.jacobian(s_ddot_ref, t_sym)

    alpha_ref = s_ddot_ref / Ac[2, 1]
    alpha_dot_ref = s_dddot_ref / Ac[2, 1]

    x_ref_sym = ca.vertcat(
        s_ref,
        alpha_ref,
        s_dot_ref,
        alpha_dot_ref
    )

    return x_ref_sym


def get_sinusoid_sym(track_freq, track_amp, phase_shift=0.0):
    t_sym = ca.SX.sym('t')

    s_ref_sym = track_amp * ca.sin(2 * ca.pi * track_freq * t_sym + phase_shift)

    return t_sym, s_ref_sym

