import numpy as np
import control


class Controller:
    def get_input(self, x, k):
        raise NotImplementedError("This method should be implemented by subclasses.")
    
    def get_gains(self):
        raise NotImplementedError("This method should be implemented by subclasses.")
    

class LQR_InfH(Controller):
    def __init__(self, Ad, Bd, Q, R):
        self.K = get_InfH_LQR_gain(Ad, Bd, Q, R)
    
    def get_input(self, x, k=None):
        return -self.K @ x

    def get_gains(self):
        return self.K
    

class LQR_FH(Controller):
    def __init__(self, Ad, Bd, Q, R, N):
        self.K_seq = get_FH_LQR_gain(Ad, Bd, Q, R, N)
    
    def get_input(self, x, k):
        return -self.K_seq[k, :, :] @ x
    
    def get_gains(self):
        return self.K_seq


def get_InfH_LQR_gain(Ad, Bd, Q, R):
    """
    Get Infinite-Horizon LQR discrete-time controller gain.
    """

    K = control.dare(Ad, Bd, Q, R)[2]

    return K


def get_FH_LQR_gain(Ad, Bd, Q, R, N):
    """
    Get Finite-Horizon LQR discrete-time controller gain sequence.
    """

    if N < 2:
        raise ValueError("Horizon too short, need at least 2 steps.")

    nx = Ad.shape[0]
    nu = Bd.shape[1]
    
    # --- Backward Riccati recursion ---
    P = np.zeros((N, nx, nx)) # Declare a 3D array to hold P(k) for k=0...N-1
    P[N-1, :, :] = Q  # Terminal cost = Q at final stage

    for k in range(N-1, 0, -1):
        Pk = P[k, :, :]

        K_k = np.linalg.inv(R + Bd.T @ Pk @ Bd) @ Bd.T @ Pk @ Ad
        
        P[k-1, :, :] = Q + Ad.T @ Pk @ Ad - Ad.T @ Pk @ Bd @ K_k

    # --- Gains sequence ---    
    K_seq = np.zeros((N-1, nu, nx))
    for i in range(N-1):
        P_ip1 = P[i+1, :, :]

        K_i = np.linalg.inv(R + Bd.T @ P_ip1 @ Bd) @ Bd.T @ P_ip1 @ Ad
        
        K_seq[i, :, :] = K_i

    return K_seq 

