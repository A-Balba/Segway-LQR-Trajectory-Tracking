import numpy as np
import numpy.linalg as la
import casadi as ca
import control

from controllers.feedforward import Feedforward


class Controller:
    def get_input(self, x, k):
        raise NotImplementedError("This method should be implemented by subclasses.")
    

class Two_DoF(Controller):
    def __init__(self, feedforward_control:Feedforward, feedback_control:Controller):
        self.feedforward_control = feedforward_control
        self.feedback_control = feedback_control

    def get_input(self, x, k):
        """
        Combine feedforward and feedback inputs:
        u = u_ff + u_fb
        """
        x = np.array(x).reshape((-1, 1))
        x_ref = self.feedforward_control.get_state_ref(k).full().reshape((-1, 1))
        u_ff = self.feedforward_control.get_input_ref(k).full().reshape((-1, 1))
        e = x - x_ref
        u_fb = self.feedback_control.get_input(e, k)
        u = u_ff + u_fb

        return u


class Feedback(Controller):
    def __init__(self, Ad, Bd, Q, R):

        self.K = control.dare(Ad, Bd, Q, R)[2]
    
    def get_input(self, e, _):

        e = np.array(e).reshape((-1, 1))
        return -self.K @ e
