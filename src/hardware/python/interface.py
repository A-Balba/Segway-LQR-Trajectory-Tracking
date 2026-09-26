import json 
from types import SimpleNamespace
import numpy as np
import time 

def load(path):
    with open(path, "r") as f:
        data = json.load(f)
    return SimpleNamespace(**data)

class SegwayInterface:
    def __init__(self,  x0=None, PORT=None, BAUD=2000000, sim=True):
        self.sim = sim
        
        if x0 is None and PORT is None:
            raise ValueError("Provide an initial state for simulation or a serial PORT for hardware mode.")

        if not sim:
            from pySerialTransfer import pySerialTransfer as txfer
            self.link = txfer.SerialTransfer(PORT, BAUD)
            self.link.open(); time.sleep(0.1)
            self.t0_ref = {"ts0": None}
        
        else:
            self.x = x0  # initial state

    def get_state(self):
        if self.sim:
            return self.x
        else:
            from hardware.python.communication import receive
            while True:
                data = receive(self.link, self.t0_ref)  # use local variable
                if data is not None:
                    self.data = data  # only store when valid
                    return np.array([self.data.x1, self.data.x2, self.data.x3, self.data.x4])

    def apply_input(self, u, Ad=None, Bd=None):
        if self.sim:
            self.x = Ad @ self.x + Bd @ u
        else:
            from hardware.python.communication import send_control
            send_control(self.link, u.item(), self.data.tick_count)

    def close(self):
        if not self.sim:
            self.link.close()
