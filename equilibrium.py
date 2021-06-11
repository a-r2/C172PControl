import multiprocessing as mp
import numpy as np

from settings import *

EQ_LEN = 12 #[pn_eq, pe_eq, pd_eq, phi_eq, theta_eq, psi_eq, u_eq, v_eq, w_eq, p_eq, q_eq, r_eq]

class Equilibrium():
    def __init__(self, eq_type, eq_point_init, eq2csv_in, eq2mod_in, rx2eq_out, event_start):
        self.t         = TELEM_WAIT
        self.dt        = 1/CM_HZ
        self.simeq     = np.zeros(EQ_LEN) #array for storing equilibrium point 
        self.csveq     = np.zeros(EQ_LEN + 1) #array for storing csv equilibrium point 
        self.simeqstr  = np.zeros(EQ_LEN) #array for storing equilibrium point as string
        self.eqstr    = str()
        if eq_type == 'constant': #constant equilibrium point
            self.proc = mp.Process(target=self.constant, args=(eq_point_init, eq2csv_in, eq2mod_in, rx2eq_out, event_start), daemon=True) #process for calculating equilibrium point
        self.proc.start()
    def constant(self, eq_point_init, eq2csv_in, eq2mod_in, rx2eq_out, event_start):
        event_start.wait() #wait for simulation start event
        while True:
            rxdata = rx2eq_out.recv() #receive RX telemetry
            if (np.any(rxdata[:,0] >= self.t)):
                i = np.where(rxdata[:,0] >= self.t)[0][0] #find first frame index
                rxdata = rxdata[i,:] #get first frame
                self.t = self.t + self.dt
                self.simeq     = eq_point_init
                self.csveq[0]  = rxdata[0] #add timestamp
                self.csveq[1:] = self.simeq
                eq2csv_in.send(self.csveq) #send equilibrium to CSV
            else:
                pass
            eq2mod_in.send(eq_point_init) #send equilibrium point to control model
