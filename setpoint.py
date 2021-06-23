import multiprocessing as mp
import numpy as np

from settings import *

SP_LEN = 12 #[pn_sp, pe_sp, pd_sp, phi_sp, theta_sp, psi_sp, u_sp, v_sp, w_sp, p_sp, q_sp, r_sp]

class Setpoint():

    def __init__(self):
        self.t        = TELEM_WAIT
        self.dt       = 1 / CM_HZ
        self.simsp    = np.zeros(SP_LEN) #array for storing setpoint point 
        self.csvsp    = np.zeros(SP_LEN + 1) #array for storing csv setpoint point 
        self.simspstr = np.zeros(SP_LEN) #array for storing setpoint point as string
        self.spstr    = str()

    def constant(self, rx2sp_out, sp2cm_in, sp2csv_in, event_start):
        event_start.wait() #wait for simulation start event
        while True:
            rxdata = rx2sp_out.recv() #receive RX telemetry
            if (np.any(rxdata[:,0] >= self.t)):
                i = np.where(rxdata[:,0] >= self.t)[0][0] #find first frame index
                rxdata = rxdata[i,:] #get first frame
                self.t = self.t + self.dt
                self.simsp     = SP_POINT_INIT
                self.csvsp[0]  = rxdata[0] #add timestamp
                self.csvsp[1:] = self.simsp
                sp2csv_in.send(self.csvsp) #send setpoint to CSV
            else:
                pass
            sp2cm_in.send(SP_POINT_INIT) #send setpoint point to control model
