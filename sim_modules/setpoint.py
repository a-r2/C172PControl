import numpy as np
import sys

from constants import *
from settings import *

class Setpoint():

    def __init__(self):
        self.t        = TELEM_WAIT
        self.dt       = 1 / CM_HZ
        self.simsp    = np.zeros(CM_STATE_LEN + CM_INPUT_LEN) #array for storing setpoint
        self.simspstr = np.zeros(CM_STATE_LEN + CM_INPUT_LEN) #array for storing setpoint as string
        self.csvsp    = np.zeros(CM_STATE_LEN + CM_INPUT_LEN + 1) #array for storing csv setpoint 
        self.spstr    = str()

    def constant(self, rx2sp_out, sp2cm_in, sp2csv_in, event_start, event_end):
        self.simsp = SP_INIT
        event_start.wait() #wait for simulation start event
        while True:
            try:
                if event_end.is_set():
                    #Close pipes
                    rx2sp_out.close()
                    sp2cm_in.close()
                    sp2csv_in.close()
                    break
                else:
                    rxdata = rx2sp_out.recv() #receive RX telemetry
                    if (np.any(rxdata[:,0] >= self.t)):
                        i = np.where(rxdata[:,0] >= self.t)[0][0] #find first frame index
                        rxdata = rxdata[i,:] #get first frame
                        sp2cm_in.send(self.simsp) #send setpoint point to control model
                        self.csvsp[0]  = rxdata[0] #add timestamp
                        self.csvsp[1:] = self.simsp
                        sp2csv_in.send(self.csvsp) #send setpoint to CSV
                        self.t = self.t + self.dt
                    else:
                        pass
            except:
                raise RuntimeError('.'.join((__name__, sys._getframe().f_code.co_name)))
