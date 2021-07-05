import control
import numpy as np
import random
import sys

from constants import *
from settings import *

class Actuation():

    def __init__(self):
        self.t         = TELEM_WAIT
        self.dt        = 1 / ACT_HZ
        self.simact    = np.zeros(ACT_LEN) #array for storing actuation
        self.simactstr = np.zeros(ACT_LEN) #array for storing actuation as string
        self.csvact    = np.zeros(ACT_LEN + 1) #array for storing csv actuation
        self.actstr    = str()

    def random(self, act2csv_in, act2tx_in, rx2act_out, event_start, event_end):
        event_start.wait() #wait for simulation start event
        while True:
            try:
                if event_end.is_set():
                    #Close pipes
                    act2csv_in.close()
                    act2tx_in.close()
                    rx2act_out.close()
                    break
                else:
                    rxdata = rx2act_out.recv() #receive RX telemetry
                    if (np.any(rxdata[:,0] >= self.t)):
                        i = np.where(rxdata[:,0] >= self.t)[0][0] #find first frame index
                        rxdata = rxdata[i,:] #get first frame
                        for i in range(ACT_LEN):
                            if i == 4 or i == 5:
                                self.simact[i] = random.gauss(mu=0.9, sigma=0.1) #throttle and mixture
                            else:
                                self.simact[i] = random.gauss(mu=0, sigma=0.1)
                        self.csvact[0]  = rxdata[0] #add timestamp
                        self.csvact[1:] = self.simact
                        act2csv_in.send(self.csvact) #send actuation to CSV
                        self.simactstr  = self.simact.astype(str)
                        self.actstr     = '\t'.join(self.simactstr)
                        self.actstr    += '\n'
                        act2tx_in.send(self.actstr) #send actuation to TX telemetry
                        self.t          = self.t + self.dt
                    else:
                        pass
            except:
                raise RuntimeError('.'.join((__name__, sys._getframe().f_code.co_name)))

    def fsf(self, act2csv_in, act2tx_in, cm2act_out, rx2act_out, event_start, event_end):
        self._init_fsf()
        event_start.wait() #wait for simulation start event
        if (CM_TYPE == 0) or (CM_TYPE == 'AL'):
            while True:
                try:
                    if event_end.is_set():
                        #Close pipes
                        act2csv_in.close()
                        act2tx_in.close()
                        cm2act_out.close()
                        break
                    else:
                        #Receive control model data
                        rxdata = rx2act_out.recv() #receive RX telemetry
                        if (np.any(rxdata[:,0] >= self.t)):
                            i = np.where(rxdata[:,0] >= self.t)[0][0] #find first frame index
                            rxdata = rxdata[i,:] #get first frame
                            cmdata = cm2act_out.recv()
                            self._preprocess(cmdata)
                            self._reduce_state_space()
                            self._find_act()
                            #Closed-loop actuation
                            x_er_ral        = self.fsf_dict['x_er_ral']
                            u_al            = self.fsf_dict['u_al']
                            u_sp_ral        = self.fsf_dict['u_sp_ral']
                            K               = self.fsf_dict['K']
                            u_er_clral      = - K @ x_er_ral #reduced closed-loop model actuation
                            u_clral         = u_er_clral + u_sp_ral
                            for i in range(len(u_clral)):
                                if i == 3:
                                    u_clral[i] = min(1,max(0,u_clral[i]))
                                else:
                                    u_clral[i] = min(1,max(-1,u_clral[i]))
                            print(' '.join(('u_clral:', str(u_clral))))
                            self.simact     = np.vstack((u_clral[:2], u_al[2], u_clral[2:], u_al[5], 0, 0, 0))
                            self.simact     = self.simact.flatten()
                            self.csvact[0]  = rxdata[0] #add timestamp
                            self.csvact[1:] = self.simact
                            act2csv_in.send(self.csvact) #send actuation to CSV
                            self.simactstr  = self.simact.astype(str)
                            self.actstr     = '\t'.join(self.simactstr)
                            self.actstr    += '\n'
                            act2tx_in.send(self.actstr) #send actuation to TX telemetry
                            self.t          = self.t + self.dt
                        else:
                            pass
                except:
                    raise RuntimeError('.'.join((__name__, sys._getframe().f_code.co_name)))

    def _init_fsf(self):
        self.fsf_dict = {}

    def _preprocess(self, cmdata):
        #Extract control model data
        x_al    = cmdata[0]
        x_sp_al = cmdata[1]
        x_er_al = cmdata[2]
        u_al    = cmdata[3]
        u_sp_al = cmdata[4]
        u_er_al = cmdata[5]
        al_sys  = cmdata[6]
        #Update FSF dictionary
        self.fsf_dict.update(x_al = x_al)
        self.fsf_dict.update(x_sp_al = x_sp_al)
        self.fsf_dict.update(x_er_al = x_er_al)
        self.fsf_dict.update(u_al = u_al)
        self.fsf_dict.update(u_sp_al = u_sp_al)
        self.fsf_dict.update(u_er_al = u_er_al)
        self.fsf_dict.update(al_sys = al_sys)

    def _reduce_state_space(self):
        x_al    = self.fsf_dict['x_al']
        x_sp_al = self.fsf_dict['x_sp_al']
        x_er_al = self.fsf_dict['x_er_al']
        u_al    = self.fsf_dict['u_al']
        u_sp_al = self.fsf_dict['u_sp_al']
        u_er_al = self.fsf_dict['u_er_al']
        al_sys  = self.fsf_dict['al_sys']
        #Construct reduced state space
        x_ral    = x_al[7:]
        x_sp_ral = x_sp_al[7:]
        x_er_ral = x_er_al[7:]
        u_ral    = u_al[np.ix_([0,1,3,4])]
        u_sp_ral = u_sp_al[np.ix_([0,1,3,4])]
        u_er_ral = u_er_al[np.ix_([0,1,3,4])]
        A        = al_sys.A
        B        = al_sys.B
        C        = al_sys.C
        D        = al_sys.D
        A_ral    = A[np.ix_([7,8,9,10,11,12],[7,8,9,10,11,12])]
        B_ral    = B[np.ix_([7,8,9,10,11,12],[0,1,3,4])]
        C_ral    = C[np.ix_([7,8,9,10,11,12],[7,8,9,10,11,12])]
        D_ral    = D[np.ix_([7,8,9,10,11,12],[0,1,3,4])]
        #Lateral and longitudinal state spaces
        ral_sys = control.StateSpace(A_ral, B_ral, C_ral, D_ral)
        ral_ctrb = control.ctrb(ral_sys.A, ral_sys.B)
        ral_obsv = control.obsv(ral_sys.A, ral_sys.C)
        #Update FSF dictionary
        self.fsf_dict.update(x_ral = x_ral)
        self.fsf_dict.update(x_sp_ral = x_sp_ral)
        self.fsf_dict.update(x_er_ral = x_er_ral)
        self.fsf_dict.update(u_ral = u_ral)
        self.fsf_dict.update(u_sp_ral = u_sp_ral)
        self.fsf_dict.update(u_er_ral = u_er_ral)
        self.fsf_dict.update(ral_sys = ral_sys)

    def _find_act(self):
        ral_sys  = self.fsf_dict['ral_sys']
        ral_poles = ral_sys.pole()
        print(' '.join(('OL RAL poles:', str(ral_poles))))
        new_poles = ral_poles
        for i in range(len(new_poles)):
            if new_poles[i].real > 0:
                new_poles[i] = - new_poles[i].real + 1j * new_poles[i].imag
            elif new_poles[i].real < 0:
                new_poles[i] = new_poles[i].real + 1j * new_poles[i].imag
            elif new_poles[i].real == 0:
                new_poles[i] = - 10 * abs(np.random.random(1)) + 1j * new_poles[i].imag
        print(' '.join(('CL RAL poles:', str(new_poles))))
        K = control.place(ral_sys.A, ral_sys.B, new_poles)
        #Closed-loop state space
        cl_ral_sys = control.StateSpace(ral_sys.A - ral_sys.B @ K, np.zeros(ral_sys.B.shape), ral_sys.C, np.zeros(ral_sys.D.shape))
        cl_ral_poles = cl_ral_sys.pole()
        #Update FSF dictionary
        self.fsf_dict.update(cl_ral_sys = cl_ral_sys)
        self.fsf_dict.update(K = K)
