from constants import *
from settings import *
from modules.func.utils import *

import numpy as np
from pyquaternion import Quaternion
import sys

''' BASE SETPOINT '''
class BaseSetpoint():

    def __init__(self):
        self.t       = TELEM_WAIT
        self.dt      = 1 / CM_HZ
        self.csvdata = np.zeros(CM_STATE_LEN + CM_INPUT_LEN + 1) #array for storing csv setpoint 

''' CONSTANT SETPOINT '''
class Constant(BaseSetpoint):

    def run(self, rx2sp_out, sp2cm_in, sp2csv_in, event_start, event_end):
        self._init()
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
                        self._preprocess(rxdata)
                        spdata = self._find_sp()
                        self._build_pipe_data(spdata)
                        self._pipe(sp2cm_in, sp2csv_in, spdata)
                        self.t = self.t + self.dt
                    else:
                        pass
            except:
                raise RuntimeError('.'.join((__name__, sys._getframe().f_code.co_name)))

    def _init(self):
        self.phys_dict = {}

    def _preprocess(self, rxdata):
        i      = np.where(rxdata[:,0] >= self.t)[0][0] #find first frame index
        rxdata = rxdata[i,:] #get first frame
        rxtime = rxdata[0]
        self.phys_dict.update(rxtime = rxtime)

    def _find_sp(self):
        spdata = SP_INIT
        return spdata

    def _build_pipe_data(self, spdata):
        rxtime           = self.phys_dict['rxtime']
        self.csvdata[0]  = rxtime #add timestamp
        self.csvdata[1:] = SP_INIT

    def _pipe(self, sp2cm_in, sp2csv_in, spdata):
        sp2cm_in.send(spdata) #send setpoint point to control model
        sp2csv_in.send(self.csvdata) #send setpoint to CSV

''' STRAIGHT-LINE '''
class StraightLine(BaseSetpoint):

    def run(self, rx2sp_out, sp2cm_in, sp2csv_in, event_start, event_end):
        self._init()
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
                        self._preprocess(rxdata)
                        spdata = self._find_sp()
                        self._build_pipe_data(spdata)
                        self._pipe(sp2cm_in, sp2csv_in, spdata)
                        self.t = self.t + self.dt
                    else:
                        pass
            except:
                raise RuntimeError('.'.join((__name__, sys._getframe().f_code.co_name)))

    def _init(self):
        self.sp_dict   = {}
        self.phys_dict = {}
        PSI_P_RAD      = deg_to_rad(PSI_START)
        GAMMA_P_RAD    = 0 #path angle
        u_p            = np.array([np.cos(PSI_P_RAD), np.sin(PSI_P_RAD), 0], dtype=float) #path unit vector
        Q_NPPSI        = Quaternion(np.cos(0.5 * PSI_P_RAD), 0, 0, np.sin(0.5 * PSI_P_RAD))
        Q_NPGAMMA      = Quaternion(np.cos(0.5 * GAMMA_P_RAD), 0, np.sin(0.5 * GAMMA_P_RAD), 0)
        Q_NP           = Q_NPPSI * Q_NPGAMMA
        #Update physical variables dictionary
        self.phys_dict.update(PSI_P_RAD = PSI_P_RAD)
        self.phys_dict.update(GAMMA_P_RAD = GAMMA_P_RAD)
        self.phys_dict.update(u_p = u_p)
        self.phys_dict.update(Q_NP = Q_NP)

    def _preprocess(self, rxdata):
        i      = np.where(rxdata[:,0] >= self.t)[0][0] #find first frame index
        rxdata = rxdata[i,:] #get first frame
        rxtime = rxdata[0]
        #RX telemetry
        longd     = rxdata[2]
        latd      = rxdata[3]
        hqfe_km   = rxdata[15]
        phi_rad   = rxdata[20]
        theta_rad = rxdata[22]
        psi_rad   = rxdata[24]
        p_rel     = np.array([latd, longd, - km_to_m(hqfe_km)], dtype=float)
        euler     = np.array([phi_rad, theta_rad, psi_rad], dtype=float)
        q_nb      = euler_to_attquat(euler)
        #Update physical variables dictionary
        self.phys_dict.update(rxtime = rxtime)
        self.phys_dict.update(p_rel = p_rel)
        self.phys_dict.update(q_nb = q_nb)

    def _find_sp(self):
        K1 = 0.1
        K2 = 0.1
        p_rel         = self.phys_dict['p_rel']
        u_p           = self.phys_dict['u_p']
        Q_NP          = self.phys_dict['Q_NP']
        q_nb          = self.phys_dict['q_nb']
        p_er          = np.dot(p_rel, u_p) * u_p - p_rel
        psi_c         = np.arctan2(p_er[1], p_er[0])
        gamma_c       = - np.arctan2(p_er[2], np.sqrt((p_er[0] ** 2) + (p_er[1] ** 2)))
        q_ncpsi       = Quaternion(np.cos(0.5 * psi_c), 0, 0, np.sin(0.5 * psi_c))
        q_ncgamma     = Quaternion(np.cos(0.5 * gamma_c), 0, np.sin(0.5 * gamma_c), 0)
        q_nc          = q_ncpsi * q_ncgamma
        q_nd          = (Q_NP - q_nc) * np.exp(- K1 * np.linalg.norm(p_er)) + q_nc
        q_nd          = q_nd.normalised
        q_er          = q_nd.conjugate * q_nb
        omega_bc      = - K2 * q_er.elements[0] * q_er.elements[1:]
        print(': '.join(('(p,q,r)',str(omega_bc))))
        spdata        = np.array(SP_INIT, dtype=float)
        spdata[10:13] = omega_bc
        return spdata

    def _build_pipe_data(self, spdata):
        rxtime           = self.phys_dict['rxtime']
        self.csvdata[0]  = rxtime #add timestamp
        self.csvdata[1:] = spdata

    def _pipe(self, sp2cm_in, sp2csv_in, spdata):
        sp2cm_in.send(spdata) #send setpoint point to control model
        sp2csv_in.send(self.csvdata) #send setpoint to CSV

''' SETPOINT MODULE '''
class Setpoint():
    
    def __init__(self):
        self.Constant     = Constant()
        self.StraightLine = StraightLine()
