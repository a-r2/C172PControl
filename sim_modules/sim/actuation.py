from constants import *
from settings import *
from c172p_model import *

import control as ctl
import control.optimal as ctlopt
import numpy as np
import sys

''' BASE ACTUATION '''
class BaseActuation():

    def __init__(self):
        self.t       = TELEM_WAIT
        self.dt      = 1 / ACT_HZ
        self.LAT_INPUT_LEN = len(LAT_INPUT_IND)
        self.LON_INPUT_LEN = len(LON_INPUT_IND)
        self.actdata = np.zeros(ACT_LEN) #array for storing actuation
        self.csvdata = np.zeros(ACT_LEN + 1) #array for piping actuation to CSV
        self.txdata  = str() #string array for piping actuation to FG

''' RANDOM ACTUATION '''
class Random(BaseActuation):

    def run(self, act2csv_in, act2tx_in, rx2act_out, event_start, event_end):
        self._init()
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
                        self._preprocess(rxdata)
                        self._find_act()
                        self._build_pipe_data()
                        self._pipe(act2tx_in, act2csv_in)
                        self.t = self.t + self.dt
                    else:
                        pass
            except:
                raise RuntimeError('.'.join((__name__, sys._getframe().f_code.co_name)))

    def _init(self):
        self.phys_dict = {}

    def _preprocess(self, rxdata):
        i      = np.where(rxdata[:,0] >= self.t)[0][0] #first RX telemetry frame index
        rxdata = rxdata[i,:] #first RX telemetry frame
        rxtime = rxdata[0]
        #Update physical variables dictionary
        self.phys_dict.update(rxtime = rxtime)

    def _find_act(self):
        self.actdata    = np.random.randint(-100, 100, ACT_LEN) / 100
        self.actdata[2] = np.random.randint(0, 100) / 100 #deltaf
        self.actdata[4] = np.random.randint(20, 100) / 100 #deltat
        self.actdata[5] = np.random.randint(20, 100) / 100 #deltam

    def _build_pipe_data(self):
        rxtime           = self.phys_dict['rxtime']
        actdata_str      = self.actdata.astype(str)
        self.txdata      = '\t'.join(actdata_str)
        self.txdata     += '\n'
        self.csvdata[0]  = rxtime #add timestamp
        self.csvdata[1:] = self.actdata
        return txdata

    def _pipe(self, act2tx_in, act2csv_in):
        act2tx_in.send(self.txdata)
        act2csv_in.send(self.csvdata)

''' FULL-STATE FEEDBACK ACTUATION '''
class FSFB(BaseActuation):

    def run(self, act2csv_in, act2tx_in, cm2act_out, rx2act_out, event_start, event_end):
        self._init()
        event_start.wait() #wait for simulation start event
        if (CM_TYPE == 0) or (CM_TYPE == 'AL') or (CM_TYPE == 2) or (CM_TYPE == 'LANL'):
            while True:
                try:
                    if event_end.is_set():
                        #Close pipes
                        act2csv_in.close()
                        act2tx_in.close()
                        cm2act_out.close()
                        break
                    else:
                        #Receive ctl.model data
                        rxdata = rx2act_out.recv() #receive RX telemetry
                        if (np.any(rxdata[:,0] >= self.t)):
                            cmdata = cm2act_out.recv()
                            self._preprocess(rxdata, cmdata)
                            self._reduce_state_space()
                            self._find_act()
                            self._build_pipe_data()
                            self._pipe(act2tx_in, act2csv_in)
                            self.t = self.t + self.dt
                        else:
                            pass
                except:
                    raise RuntimeError('.'.join((__name__, sys._getframe().f_code.co_name)))

    def _init(self):
        self.act_dict  = {}
        self.phys_dict = {}
        self.actdata[np.ix_([2, 5])] = [0, 1] #deltaf, deltam

    def _preprocess(self, rxdata, cmdata):
        i      = np.where(rxdata[:,0] >= self.t)[0][0] #first RX telemetry frame index
        rxdata = rxdata[i,:] #first RX telemetry frame
        rxtime = rxdata[0]
        #Extract control model data
        cm_sys  = cmdata[0]
        x_cm    = cmdata[1]
        x_eq_cm = cmdata[2]
        x_er_cm = cmdata[3]
        x_sp_cm = cmdata[4]
        u_cm    = cmdata[5]
        u_eq_cm = cmdata[6]
        u_er_cm = cmdata[7]
        u_sp_cm = cmdata[8]
        #Update actuation dictionary
        self.act_dict.update(cm_sys = cm_sys)
        self.act_dict.update(x_cm = x_cm)
        self.act_dict.update(x_eq_cm = x_eq_cm)
        self.act_dict.update(x_er_cm = x_er_cm)
        self.act_dict.update(x_sp_cm = x_sp_cm)
        self.act_dict.update(u_cm = u_cm)
        self.act_dict.update(u_eq_cm = u_eq_cm)
        self.act_dict.update(u_er_cm = u_er_cm)
        self.act_dict.update(u_sp_cm = u_sp_cm)
        #Update physical variables dictionary
        self.phys_dict.update(rxtime = rxtime)

    def _reduce_state_space(self):
        cm_sys  = self.act_dict['cm_sys']
        x_cm    = self.act_dict['x_cm']
        x_eq_cm = self.act_dict['x_eq_cm']
        x_er_cm = self.act_dict['x_er_cm']
        x_sp_cm = self.act_dict['x_sp_cm']
        u_cm    = self.act_dict['u_cm']
        u_eq_cm = self.act_dict['u_eq_cm']
        u_er_cm = self.act_dict['u_er_cm']
        u_sp_cm = self.act_dict['u_sp_cm']
        #Construct lateral state space
        x_eq_lat = x_eq_cm[np.ix_(LAT_STATE_IND)]
        x_sp_lat = x_sp_cm[np.ix_(LAT_STATE_IND)]
        x_er_lat = x_er_cm[np.ix_(LAT_STATE_IND)]
        u_eq_lat = u_eq_cm[np.ix_(LAT_INPUT_IND)]
        u_sp_lat = u_sp_cm[np.ix_(LAT_INPUT_IND)]
        u_er_lat = u_er_cm[np.ix_(LAT_INPUT_IND)]
        A        = cm_sys.A
        B        = cm_sys.B
        C        = cm_sys.C
        D        = cm_sys.D
        A_lat    = A[np.ix_(LAT_STATE_IND, LAT_STATE_IND)]
        B_lat    = B[np.ix_(LAT_STATE_IND, LAT_INPUT_IND)]
        C_lat    = C[np.ix_(LAT_STATE_IND, LAT_STATE_IND)]
        D_lat    = D[np.ix_(LAT_STATE_IND, LAT_INPUT_IND)]
        #Lateral dynamics model
        lat_sys = ctl.StateSpace(A_lat, B_lat, C_lat, D_lat)
        #Construct longitudinal state space
        x_eq_lon = x_eq_cm[np.ix_(LON_STATE_IND)]
        x_sp_lon = x_sp_cm[np.ix_(LON_STATE_IND)]
        x_er_lon = x_er_cm[np.ix_(LON_STATE_IND)]
        u_eq_lon = u_eq_cm[np.ix_(LON_INPUT_IND)]
        u_sp_lon = u_sp_cm[np.ix_(LON_INPUT_IND)]
        u_er_lon = u_er_cm[np.ix_(LON_INPUT_IND)]
        A        = cm_sys.A
        B        = cm_sys.B
        C        = cm_sys.C
        D        = cm_sys.D
        A_lon    = A[np.ix_(LON_STATE_IND, LON_STATE_IND)]
        B_lon    = B[np.ix_(LON_STATE_IND, LON_INPUT_IND)]
        C_lon    = C[np.ix_(LON_STATE_IND, LON_STATE_IND)]
        D_lon    = D[np.ix_(LON_STATE_IND, LON_INPUT_IND)]
        #Longitudinal dynamics model
        lon_sys = ctl.StateSpace(A_lon, B_lon, C_lon, D_lon)
        #Update actuation dictionary
        self.act_dict.update(lat_sys = lat_sys)
        self.act_dict.update(x_eq_lat = x_eq_lat)
        self.act_dict.update(x_er_lat = x_er_lat)
        self.act_dict.update(x_sp_lat = x_sp_lat)
        self.act_dict.update(u_eq_lat = u_eq_lat)
        self.act_dict.update(u_er_lat = u_er_lat)
        self.act_dict.update(u_sp_lat = u_sp_lat)
        self.act_dict.update(lon_sys = lon_sys)
        self.act_dict.update(x_eq_lon = x_eq_lon)
        self.act_dict.update(x_er_lon = x_er_lon)
        self.act_dict.update(x_sp_lon = x_sp_lon)
        self.act_dict.update(u_eq_lon = u_eq_lon)
        self.act_dict.update(u_er_lon = u_er_lon)
        self.act_dict.update(u_sp_lon = u_sp_lon)

    def _find_act(self):
        u_cm      = self.act_dict['u_cm']
        u_eq_cm   = self.act_dict['u_eq_cm']
        lat_sys   = self.act_dict['lat_sys']
        x_eq_lat  = self.act_dict['x_eq_lat']
        x_er_lat  = self.act_dict['x_er_lat']
        x_sp_lat  = self.act_dict['x_sp_lat']
        u_eq_lat  = self.act_dict['u_eq_lat']
        u_er_lat  = self.act_dict['u_er_lat']
        u_sp_lat  = self.act_dict['u_sp_lat']
        lon_sys   = self.act_dict['lon_sys']
        x_eq_lon  = self.act_dict['x_eq_lon']
        x_er_lon  = self.act_dict['x_er_lon']
        x_sp_lon  = self.act_dict['x_sp_lon']
        u_eq_lon  = self.act_dict['u_eq_lon']
        u_er_lon  = self.act_dict['u_er_lon']
        u_sp_lon  = self.act_dict['u_sp_lon']
        print('$$$$$$$$$$$$$$$$$$$$$$$$$$ ACTUATION $$$$$$$$$$$$$$$$$$$$$$$$$$')
        print(' '.join(('x_eq_lat:', str(x_eq_lat))))
        print(' '.join(('x_er_lat:', str(x_er_lat))))
        print(' '.join(('x_sp_lat:', str(x_sp_lat))))
        print(' '.join(('u_eq_lat:', str(u_eq_lat))))
        print(' '.join(('u_er_lat:', str(u_er_lat))))
        print(' '.join(('u_sp_lat:', str(u_sp_lat))))
        cm_sys  = self.act_dict['cm_sys']
        print(': '.join(('A', str(cm_sys.A))))
        print(': '.join(('B', str(cm_sys.B))))
        if np.linalg.matrix_rank(ctl.ctrb(lat_sys.A, lat_sys.B)) == len(LAT_STATE_IND):
            lat_poles = - np.array([1, 2, 3, 4, 5]) #desired closed-loop poles
            K_lat = ctl.place(lat_sys.A, lat_sys.B, lat_poles) #needed gain matrix to move the poles to the desired location
            cllat_sys   = ctl.StateSpace(lat_sys.A - lat_sys.B @ K_lat, np.zeros(lat_sys.B.shape), lat_sys.C, lat_sys.D) #closed-loop lateral state space model
            print(': '.join(('A_lat', str(lat_sys.A))))
            print(': '.join(('B_lat', str(lat_sys.B))))
            print(': '.join(('K_lat', str(K_lat))))
            print(' '.join(('OL LAT poles:', str(lat_sys.pole()))))
            print(' '.join(('CL LAT poles:', str(cllat_sys.pole()))))
            #if (SP_TYPE == 0) or (SP_TYPE == 'constant'):
            if False:
                u_r        = - np.linalg.pinv(lat_sys.C @ np.linalg.inv(lat_sys.A - lat_sys.B @ K_lat) @ lat_sys.B) @ (x_sp_lat - x_eq_lat)
                u_er_cllat = u_r - K_lat @ x_er_lat #lateral closed-loop model actuation
                print(' '.join(('u_r:', str(u_r))))
            else:
                u_er_cllat = - K_lat @ x_er_lat #lateral closed-loop model actuation
            u_cllat = u_er_cllat + u_eq_lat
            #Saturate actuation
            for i in range(self.LAT_INPUT_LEN):
                u_cllat[i] = min(1, max(-1, u_cllat[i]))
            u_er_cllat = u_cllat - u_eq_lat
            print(' '.join(('u_cllat:', str(u_cllat))))
            print(' '.join(('actdata_lat:', str(self.actdata[np.ix_(LAT_INPUT_IND)]))))
            self.actdata[np.ix_(LAT_INPUT_IND)] = u_er_cllat
        if np.linalg.matrix_rank(ctl.ctrb(lon_sys.A, lon_sys.B)) == len(LON_STATE_IND):
            lon_poles = - np.array([1, 2, 3, 4, 5]) #desired closed-loop poles
            K_lon = ctl.place(lon_sys.A, lon_sys.B, lon_poles) #needed gain matrix to move the poles to the desired location
            cllon_sys   = ctl.StateSpace(lon_sys.A - lon_sys.B @ K_lon, np.zeros(lon_sys.B.shape), lon_sys.C, lon_sys.D) #closed-loop loneral state space model
            print(': '.join(('A_lon', str(lon_sys.A))))
            print(': '.join(('B_lon', str(lon_sys.B))))
            print(': '.join(('K_lon', str(K_lon))))
            print(' '.join(('OL LON poles:', str(lon_sys.pole()))))
            print(' '.join(('CL LON poles:', str(cllon_sys.pole()))))
            #if (SP_TYPE == 0) or (SP_TYPE == 'constant'):
            if False:
                u_r        = - np.linalg.pinv(lon_sys.C @ np.linalg.inv(lon_sys.A - lon_sys.B @ K_lon) @ lon_sys.B) @ (x_sp_lon - x_eq_lon)
                u_er_cllon = u_r - K_lon @ x_er_lon #longitudinal closed-loop model actuation
                print(' '.join(('u_r:', str(u_r))))
            else:
                u_er_cllon = - K_lon @ x_er_lon #longitudinal closed-loop model actuation
            u_cllon = u_er_cllon + u_eq_lon
            #Saturate actuation
            for i in range(self.LON_INPUT_LEN):
                if LON_INPUT_IND[i] == 4: #deltat
                    u_cllon[i] = min(1, max(0, u_cllon[i]))
                else:
                    u_cllon[i] = min(1, max(-1, u_cllon[i]))
            u_er_cllon = u_cllon - u_eq_lon
            print(' '.join(('u_cllon:', str(u_cllon))))
            print(' '.join(('actdata_lon:', str(self.actdata[np.ix_(LON_INPUT_IND)]))))
            self.actdata[np.ix_(LON_INPUT_IND)] = u_er_cllon
        self.actdata[-3:] = u_eq_cm[np.ix_([0,1,3])] #deltaa_eq, deltae_eq, deltar_eq
        print(' '.join(('actdata:', str(self.actdata))))

    def _build_pipe_data(self):
        rxtime           = self.phys_dict['rxtime']
        actdata_str      = self.actdata.astype(str)
        self.txdata      = '\t'.join(actdata_str)
        self.txdata     += '\n'
        self.csvdata[0]  = rxtime #add timestamp
        self.csvdata[1:] = self.actdata

    def _pipe(self, act2tx_in, act2csv_in):
        act2tx_in.send(self.txdata)
        act2csv_in.send(self.csvdata)

''' LINER-QUADRATIC REGULATOR '''
class LQR(FSFB):

    def _find_act(self):
        lat_sys   = self.act_dict['lat_sys']
        x_eq_lat  = self.act_dict['x_eq_lat']
        x_er_lat  = self.act_dict['x_er_lat']
        x_sp_lat  = self.act_dict['x_sp_lat']
        u_cm      = self.act_dict['u_cm']
        u_eq_lat  = self.act_dict['u_eq_lat']
        u_er_lat  = self.act_dict['u_er_lat']
        u_sp_lat  = self.act_dict['u_sp_lat']
        if np.linalg.matrix_rank(ctl.ctrb(lat_sys.A, lat_sys.B)) == len(LAT_STATE_IND):
            rcm_poles = lat_sys.pole()
            print('$$$$$$$$$$$$$$$$$$$$$$$$$$ ACTUATION $$$$$$$$$$$$$$$$$$$$$$$$$$')
            print(': '.join(('A', str(lat_sys.A))))
            print(': '.join(('B', str(lat_sys.B))))
            print(' '.join(('OL RCM poles:', str(rcm_poles))))
            '''
            desired_poles = rcm_poles
            for i in range(len(desired_poles)):
                if desired_poles[i].real > 0:
                    desired_poles[i] = - desired_poles[i].real + 1j * desired_poles[i].imag
                elif desired_poles[i].real < 0:
                    desired_poles[i] = desired_poles[i].real + 1j * desired_poles[i].imag
                elif desired_poles[i].real == 0:
                    desired_poles[i] = - abs(np.random.random(1)) + 1j * desired_poles[i].imag
            '''
            desired_poles = -np.array([1,1.1,1.2,1.3,1.4,1.5,1.6,1.7,1.8,1.9])
            Q = np.array([
                          [1, 0, 0, 0, 0, 0],
                          [0, 1, 0, 0, 0, 0],
                          [0, 0, 1, 0, 0, 0],
                          [0, 0, 0, 1, 0, 0],
                          [0, 0, 0, 0, 1, 0],
                          [0, 0, 0, 0, 0, 1]
                         ], dtype=float)
            R = np.array([
                          [1, 0, 0, 0],
                          [0, 1, 0, 0],
                          [0, 0, 1, 0],
                          [0, 0, 0, 1]
                         ], dtype=float)
            K, S, E = ctl.lqr(lat_sys, Q, R)
            #Closed-loop state space
            cllat_sys   = ctl.StateSpace(lat_sys.A - lat_sys.B @ K, np.zeros(lat_sys.B.shape), lat_sys.C, np.zeros(lat_sys.D.shape))
            clrcm_poles = cllat_sys.pole()
            print(' '.join(('CL RCM poles:', str(clrcm_poles))))
            if (SP_TYPE == 0) or (SP_TYPE == 'constant'):
                u_r         = - np.linalg.pinv(lat_sys.C @ np.linalg.inv(lat_sys.A - lat_sys.B @ K) @ lat_sys.B) @ (x_sp_lat - x_eq_lat)
                print(' '.join(('u_r:', str(u_r))))
                u_er_clrcm  = u_r - K @ x_er_lat #reduced closed-loop model actuation
            else:
                u_er_clrcm  = - K @ x_er_lat #reduced closed-loop model actuation
            u_clrcm = u_er_clrcm + u_eq_lat
            #Saturate actuation
            for i in range(len(u_clrcm)):
                if i == 3:
                    u_clrcm[i] = min(1, max(0, u_clrcm[i]))
                else:
                    u_clrcm[i] = min(1, max(-1, u_clrcm[i]))
            u_er_clrcm = u_clrcm - u_eq_lat
            actdata = np.hstack((u_clrcm[:2], u_cm[2], u_clrcm[2:], u_cm[5], 0, 0, 0))
            print(' '.join(('u_clrcm:', str(u_clrcm))))
            print(' '.join(('actdata:', str(actdata))))
        else:
            actdata = np.hstack((u_cm, 0, 0, 0))
        return actdata

''' MPC METHOD '''
class MPC(FSFB):

    def _find_act(self):
        lat_sys   = self.act_dict['lat_sys']
        x_eq_lat  = self.act_dict['x_eq_lat']
        x_er_lat  = self.act_dict['x_er_lat']
        x_sp_lat  = self.act_dict['x_sp_lat']
        u_cm      = self.act_dict['u_cm']
        u_eq_lat  = self.act_dict['u_eq_lat']
        u_er_lat  = self.act_dict['u_er_lat']
        u_sp_lat  = self.act_dict['u_sp_lat']
        if np.linalg.matrix_rank(ctl.ctrb(lat_sys.A, lat_sys.B)) == len(LAT_STATE_IND):
            rcm_io_sys = ctl.iosys.LinearIOSystem(lat_sys)
            print('$$$$$$$$$$$$$$$$$$$$$$$$$$ ACTUATION $$$$$$$$$$$$$$$$$$$$$$$$$$')
            print(': '.join(('A', str(lat_sys.A))))
            print(': '.join(('B', str(lat_sys.B))))
            horizon = np.arange(0, MPC_HORSTEPS * self.dt, self.dt)
            Q = np.array([
                          [1, 0, 0, 0, 0, 0],
                          [0, 1, 0, 0, 0, 0],
                          [0, 0, 1, 0, 0, 0],
                          [0, 0, 0, 1, 0, 0],
                          [0, 0, 0, 0, 1, 0],
                          [0, 0, 0, 0, 0, 1]
                         ], dtype=float)
            R = np.array([
                          [1, 0, 0, 0],
                          [0, 1, 0, 0],
                          [0, 0, 1, 0],
                          [0, 0, 0, 1]
                         ], dtype=float)
            cost    = ctlopt.quadratic_cost(rcm_io_sys, Q, R)
            constr  = [ctlopt.input_range_constraint(rcm_io_sys, [DELTAA_RANGE[0], DELTAE_RANGE[0], DELTAR_RANGE[0], DELTAT_RANGE[0]], [DELTAA_RANGE[1], DELTAE_RANGE[1], DELTAR_RANGE[1], DELTAT_RANGE[1]])]
            mpc_res = ctlopt.solve_ocp(rcm_io_sys, horizon, x_er_lat, cost, constraints=constr)
            u_mpc   = mpc_res.inputs[:,0]
            u_er_clrcm = u_mpc #reduced closed-loop model actuation
            u_clrcm = u_er_clrcm + u_eq_lat
            #Saturate actuation
            for i in range(len(u_clrcm)):
                if i == 3:
                    u_clrcm[i] = min(1, max(0, u_clrcm[i]))
                else:
                    u_clrcm[i] = min(1, max(-1, u_clrcm[i]))
            u_er_clrcm = u_clrcm - u_eq_lat
            actdata = np.hstack((u_clrcm[:2], u_cm[2], u_clrcm[2:], u_cm[5], 0, 0, 0))
            print(' '.join(('u_clrcm:', str(u_clrcm))))
            print(' '.join(('actdata:', str(actdata))))
        else:
            actdata = np.hstack((u_cm, 0, 0, 0))
        return actdata

''' ACTUATION MODULE '''
class Actuation():

    def __init__(self):
        self.Random = Random()
        self.FSFB   = FSFB()
        self.LQR    = LQR()
        self.MPC    = MPC()
