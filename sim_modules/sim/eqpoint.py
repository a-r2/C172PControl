from c172p_model import *
from constants import *
from settings import *
from modules.func.utils import *
from modules.sim.control_models import *

import numpy as np
from pyquaternion import Quaternion
from scipy import optimize as opt
import sys

''' EQUILIBRIUM POINT MODULE '''
class Equilibrium():

    def __init__(self):
        self.t           = TELEM_WAIT
        self.dt          = 1 / CM_HZ
        self.csvdata     = np.zeros(CM_STATE_LEN + CM_INPUT_LEN + 1) #array for storing csv equilibrium
        self.Z_STATE_LEN = len(EQ_STATE_IND)
        self.Z_INPUT_LEN = len(EQ_INPUT_IND)

    def run(self, eq2cm_in, eq2csv_in, rx2eq_out, event_start, event_end):
        if not ((CM_TYPE == 1) or (CM_TYPE == 'ANL')): #analytic non-linear control model
            self._init()
            event_start.wait() #wait for simulation start event
            while True:
                try:
                    if event_end.is_set():
                        #Close pipes
                        eq2cm_in.close()
                        eq2csv_in.close()
                        rx2eq_out.close()
                        break
                    else:
                        rxdata = rx2eq_out.recv() #receive RX telemetry
                        if (np.any(rxdata[:,0] >= self.t)):
                            self._preprocess(rxdata)
                            self._find_eq()
                            eqdata = self._build_pipe_data()
                            self._pipe(eq2cm_in, eq2csv_in, eqdata)
                            self.t = self.t + self.dt
                        else:
                            pass
                except:
                    raise RuntimeError('.'.join((__name__, sys._getframe().f_code.co_name)))

    def _init(self):
        #Initialize dictionaries
        self.eq_dict   = {} #equilibrium point dictionary
        self.aacm_dict = {} #analytic control models assumptions dictionary
        self.phys_dict = {} #physical variables dictionary
        #Initialize equilibrium point variables
        x_cm         = np.zeros(CM_STATE_LEN) #state
        x_eq_cm      = np.zeros(CM_STATE_LEN) #state equilibrium
        prev_x_eq_cm = np.zeros(CM_STATE_LEN) #previous state equilibrium
        u_cm         = np.zeros(CM_INPUT_LEN) #input
        u_eq_cm      = np.zeros(CM_INPUT_LEN) #input equilibrium
        prev_u_eq_cm = np.zeros(CM_INPUT_LEN) #previous input equilibrium
        self.success = False
        #Update control model dictionary
        self.eq_dict.update(x_cm = x_cm)
        self.eq_dict.update(x_eq_cm = x_eq_cm)
        self.eq_dict.update(prev_x_eq_cm = prev_x_eq_cm)
        self.eq_dict.update(u_cm = u_cm)
        self.eq_dict.update(u_eq_cm = u_eq_cm)
        self.eq_dict.update(prev_u_eq_cm = prev_u_eq_cm)

    def _preprocess(self, rxdata):
        i      = np.where(rxdata[:,0] >= self.t)[0][0] #find first frame index
        rxdata = rxdata[i,:] #get first frame
        rxtime = rxdata[0]
        self.phys_dict.update(rxtime = rxtime)
        #RX telemetry
        long_delta = rxdata[2]
        lat_delta  = rxdata[3]
        h_qfe      = rxdata[11]
        phi        = rxdata[14]
        theta      = rxdata[15]
        psi        = rxdata[16]
        u          = rxdata[23]
        v          = rxdata[24]
        w          = rxdata[25]
        p          = rxdata[35]
        q          = rxdata[36]
        r          = rxdata[37]
        alphadot   = rxdata[41]
        sigmara    = rxdata[79] 
        deltara    = rxdata[80]
        sigmala    = rxdata[81]
        deltala    = rxdata[82]
        sigmae     = rxdata[83]
        deltae     = rxdata[84]
        sigmar     = rxdata[87]
        deltar     = rxdata[88]
        deltat     = rxdata[89]
        qbar       = rxdata[94]
        revprop    = rxdata[101]
        Ixx        = rxdata[102]
        Ixy        = rxdata[103]
        Ixz        = rxdata[104]
        Iyy        = rxdata[105]
        Iyz        = rxdata[106]
        Izz        = rxdata[107]
        mass       = rxdata[108]
        #Conversions
        pn             = lat_delta
        pe             = long_delta
        pd             = - h_qfe
        deltaa         = deltaa_avg_acm(deltala, deltara) 
        euler          = np.array([phi, theta, psi], dtype=float)
        #Assumptions of analytic control models
        self.aacm_dict = assumptions_acm(deltat, pd)
        sigmaf         = self.aacm_dict['sigmaf']
        deltaf         = self.aacm_dict['deltaf']
        deltam         = self.aacm_dict['deltam']
        rho            = self.aacm_dict['rho']
        grav           = self.aacm_dict['grav']
        stall          = self.aacm_dict['stall']
        hmacb          = self.aacm_dict['hmacb']
        #Moments of inertia
        Gamma  = mominert_gamma(Ixx, Ixz, Izz)
        Gamma1 = mominert_gamma1(Ixx, Ixz, Iyy, Izz, Gamma)
        Gamma2 = mominert_gamma2(Ixz, Iyy, Izz, Gamma)
        Gamma3 = mominert_gamma3(Izz, Gamma)
        Gamma4 = mominert_gamma4(Ixz, Gamma)
        Gamma5 = mominert_gamma5(Ixx, Iyy, Izz)
        Gamma6 = mominert_gamma6(Ixz, Iyy)
        Gamma7 = mominert_gamma7(Ixx, Iyy, Ixz, Gamma)
        Gamma8 = mominert_gamma8(Ixx, Gamma)
        #State
        x_cm = np.array(
                        [
                         pn,
                         pe,
                         pd,
                         phi,
                         theta,
                         psi,
                         u,
                         v,
                         w,
                         p,
                         q,
                         r
                        ]
                       )
        #Input
        u_cm = np.array(
                        [
                         deltaa,
                         deltae,
                         deltaf,
                         deltar,
                         deltat,
                         deltam
                        ]
                       )
        #Update equilibrium point dictionary
        self.eq_dict.update(x_cm = x_cm)
        self.eq_dict.update(u_cm = u_cm)
        #Equilibrium point parameters dictionary
        params_eq = {}
        params_eq.update(alphadot = alphadot)
        params_eq.update(sigmaf = sigmaf)
        params_eq.update(hmacb = hmacb)
        params_eq.update(stall = stall)
        params_eq.update(Iyy = Iyy)
        params_eq.update(mass = mass)
        params_eq.update(grav = grav)
        params_eq.update(rho = rho)
        params_eq.update(revprop = revprop)
        params_eq.update(Gamma = Gamma)
        params_eq.update(Gamma1 = Gamma1)
        params_eq.update(Gamma2 = Gamma2)
        params_eq.update(Gamma3 = Gamma3)
        params_eq.update(Gamma4 = Gamma4)
        params_eq.update(Gamma5 = Gamma5)
        params_eq.update(Gamma6 = Gamma6)
        params_eq.update(Gamma7 = Gamma7)
        params_eq.update(Gamma8 = Gamma8)
        self.eq_dict.update(params_eq = params_eq)

    def _find_eq(self):
        params_eq     = self.eq_dict['params_eq']
        x_cm          = self.eq_dict['x_cm']
        x_eq_cm       = self.eq_dict['x_eq_cm']
        prev_x_eq_cm  = self.eq_dict['prev_x_eq_cm']
        u_cm          = self.eq_dict['u_cm']
        u_eq_cm       = self.eq_dict['u_eq_cm']
        prev_u_eq_cm  = self.eq_dict['prev_u_eq_cm']
        if not self.success:
            #Build the variables array (z) for the optimization problem 
            z_x = x_cm[np.ix_(EQ_STATE_IND)]
            z_u = u_cm[np.ix_(EQ_INPUT_IND)]
            z0  = np.hstack((z_x, z_u))
            #Actuation constraints (inequality constraints)
            M_ineq      = np.zeros((4, len(EQ_STATE_IND) + len(EQ_INPUT_IND))) #row: inequality, column: z variable
            M_ineq[0,8] = 1 #deltaa
            M_ineq[1,9] = 1 #deltae
            M_ineq[2,10] = 1 #deltar
            M_ineq[3,11] = 1 #deltat
            lb_ineq     = np.array([-1, -1, -1, 0])
            ub_ineq     = np.array([1, 1, 1, 1])
            ineq_lc     = opt.LinearConstraint(M_ineq, lb_ineq, ub_ineq)
            #Result
            eq_result = opt.minimize(self._fcn_eq, z0, args=params_eq, constraints=(ineq_lc,))
            x_eq      = eq_result.x[:self.Z_STATE_LEN]
            u_eq      = eq_result.x[self.Z_STATE_LEN:]
            if not eq_result.success:
                x_eq_cm = prev_x_eq_cm
                u_eq_cm = prev_u_eq_cm
            else:
                self.success = True
                j = 0
                for i in range(CM_STATE_LEN):
                    if i in EQ_STATE_IND:
                        x_eq_cm[i] = x_eq[j]
                        j += 1
                    else:
                        x_eq_cm[i] = x_cm[i]
                j = 0
                for i in range(CM_INPUT_LEN):
                    if i in EQ_INPUT_IND:
                        u_eq_cm[i] = u_eq[j]
                        j += 1
                    else:
                        u_eq_cm[i] = u_cm[i]
            x_eq_cm[np.ix_([0,1,2,5])] = x_cm[np.ix_([0,1,2,5])] #maintain pn, pe, pd, psi
            prev_x_eq_cm = x_eq_cm
            prev_u_eq_cm = u_eq_cm
            self.eq_dict.update(x_eq_cm = x_eq_cm)
            self.eq_dict.update(prev_x_eq_cm = prev_x_eq_cm)
            self.eq_dict.update(u_eq_cm = u_eq_cm)
            self.eq_dict.update(prev_u_eq_cm = prev_u_eq_cm)
            print('#################### EQUILIBRIUM ####################')
            print(': '.join(('eq_result', str(eq_result))))
            print(': '.join(('x_eq_cm', str(x_eq_cm))))
            print(': '.join(('u_eq_cm', str(u_eq_cm))))

    def _build_pipe_data(self):
        x_eq_cm          = self.eq_dict['x_eq_cm']
        u_eq_cm          = self.eq_dict['u_eq_cm']
        rxtime           = self.phys_dict['rxtime']
        eqdata           = np.hstack((x_eq_cm, u_eq_cm))
        self.csvdata[0]  = rxtime
        self.csvdata[1:] = np.hstack(eqdata)
        return eqdata

    def _pipe(self, eq2cm_in, eq2csv_in, eqdata):
        eq2cm_in.send(eqdata) #send equilibrium point to control model
        eq2csv_in.send(self.csvdata) #send equilibrium point to CSV

    def _fcn_eq(self, z_in, params_eq):
        #Parameters
        alphadot = params_eq['alphadot']
        sigmaf   = params_eq['sigmaf']
        hmacb    = params_eq['hmacb']
        stall    = params_eq['stall']
        Iyy      = params_eq['Iyy']
        mass     = params_eq['mass']
        grav     = params_eq['grav']
        rho      = params_eq['rho']
        revprop  = params_eq['revprop']
        Gamma    = params_eq['Gamma']
        Gamma1   = params_eq['Gamma1']
        Gamma2   = params_eq['Gamma2']
        Gamma3   = params_eq['Gamma3']
        Gamma4   = params_eq['Gamma4']
        Gamma5   = params_eq['Gamma5']
        Gamma6   = params_eq['Gamma6']
        Gamma7   = params_eq['Gamma7']
        Gamma8   = params_eq['Gamma8']
        #State
        phi   = z_in[0]
        theta = z_in[1]
        psi   = 0 #does not matter which value
        u     = z_in[2]
        v     = z_in[3]
        w     = z_in[4]
        p     = z_in[5]
        q     = z_in[6]
        r     = z_in[7]
        euler = np.array([phi, theta, psi], dtype=float)
        #Input
        deltaa = z_in[8]
        deltae = z_in[9]
        deltar = z_in[10]
        deltat = z_in[11]
        #Physical variables
        sigmala    = deltaa_to_sigmala(deltaa)
        sigmara    = deltaa_to_sigmara(deltaa)
        sigmaa_avg = sigmaa_avg_acm(sigmala, sigmara)
        sigmae     = deltae_to_sigmae(deltae)
        sigmar     = deltar_to_sigmar(deltar)
        J          = J_acm(u, revprop)
        TT_out     = TT_interp(J)
        CT         = TT_out
        TP_out     = TP_interp(J)
        CP         = TP_out
        T          = thrust_eng(CT, rho, revprop)
        P          = power_eng(CP, rho, revprop)
        Vauw       = Vauw_acm(u, w)
        Va         = Va_acm(u, v, w)
        Bw2Va      = BW / (2 * (Va + 1e-6))
        Cw2Va      = CW / (2 * (Va + 1e-6))
        Vprop2     = Vprop2_acm(u, rho, T)
        Vprop      = Vprop_acm(u, Vprop2)
        Vind       = Vind_acm(u, Vprop)
        alpha      = alpha_acm(u, w)
        beta       = beta_acm(v, Vauw)
        qbar       = qbar_acm(rho, Va)
        qbaruw     = qbaruw_acm(rho, Vauw)
        qbarind    = qbarind_acm(rho, Vind)
        qbarprop   = qbarprop_acm(rho, Vprop)
        omega      = omega_acm(revprop)
        H          = H_acm(omega)
        tau        = tau_acm(P, omega)
        #Aerodynamics tables
        TDge_out = TDge_interp(hmacb)
        TD2_out  = TD2_interp(sigmaf)
        TD3_out  = TD3_interp(alpha, sigmaf)
        TC1_out  = TC1_interp(beta, sigmaf)
        TLge_out = TLge_interp(hmacb)
        TL1_out  = TL1_interp(alpha, stall)
        TL2_out  = TL2_interp(sigmaf)
        Tl1_out  = Tl1_interp(alpha)
        Tl31_out = Tl31_interp(sigmaf)
        Tl32_out = Tl32_interp(alpha, r)
        Tl33_out = Tl33_interp(alpha, r)
        Tl4_out  = Tl4_interp(alpha, stall)
        Tm1_out  = Tm1_interp(qbar)
        Tm2_out  = Tm2_interp(alpha)
        Tm4_out  = Tm4_interp(sigmaf)
        Tm5_out  = Tm5_interp(sigmae, alpha)
        Tn1_out  = Tn1_interp(beta)
        Tn3_out  = Tn3_interp(r, alpha)
        Tn4_out  = Tn4_interp(alpha, beta)
        #Aerodynamics coefficients
        CD1 = aerocoeff_CD1()
        CD2 = aerocoeff_CD2(TDge_out, TD2_out)
        CD3 = aerocoeff_CD3(TDge_out, TD3_out)
        CD4 = aerocoeff_CD4(beta)
        CC1 = aerocoeff_CC1(TC1_out)
        CC2 = aerocoeff_CC2(sigmar)
        CL1 = aerocoeff_CL1(TLge_out, TL1_out)
        CL2 = aerocoeff_CL2(TLge_out, TL2_out)
        CL3 = aerocoeff_CL3(sigmae)
        CL4 = aerocoeff_CL4(Cw2Va, q)
        CL5 = aerocoeff_CL5(Cw2Va, alphadot)
        Cl1 = aerocoeff_Cl1(beta, Tl1_out)
        Cl2 = aerocoeff_Cl2(Bw2Va, p)
        Cl3 = aerocoeff_Cl3(Bw2Va, r, Tl31_out, Tl32_out, Tl33_out, stall)
        Cl4 = aerocoeff_Cl4(sigmaa_avg, Tl4_out)
        Cl5 = aerocoeff_Cl5(sigmar)
        Cm1 = aerocoeff_Cm1(Tm1_out)
        Cm2 = aerocoeff_Cm2(alpha, Tm2_out)
        Cm3 = aerocoeff_Cm3(Cw2Va, q)
        Cm4 = aerocoeff_Cm4(Tm4_out)
        Cm5 = aerocoeff_Cm5(Cw2Va, alphadot)
        Cm6 = aerocoeff_Cm6(sigmae, Tm5_out)
        Cn1 = aerocoeff_Cn1(Tn1_out)
        Cn2 = aerocoeff_Cn2(Bw2Va, r)
        Cn3 = aerocoeff_Cn3(Bw2Va, Tn3_out)
        Cn4 = aerocoeff_Cn4(sigmaa_avg, Tn4_out)
        Cn5 = aerocoeff_Cn5(sigmar)
        Cn6 = aerocoeff_Cn6()
        #Dynamics
        D          = qbar * SW * (CD1 + CD2 + CD3 + CD4) 
        C          = qbar * SW * (CC1 + CC2)
        L          = SW * (qbar * (CL1 + CL2 + CL3 + CL4) + qbaruw * CL5)
        gx, gy, gz = gravity_body(euler, mass, grav)
        fx         = - D + gx + T
        fy         = C + gy
        fz         = - L + gz
        l          = qbar * SW * BW * (Cl1 + Cl2 + Cl3 + Cl4 + Cl5) + tau
        m          = SW * CW * (qbar * (Cm1 + Cm2 + Cm3 + Cm4) + qbaruw * Cm5 + qbarind * Cm6) + H * r
        n          = SW * BW * (qbar * (Cn1 + Cn2 + Cn3 + Cn4) + qbarind * Cn5 + qbarprop * Cn6) - H * q
        #Trigonometric values
        cos_phi   = np.cos(phi)
        cos_theta = np.cos(theta)
        cos_psi   = np.cos(psi)
        sin_phi   = np.sin(phi)
        sin_theta = np.sin(theta)
        sin_psi   = np.sin(psi)
        tan_theta = sin_theta / cos_theta
        #State derivative respect to time
        xdot_cm        = np.zeros(CM_STATE_LEN) #initialize xdot
        xdot_cm[0:3]   = np.array([[cos_theta * cos_psi, sin_phi * sin_theta * cos_psi - cos_phi * sin_psi, cos_phi * sin_theta * cos_psi + sin_phi * sin_psi], [cos_theta * sin_psi, sin_phi * sin_theta * sin_psi + cos_phi * cos_psi, cos_phi * sin_theta * sin_psi - sin_phi * cos_psi], [- sin_theta, sin_phi * cos_theta, cos_phi * cos_theta]], dtype=float) @ np.array([u, v, w], dtype=float) #position in NED frame (pn, pe, pd)
        xdot_cm[3:6]   = np.array([[1, sin_phi * tan_theta, cos_phi * tan_theta], [0, cos_phi, -sin_phi], [0, sin_phi / cos_theta, cos_phi / cos_theta]], dtype=float) @ np.array([p, q, r], dtype=float) #attitude in vehicle frame (q0dot, q1dot, q2dot, q3dot)
        xdot_cm[6:9]  = np.array([[0, -w, v], [w, 0, -u], [-v, u, 0]], dtype=float) @ np.array([p, q, r]) + np.array([fx, fy, fz], dtype=float) / mass #linear velocity in body frame (udot, vdot, wdot)
        xdot_cm[9:12] = np.array([Gamma1*p*q-Gamma2*q*r, Gamma5*p*r-Gamma6*(p**2-r**2), Gamma7*p*q-Gamma1*q*r], dtype=float) + np.array([[Gamma3, 0, Gamma4], [0, 1/Iyy, 0], [Gamma4, 0, Gamma8]], dtype=float) @ np.array([l, m, n], dtype=float) #angular velocity in body frame (pdot, qdot, rdot)
        #Scalar output function to minimize
        #z_out = abs(xdot_cm[np.ix_(EQ_STATE_IND)])
        z_out = abs(xdot_cm[np.ix_([2,3,4,5,6,7,8,9,10,11])])
        z_out = z_out.sum()
        return z_out

