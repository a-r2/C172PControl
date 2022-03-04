from constants import *
from settings import *
from modules.func.utils import *

from c172p_model import *

import control as ctl
import numpy as np
from pyquaternion import Quaternion
from scipy import interpolate
import sys

''' CONVERSIONS OF ACTUATION VARIABLES '''
def deltaa_to_sigmaa(deltaa):
    #Normalized aileron deflection to physical aileron deflection (SI)
    return min((SIGMAA_RANGE[1] / DELTAA_RANGE[1]) * deltaa, SIGMAA_RANGE[1]) if deltaa >= 0 else max((SIGMAA_RANGE[0] / DELTAA_RANGE[0]) * deltaa, SIGMAA_RANGE[0])

def deltaa_to_sigmala(deltaa):
    #Normalized aileron deflection to physical left aileron deflection (SI)
    return deltaa_to_sigmaa(deltaa)

def deltaa_to_sigmara(deltaa):
    #Normalized aileron deflection to physical right aileron deflection (SI)
    return deltaa_to_sigmaa(-deltaa)

def sigmaa_to_deltaa(sigmaa):
    #Physical aileron deflection (SI) to normalized aileron deflection
    return min((DELTAA_RANGE[1] / SIGMAA_RANGE[1]) * sigmaa, DELTAA_RANGE[1]) if sigmaa >= 0 else max((DELTAA_RANGE[0] / SIGMAA_RANGE[0]) * sigmaa, DELTAA_RANGE[0])

def deltae_to_sigmae(deltae):
    #Normalized elevator deflection to physical elevator deflection (SI)
    return min((SIGMAE_RANGE[1] / DELTAE_RANGE[1]) * deltae, SIGMAE_RANGE[1]) if deltae >= 0 else max((SIGMAE_RANGE[0] / DELTAE_RANGE[0]) * deltae, SIGMAE_RANGE[0])

def sigmae_to_deltae(sigmae):
    #Physical elevator deflection (SI) to normalized elevator deflection
    return min((DELTAE_RANGE[1] / SIGMAE_RANGE[1]) * sigmae, DELTAE_RANGE[1]) if sigmae >= 0 else max((DELTAE_RANGE[0] / SIGMAE_RANGE[0]) * sigmae, DELTAE_RANGE[0])

def deltaf_to_sigmaf(deltaf):
    #Normalized flaps deflection to physical flaps deflection (SI)
    return min((SIGMAF_RANGE[1] / DELTAF_RANGE[1]) * deltaf, SIGMAF_RANGE[1]) if deltaf >= 0 else max((SIGMAF_RANGE[0] / DELTAF_RANGE[0]) * deltaf, SIGMAF_RANGE[0])

def sigmaf_to_deltaf(sigmaf):
    #Physical flaps deflection (SI) to normalized flaps deflection
    return min((DELTAF_RANGE[1] / SIGMAF_RANGE[1]) * sigmaf, DELTAF_RANGE[1]) if sigmaf >= 0 else max((DELTAF_RANGE[0] / SIGMAF_RANGE[0]) * sigmaf, DELTAF_RANGE[0])

def deltar_to_sigmar(deltar):
    #Normalized rudder deflection to physical rudder deflection (SI)
    return min((SIGMAR_RANGE[1] / DELTAR_RANGE[1]) * deltar, SIGMAR_RANGE[1]) if deltar >= 0 else max((SIGMAR_RANGE[0] / DELTAR_RANGE[0]) * deltar, SIGMAR_RANGE[0])

def sigmar_to_deltar(sigmar):
    #Physical rudder deflection (SI) to normalized rudder deflection
    return min((DELTAR_RANGE[1] / SIGMAR_RANGE[1]) * sigmar, DELTAR_RANGE[1]) if sigmar >= 0 else max((DELTAR_RANGE[0] / SIGMAR_RANGE[0]) * sigmar, DELTAR_RANGE[0])

''' PARTIAL DERIVATIVES OF ACTUATION VARIABLES '''
def parder_deltaa_sigmaa_acm(deltaa):
    #Partial derivative of deltaa respect to sigmaa
    return SIGMAA_RANGE[1] / DELTAA_RANGE[1] if deltaa >= 0 else SIGMAA_RANGE[0] / DELTAA_RANGE[0]

def parder_deltaa_sigmala_acm(deltaa):
    #Partial derivative of physical left aileron deflection (SI) respect to normalized aileron deflection
    return (DELTAA_RANGE[1] / SIGMAA_RANGE[1]) * parder_deltaa_sigmaa_acm(deltaa) if deltaa >= 0 else (DELTAA_RANGE[0] / SIGMAA_RANGE[0]) * parder_deltaa_sigmaa_acm(deltaa)

def parder_deltaa_sigmara_acm(deltaa):
    #Partial derivative of physical right aileron deflection (SI) respect to normalized aileron deflection
    return - (DELTAA_RANGE[1] / SIGMAA_RANGE[1]) * parder_deltaa_sigmaa_acm(deltaa) if deltaa >= 0 else - (DELTAA_RANGE[0] / SIGMAA_RANGE[0]) * parder_deltaa_sigmaa_acm(deltaa)

def parder_deltae_sigmae_acm(deltae):
    #Partial derivative of deltae respect to sigmae
    return SIGMAE_RANGE[1] / DELTAE_RANGE[1] if deltae >= 0 else SIGMAE_RANGE[0] / DELTAE_RANGE[0]

def parder_deltar_sigmar_acm(deltar):
    #Partial derivative of deltar respect to sigmar
    return SIGMAR_RANGE[1] / DELTAR_RANGE[1] if deltar >= 0 else SIGMAR_RANGE[0] / DELTAR_RANGE[0]

''' ASSUMPTIONS OF ANALYTIC CONTROL MODELS '''
def assumptions_acm(deltat, pd):
    #Assumptions of analytic control models
    sigmaf  = 0
    deltaf  = 0
    deltam  = 1
    rho     = barometric_density(pd)
    grav    = G0_SI
    stall   = 0
    hmacb   = 2
    #Build assumptions dictionary
    aacm_dict = dict()
    aacm_dict.update(sigmaf = sigmaf)
    aacm_dict.update(deltaf = deltaf)
    aacm_dict.update(deltam = deltam)
    aacm_dict.update(rho = rho)
    aacm_dict.update(grav = grav)
    aacm_dict.update(stall = stall)
    aacm_dict.update(hmacb= hmacb)
    return aacm_dict

''' PHYSICAL VARIABLES OF ANALYTIC CONTROL MODELS '''
def alpha_acm(u, w):
    #Angle of attack (alpha) as defined in the analytic control models
    return np.arctan2(w, u)

def beta_acm(v, Vauw):
    #Side-slip angle (beta) as defined in the analytic control models
    return np.arctan2(v, Vauw)

def J_acm(u, revprop):
    #Advance ratio (J) as defined in the analytic control models
    if revprop == 0:
        return 0
    else:
        return u / (revprop * D_PROP + 1e-6)

def revprop_acm(deltat):
    #Propeller revolutions rate law as defined in the analytic control models
    return (N_MAX - N_MIN) * deltat + N_MIN

def omega_acm(revprop):
    #Propeller angular velocity (omega) as defined in the analytic control models
    return 2 * np.pi * revprop

def H_acm(omega):
    #Propeller angular momentum (H) as defined in the analytic control models
    return I_XX_PROP * omega

def tau_acm(P, omega):
    #Propeller torque (tau) as defined in the analytic control models
    if omega >= 0.01:
        return - P / omega
    else:
        return - 100 * P

def qbar_acm(rho, Va):
    #Dynamic pressure (qbar) as defined in the analytic control models
    return 0.5 * rho * (Va ** 2)

def qbaruw_acm(rho, Vauw):
    #Dynamic pressure from components u and w (qbaruw) as defined in the analytic control models
    return 0.5 * rho * (Vauw ** 2)

def qbarprop_acm(rho, Vprop):
    #Propeller induced dynamic pressure (qbarprop) as defined in the analytic control models
    return 0.5 * rho * (Vprop ** 2)

def qbarind_acm(rho, Vind):
    #Dynamic pressure including propeller induced velocity effect (qbarind) as defined in the analytic control models
    return 0.5 * rho * (Vind ** 2)

def Va_acm(u, v, w):
    #Aerodynamic velocity (Va) as defined in the analytic control models
    return np.sqrt((u ** 2) + (v ** 2) + (w ** 2))

def Vauw_acm(u, w):
    #Aerodynamic velocity from components u and w (Vauw) as defined in the analytic control models
    return np.sqrt((u ** 2) + (w ** 2))

def Vprop_acm(u, Vprop2):
    #Propeller induced velocity (Vprop) as defined in the analytic control models
    if Vprop2 >= 0:
        return 0.5 * (- u + np.sqrt(Vprop2))
    else:
        return 0.5 * (- u - np.sqrt(- Vprop2))

def Vprop2_acm(u, rho, T):
    #Squared propeller induced velocity (Vprop^2) as defined in the analytic control models
    return u * abs(u) + ((2 * T) / (rho * A_PROP))

def Vind_acm(u, Vprop):
    #Velocity including propeller induced velocity (Vind) as defined in the analytic control models
    return u + Vprop

''' PARTIAL DERIVATIVES OF PHYSICAL VARIABLES '''
def parder_pd_J_alcm(u, revprop, parder_pd_revprop):
    #Partial derivative of J respect to pd
    return - u * parder_pd_revprop / ((revprop ** 2) * D_PROP)

def parder_u_J_alcm(u, revprop, parder_u_revprop):
    #Partial derivative of J respect to u
    return 1 / ((revprop ** 2) * D_PROP) - u * parder_u_revprop / ((revprop ** 2) * D_PROP)

def parder_deltat_J_alcm(u, revprop, parder_deltat_revprop):
    #Partial derivative of J respect to deltat
    return - u * parder_deltat_revprop / ((revprop ** 2) * D_PROP)

def parder_u_alpha_alcm(w, Vauw):
    #Partial derivative of alpha respect to u
    return - w / ((Vauw ** 2) + 1e-6)

def parder_w_alpha_alcm(u, Vauw):
    #Partial derivative of alpha respect to w
    return u / ((Vauw ** 2) + 1e-6)

def parder_u_beta_alcm(u, v, Vauw, Va):
    #Partial derivative of beta respect to u
    return - (u * v) / (Vauw * (Va ** 2) + 1e-6)

def parder_v_beta_alcm(Vauw, Va):
    #Partial derivative of beta respect to v
    return Vauw / ((Va ** 2) + 1e-6)

def parder_w_beta_alcm(v, w, Vauw, Va):
    #Partial derivative of beta respect to w
    return - (v * w) / (Vauw * (Va ** 2) + 1e-6)

def parder_u_Bw2Va_alcm(Va, parder_u_Va):
    #Partial derivative of Va respect to u
    return - 0.5 * BW * parder_u_Va / ((Va ** 2) + 1e-6)

def parder_v_Bw2Va_alcm(Va, parder_v_Va):
    #Partial derivative of Va respect to v
    return - 0.5 * BW * parder_v_Va / ((Va ** 2) + 1e-6)

def parder_w_Bw2Va_alcm(Va, parder_w_Va):
    #Partial derivative of Va respect to w
    return - 0.5 * BW * parder_w_Va / ((Va ** 2) + 1e-6)

def parder_u_Cw2Va_alcm(Va, parder_u_Va):
    #Partial derivative of Va respect to u
    return - 0.5 * CW * parder_u_Va / ((Va ** 2) + 1e-6)

def parder_v_Cw2Va_alcm(Va, parder_v_Va):
    #Partial derivative of Va respect to v
    return - 0.5 * CW * parder_v_Va / ((Va ** 2) + 1e-6)

def parder_w_Cw2Va_alcm(Va, parder_w_Va):
    #Partial derivative of Va respect to w
    return - 0.5 * CW * parder_w_Va / ((Va ** 2) + 1e-6)

def parder_pd_revprop_alcm(parder_pd_tau):
    #Partial derivative of revprop respect to pd
    return 2 * np.pi * parder_pd_tau / I_XX_PROP

def parder_u_revprop_alcm(parder_u_tau):
    #Partial derivative of revprop respect to u
    return 2 * np.pi * parder_u_tau / I_XX_PROP

def parder_deltat_revprop_alcm(parder_deltat_tau):
    #Partial derivative of revprop respect to deltat
    return 2 * np.pi * parder_deltat_tau / I_XX_PROP

def parder_deltat_omega_alcm(parder_deltat_revprop):
    #Partial derivative of omega respect to deltat
    return 2 * np.pi * parder_deltat_revprop

def parder_deltat_L_alcm(parder_deltat_omega):
    #Partial derivative of H respect to deltat
    return I_XX_PROP * parder_deltat_omega

def parder_pd_tau_alcm(omega, parder_pd_P):
    #Partial derivative of tau respect to pd
    if omega >= 0.01:
        return - parder_pd_P / omega
    else:
        return - 100 * parder_pd_P

def parder_u_tau_alcm(omega, parder_u_P):
    #Partial derivative of tau respect to u
    if omega >= 0.01:
        return - parder_u_P / omega
    else:
        return - 100 * parder_u_P

def parder_deltat_tau_alcm(omega, P, parder_deltat_omega, parder_deltat_P):
    #Partial derivative of tau respect to deltat
    if omega >= 0.01:
        return (P * parder_deltat_omega - omega * parder_deltat_P) / (omega ** 2)
    else:
        return - 100 * parder_deltat_P

def parder_pd_rho_alcm(pd):
    #Partial derivative of rho respect to pd
    return parder_pd_barometric_density(pd)

def parder_pd_qbar_alcm(Va, parder_pd_rho):
    #Partial derivative of qbar respect to pd
    return 0.5 * parder_pd_rho * (Va ** 2)

def parder_u_qbar_alcm(u, rho):
    #Partial derivative of qbar respect to u
    return rho * u

def parder_v_qbar_alcm(v, rho):
    #Partial derivative of qbar respect to v
    return rho * v

def parder_w_qbar_alcm(w, rho):
    #Partial derivative of qbar respect to w
    return rho * w

def parder_pd_qbarind_alcm(rho, Vind, parder_pd_rho, parder_pd_Vind):
    #Partial derivative of qbarind respect to pd
    return 0.5 * Vind * (parder_pd_rho * Vind + 2 * rho * parder_pd_Vind)

def parder_u_qbarind_alcm(rho, Vind, parder_u_Vind):
    #Partial derivative of qbarind respect to u
    return rho * Vind * parder_u_Vind

def parder_deltat_qbarind_alcm(rho, Vind, parder_deltat_Vind):
    #Partial derivative of qbarind respect to deltat
    return rho * Vind * parder_deltat_Vind

def parder_pd_qbarprop_alcm(rho, Vprop, parder_pd_rho, parder_pd_Vprop):
    #Partial derivative of qbarprop respect to pd
    return 0.5 * Vprop * (parder_pd_rho * Vprop + 2 * rho * parder_pd_Vprop)

def parder_u_qbarprop_alcm(rho, Vprop, parder_u_Vprop):
    #Partial derivative of qbarprop respect to u
    return rho * Vprop * parder_u_Vprop

def parder_deltat_qbarprop_alcm(rho, Vprop, parder_deltat_Vprop):
    #Partial derivative of qbarprop respect to deltat
    return rho * Vprop * parder_deltat_Vprop

def parder_pd_qbaruw_alcm(Vauw, parder_pd_rho):
    #Partial derivative of qbaruw respect to pd
    return 0.5 * parder_pd_rho * (Vauw ** 2)

def parder_u_qbaruw_alcm(parder_u_qbar):
    #Partial derivative of qbaruw respect to u
    return parder_u_qbar

def parder_w_qbaruw_alcm(parder_w_qbar):
    #Partial derivative of qbaruw respect to w
    return parder_w_qbar

def parder_pd_T_alcm(CT, rho, revprop, parder_J_CT, parder_pd_J, parder_pd_rho, parder_pd_revprop):
    #Partial derivative of T respect to pd
    return revprop * (D_PROP ** 4) * (rho * revprop * parder_J_CT * parder_pd_J + CT * revprop * parder_pd_rho + 2 * CT * rho * parder_pd_revprop)

def parder_u_T_alcm(CT, rho, revprop, parder_J_CT, parder_u_J, parder_u_revprop):
    #Partial derivative of T respect to u
    return rho * revprop * (D_PROP ** 4) * (revprop * parder_J_CT * parder_u_J + 2 * CT * parder_u_revprop)

def parder_deltat_T_alcm(CT, rho, revprop, parder_J_CT, parder_deltat_J, parder_deltat_revprop):
    #Partial derivative of T respect to deltat
    return rho * revprop * (D_PROP ** 4) * (revprop * parder_J_CT * parder_deltat_J + 2 * CT * parder_deltat_revprop)

def parder_pd_P_alcm(CP, rho, revprop, parder_J_CP, parder_pd_J, parder_pd_rho, parder_pd_revprop):
    #Partial derivative of P respect to pd
    return (revprop ** 2) * (D_PROP ** 5) * (rho * revprop * parder_J_CP * parder_pd_J + CP * revprop * parder_pd_rho + 3 * CP * rho * parder_pd_revprop)

def parder_u_P_alcm(CP, rho, revprop, parder_J_CP, parder_u_J, parder_u_revprop):
    #Partial derivative of P respect to u
    return rho * (revprop ** 2) * (D_PROP ** 5) * (revprop * parder_J_CP * parder_u_J + 3 * CP * parder_u_revprop)

def parder_deltat_P_alcm(CP, rho, revprop, parder_J_CP, parder_deltat_J, parder_deltat_revprop):
    #Partial derivative of P respect to deltat
    return rho * (revprop ** 2) * (D_PROP ** 5) * (revprop * parder_J_CP * parder_deltat_J + 3 * CP * parder_deltat_revprop)

def parder_u_Va_alcm(u, Va):
    #Partial derivative of Va respect to u
    return u / (Va + 1e-6)

def parder_v_Va_alcm(v, Va):
    #Partial derivative of Va respect to v
    return v / (Va + 1e-6)

def parder_w_Va_alcm(w, Va):
    #Partial derivative of Va respect to w
    return w / (Va + 1e-6)

def parder_pd_Vprop_alcm(u, rho, T, Vprop, Vprop2, parder_pd_rho):
    #Partial derivative of Vprop respect to pd
    if Vprop2 >= 0:
        return - 0.5 * parder_pd_rho * ((T * (rho * A_PROP * u * abs(u) + 2 * T)) / ((rho ** 3) * (A_PROP ** 2) * (Vprop ** 3) + 1e-6))
    else:
        return 0.5 * parder_pd_rho * ((T * (rho * A_PROP * u * abs(u) + 2 * T)) / ((rho ** 3) * (A_PROP ** 2) * (Vprop ** 3) + 1e-6))

def parder_u_Vprop_alcm(u, rho, T, Vprop, Vprop2):
    #Partial derivative of Vprop respect to u
    if Vprop2 >= 0:
        return 0.5 * (1 + (u ** 2) * (rho * A_PROP * u * abs(u) + 2 * T) / (rho * A_PROP * abs(u) * (Vprop ** 3) + 1e-6))
    else:
        return 0.5 * (1 - (u ** 2) * (rho * A_PROP * u * abs(u) + 2 * T) / (rho * A_PROP * abs(u) * (Vprop ** 3) + 1e-6))

def parder_deltat_Vprop_alcm(u, rho, T, Vprop, Vprop2, parder_deltat_T):
    #Partial derivative of Vprop respect to deltat
    if Vprop2 >= 0:
        return 0.5 * parder_deltat_T * (rho * A_PROP * u * abs(u) + 2 * T) / ((rho ** 2) * (A_PROP ** 2) * (Vprop ** 3) + 1e-6)
    else:
        return - 0.5 * parder_deltat_T * (rho * A_PROP * u * abs(u) + 2 * T) / ((rho ** 2) * (A_PROP ** 2) * (Vprop ** 3) + 1e-6)

def parder_pd_Vind_alcm(parder_pd_Vprop):
    #Partial derivative of Vind respect to pd
    return parder_pd_Vprop

def parder_u_Vind_alcm(parder_u_Vprop):
    #Partial derivative of Vind respect to u
    return 1 + parder_u_Vprop

def parder_deltat_Vind_alcm(parder_deltat_Vprop):
    #Partial derivative of Vind respect to deltat
    return parder_deltat_Vprop

''' PARTIAL DERIVATIVES DYNAMIC COEFFICIENTS '''
def parder_u_CD3_alcm(TDge_out, parder_alpha_TD3, parder_u_alpha):
    #Partial derivative of CD3 respect to u
    return TDge_out * parder_alpha_TD3 * parder_u_alpha

def parder_w_CD3_alcm(TDge_out, parder_alpha_TD3, parder_w_alpha):
    #Partial derivative of CD3 respect to w
    return TDge_out * parder_alpha_TD3 * parder_w_alpha

def parder_u_CD4_alcm(beta, parder_u_beta):
    #Partial derivative of CD4 respect to u
    return 0 if beta == 0 else 0.15 * (beta / abs(beta)) * parder_u_beta

def parder_v_CD4_alcm(beta, parder_v_beta):
    #Partial derivative of CD4 respect to v
    return 0 if beta == 0 else 0.15 * (beta / abs(beta)) * parder_v_beta

def parder_w_CD4_alcm(beta, parder_w_beta):
    #Partial derivative of CD4 respect to w
    return 0 if beta == 0 else 0.15 * (beta / abs(beta)) * parder_w_beta

def parder_u_CC1_alcm(parder_beta_TC1, parder_u_beta):
    #Partial derivative of CC1 respect to u
    return parder_beta_TC1 * parder_u_beta

def parder_v_CC1_alcm(parder_beta_TC1, parder_v_beta):
    #Partial derivative of CC1 respect to v
    return parder_beta_TC1 * parder_v_beta

def parder_w_CC1_alcm(parder_beta_TC1, parder_w_beta):
    #Partial derivative of CC1 respect to w
    return parder_beta_TC1 * parder_w_beta

def parder_deltar_CC2_alcm(parder_deltar_sigmar):
    #Partial derivative of CC2 respect to deltar
    return 0.15 * parder_deltar_sigmar

def parder_u_CL1_alcm(TLge_out, parder_alpha_TL1, parder_u_alpha):
    #Partial derivative of CL1 respect to u
    return TLge_out * parder_alpha_TL1 * parder_u_alpha

def parder_w_CL1_alcm(TLge_out, parder_alpha_TL1, parder_w_alpha):
    #Partial derivative of CL1 respect to w
    return TLge_out * parder_alpha_TL1 * parder_w_alpha

def parder_deltae_CL3_alcm(parder_deltae_sigmae):
    #Partial derivative of CL3 respect to deltae
    return 0.43 * parder_deltae_sigmae

def parder_u_CL4_alcm(q, parder_u_Cw2Va):
    #Partial derivative of CL4 respect to u
    return  3.9 * q * parder_u_Cw2Va

def parder_v_CL4_alcm(q, parder_v_Cw2Va):
    #Partial derivative of CL4 respect to v
    return  3.9 * q * parder_v_Cw2Va

def parder_w_CL4_alcm(q, parder_w_Cw2Va):
    #Partial derivative of CL4 respect to w
    return  3.9 * q * parder_w_Cw2Va

def parder_q_CL4_alcm(Cw2Va):
    #Partial derivative of CL4 respect to q
    return 3.9 * Cw2Va

def parder_u_CL5_alcm(alphadot, parder_u_Cw2Va):
    #Partial derivative of CL5 respect to u
    return  1.7 * alphadot  * parder_u_Cw2Va

def parder_v_CL5_alcm(alphadot, parder_v_Cw2Va):
    #Partial derivative of CL5 respect to v
    return  1.7 * alphadot  * parder_v_Cw2Va

def parder_w_CL5_alcm(alphadot, parder_w_Cw2Va):
    #Partial derivative of CL5 respect to w
    return  1.7 * alphadot  * parder_w_Cw2Va

def parder_u_Cl1_alcm(beta, Tl1_out, parder_u_beta, parder_alpha_Tl1, parder_u_alpha):
    #Partial derivative of Cl1 respect to u
    return - 0.092 * (parder_u_beta * Tl1_out + beta * parder_alpha_Tl1 * parder_u_alpha)

def parder_v_Cl1_alcm(Tl1_out, parder_v_beta):
    #Partial derivative of Cl1 respect to v
    return - 0.092 * parder_v_beta * Tl1_out

def parder_w_Cl1_alcm(beta, Tl1_out, parder_w_beta, parder_alpha_Tl1, parder_w_alpha):
    #Partial derivative of Cl1 respect to w
    return - 0.092 * (parder_w_beta * Tl1_out + beta * parder_alpha_Tl1 * parder_w_alpha)

def parder_u_Cl2_alcm(p, parder_u_Bw2Va):
    #Partial derivative of Cl2 respect to u
    return - 0.484 * parder_u_Bw2Va * p

def parder_v_Cl2_alcm(p, parder_v_Bw2Va):
    #Partial derivative of Cl2 respect to v
    return - 0.484 * parder_v_Bw2Va * p

def parder_w_Cl2(p, parder_w_Bw2Va):
    #Partial derivative of Cl2 respect to w
    return - 0.484 * parder_w_Bw2Va * p

def parder_p_Cl2(Bw2Va):
    #Partial derivative of Cl2 respect to w
    return - 0.484 * Bw2Va

def parder_u_Cl3(Bw2Va, r, Tl31_out, Tl32_out, Tl33_out, stall, parder_u_Bw2Va, parder_alpha_Tl32, parder_alpha_Tl33, parder_u_alpha):
    #Partial derivative of Cl3 respect to u
    if stall:
        return r * Tl31_out * (parder_u_Bw2Va * Tl32_out + Bw2Va * parder_alpha_Tl32 * parder_u_alpha)

    else:
        return r * Tl31_out * (parder_u_Bw2Va * Tl33_out + Bw2Va * parder_alpha_Tl33 * parder_u_alpha)

def parder_v_Cl3(r, Tl31_out, Tl32_out, Tl33_out, stall, parder_v_Bw2Va):
    #Partial derivative of Cl3 respect to v
    if stall:
        return r * Tl31_out * parder_v_Bw2Va * Tl32_out

    else:
        return r * Tl31_out * parder_v_Bw2Va * Tl33_out

def parder_w_Cl3_alcm(Bw2Va, r, Tl31_out, Tl32_out, Tl33_out, stall, parder_w_Bw2Va, parder_alpha_Tl32, parder_alpha_Tl33, parder_w_alpha):
    #Partial derivative of Cl3 respect to w
    if stall:
        return r * Tl31_out * (parder_w_Bw2Va * Tl32_out + Bw2Va * parder_alpha_Tl32 * parder_w_alpha)

    else:
        return r * Tl31_out * (parder_w_Bw2Va * Tl33_out + Bw2Va * parder_alpha_Tl33 * parder_w_alpha)

def parder_r_Cl3_alcm(Bw2Va, r, Tl31_out, Tl32_out, Tl33_out, stall, parder_r_Tl32, parder_r_Tl33):
    #Partial derivative of Cl3 respect to r
    if stall:
        return Bw2Va * Tl31_out * (Tl32_out + r * parder_r_Tl32)
    else:
        return Bw2Va * Tl31_out * (Tl33_out + r * parder_r_Tl33)

def parder_u_Cl4_alcm(sigmaa_avg, parder_alpha_Tl4, parder_u_alpha):
    #Partial derivative of Cl4 respect to u
    return 0.229 * sigmaa_avg * parder_alpha_Tl4 * parder_u_alpha

def parder_w_Cl4_alcm(sigmaa_avg, parder_alpha_Tl4, parder_w_alpha):
    #Partial derivative of Cl4 respect to w
    return 0.229 * sigmaa_avg * parder_alpha_Tl4 * parder_w_alpha

def parder_deltaa_Cl4_alcm(Tl4_out, parder_deltaa_sigmaa, parder_deltaa_sigmala, parder_deltaa_sigmara):
    #Partial derivative of Cl4 respect to deltaa
    return 0.1145 * parder_deltaa_sigmaa * (parder_deltaa_sigmala - parder_deltaa_sigmara) * Tl4_out

def parder_deltar_Cl5_alcm(parder_deltar_sigmar):
    #Partial derivative of Cl5 respect to deltar
    return 0.0147 * parder_deltar_sigmar

def parder_pd_Cm1_alcm(parder_qbar_Tm1, parder_pd_qbar):
    #Partial derivative of Cm1 respect to pd
    return  parder_qbar_Tm1 * parder_pd_qbar

def parder_u_Cm1_alcm(parder_qbar_Tm1, parder_u_qbar):
    #Partial derivative of Cm1 respect to u
    return  parder_qbar_Tm1 * parder_u_qbar

def parder_v_Cm1_alcm(parder_qbar_Tm1, parder_v_qbar):
    #Partial derivative of Cm1 respect to v
    return  parder_qbar_Tm1 * parder_v_qbar

def parder_w_Cm1_alcm(parder_qbar_Tm1, parder_w_qbar):
    #Partial derivative of Cm1 respect to w
    return  parder_qbar_Tm1 * parder_w_qbar

def parder_u_Cm2_alcm(alpha, Tm2_out, parder_alpha_Tm2, parder_u_alpha):
    #Partial derivative of Cm2 respect to u
    return - 1.8 * parder_u_alpha * (np.cos(alpha) * Tm2_out + np.sin(alpha) * parder_alpha_Tm2)

def parder_w_Cm2_alcm(alpha, Tm2_out, parder_alpha_Tm2, parder_w_alpha):
    #Partial derivative of Cm2 respect to w
    return - 1.8 * parder_w_alpha * (np.cos(alpha) * Tm2_out + np.sin(alpha) * parder_alpha_Tm2)

def parder_u_Cm3_alcm(q, parder_u_Cw2Va):
    #Partial derivative of Cm3 respect to u
    return - 12.4 * q * parder_u_Cw2Va  

def parder_v_Cm3_alcm(q, parder_v_Cw2Va):
    #Partial derivative of Cm3 respect to v
    return - 12.4 * q * parder_v_Cw2Va  

def parder_w_Cm3_alcm(q, parder_w_Cw2Va):
    #Partial derivative of Cm3 respect to w
    return - 12.4 * q * parder_w_Cw2Va  

def parder_q_Cm3_alcm(Cw2Va):
    #Partial derivative of Cm3 respect to q
    return - 12.4 * Cw2Va

def parder_u_Cm5_alcm(alphadot, parder_u_Cw2Va):
    #Partial derivative of Cm5 respect to u
    return - 7.27 * alphadot  * parder_u_Cw2Va  

def parder_v_Cm5_alcm(alphadot, parder_v_Cw2Va):
    #Partial derivative of Cm5 respect to v
    return - 7.27 * alphadot * parder_v_Cw2Va  

def parder_w_Cm5_alcm(alphadot, parder_w_Cw2Va):
    #Partial derivative of Cm5 respect to w
    return - 7.27  * alphadot * parder_w_Cw2Va  

def parder_u_Cm6_alcm(sigmae, parder_alpha_Tm5, parder_u_alpha):
    #Partial derivative of Cm6 respect to u
    return - 1.28 * sigmae * parder_alpha_Tm5 * parder_u_alpha

def parder_w_Cm6_alcm(sigmae, parder_alpha_Tm5, parder_w_alpha):
    #Partial derivative of Cm6 respect to w
    return - 1.28 * sigmae * parder_alpha_Tm5 * parder_w_alpha

def parder_deltae_Cm6_alcm(sigmae, alpha, Tm5_out, parder_deltae_sigmae, parder_sigmae_Tm5):
    #Partial derivative of Cm6 respect to deltae
    return - 1.28 * parder_deltae_sigmae * (Tm5_out + sigmae * parder_sigmae_Tm5)

def parder_u_Cn1_alcm(parder_beta_Tn1, parder_u_beta):
    #Partial derivative of Cn1 respect to u 
    return parder_beta_Tn1 * parder_u_beta

def parder_v_Cn1_alcm(parder_beta_Tn1, parder_v_beta):
    #Partial derivative of Cn1 respect to v 
    return parder_beta_Tn1 * parder_v_beta

def parder_w_Cn1_alcm(parder_beta_Tn1, parder_w_beta):
    #Partial derivative of Cn1 respect to w 
    return parder_beta_Tn1 * parder_w_beta

def parder_u_Cn2_alcm(r, parder_u_Bw2Va):
    #Partial derivative of Cn2 respect to u
    return - 0.0937 * r * parder_u_Bw2Va  

def parder_v_Cn2_alcm(r, parder_v_Bw2Va):
    #Partial derivative of Cn2 respect to v
    return - 0.0937 * r * parder_v_Bw2Va  

def parder_w_Cn2_alcm(r, parder_w_Bw2Va):
    #Partial derivative of Cn2 respect to w
    return - 0.0937  * r * parder_w_Bw2Va  

def parder_r_Cn2_alcm(Bw2Va):
    #Partial derivative of Cn2 respect to w
    return - 0.0937  * Bw2Va  

def parder_u_Cn3_alcm(Bw2Va, Tn3_out, parder_u_Bw2Va, parder_alpha_Tn3, parder_u_alpha):
    #Partial derivative of Cn3 respect to u
    return parder_u_Bw2Va * Tn3_out + Bw2Va * parder_alpha_Tn3 * parder_u_alpha

def parder_v_Cn3_alcm(Tn3_out, parder_v_Bw2Va):
    #Partial derivative of Cn3 respect to v
    return parder_v_Bw2Va * Tn3_out

def parder_w_Cn3_alcm(Bw2Va, Tn3_out, parder_w_Bw2Va, parder_alpha_Tn3, parder_w_alpha):
    #Partial derivative of Cn3 respect to w
    return parder_w_Bw2Va * Tn3_out + Bw2Va * parder_alpha_Tn3 * parder_w_alpha

def parder_r_Cn3_alcm(Bw2Va, parder_r_Tn3):
    #Partial derivative of Cn3 respect to r
    return Bw2Va * parder_r_Tn3

def parder_u_Cn4_alcm(sigmaa_avg, parder_alpha_Tn4, parder_beta_Tn4, parder_u_alpha, parder_u_beta):
    #Partial derivative of Cn4 respect to u
    return sigmaa_avg * (parder_alpha_Tn4 * parder_u_alpha + parder_beta_Tn4 * parder_u_beta)

def parder_v_Cn4_alcm(sigmaa_avg, parder_beta_Tn4, parder_v_beta):
    #Partial derivative of Cn4 respect to v
    return sigmaa_avg * parder_beta_Tn4 * parder_v_beta

def parder_w_Cn4_alcm(sigmaa_avg, parder_alpha_Tn4, parder_beta_Tn4, parder_w_alpha, parder_w_beta):
    #Partial derivative of Cn4 respect to w
    return sigmaa_avg * (parder_alpha_Tn4 * parder_w_alpha + parder_beta_Tn4 * parder_w_beta)

def parder_deltaa_Cn4_alcm(Tn4_out, parder_deltaa_sigmaa, parder_deltaa_sigmala, parder_deltaa_sigmara):
    #Partial derivative of Cn4 respect to deltaa
    return parder_deltaa_sigmaa * (parder_deltaa_sigmala - parder_deltaa_sigmara) * Tn4_out

def parder_deltar_Cn5_alcm(parder_deltar_sigmar):
    #Partial derivative of Cn5 respect to deltar
    return - 0.0645 * parder_deltar_sigmar

''' BASE CONTROL MODEL '''
class BaseControlModel():
    #Control model base class

    def __init__(self):
        self.t          = TELEM_WAIT
        self.dt         = 1 / CM_HZ
        self.inputs_str = CM_INPUT_STR
        self.states_str = CM_STATE_STR
        self.csvdata    = np.zeros(CM_STATE_LEN + 1) #array for storing csv actuation

''' ANALYTIC LINEAR CONTROL MODEL '''
class ALCM(BaseControlModel):
    #Analytic Linear Control Model (ALCM)

    def _init(self):
        #Initialize dictionaries
        self.cm_dict   = {} #control model dictionary
        self.phys_dict = {} #physical variables dictionary
        #Create states and inputs error strings tuple
        self.inputs_er_str = [item + '_er' for item in self.inputs_str]
        self.states_er_str = [item + '_er' for item in self.states_str]
        #Initialize ALCM
        x_cm       = np.zeros(CM_STATE_LEN) #state
        x_eq_cm    = np.zeros(CM_STATE_LEN) #equilibrium point state
        x_sp_cm    = np.zeros(CM_STATE_LEN) #setpoint state
        x_er_cm    = np.zeros(CM_STATE_LEN) #state error
        u_cm       = np.zeros(CM_INPUT_LEN) #input
        u_eq_cm    = np.zeros(CM_INPUT_LEN) #equilibrium point input
        u_sp_cm    = np.zeros(CM_INPUT_LEN) #setpoint input
        u_er_cm    = np.zeros(CM_INPUT_LEN) #input error
        A          = np.zeros((CM_STATE_LEN, CM_STATE_LEN)) #state-state jacobian
        B          = np.zeros((CM_STATE_LEN, CM_INPUT_LEN)) #state-input jacobian
        C          = np.eye(CM_STATE_LEN) #output-state jacobian (y = x)
        D          = np.zeros((CM_STATE_LEN, CM_INPUT_LEN)) #output-input jacobian
        #Update control model dictionary
        self.cm_dict.update(x_cm = x_cm)
        self.cm_dict.update(x_eq_cm = x_eq_cm)
        self.cm_dict.update(x_sp_cm = x_sp_cm)
        self.cm_dict.update(x_er_cm = x_er_cm)
        self.cm_dict.update(u_cm = u_cm)
        self.cm_dict.update(u_eq_cm = u_eq_cm)
        self.cm_dict.update(u_sp_cm = u_sp_cm)
        self.cm_dict.update(u_er_cm = u_er_cm)
        self.cm_dict.update(A = A)
        self.cm_dict.update(B = B)
        self.cm_dict.update(C = C)
        self.cm_dict.update(D = D)
        self.cm_dict.update(prev_parder_pd_revprop_eq = 0.01)
        self.cm_dict.update(prev_parder_u_revprop_eq = 0.01)
        self.cm_dict.update(prev_parder_deltat_revprop_eq  = 0.01)
        self.cm_dict.update(prev_parder_pd_tau_eq = 0.01)
        self.cm_dict.update(prev_parder_u_tau_eq = 0.01)
        self.cm_dict.update(prev_parder_deltat_tau_eq = 0.01)

    def run(self, cm2act_in, cm2csv_in, eq2cm_out, rx2cm_out, sp2cm_out, event_start, event_end):
        self._init() #initialize ALCM
        event_start.wait() #wait for simulation start event
        while True:
            try:
                if event_end.is_set():
                    #Close pipes
                    cm2act_in.close()
                    cm2csv_in.close()
                    eq2cm_out.close()
                    rx2cm_out.close()
                    sp2cm_out.close()
                    break
                else:
                    rxdata = rx2cm_out.recv() #receive RX telemetry
                    if (np.any(rxdata[:,0] >= self.t)):
                        spdata = sp2cm_out.recv() #receive setpoint
                        eqdata = eq2cm_out.recv() #receive equilibrium point 
                        self._preprocess(rxdata, eqdata, spdata) #preprocess piped-in data
                        self._build() #build ALCM model
                        actdata = self._build_pipe_data() #build pipe data
                        self._pipe(cm2act_in, cm2csv_in, actdata) #pipe out to actuation and CSV logging
                        self.t += self.dt
            except:
                raise RuntimeError('.'.join((__name__, sys._getframe().f_code.co_name)))

    def _preprocess(self, rxdata, eqdata, spdata):
        i      = np.where(rxdata[:,0] >= self.t)[0][0] #first RX telemetry frame index
        rxdata = rxdata[i,:] #first RX telemetry frame
        rxtime = rxdata[0]
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
        #Equilibrium point
        pn_eq     = eqdata[0]
        pe_eq     = eqdata[1]
        pd_eq     = eqdata[2]
        phi_eq    = eqdata[3]
        theta_eq  = eqdata[4]
        psi_eq    = eqdata[5]
        u_eq      = eqdata[6]
        v_eq      = eqdata[7]
        w_eq      = eqdata[8]
        p_eq      = eqdata[9]
        q_eq      = eqdata[10]
        r_eq      = eqdata[11]
        deltaa_eq = eqdata[12]
        deltae_eq = eqdata[13]
        deltaf_eq = eqdata[14]
        deltar_eq = eqdata[15]
        deltat_eq = eqdata[16]
        deltam_eq = eqdata[17]
        #Setpoint
        pn_sp     = spdata[0]
        pe_sp     = spdata[1]
        pd_sp     = spdata[2]
        phi_eq    = spdata[3]
        theta_eq  = spdata[4]
        psi_eq    = spdata[5]
        u_sp      = spdata[6]
        v_sp      = spdata[7]
        w_sp      = spdata[8]
        p_sp      = spdata[9]
        q_sp      = spdata[10]
        r_sp      = spdata[11]
        deltaa_sp = spdata[12]
        deltae_sp = spdata[13]
        deltaf_sp = spdata[14]
        deltar_sp = spdata[15]
        deltat_sp = spdata[16]
        deltam_sp = spdata[17]
        #Conversions
        pn             = lat_delta
        pe             = long_delta
        pd             = - h_qfe
        deltaa         = deltaa_avg_acm(deltala, deltara) 
        euler          = np.array([phi, theta, psi], dtype=float)
        #Assumptions of analytic control models
        aacm_dict = assumptions_acm(deltat, pd)
        sigmaf    = aacm_dict['sigmaf']
        deltaf    = aacm_dict['deltaf']
        deltam    = aacm_dict['deltam']
        rho       = aacm_dict['rho']
        grav      = aacm_dict['grav']
        stall     = aacm_dict['stall']
        hmacb     = aacm_dict['hmacb']
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
        #State equilibrium point
        x_eq_cm = np.array(eqdata[:CM_STATE_LEN])
        #State error
        x_er_cm = x_cm - x_eq_cm
        #State setpoint
        ##x_sp_cm = np.array(spdata[:CM_STATE_LEN])
        x_sp_cm = x_eq_cm
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
        #Input equilibrium point
        u_eq_cm = np.array(eqdata[CM_STATE_LEN:])
        #Input error
        u_er_cm = u_cm - u_eq_cm
        #Input setpoint
        #u_sp_cm = np.array(spdata[CM_STATE_LEN:])
        u_sp_cm = u_eq_cm
        print('$$$$$$$$$$$$$$$$$$$$$$$$$$ CONTROL MODEL $$$$$$$$$$$$$$$$$$$$$$$$$$')
        print(' '.join(('x_cm:', str(x_cm))))
        print(' '.join(('x_eq_cm:', str(x_eq_cm))))
        print(' '.join(('x_er_cm:', str(x_er_cm))))
        print(' '.join(('x_sp_cm:', str(x_sp_cm))))
        print(' '.join(('u_cm:', str(u_cm))))
        print(' '.join(('u_eq_cm:', str(u_eq_cm))))
        print(' '.join(('u_er_cm:', str(u_er_cm))))
        print(' '.join(('u_sp_cm:', str(u_sp_cm))))
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
        #Physical variables evaluated at the equilibrium point 
        sigmala_eq    = deltaa_to_sigmala(deltaa_eq)
        sigmara_eq    = deltaa_to_sigmara(deltaa_eq)
        sigmaa_avg_eq = sigmaa_avg_acm(sigmala_eq, sigmara_eq)
        sigmae_eq     = deltae_to_sigmae(deltae_eq)
        sigmaf_eq     = deltaf_to_sigmaf(deltaf_eq)
        sigmar_eq     = deltar_to_sigmar(deltar_eq)
        J_eq          = J_acm(u_eq, revprop)
        TT_eq         = TT_interp(J_eq)
        CT_eq         = TT_eq
        TP_eq         = TP_interp(J_eq)
        CP_eq         = TP_eq
        T_eq          = thrust_eng(CT_eq, rho, revprop)
        P_eq          = power_eng(CP_eq, rho, revprop)
        Va_eq         = Va_acm(u_eq, v_eq, w_eq)
        Vauw_eq       = Vauw_acm(u_eq, w_eq)
        Bw2Va_eq      = BW / (2 * (Va_eq + 1e-6))
        Cw2Va_eq      = CW / (2 * (Va_eq + 1e-6))
        Vprop2_eq     = Vprop2_acm(u_eq, rho, T_eq)
        Vprop_eq      = Vprop_acm(u_eq, Vprop2_eq)
        Vind_eq       = Vind_acm(u_eq, Vprop_eq)
        alpha_eq      = alpha_acm(u_eq, w_eq)
        beta_eq       = beta_acm(v_eq, Vauw_eq)
        qbar_eq       = qbar_acm(rho, Va_eq)
        qbaruw_eq     = qbaruw_acm(rho, Vauw_eq)
        qbarprop_eq   = qbarprop_acm(rho, Vprop_eq)
        qbarind_eq    = qbarind_acm(rho, Vind_eq)
        omega_eq      = omega_acm(revprop)
        H_eq          = H_acm(omega_eq)
        tau_eq        = tau_acm(P_eq, omega_eq)
        #Aerodynamics tables evaluated at the equilibrium point
        TDge_eq = TDge_interp(hmacb)
        TD2_eq  = TD2_interp(sigmaf_eq)
        TD3_eq  = TD3_interp(alpha_eq, sigmaf_eq)
        TC1_eq  = TC1_interp(beta_eq, sigmaf_eq)
        TLge_eq = TLge_interp(hmacb)
        TL1_eq  = TL1_interp(alpha_eq, stall)
        TL2_eq  = TL2_interp(sigmaf_eq)
        Tl1_eq  = Tl1_interp(alpha_eq)
        Tl31_eq = Tl31_interp(sigmaf_eq)
        Tl32_eq = Tl32_interp(alpha_eq, r_eq)
        Tl33_eq = Tl33_interp(alpha_eq, r_eq)
        Tl4_eq  = Tl4_interp(alpha_eq, stall)
        Tm1_eq  = Tm1_interp(qbar_eq)
        Tm2_eq  = Tm2_interp(alpha_eq)
        Tm4_eq  = Tm4_interp(sigmaf_eq)
        Tm5_eq  = Tm5_interp(sigmae_eq, alpha_eq)
        Tn1_eq  = Tn1_interp(beta_eq)
        Tn3_eq  = Tn3_interp(r_eq, alpha_eq)
        Tn4_eq  = Tn4_interp(alpha_eq, beta_eq)
        #Aerodynamics coefficients evaluated at the equilibrium
        CD1_eq = aerocoeff_CD1()
        CD2_eq = aerocoeff_CD2(TDge_eq, TD2_eq)
        CD3_eq = aerocoeff_CD3(TDge_eq, TD3_eq)
        CD4_eq = aerocoeff_CD4(beta_eq)
        CC1_eq = aerocoeff_CC1(TC1_eq)
        CC2_eq = aerocoeff_CC2(sigmar_eq)
        CL1_eq = aerocoeff_CL1(TLge_eq, TL1_eq)
        CL2_eq = aerocoeff_CL2(TLge_eq, TL2_eq)
        CL3_eq = aerocoeff_CL3(sigmae_eq)
        CL4_eq = aerocoeff_CL4(Cw2Va_eq, q_eq)
        CL5_eq = aerocoeff_CL5(Cw2Va_eq, alphadot)
        Cl1_eq = aerocoeff_Cl1(beta_eq, Tl1_eq)
        Cl2_eq = aerocoeff_Cl2(Bw2Va_eq, p_eq)
        Cl3_eq = aerocoeff_Cl3(Bw2Va_eq, r_eq, Tl31_eq, Tl32_eq, Tl33_eq, stall)
        Cl4_eq = aerocoeff_Cl4(sigmaa_avg_eq, Tl4_eq)
        Cl5_eq = aerocoeff_Cl5(sigmar_eq)
        Cm1_eq = aerocoeff_Cm1(Tm1_eq)
        Cm2_eq = aerocoeff_Cm2(alpha_eq, Tm2_eq)
        Cm3_eq = aerocoeff_Cm3(Cw2Va_eq, q_eq)
        Cm4_eq = aerocoeff_Cm4(Tm4_eq)
        Cm5_eq = aerocoeff_Cm5(Cw2Va_eq, alphadot)
        Cm6_eq = aerocoeff_Cm6(sigmae_eq, Tm5_eq)
        Cn1_eq = aerocoeff_Cn1(Tn1_eq)
        Cn2_eq = aerocoeff_Cn2(Bw2Va_eq, r_eq)
        Cn3_eq = aerocoeff_Cn3(Bw2Va_eq, Tn3_eq)
        Cn4_eq = aerocoeff_Cn4(sigmaa_avg_eq, Tn4_eq)
        Cn5_eq = aerocoeff_Cn5(sigmar_eq)
        Cn6_eq = aerocoeff_Cn6()
        #Update control model dictionary
        self.cm_dict.update(x_cm = x_cm)
        self.cm_dict.update(x_eq_cm = x_eq_cm)
        self.cm_dict.update(x_er_cm = x_er_cm)
        self.cm_dict.update(x_sp_cm = x_sp_cm)
        self.cm_dict.update(u_cm = u_cm)
        self.cm_dict.update(u_eq_cm = u_eq_cm)
        self.cm_dict.update(u_er_cm = u_er_cm)
        self.cm_dict.update(u_sp_cm = u_sp_cm)
        #Update physical variables dictionary
        self.phys_dict.update(rxtime = rxtime)
        self.phys_dict.update(phi = phi)
        self.phys_dict.update(theta = theta)
        self.phys_dict.update(psi = psi)
        self.phys_dict.update(alphadot = alphadot)
        self.phys_dict.update(sigmara = sigmara)
        self.phys_dict.update(sigmara_eq = sigmara_eq)
        self.phys_dict.update(deltara = deltara)
        self.phys_dict.update(sigmala = sigmala)
        self.phys_dict.update(sigmala_eq = sigmala_eq)
        self.phys_dict.update(sigmaa_avg_eq = sigmaa_avg_eq)
        self.phys_dict.update(deltala = deltala)
        self.phys_dict.update(deltaa = deltaa)
        self.phys_dict.update(sigmae = sigmae)
        self.phys_dict.update(sigmae_eq = sigmae_eq)
        self.phys_dict.update(deltae = deltae)
        self.phys_dict.update(sigmar = sigmar)
        self.phys_dict.update(sigmar_eq = sigmar_eq)
        self.phys_dict.update(deltar = deltar)
        self.phys_dict.update(deltat = deltat)
        self.phys_dict.update(hmacb = hmacb)
        self.phys_dict.update(Ixx = Ixx)
        self.phys_dict.update(Ixy = Ixy)
        self.phys_dict.update(Ixz = Ixz)
        self.phys_dict.update(Iyy = Iyy)
        self.phys_dict.update(Iyz = Iyz)
        self.phys_dict.update(Izz = Izz)
        self.phys_dict.update(mass = mass)
        self.phys_dict.update(Gamma = Gamma)
        self.phys_dict.update(Gamma1 = Gamma1)
        self.phys_dict.update(Gamma2 = Gamma2)
        self.phys_dict.update(Gamma3 = Gamma3)
        self.phys_dict.update(Gamma4 = Gamma4)
        self.phys_dict.update(Gamma5 = Gamma5)
        self.phys_dict.update(Gamma6 = Gamma6)
        self.phys_dict.update(Gamma7 = Gamma7)
        self.phys_dict.update(Gamma8 = Gamma8)
        self.phys_dict.update(J_eq = J_eq)
        self.phys_dict.update(T_eq = T_eq)
        self.phys_dict.update(P_eq = P_eq)
        self.phys_dict.update(Vauw_eq = Vauw_eq)
        self.phys_dict.update(Va_eq = Va_eq)
        self.phys_dict.update(Bw2Va_eq = Bw2Va_eq)
        self.phys_dict.update(Cw2Va_eq = Cw2Va_eq)
        self.phys_dict.update(Vprop_eq = Vprop_eq)
        self.phys_dict.update(Vprop2_eq = Vprop2_eq)
        self.phys_dict.update(Vind_eq = Vind_eq)
        self.phys_dict.update(alpha_eq = alpha_eq)
        self.phys_dict.update(beta_eq = beta_eq)
        self.phys_dict.update(qbar_eq = qbar_eq)
        self.phys_dict.update(qbaruw_eq = qbaruw_eq)
        self.phys_dict.update(qbarind_eq = qbarind_eq)
        self.phys_dict.update(qbarprop_eq = qbarprop_eq)
        self.phys_dict.update(sigmaf = sigmaf)
        self.phys_dict.update(sigmaf_eq = sigmaf_eq)
        self.phys_dict.update(deltaf = deltaf)
        self.phys_dict.update(deltam = deltam)
        self.phys_dict.update(revprop = revprop)
        self.phys_dict.update(rho = rho)
        self.phys_dict.update(grav = grav)
        self.phys_dict.update(stall = stall)
        self.phys_dict.update(omega_eq = omega_eq)
        self.phys_dict.update(H_eq = H_eq)
        self.phys_dict.update(tau_eq = tau_eq)
        self.phys_dict.update(CT_eq = CT_eq)
        self.phys_dict.update(CP_eq = CP_eq)
        self.phys_dict.update(TDge_eq = TDge_eq)
        self.phys_dict.update(TD2_eq = TD2_eq)
        self.phys_dict.update(TD3_eq = TD3_eq)
        self.phys_dict.update(TC1_eq = TC1_eq)
        self.phys_dict.update(TLge_eq = TLge_eq)
        self.phys_dict.update(TL1_eq = TL1_eq)
        self.phys_dict.update(TL2_eq = TL2_eq)
        self.phys_dict.update(Tl1_eq = Tl1_eq)
        self.phys_dict.update(Tl31_eq = Tl31_eq)
        self.phys_dict.update(Tl32_eq = Tl32_eq)
        self.phys_dict.update(Tl33_eq = Tl33_eq)
        self.phys_dict.update(Tl4_eq = Tl4_eq)
        self.phys_dict.update(Tm1_eq = Tm1_eq)
        self.phys_dict.update(Tm2_eq = Tm2_eq)
        self.phys_dict.update(Tm4_eq = Tm4_eq)
        self.phys_dict.update(Tm5_eq = Tm5_eq)
        self.phys_dict.update(Tn1_eq = Tn1_eq)
        self.phys_dict.update(Tn3_eq = Tn3_eq)
        self.phys_dict.update(Tn4_eq = Tn4_eq)
        self.phys_dict.update(CD1_eq = CD1_eq)
        self.phys_dict.update(CD2_eq = CD2_eq)
        self.phys_dict.update(CD3_eq = CD3_eq)
        self.phys_dict.update(CD4_eq = CD4_eq)
        self.phys_dict.update(CC1_eq = CC1_eq)
        self.phys_dict.update(CC2_eq = CC2_eq)
        self.phys_dict.update(CL1_eq = CL1_eq)
        self.phys_dict.update(CL2_eq = CL2_eq)
        self.phys_dict.update(CL3_eq = CL3_eq)
        self.phys_dict.update(CL4_eq = CL4_eq)
        self.phys_dict.update(CL5_eq = CL5_eq)
        self.phys_dict.update(Cl1_eq = Cl1_eq)
        self.phys_dict.update(Cl2_eq = Cl2_eq)
        self.phys_dict.update(Cl3_eq = Cl3_eq)
        self.phys_dict.update(Cl4_eq = Cl4_eq)
        self.phys_dict.update(Cl5_eq = Cl5_eq)
        self.phys_dict.update(Cm1_eq = Cm1_eq)
        self.phys_dict.update(Cm2_eq = Cm2_eq)
        self.phys_dict.update(Cm3_eq = Cm3_eq)
        self.phys_dict.update(Cm4_eq = Cm4_eq)
        self.phys_dict.update(Cm5_eq = Cm5_eq)
        self.phys_dict.update(Cm6_eq = Cm6_eq)
        self.phys_dict.update(Cn1_eq = Cn1_eq)
        self.phys_dict.update(Cn2_eq = Cn2_eq)
        self.phys_dict.update(Cn3_eq = Cn3_eq)
        self.phys_dict.update(Cn4_eq = Cn4_eq)
        self.phys_dict.update(Cn5_eq = Cn5_eq)
        self.phys_dict.update(Cn6_eq = Cn6_eq)

    def _build(self):
        #Retrieve data
        x_er_cm = self.cm_dict['x_er_cm']
        u_er_cm = self.cm_dict['u_er_cm']
        A       = self.cm_dict['A']
        B       = self.cm_dict['B']
        C       = self.cm_dict['C']
        D       = self.cm_dict['D']
        #Compute partial derivatives
        parder_x_eq, parder_u_eq = self._parder()
        #Update state space
        A = self._update_A(A, parder_x_eq)
        B = self._update_B(B, parder_u_eq)
        #Analytic linear control model
        al_ss       = ctl.StateSpace(A, B, C, D) #state space
        cm_sys      = ctl.iosys.LinearIOSystem(al_ss, inputs=self.inputs_er_str, outputs=self.states_er_str, states=self.states_er_str, name='AL') #linear input-output system
        #Update control model dictionary
        self.cm_dict.update(cm_sys = cm_sys)

    def _build_pipe_data(self):
        #Actuation pipe data
        x_cm            = self.cm_dict['x_cm']
        x_sp_cm         = self.cm_dict['x_sp_cm']
        x_er_cm         = self.cm_dict['x_er_cm']
        x_eq_cm         = self.cm_dict['x_eq_cm']
        u_cm            = self.cm_dict['u_cm']
        u_sp_cm         = self.cm_dict['u_sp_cm']
        u_er_cm         = self.cm_dict['u_er_cm']
        u_eq_cm         = self.cm_dict['u_eq_cm']
        cm_sys          = self.cm_dict['cm_sys']
        rxtime          = self.phys_dict['rxtime']
        actdata         = (cm_sys, x_cm, x_eq_cm, x_er_cm, x_sp_cm, u_cm, u_eq_cm, u_er_cm, u_sp_cm)
        #CSV pipe data
        self.csvdata[0]  = rxtime
        self.csvdata[1:] = x_cm.squeeze()
        return actdata

    def _pipe(self, cm2act_in, cm2csv_in, actdata):
        if (ACT_TYPE == 0) or (ACT_TYPE == 'random'): #random control
            pass
        else: 
            cm2act_in.send(actdata)
        cm2csv_in.send(self.csvdata)

    def _update_A(self, A, parder_x_eq):
        #Partial derivatives of the evolution of the state respect to state variables evaluated at the equilibrium point
        parder_phi_pndot_eq    = parder_x_eq[0]
        parder_theta_pndot_eq  = parder_x_eq[1]
        parder_psi_pndot_eq    = parder_x_eq[2]
        parder_u_pndot_eq      = parder_x_eq[3]
        parder_v_pndot_eq      = parder_x_eq[4]
        parder_w_pndot_eq      = parder_x_eq[5]
        parder_phi_pedot_eq    = parder_x_eq[6]
        parder_theta_pedot_eq  = parder_x_eq[7]
        parder_psi_pedot_eq    = parder_x_eq[8]
        parder_u_pedot_eq      = parder_x_eq[9]
        parder_v_pedot_eq      = parder_x_eq[10]
        parder_w_pedot_eq      = parder_x_eq[11]
        parder_phi_pddot_eq    = parder_x_eq[12]
        parder_theta_pddot_eq  = parder_x_eq[13]
        parder_u_pddot_eq      = parder_x_eq[14]
        parder_v_pddot_eq      = parder_x_eq[15]
        parder_w_pddot_eq      = parder_x_eq[16]
        parder_phi_phidot_eq   = parder_x_eq[17]
        parder_theta_phidot_eq = parder_x_eq[18]
        parder_p_phidot_eq     = parder_x_eq[19]
        parder_q_phidot_eq     = parder_x_eq[20]
        parder_r_phidot_eq     = parder_x_eq[21]
        parder_phi_thetadot_eq = parder_x_eq[22]
        parder_q_thetadot_eq   = parder_x_eq[23]
        parder_r_thetadot_eq   = parder_x_eq[24]
        parder_phi_psidot_eq   = parder_x_eq[25]
        parder_theta_psidot_eq = parder_x_eq[26]
        parder_q_psidot_eq     = parder_x_eq[27]
        parder_r_psidot_eq     = parder_x_eq[28]
        parder_pd_udot_eq      = parder_x_eq[29]
        parder_theta_udot_eq   = parder_x_eq[30]
        parder_u_udot_eq       = parder_x_eq[31]
        parder_v_udot_eq       = parder_x_eq[32]
        parder_w_udot_eq       = parder_x_eq[33]
        parder_q_udot_eq       = parder_x_eq[34]
        parder_r_udot_eq       = parder_x_eq[35]
        parder_pd_vdot_eq      = parder_x_eq[36]
        parder_phi_vdot_eq     = parder_x_eq[37]
        parder_theta_vdot_eq   = parder_x_eq[38]
        parder_u_vdot_eq       = parder_x_eq[39]
        parder_v_vdot_eq       = parder_x_eq[40]
        parder_w_vdot_eq       = parder_x_eq[41]
        parder_p_vdot_eq       = parder_x_eq[42]
        parder_r_vdot_eq       = parder_x_eq[43]
        parder_pd_wdot_eq      = parder_x_eq[44]
        parder_phi_wdot_eq     = parder_x_eq[45]
        parder_theta_wdot_eq   = parder_x_eq[46]
        parder_u_wdot_eq       = parder_x_eq[47]
        parder_v_wdot_eq       = parder_x_eq[48]
        parder_w_wdot_eq       = parder_x_eq[49]
        parder_p_wdot_eq       = parder_x_eq[50]
        parder_q_wdot_eq       = parder_x_eq[51]
        parder_pd_pdot_eq      = parder_x_eq[52]
        parder_u_pdot_eq       = parder_x_eq[53]
        parder_v_pdot_eq       = parder_x_eq[54]
        parder_w_pdot_eq       = parder_x_eq[55]
        parder_p_pdot_eq       = parder_x_eq[56]
        parder_q_pdot_eq       = parder_x_eq[57]
        parder_r_pdot_eq       = parder_x_eq[58]
        parder_pd_qdot_eq      = parder_x_eq[59]
        parder_u_qdot_eq       = parder_x_eq[60]
        parder_v_qdot_eq       = parder_x_eq[61]
        parder_w_qdot_eq       = parder_x_eq[62]
        parder_p_qdot_eq       = parder_x_eq[63]
        parder_q_qdot_eq       = parder_x_eq[64]
        parder_r_qdot_eq       = parder_x_eq[65]
        parder_pd_rdot_eq      = parder_x_eq[66]
        parder_u_rdot_eq       = parder_x_eq[67]
        parder_v_rdot_eq       = parder_x_eq[68]
        parder_w_rdot_eq       = parder_x_eq[69]
        parder_p_rdot_eq       = parder_x_eq[70]
        parder_q_rdot_eq       = parder_x_eq[71]
        parder_r_rdot_eq       = parder_x_eq[72]
        #Partial derivatives of pndot respect to state variables
        A[0,3] = parder_phi_pndot_eq
        A[0,4] = parder_theta_pndot_eq
        A[0,5] = parder_psi_pndot_eq
        A[0,6] = parder_u_pndot_eq
        A[0,7] = parder_v_pndot_eq
        A[0,8] = parder_w_pndot_eq
        #Partial derivatives of pedot respect to state variables
        A[1,3] = parder_phi_pedot_eq
        A[1,4] = parder_theta_pedot_eq
        A[1,5] = parder_psi_pedot_eq
        A[1,6] = parder_u_pedot_eq
        A[1,7] = parder_v_pedot_eq
        A[1,8] = parder_w_pedot_eq
        #Partial derivatives of pddot respect to state variables
        A[2,3] = parder_phi_pddot_eq
        A[2,4] = parder_theta_pddot_eq
        A[2,6] = parder_u_pddot_eq
        A[2,7] = parder_v_pddot_eq
        A[2,8] = parder_w_pddot_eq
        #Partial derivatives of phidot respect to state variables
        A[3,3]  = parder_phi_phidot_eq
        A[3,4]  = parder_theta_phidot_eq
        A[3,9]  = parder_p_phidot_eq
        A[3,10] = parder_q_phidot_eq
        A[3,11] = parder_r_phidot_eq
        #Partial derivatives of thetadot respect to state variables
        A[4,3]  = parder_phi_thetadot_eq
        A[4,10] = parder_q_thetadot_eq
        A[4,11] = parder_r_thetadot_eq
        #Partial derivatives of psidot respect to state variables
        A[5,3]  = parder_phi_psidot_eq
        A[5,4]  = parder_theta_psidot_eq
        A[5,10] = parder_q_psidot_eq
        A[5,11] = parder_r_psidot_eq
        #Partial derivatives of udot respect to state variables
        A[6,2]  = parder_pd_udot_eq
        A[6,4]  = parder_theta_udot_eq
        A[6,6]  = parder_u_udot_eq
        A[6,7]  = parder_v_udot_eq
        A[6,8]  = parder_w_udot_eq
        A[6,10] = parder_q_udot_eq
        A[6,11] = parder_r_udot_eq
        #Partial derivatives of vdot respect to state variables
        A[7,2]  = parder_pd_vdot_eq
        A[7,3]  = parder_phi_vdot_eq
        A[7,4]  = parder_theta_vdot_eq
        A[7,6]  = parder_u_vdot_eq
        A[7,7]  = parder_v_vdot_eq
        A[7,8]  = parder_w_vdot_eq
        A[7,9]  = parder_p_vdot_eq
        A[7,11] = parder_r_vdot_eq
        #Partial derivatives of wdot respect to state variables
        A[8,2]  = parder_pd_wdot_eq
        A[8,3]  = parder_phi_wdot_eq
        A[8,4]  = parder_theta_wdot_eq
        A[8,6]  = parder_u_wdot_eq
        A[8,7]  = parder_v_wdot_eq
        A[8,8]  = parder_w_wdot_eq
        A[8,9]  = parder_p_wdot_eq
        A[8,10] = parder_q_wdot_eq
        #Partial derivatives of pdot respect to state variables
        A[9,2]  = parder_pd_pdot_eq
        A[9,6]  = parder_u_pdot_eq
        A[9,7]  = parder_v_pdot_eq
        A[9,8]  = parder_w_pdot_eq
        A[9,9] = parder_p_pdot_eq
        A[9,10] = parder_q_pdot_eq
        A[9,11] = parder_r_pdot_eq
        #Partial derivatives of qdot respect to state variables
        A[10,2]  = parder_pd_qdot_eq
        A[10,6]  = parder_u_qdot_eq
        A[10,7]  = parder_v_qdot_eq
        A[10,8]  = parder_w_qdot_eq
        A[10,9] = parder_p_qdot_eq
        A[10,10] = parder_q_qdot_eq
        A[10,11] = parder_r_qdot_eq
        #Partial derivatives of rdot respect to state variables
        A[11,2]  = parder_pd_rdot_eq
        A[11,6]  = parder_u_rdot_eq
        A[11,7]  = parder_v_rdot_eq
        A[11,8]  = parder_w_rdot_eq
        A[11,9] = parder_p_rdot_eq
        A[11,10] = parder_q_rdot_eq
        A[11,11] = parder_r_rdot_eq
        return A

    def _update_B(self, B, parder_u_eq):
        #B is reduced by 2 columns corresponding to deltaf and deltam because they are assumed constant in the analytic linear control model
        #Partial derivatives of the evolution of the state respect to input variables evaluated at the equilibrium point
        parder_deltat_udot_eq = parder_u_eq[0]
        parder_deltar_vdot_eq = parder_u_eq[1]
        parder_deltae_wdot_eq = parder_u_eq[2]
        parder_deltaa_pdot_eq = parder_u_eq[3]
        parder_deltar_pdot_eq = parder_u_eq[4]
        parder_deltat_pdot_eq = parder_u_eq[5]
        parder_deltae_qdot_eq = parder_u_eq[6]
        parder_deltat_qdot_eq = parder_u_eq[7]
        parder_deltaa_rdot_eq = parder_u_eq[8]
        parder_deltar_rdot_eq = parder_u_eq[9]
        parder_deltat_rdot_eq = parder_u_eq[10]
        #Partial derivatives of udot respect to input variables
        B[6,4]  = parder_deltat_udot_eq
        #Partial derivatives of vdot respect to input variables
        B[7,3]  = parder_deltar_vdot_eq
        #Partial derivatives of wdot respect to input variables
        B[8,1]  = parder_deltae_wdot_eq
        #Partial derivatives of pdot respect to input variables
        B[9,0] = parder_deltaa_pdot_eq
        B[9,3] = parder_deltar_pdot_eq
        B[9,4] = parder_deltat_pdot_eq
        #Partial derivatives of qdot respect to input variables
        B[10,1] = parder_deltae_qdot_eq
        B[10,4] = parder_deltat_qdot_eq
        #Partial derivatives of rdot respect to input variables
        B[11,0] = parder_deltaa_rdot_eq
        B[11,3] = parder_deltar_rdot_eq
        B[11,4] = parder_deltat_rdot_eq
        return B

    def _parder(self):
        x_eq_cm = self.cm_dict['x_eq_cm']
        u_eq_cm = self.cm_dict['u_eq_cm']
        #State setpoint
        pn_eq    = x_eq_cm[0]
        pe_eq    = x_eq_cm[1]
        pd_eq    = x_eq_cm[2]
        phi_eq   = x_eq_cm[3]
        theta_eq = x_eq_cm[4]
        psi_eq   = x_eq_cm[5]
        u_eq     = x_eq_cm[6]
        v_eq     = x_eq_cm[7]
        w_eq     = x_eq_cm[8]
        p_eq     = x_eq_cm[9]
        q_eq     = x_eq_cm[10]
        r_eq     = x_eq_cm[11]
        #Input setpoint
        deltaa_eq = u_eq_cm[0]
        deltae_eq = u_eq_cm[1]
        deltaf_eq = u_eq_cm[2]
        deltar_eq = u_eq_cm[3]
        deltat_eq = u_eq_cm[4]
        deltam_eq = u_eq_cm[5]
        #Physical variables
        phi           = self.phys_dict['phi']
        theta         = self.phys_dict['theta']
        psi           = self.phys_dict['psi']
        alphadot      = self.phys_dict['alphadot']
        sigmara       = self.phys_dict['sigmara']
        sigmara_eq    = self.phys_dict['sigmara_eq']
        deltara       = self.phys_dict['deltara']
        sigmala       = self.phys_dict['sigmala']
        sigmala_eq    = self.phys_dict['sigmala_eq']
        sigmaa_avg_eq = self.phys_dict['sigmaa_avg_eq']
        deltala       = self.phys_dict['deltala']
        sigmae        = self.phys_dict['sigmae']
        sigmae_eq     = self.phys_dict['sigmae_eq']
        deltae        = self.phys_dict['deltae']
        sigmar        = self.phys_dict['sigmar']
        sigmar_eq     = self.phys_dict['sigmar_eq']
        deltar        = self.phys_dict['deltar']
        deltat        = self.phys_dict['deltat']
        hmacb         = self.phys_dict['hmacb']
        Ixx           = self.phys_dict['Ixx']
        Ixy           = self.phys_dict['Ixy']
        Ixz           = self.phys_dict['Ixz']
        Iyy           = self.phys_dict['Iyy']
        Iyz           = self.phys_dict['Iyz']
        Izz           = self.phys_dict['Izz']
        mass          = self.phys_dict['mass']
        Gamma         = self.phys_dict['Gamma']
        Gamma1        = self.phys_dict['Gamma1']
        Gamma2        = self.phys_dict['Gamma2']
        Gamma3        = self.phys_dict['Gamma3']
        Gamma4        = self.phys_dict['Gamma4']
        Gamma5        = self.phys_dict['Gamma5']
        Gamma6        = self.phys_dict['Gamma6']
        Gamma7        = self.phys_dict['Gamma7']
        Gamma8        = self.phys_dict['Gamma8']
        J_eq          = self.phys_dict['J_eq']
        T_eq          = self.phys_dict['T_eq']
        P_eq          = self.phys_dict['P_eq']
        Vauw_eq       = self.phys_dict['Vauw_eq']
        Va_eq         = self.phys_dict['Va_eq']
        Bw2Va_eq      = self.phys_dict['Bw2Va_eq']
        Cw2Va_eq      = self.phys_dict['Cw2Va_eq']
        Vprop_eq      = self.phys_dict['Vprop_eq']
        Vprop2_eq     = self.phys_dict['Vprop2_eq']
        Vind_eq       = self.phys_dict['Vind_eq']
        alpha_eq      = self.phys_dict['alpha_eq']
        beta_eq       = self.phys_dict['beta_eq']
        qbar_eq       = self.phys_dict['qbar_eq']
        qbaruw_eq     = self.phys_dict['qbaruw_eq']
        qbarind_eq    = self.phys_dict['qbarind_eq']
        qbarprop_eq   = self.phys_dict['qbarprop_eq']
        sigmaf        = self.phys_dict['sigmaf']
        sigmaf_eq     = self.phys_dict['sigmaf_eq']
        deltaf        = self.phys_dict['deltaf']
        deltam        = self.phys_dict['deltam']
        revprop       = self.phys_dict['revprop']
        rho           = self.phys_dict['rho']
        grav          = self.phys_dict['grav']
        stall         = self.phys_dict['stall']
        omega_eq      = self.phys_dict['omega_eq']
        H_eq          = self.phys_dict['H_eq']
        tau_eq        = self.phys_dict['tau_eq']
        CT_eq         = self.phys_dict['CT_eq']
        CP_eq         = self.phys_dict['CP_eq']
        TDge_eq       = self.phys_dict['TDge_eq']
        TD2_eq        = self.phys_dict['TD2_eq']
        TD3_eq        = self.phys_dict['TD3_eq']
        TC1_eq        = self.phys_dict['TC1_eq']
        TLge_eq       = self.phys_dict['TLge_eq']
        TL1_eq        = self.phys_dict['TL1_eq']
        TL2_eq        = self.phys_dict['TL2_eq']
        Tl1_eq        = self.phys_dict['Tl1_eq']
        Tl31_eq       = self.phys_dict['Tl31_eq']
        Tl32_eq       = self.phys_dict['Tl32_eq']
        Tl33_eq       = self.phys_dict['Tl33_eq']
        Tl4_eq        = self.phys_dict['Tl4_eq']
        Tm1_eq        = self.phys_dict['Tm1_eq']
        Tm2_eq        = self.phys_dict['Tm2_eq']
        Tm4_eq        = self.phys_dict['Tm4_eq']
        Tm5_eq        = self.phys_dict['Tm5_eq']
        Tn1_eq        = self.phys_dict['Tn1_eq']
        Tn3_eq        = self.phys_dict['Tn3_eq']
        Tn4_eq        = self.phys_dict['Tn4_eq']
        CD1_eq        = self.phys_dict['CD1_eq']
        CD2_eq        = self.phys_dict['CD2_eq']
        CD3_eq        = self.phys_dict['CD3_eq']
        CD4_eq        = self.phys_dict['CD4_eq']
        CC1_eq        = self.phys_dict['CC1_eq']
        CC2_eq        = self.phys_dict['CC2_eq']
        CL1_eq        = self.phys_dict['CL1_eq']
        CL2_eq        = self.phys_dict['CL2_eq']
        CL3_eq        = self.phys_dict['CL3_eq']
        CL4_eq        = self.phys_dict['CL4_eq']
        CL5_eq        = self.phys_dict['CL5_eq']
        Cl1_eq        = self.phys_dict['Cl1_eq']
        Cl2_eq        = self.phys_dict['Cl2_eq']
        Cl3_eq        = self.phys_dict['Cl3_eq']
        Cl4_eq        = self.phys_dict['Cl4_eq']
        Cl5_eq        = self.phys_dict['Cl5_eq']
        Cm1_eq        = self.phys_dict['Cm1_eq']
        Cm2_eq        = self.phys_dict['Cm2_eq']
        Cm3_eq        = self.phys_dict['Cm3_eq']
        Cm4_eq        = self.phys_dict['Cm4_eq']
        Cm5_eq        = self.phys_dict['Cm5_eq']
        Cm6_eq        = self.phys_dict['Cm6_eq']
        Cn1_eq        = self.phys_dict['Cn1_eq']
        Cn2_eq        = self.phys_dict['Cn2_eq']
        Cn3_eq        = self.phys_dict['Cn3_eq']
        Cn4_eq        = self.phys_dict['Cn4_eq']
        Cn5_eq        = self.phys_dict['Cn5_eq']
        Cn6_eq        = self.phys_dict['Cn6_eq']
        #Last iteration
        prev_parder_pd_revprop_eq     = self.cm_dict['prev_parder_pd_revprop_eq']
        prev_parder_u_revprop_eq      = self.cm_dict['prev_parder_u_revprop_eq']
        prev_parder_deltat_revprop_eq = self.cm_dict['prev_parder_deltat_revprop_eq']
        prev_parder_pd_tau_eq         = self.cm_dict['prev_parder_pd_tau_eq']
        prev_parder_u_tau_eq          = self.cm_dict['prev_parder_u_tau_eq']
        prev_parder_deltat_tau_eq     = self.cm_dict['prev_parder_deltat_tau_eq']
        #Partial derivatives of dynamic tables respect to physical variables evaluated at the equilibrium
        parder_J_CT_eq       = parder_J_TT_interp(J_eq)
        parder_J_CP_eq       = parder_J_TP_interp(J_eq)
        parder_alpha_TD3_eq  = parder_alpha_TD3_interp(alpha_eq, sigmaf)
        parder_beta_TC1_eq   = parder_beta_TC1_interp(beta_eq, sigmaf)
        parder_alpha_TL1_eq  = parder_alpha_TL1_interp(alpha_eq, stall)
        parder_alpha_Tl1_eq  = parder_alpha_Tl1_interp(alpha_eq)
        parder_alpha_Tl32_eq = parder_alpha_Tl32_interp(alpha_eq, r_eq)
        parder_r_Tl32_eq     = parder_r_Tl32_interp(alpha_eq, r_eq)
        parder_alpha_Tl33_eq = parder_alpha_Tl33_interp(alpha_eq, r_eq)
        parder_r_Tl33_eq     = parder_r_Tl33_interp(alpha_eq, r_eq)
        parder_alpha_Tl4_eq  = parder_alpha_Tl4_interp(alpha_eq, stall)
        parder_qbar_Tm1_eq   = parder_qbar_Tm1_interp(qbar_eq)
        parder_alpha_Tm2_eq  = parder_alpha_Tm2_interp(alpha_eq)
        parder_sigmae_Tm5_eq = parder_sigmae_Tm5_interp(sigmae_eq, alpha_eq)
        parder_alpha_Tm5_eq  = parder_alpha_Tm5_interp(sigmae_eq, alpha_eq)
        parder_beta_Tn1_eq   = parder_beta_Tn1_interp(beta_eq)
        parder_r_Tn3_eq      = parder_r_Tn3_interp(r_eq, alpha_eq)
        parder_alpha_Tn3_eq  = parder_alpha_Tn3_interp(r_eq, alpha_eq)
        parder_alpha_Tn4_eq  = parder_alpha_Tn4_interp(alpha_eq, beta_eq)
        parder_beta_Tn4_eq   = parder_beta_Tn4_interp(alpha_eq, beta_eq)
        #Partial derivatives of actuation variables evaluated at the equibrium point
        parder_deltaa_sigmaa_eq  = parder_deltaa_sigmaa_acm(deltaa_eq)
        parder_deltaa_sigmala_eq = parder_deltaa_sigmala_acm(deltaa_eq)
        parder_deltaa_sigmara_eq = parder_deltaa_sigmara_acm(deltaa_eq)
        parder_deltae_sigmae_eq  = parder_deltae_sigmae_acm(deltae_eq)
        parder_deltar_sigmar_eq  = parder_deltar_sigmar_acm(deltar_eq)
        #Partial derivatives of physical variables respect to state variables evaluated at the equilibrium point
        parder_pd_rho_eq          = parder_pd_rho_alcm(pd_eq)
        parder_pd_J_eq            = parder_pd_J_alcm(u_eq, revprop, prev_parder_pd_revprop_eq)
        parder_u_J_eq             = parder_u_J_alcm(u_eq, revprop, prev_parder_u_revprop_eq)
        parder_deltat_J_eq        = parder_deltat_J_alcm(u_eq, revprop, prev_parder_deltat_revprop_eq)
        parder_pd_revprop_eq      = parder_pd_revprop_alcm(prev_parder_pd_tau_eq)
        parder_u_revprop_eq       = parder_u_revprop_alcm(prev_parder_u_tau_eq)
        parder_deltat_revprop_eq  = parder_deltat_revprop_alcm(prev_parder_deltat_tau_eq)
        parder_pd_T_eq            = parder_pd_T_alcm(CT_eq, rho, revprop, parder_J_CT_eq, parder_pd_J_eq, parder_pd_rho_eq, parder_pd_revprop_eq)
        parder_u_T_eq             = parder_u_T_alcm(CT_eq, rho, revprop, parder_J_CT_eq, parder_u_J_eq, parder_u_revprop_eq)
        parder_deltat_T_eq        = parder_deltat_T_alcm(CT_eq, rho, revprop, parder_J_CT_eq, parder_deltat_J_eq, parder_deltat_revprop_eq)
        parder_pd_P_eq            = parder_pd_P_alcm(CP_eq, rho, revprop, parder_J_CP_eq, parder_pd_J_eq, parder_pd_rho_eq, parder_pd_revprop_eq)
        parder_u_P_eq             = parder_u_P_alcm(CP_eq, rho, revprop, parder_J_CP_eq, parder_u_J_eq, parder_u_revprop_eq)
        parder_deltat_P_eq        = parder_deltat_P_alcm(CP_eq, rho, revprop, parder_J_CP_eq, parder_deltat_J_eq, parder_deltat_revprop_eq)
        parder_u_Va_eq            = parder_u_Va_alcm(u_eq, Va_eq)
        parder_v_Va_eq            = parder_v_Va_alcm(v_eq, Va_eq)
        parder_w_Va_eq            = parder_w_Va_alcm(w_eq, Va_eq)
        parder_u_Bw2Va_eq         = parder_u_Bw2Va_alcm(Va_eq, parder_u_Va_eq)
        parder_v_Bw2Va_eq         = parder_v_Bw2Va_alcm(Va_eq, parder_v_Va_eq)
        parder_w_Bw2Va_eq         = parder_w_Bw2Va_alcm(Va_eq, parder_w_Va_eq)
        parder_u_Cw2Va_eq         = parder_u_Cw2Va_alcm(Va_eq, parder_u_Va_eq)
        parder_v_Cw2Va_eq         = parder_v_Cw2Va_alcm(Va_eq, parder_v_Va_eq)
        parder_w_Cw2Va_eq         = parder_w_Cw2Va_alcm(Va_eq, parder_w_Va_eq)
        parder_pd_Vprop_eq        = parder_pd_Vprop_alcm(u_eq, rho, T_eq, Vprop_eq, Vprop2_eq, parder_pd_rho_eq)
        parder_u_Vprop_eq         = parder_u_Vprop_alcm(u_eq, rho, T_eq, Vprop_eq, Vprop2_eq)
        parder_deltat_Vprop_eq    = parder_deltat_Vprop_alcm(u_eq, rho, T_eq, Vprop_eq, Vprop2_eq, parder_deltat_T_eq)
        parder_pd_Vind_eq         = parder_pd_Vind_alcm(parder_pd_Vprop_eq)
        parder_u_Vind_eq          = parder_u_Vind_alcm(parder_u_Vprop_eq)
        parder_deltat_Vind_eq     = parder_deltat_Vind_alcm(parder_deltat_Vprop_eq)
        parder_u_alpha_eq         = parder_u_alpha_alcm(w_eq, Vauw_eq)
        parder_w_alpha_eq         = parder_w_alpha_alcm(u_eq, Vauw_eq)
        parder_u_beta_eq          = parder_u_beta_alcm(u_eq, v_eq, Vauw_eq, Va_eq)
        parder_v_beta_eq          = parder_v_beta_alcm(Vauw_eq, Va_eq)
        parder_w_beta_eq          = parder_w_beta_alcm(v_eq, w_eq, Vauw_eq, Va_eq)
        parder_pd_qbar_eq         = parder_pd_qbar_alcm(Va_eq, parder_pd_rho_eq)
        parder_u_qbar_eq          = parder_u_qbar_alcm(u_eq, rho)
        parder_v_qbar_eq          = parder_v_qbar_alcm(v_eq, rho)
        parder_w_qbar_eq          = parder_w_qbar_alcm(w_eq, rho)
        parder_pd_qbarprop_eq     = parder_pd_qbarprop_alcm(rho, Vprop_eq, parder_pd_rho_eq, parder_pd_Vprop_eq)
        parder_u_qbarprop_eq      = parder_u_qbarprop_alcm(rho, Vprop_eq, parder_u_Vprop_eq)
        parder_deltat_qbarprop_eq = parder_deltat_qbarprop_alcm(rho, Vprop_eq, parder_deltat_Vprop_eq)
        parder_pd_qbarind_eq      = parder_pd_qbarind_alcm(rho, Vind_eq, parder_pd_rho_eq, parder_pd_Vind_eq)
        parder_u_qbarind_eq       = parder_u_qbarind_alcm(rho, Vind_eq, parder_u_Vind_eq)
        parder_deltat_qbarind_eq  = parder_deltat_qbarind_alcm(rho, Vind_eq, parder_deltat_Vind_eq)
        parder_pd_qbaruw_eq       = parder_pd_qbaruw_alcm(Vauw_eq, parder_pd_rho_eq)
        parder_u_qbaruw_eq        = parder_u_qbaruw_alcm(parder_u_qbar_eq)
        parder_w_qbaruw_eq        = parder_w_qbaruw_alcm(parder_w_qbar_eq)
        parder_deltat_omega_eq    = parder_deltat_omega_alcm(parder_deltat_revprop_eq)
        parder_deltat_H_eq        = parder_deltat_L_alcm(parder_deltat_omega_eq)
        parder_pd_tau_eq          = parder_pd_tau_alcm(omega_eq, parder_pd_P_eq)
        parder_u_tau_eq           = parder_u_tau_alcm(omega_eq, parder_u_P_eq)
        parder_deltat_tau_eq      = parder_deltat_tau_alcm(omega_eq, P_eq, parder_deltat_omega_eq, parder_deltat_P_eq)
        #Partial derivatives of dynamics coefficients respect to state variables evaluated at the equilibrium point 
        parder_u_CD3_eq      = parder_u_CD3_alcm(TDge_eq, parder_alpha_TD3_eq, parder_u_alpha_eq)
        parder_w_CD3_eq      = parder_w_CD3_alcm(TDge_eq, parder_alpha_TD3_eq, parder_w_alpha_eq)
        parder_u_CD4_eq      = parder_u_CD4_alcm(beta_eq, parder_u_beta_eq)
        parder_v_CD4_eq      = parder_v_CD4_alcm(beta_eq, parder_v_beta_eq)
        parder_w_CD4_eq      = parder_w_CD4_alcm(beta_eq, parder_w_beta_eq)
        parder_u_CC1_eq      = parder_u_CC1_alcm(parder_beta_TC1_eq, parder_u_beta_eq)
        parder_v_CC1_eq      = parder_v_CC1_alcm(parder_beta_TC1_eq, parder_v_beta_eq)
        parder_w_CC1_eq      = parder_w_CC1_alcm(parder_beta_TC1_eq, parder_w_beta_eq)
        parder_deltar_CC2_eq = parder_deltar_CC2_alcm(parder_deltar_sigmar_eq)
        parder_u_CL1_eq      = parder_u_CL1_alcm(TLge_eq, parder_alpha_TL1_eq, parder_u_alpha_eq)
        parder_w_CL1_eq      = parder_w_CL1_alcm(TLge_eq, parder_alpha_TL1_eq, parder_w_alpha_eq)
        parder_deltae_CL3_eq = parder_deltae_CL3_alcm(parder_deltae_sigmae_eq)
        parder_u_CL4_eq      = parder_u_CL4_alcm(q_eq, parder_u_Cw2Va_eq)
        parder_v_CL4_eq      = parder_v_CL4_alcm(q_eq, parder_v_Cw2Va_eq)
        parder_w_CL4_eq      = parder_w_CL4_alcm(q_eq, parder_w_Cw2Va_eq)
        parder_q_CL4_eq      = parder_q_CL4_alcm(Cw2Va_eq)
        parder_u_CL5_eq      = parder_u_CL5_alcm(alphadot, parder_u_Cw2Va_eq)
        parder_v_CL5_eq      = parder_v_CL5_alcm(alphadot, parder_v_Cw2Va_eq)
        parder_w_CL5_eq      = parder_w_CL5_alcm(alphadot, parder_w_Cw2Va_eq)
        parder_u_Cl1_eq      = parder_u_Cl1_alcm(beta_eq, Tl1_eq, parder_u_beta_eq, parder_alpha_Tl1_eq, parder_u_alpha_eq)
        parder_v_Cl1_eq      = parder_v_Cl1_alcm(Tl1_eq, parder_v_beta_eq)
        parder_w_Cl1_eq      = parder_w_Cl1_alcm(beta_eq, Tl1_eq, parder_w_beta_eq, parder_alpha_Tl1_eq, parder_w_alpha_eq)
        parder_u_Cl2_eq      = parder_u_Cl2_alcm(p_eq, parder_u_Bw2Va_eq)
        parder_v_Cl2_eq      = parder_v_Cl2_alcm(p_eq, parder_v_Bw2Va_eq)
        parder_w_Cl2_eq      = parder_w_Cl2(p_eq, parder_w_Bw2Va_eq)
        parder_p_Cl2_eq      = parder_p_Cl2(Bw2Va_eq)
        parder_u_Cl3_eq      = parder_u_Cl3(Bw2Va_eq, r_eq, Tl31_eq, Tl32_eq, Tl33_eq, stall, parder_u_Bw2Va_eq, parder_alpha_Tl32_eq, parder_alpha_Tl33_eq, parder_u_alpha_eq)
        parder_v_Cl3_eq      = parder_v_Cl3(r_eq, Tl31_eq, Tl32_eq, Tl33_eq, stall, parder_v_Bw2Va_eq)
        parder_w_Cl3_eq      = parder_w_Cl3_alcm(Bw2Va_eq, r_eq, Tl31_eq, Tl32_eq, Tl33_eq, stall, parder_w_Bw2Va_eq, parder_alpha_Tl32_eq, parder_alpha_Tl33_eq, parder_w_alpha_eq)
        parder_r_Cl3_eq      = parder_r_Cl3_alcm(Bw2Va_eq, r_eq, Tl31_eq, Tl32_eq, Tl33_eq, stall, parder_r_Tl32_eq, parder_r_Tl33_eq)
        parder_u_Cl4_eq      = parder_u_Cl4_alcm(sigmaa_avg_eq, parder_alpha_Tl4_eq, parder_u_alpha_eq)
        parder_w_Cl4_eq      = parder_w_Cl4_alcm(sigmaa_avg_eq, parder_alpha_Tl4_eq, parder_w_alpha_eq)
        parder_deltaa_Cl4_eq = parder_deltaa_Cl4_alcm(Tl4_eq, parder_deltaa_sigmaa_eq, parder_deltaa_sigmala_eq, parder_deltaa_sigmara_eq)
        parder_deltar_Cl5_eq = parder_deltar_Cl5_alcm(parder_deltar_sigmar_eq)
        parder_pd_Cm1_eq     = parder_pd_Cm1_alcm(parder_qbar_Tm1_eq, parder_pd_qbar_eq)
        parder_u_Cm1_eq      = parder_u_Cm1_alcm(parder_qbar_Tm1_eq, parder_u_qbar_eq)
        parder_v_Cm1_eq      = parder_v_Cm1_alcm(parder_qbar_Tm1_eq, parder_v_qbar_eq)
        parder_w_Cm1_eq      = parder_w_Cm1_alcm(parder_qbar_Tm1_eq, parder_w_qbar_eq)
        parder_u_Cm2_eq      = parder_u_Cm2_alcm(alpha_eq, Tm2_eq, parder_alpha_Tm2_eq, parder_u_alpha_eq)
        parder_w_Cm2_eq      = parder_w_Cm2_alcm(alpha_eq, Tm2_eq, parder_alpha_Tm2_eq, parder_w_alpha_eq)
        parder_u_Cm3_eq      = parder_u_Cm3_alcm(q_eq, parder_u_Cw2Va_eq)
        parder_v_Cm3_eq      = parder_v_Cm3_alcm(q_eq, parder_v_Cw2Va_eq)
        parder_w_Cm3_eq      = parder_w_Cm3_alcm(q_eq, parder_w_Cw2Va_eq)
        parder_q_Cm3_eq      = parder_q_Cm3_alcm(Cw2Va_eq)
        parder_u_Cm5_eq      = parder_u_Cm5_alcm(alphadot, parder_u_Cw2Va_eq)
        parder_v_Cm5_eq      = parder_v_Cm5_alcm(alphadot, parder_v_Cw2Va_eq)
        parder_w_Cm5_eq      = parder_w_Cm5_alcm(alphadot, parder_w_Cw2Va_eq)
        parder_u_Cm6_eq      = parder_u_Cm6_alcm(sigmae_eq, parder_alpha_Tm5_eq, parder_u_alpha_eq)
        parder_w_Cm6_eq      = parder_w_Cm6_alcm(sigmae_eq, parder_alpha_Tm5_eq, parder_w_alpha_eq)
        parder_deltae_Cm6_eq = parder_deltae_Cm6_alcm(sigmae_eq, alpha_eq, Tm5_eq, parder_deltae_sigmae_eq, parder_sigmae_Tm5_eq)
        parder_u_Cn1_eq      = parder_u_Cn1_alcm(parder_beta_Tn1_eq, parder_u_beta_eq)
        parder_v_Cn1_eq      = parder_v_Cn1_alcm(parder_beta_Tn1_eq, parder_v_beta_eq)
        parder_w_Cn1_eq      = parder_w_Cn1_alcm(parder_beta_Tn1_eq, parder_w_beta_eq)
        parder_u_Cn2_eq      = parder_u_Cn2_alcm(r_eq, parder_u_Bw2Va_eq)
        parder_v_Cn2_eq      = parder_v_Cn2_alcm(r_eq, parder_v_Bw2Va_eq)
        parder_w_Cn2_eq      = parder_w_Cn2_alcm(r_eq, parder_w_Bw2Va_eq)
        parder_r_Cn2_eq      = parder_r_Cn2_alcm(Bw2Va_eq)
        parder_u_Cn3_eq      = parder_u_Cn3_alcm(Bw2Va_eq, Tn3_eq, parder_u_Bw2Va_eq, parder_alpha_Tn3_eq, parder_u_alpha_eq)
        parder_v_Cn3_eq      = parder_v_Cn3_alcm(Tn3_eq, parder_v_Bw2Va_eq)
        parder_w_Cn3_eq      = parder_w_Cn3_alcm(Bw2Va_eq, Tn3_eq, parder_w_Bw2Va_eq, parder_alpha_Tn3_eq, parder_w_alpha_eq)
        parder_r_Cn3_eq      = parder_r_Cn3_alcm(Bw2Va_eq, parder_r_Tn3_eq)
        parder_u_Cn4_eq      = parder_u_Cn4_alcm(sigmaa_avg_eq, parder_alpha_Tn4_eq, parder_beta_Tn4_eq, parder_u_alpha_eq, parder_u_beta_eq)
        parder_v_Cn4_eq      = parder_v_Cn4_alcm(sigmaa_avg_eq, parder_beta_Tn4_eq, parder_v_beta_eq)
        parder_w_Cn4_eq      = parder_w_Cn4_alcm(sigmaa_avg_eq, parder_alpha_Tn4_eq, parder_beta_Tn4_eq, parder_w_alpha_eq, parder_w_beta_eq)
        parder_deltaa_Cn4_eq = parder_deltaa_Cn4_alcm(Tn4_eq, parder_deltaa_sigmaa_eq, parder_deltaa_sigmala_eq, parder_deltaa_sigmara_eq)
        parder_deltar_Cn5_eq = parder_deltar_Cn5_alcm(parder_deltar_sigmar_eq)
        #Trigonometric values
        cos_phi_eq   = np.cos(phi_eq)
        cos_theta_eq = np.cos(theta_eq)
        cos_psi_eq   = np.cos(psi_eq)
        sin_phi_eq   = np.sin(phi_eq)
        sin_theta_eq = np.sin(theta_eq)
        sin_psi_eq   = np.sin(psi_eq)
        tan_theta_eq = sin_theta_eq / cos_theta_eq
        sec_theta_eq = 1 / cos_theta_eq
        #Partial derivatives of pndot respect to state variables evaluated at the equilibrium point
        parder_phi_pndot_eq   = (cos_phi_eq * sin_theta_eq * cos_phi_eq + sin_phi_eq * sin_psi_eq) * v_eq + (cos_phi_eq * sin_psi_eq - sin_phi_eq * sin_theta_eq * cos_psi_eq) * w_eq
        parder_theta_pndot_eq = - (sin_theta_eq * cos_psi_eq) * u_eq + (sin_phi_eq * cos_theta_eq * cos_psi_eq) * v_eq + (cos_phi_eq * cos_theta_eq * cos_psi_eq) * w_eq
        parder_psi_pndot_eq   = - (cos_theta_eq * sin_psi_eq) * u_eq - (sin_phi_eq * sin_theta_eq * sin_psi_eq + cos_phi_eq * cos_psi_eq) * v_eq + (sin_phi_eq * cos_psi_eq - cos_phi_eq * sin_theta_eq * sin_psi_eq) * w_eq
        parder_u_pndot_eq     = cos_theta_eq * cos_psi_eq
        parder_v_pndot_eq     = sin_phi_eq * sin_theta_eq * cos_psi_eq - cos_phi_eq * sin_psi_eq
        parder_w_pndot_eq     = cos_phi_eq * sin_theta_eq * cos_psi_eq + sin_phi_eq * sin_psi_eq
        #Partial derivatives of pedot respect to state variables evaluated at the equilibrium point
        parder_phi_pedot_eq   = (cos_phi_eq * sin_theta_eq * sin_psi_eq - sin_phi_eq * cos_psi_eq) * v_eq - (sin_phi_eq * sin_theta_eq * sin_psi_eq + cos_phi_eq * cos_psi_eq) * w_eq
        parder_theta_pedot_eq = - (sin_theta_eq * sin_psi_eq) * u_eq + (sin_phi_eq * cos_theta_eq * sin_psi_eq) * v_eq + (cos_phi_eq * cos_theta_eq * sin_psi_eq) * w_eq
        parder_psi_pedot_eq   = (cos_theta_eq * cos_psi_eq) * u_eq + (sin_phi_eq * sin_theta_eq * cos_psi_eq - cos_phi_eq * sin_psi_eq) * v_eq + (cos_phi_eq * sin_theta_eq * cos_psi_eq + sin_phi_eq * sin_psi_eq) * w_eq
        parder_u_pedot_eq     = cos_theta_eq * sin_psi_eq
        parder_v_pedot_eq     = sin_phi_eq * sin_theta_eq * sin_psi_eq + cos_phi_eq * cos_psi_eq
        parder_w_pedot_eq     = cos_phi_eq * sin_theta_eq * sin_psi_eq - sin_phi_eq * cos_psi_eq
        #Partial derivatives of pddot respect to state variables evaluated at the equilibrium point
        parder_phi_pddot_eq   = (cos_phi_eq * cos_theta_eq) * v_eq - (sin_phi_eq * cos_theta_eq) * w_eq
        parder_theta_pddot_eq = - (cos_theta_eq) * u_eq - (sin_phi_eq * sin_theta_eq) * v_eq - (cos_phi_eq * sin_theta_eq) * w_eq
        parder_u_pddot_eq     = - sin_theta_eq
        parder_v_pddot_eq     = sin_phi_eq * cos_theta_eq
        parder_w_pddot_eq     = cos_phi_eq * cos_theta_eq
        #Partial derivatives of phidot respect to state variables evaluated at the equilibrium point
        parder_phi_phidot_eq   = (cos_phi_eq * tan_theta_eq) * q_eq - (sin_phi_eq * tan_theta_eq) * r_eq
        parder_theta_phidot_eq = (sin_phi_eq * (1 + tan_theta_eq ** 2)) * q_eq + (cos_phi_eq * (1 + tan_theta_eq ** 2)) * r_eq
        parder_p_phidot_eq     = 1
        parder_q_phidot_eq     = sin_phi_eq * tan_theta_eq
        parder_r_phidot_eq     = cos_phi_eq * tan_theta_eq
        #Partial derivatives of thetadot respect to state variables evaluated at the equilibrium point
        parder_phi_thetadot_eq = - (sin_phi_eq) * q_eq - (cos_phi_eq) * r_eq
        parder_q_thetadot_eq   = cos_phi_eq
        parder_r_thetadot_eq   = - sin_phi_eq 
        #Partial derivatives of psidot respect to state variables evaluated at the equilibrium point
        parder_phi_psidot_eq   = (cos_phi_eq / cos_theta_eq) * q_eq - (sin_phi_eq / cos_theta_eq) * r_eq
        parder_theta_psidot_eq = (sin_phi_eq * tan_theta_eq * sec_theta_eq) * q_eq + (cos_phi_eq * tan_theta_eq * sec_theta_eq) * r_eq
        parder_q_psidot_eq     = sin_phi_eq / cos_theta_eq
        parder_r_psidot_eq     = cos_phi_eq / cos_theta_eq
        #Partial derivatives of udot respect to state variables evaluated at the equilibrium point
        parder_pd_udot_eq    = (1 / mass) * (- parder_pd_qbar_eq * SW * (CD1_eq + CD2_eq + CD3_eq + CD4_eq) + parder_pd_T_eq)
        parder_theta_udot_eq = - grav * cos_theta_eq
        parder_u_udot_eq     = (1 / mass) * (- SW * (parder_u_qbar_eq * (CD1_eq + CD2_eq + CD3_eq + CD4_eq) + qbar_eq * (parder_u_CD3_eq + parder_u_CD4_eq)) + parder_u_T_eq)
        parder_v_udot_eq      = r_eq - (SW / mass) * (parder_v_qbar_eq * (CD1_eq + CD2_eq + CD3_eq + CD4_eq) + qbar_eq * parder_v_CD4_eq)
        parder_w_udot_eq      = - q_eq - (SW / mass) * (parder_w_qbar_eq * (CD1_eq + CD2_eq + CD3_eq + CD4_eq) + qbar_eq * (parder_w_CD3_eq + parder_w_CD4_eq))
        parder_q_udot_eq      = - w_eq
        parder_r_udot_eq      = v_eq
        #Partial derivatives of vdot respect to state variables evaluated at the equilibrium point
        parder_pd_vdot_eq    = (1 / mass) * (parder_pd_qbar_eq * SW * (CC1_eq + CC2_eq))
        parder_phi_vdot_eq   = grav * cos_phi_eq * cos_theta_eq
        parder_theta_vdot_eq = - grav * sin_phi_eq * sin_theta_eq
        parder_u_vdot_eq     = - r_eq + (SW / mass) * (parder_u_qbar_eq * (CC1_eq + CC2_eq) + qbar_eq * parder_u_CC1_eq)
        parder_v_vdot_eq     = (SW / mass) * (parder_v_qbar_eq * (CC1_eq + CC2_eq) + qbar_eq * parder_v_CC1_eq)
        parder_w_vdot_eq     = p_eq + (SW / mass) * (parder_w_qbar_eq * (CC1_eq + CC2_eq) + qbar_eq * parder_w_CC1_eq)
        parder_p_vdot_eq     = w_eq
        parder_r_vdot_eq     = - u_eq
        #Partial derivatives of wdot respect to state variables evaluated at the equilibrium point
        parder_pd_wdot_eq    = - (SW / mass) * (parder_pd_qbar_eq * (CL1_eq + CL2_eq + CL3_eq + CL4_eq) + parder_pd_qbaruw_eq * CL5_eq)
        parder_phi_wdot_eq   = - grav * sin_phi_eq * cos_theta_eq
        parder_theta_wdot_eq = - grav * cos_phi_eq * sin_theta_eq
        parder_u_wdot_eq     = q_eq - (SW / mass) * (parder_u_qbar_eq * (CL1_eq + CL2_eq + CL3_eq + CL4_eq) + qbar_eq * (parder_u_CL1_eq + parder_u_CL4_eq) + parder_u_qbaruw_eq * CL5_eq + qbaruw_eq * parder_u_CL5_eq)
        parder_v_wdot_eq     = - p_eq - (SW / mass) * (parder_v_qbar_eq * (CL1_eq + CL2_eq + CL3_eq + CL4_eq) + qbar_eq * parder_v_CL4_eq + qbaruw_eq * CL5_eq)
        parder_w_wdot_eq     = - (SW / mass) * (parder_w_qbar_eq * (CL1_eq + CL2_eq + CL3_eq + CL4_eq) + qbar_eq * (parder_w_CL1_eq + parder_w_CL4_eq) + parder_w_qbaruw_eq * CL5_eq + qbaruw_eq * parder_w_CL5_eq)
        parder_p_wdot_eq     = - v_eq
        parder_q_wdot_eq     = u_eq - ((qbar_eq * SW) / mass) * parder_q_CL4_eq
        #Partial derivatives of pdot respect to state variables evaluated at the equilibrium point
        parder_pd_pdot_eq = SW * BW * (parder_pd_qbar_eq * (Gamma3 * (Cl1_eq + Cl2_eq + Cl3_eq + Cl4_eq + Cl5_eq) + Gamma4 * (Cn1_eq + Cn2_eq + Cn3_eq + Cn4_eq)) + Gamma4 * (parder_pd_qbarind_eq * Cn5_eq + parder_pd_qbarprop_eq * Cn6_eq)) + Gamma3 * parder_pd_tau_eq
        parder_u_pdot_eq  = SW * BW * (parder_u_qbar_eq * (Gamma3 * (Cl1_eq + Cl2_eq + Cl3_eq + Cl4_eq + Cl5_eq) + Gamma4 * (Cn1_eq + Cn2_eq + Cn3_eq + Cn4_eq)) + qbar_eq * (Gamma3 * (parder_u_Cl1_eq + parder_u_Cl2_eq + parder_u_Cl3_eq + parder_u_Cl4_eq) + Gamma4 * (parder_u_Cn1_eq + parder_u_Cn2_eq + parder_u_Cn3_eq + parder_u_Cn4_eq)) + Gamma4 * (parder_u_qbarind_eq * Cn5_eq + parder_u_qbarprop_eq * Cn6_eq)) + Gamma3 * parder_u_tau_eq
        parder_v_pdot_eq  = SW * BW * (parder_v_qbar_eq * (Gamma3 * (Cl1_eq + Cl2_eq + Cl3_eq + Cl4_eq + Cl5_eq) + Gamma4 * (Cn1_eq + Cn2_eq + Cn3_eq + Cn4_eq)) + qbar_eq * (Gamma3 * (parder_v_Cl1_eq + parder_v_Cl2_eq + parder_v_Cl3_eq) + Gamma4 * (parder_v_Cn1_eq + parder_v_Cn2_eq + parder_v_Cn3_eq + parder_v_Cn4_eq)))
        parder_w_pdot_eq  = SW * BW * (parder_w_qbar_eq * (Gamma3 * (Cl1_eq + Cl2_eq + Cl3_eq + Cl4_eq + Cl5_eq) + Gamma4 * (Cn1_eq + Cn2_eq + Cn3_eq + Cn4_eq)) + qbar_eq * (Gamma3 * (parder_w_Cl1_eq + parder_w_Cl2_eq + parder_w_Cl3_eq + parder_w_Cl4_eq) + Gamma4 * (parder_w_Cn1_eq + parder_w_Cn2_eq + parder_w_Cn3_eq + parder_w_Cn4_eq)))
        parder_p_pdot_eq  = Gamma1 * q_eq + SW * BW * qbar_eq * Gamma3 * parder_p_Cl2_eq
        parder_q_pdot_eq  = Gamma1 * p_eq - Gamma2 * r_eq - Gamma4 * H_eq
        parder_r_pdot_eq  = - Gamma2 * q_eq + SW * BW * qbar_eq * (Gamma3 * parder_r_Cl3_eq + Gamma4 * (parder_r_Cn2_eq + parder_r_Cn3_eq))
        #Partial derivatives of qdot respect to state variables evaluated at the equilibrium point
        parder_pd_qdot_eq = ((SW * CW) / Iyy) * (parder_pd_qbar_eq * (Cm1_eq + Cm2_eq + Cm3_eq + Cm4_eq) + qbar_eq * parder_pd_Cm1_eq + parder_pd_qbaruw_eq * Cm5_eq + parder_pd_qbarind_eq * Cm6_eq)
        parder_u_qdot_eq  = ((SW * CW) / Iyy) * (parder_u_qbar_eq * (Cm1_eq + Cm2_eq + Cm3_eq + Cm4_eq) + qbar_eq * (parder_u_Cm1_eq + parder_u_Cm2_eq + parder_u_Cm3_eq) + parder_u_qbaruw_eq * Cm5_eq + qbaruw_eq * parder_u_Cm5_eq + parder_u_qbarind_eq * Cm6_eq + qbarind_eq * parder_u_Cm6_eq)
        parder_v_qdot_eq  = ((SW * CW) / Iyy) * (parder_v_qbar_eq * (Cm1_eq + Cm2_eq + Cm3_eq + Cm4_eq) + qbar_eq * (parder_v_Cm1_eq + parder_v_Cm3_eq) + qbaruw_eq * parder_v_Cm5_eq)
        parder_w_qdot_eq  = ((SW * CW) / Iyy) * (parder_w_qbar_eq * (Cm1_eq + Cm2_eq + Cm3_eq + Cm4_eq) + qbar_eq * (parder_w_Cm1_eq + parder_w_Cm2_eq + parder_w_Cm3_eq) + parder_w_qbaruw_eq * Cm5_eq + qbaruw_eq * parder_w_Cm5_eq + qbarind_eq * parder_w_Cm6_eq)
        parder_p_qdot_eq  = Gamma5 * r_eq - 2 * Gamma6 * p_eq
        parder_q_qdot_eq  = ((SW * CW * qbar_eq) / Iyy) * parder_q_Cm3_eq
        parder_r_qdot_eq  = Gamma5 * p_eq + 2 * Gamma6 * r_eq + H_eq / Iyy
        #Partial derivatives of rdot respect to state variables evaluated at the equilibrium point
        parder_pd_rdot_eq = SW * BW * (parder_pd_qbar_eq * (Gamma4 * (Cl1_eq + Cl2_eq + Cl3_eq + Cl4_eq + Cl5_eq) + Gamma8 * (Cn1_eq + Cn2_eq + Cn3_eq + Cn4_eq)) + Gamma8 * (parder_pd_qbarind_eq * Cn5_eq + parder_pd_qbarprop_eq * Cn6_eq)) + Gamma4 * parder_pd_tau_eq
        parder_u_rdot_eq  = SW * BW * (parder_u_qbar_eq * (Gamma4 * (Cl1_eq + Cl2_eq + Cl3_eq + Cl4_eq + Cl5_eq) + Gamma8 * (Cn1_eq + Cn2_eq + Cn3_eq + Cn4_eq)) + qbar_eq * (Gamma4 * (parder_u_Cl1_eq + parder_u_Cl2_eq + parder_u_Cl3_eq + parder_u_Cl4_eq) + Gamma8 * (parder_u_Cn1_eq + parder_u_Cn2_eq + parder_u_Cn3_eq + parder_u_Cn4_eq)) + Gamma8 * (parder_u_qbarind_eq * Cn5_eq + parder_u_qbarprop_eq * Cn6_eq)) + Gamma4 * parder_u_tau_eq
        parder_v_rdot_eq  = SW * BW * (parder_v_qbar_eq * (Gamma4 * (Cl1_eq + Cl2_eq + Cl3_eq + Cl4_eq + Cl5_eq) + Gamma8 * (Cn1_eq + Cn2_eq + Cn3_eq + Cn4_eq)) + qbar_eq * (Gamma4 * (parder_v_Cl1_eq + parder_v_Cl2_eq + parder_v_Cl3_eq) + Gamma8 * (parder_v_Cn1_eq + parder_v_Cn2_eq + parder_v_Cn3_eq + parder_v_Cn4_eq)))
        parder_w_rdot_eq  = SW * BW * (parder_w_qbar_eq * (Gamma4 * (Cl1_eq + Cl2_eq + Cl3_eq + Cl4_eq + Cl5_eq) + Gamma8 * (Cn1_eq + Cn2_eq + Cn3_eq + Cn4_eq)) + qbar_eq * (Gamma4 * (parder_w_Cl1_eq + parder_w_Cl2_eq + parder_w_Cl3_eq + parder_w_Cl4_eq) + Gamma8 * (parder_w_Cn1_eq + parder_w_Cn2_eq + parder_w_Cn3_eq + parder_w_Cn4_eq)))
        parder_p_rdot_eq  = Gamma7 * q_eq + SW * BW * qbar_eq * Gamma4 * parder_p_Cl2_eq
        parder_q_rdot_eq  = Gamma7 * p_eq - Gamma1 * r_eq - Gamma8 * H_eq
        parder_r_rdot_eq  = - Gamma1 * q_eq + SW * BW * qbar_eq * (Gamma4 * parder_r_Cl3_eq + Gamma8 * (parder_r_Cn2_eq + parder_r_Cn3_eq))
        #Partial derivatives of udot respect to input variables evaluated at the equilibrium point
        parder_deltat_udot_eq = parder_deltat_T_eq / mass
        #Partial derivatives of vdot respect to input variables evaluated at the equilibrium point
        parder_deltar_vdot_eq = ((SW * qbar_eq) / mass) * parder_deltar_CC2_eq
        #Partial derivatives of wdot respect to input variables evaluated at the equilibrium point
        parder_deltae_wdot_eq = - ((SW * qbar_eq) / mass) * parder_deltae_CL3_eq
        #Partial derivatives of pdot respect to input variables evaluated at the equilibrium point
        parder_deltaa_pdot_eq = SW * BW * qbar_eq * (Gamma3 * parder_deltaa_Cl4_eq + Gamma4 * parder_deltaa_Cn4_eq)
        parder_deltar_pdot_eq = SW * BW * (Gamma3 * qbar_eq * parder_deltar_Cl5_eq + Gamma4 * qbarind_eq * parder_deltar_Cn5_eq)
        parder_deltat_pdot_eq = SW * BW * Gamma4 * (parder_deltat_qbarind_eq * Cn5_eq + parder_deltat_qbarprop_eq * Cn6_eq) + Gamma3 * parder_deltat_tau_eq - Gamma4 * q_eq * parder_deltat_H_eq
        #Partial derivatives of qdot respect to input variables evaluated at the equilibrium point
        parder_deltae_qdot_eq = ((SW * CW * qbarind_eq) / Iyy) * parder_deltae_Cm6_eq
        parder_deltat_qdot_eq = ((SW * CW) / Iyy) * parder_deltat_qbarind_eq * Cm6_eq + r_eq * parder_deltat_H_eq / Iyy
        #Partial derivatives of rdot respect to input variables evaluated at the equilibrium point
        parder_deltaa_rdot_eq = SW * BW * qbar_eq * (Gamma4 * parder_deltaa_Cl4_eq + Gamma8 * parder_deltaa_Cn4_eq)
        parder_deltar_rdot_eq = SW * BW * (Gamma4 * qbar_eq * parder_deltar_Cl5_eq + Gamma8 * qbarind_eq * parder_deltar_Cn5_eq)
        parder_deltat_rdot_eq = SW * BW * Gamma8 * (parder_deltat_qbarind_eq * Cn5_eq + parder_deltat_qbarprop_eq * Cn6_eq) + Gamma4 * parder_deltat_tau_eq - Gamma8 * q_eq * parder_deltat_H_eq
        #Partial derivatives of state respect to state (A) evaluated at the equilibrium point
        parder_x_eq = np.array(
                               [ 
                                parder_phi_pndot_eq,
                                parder_theta_pndot_eq,
                                parder_psi_pndot_eq,
                                parder_u_pndot_eq,
                                parder_v_pndot_eq,
                                parder_w_pndot_eq,
                                parder_phi_pedot_eq,
                                parder_theta_pedot_eq,
                                parder_psi_pedot_eq,
                                parder_u_pedot_eq,
                                parder_v_pedot_eq,
                                parder_w_pedot_eq,
                                parder_phi_pddot_eq,
                                parder_theta_pddot_eq,
                                parder_u_pddot_eq,
                                parder_v_pddot_eq,
                                parder_w_pddot_eq,
                                parder_phi_phidot_eq,
                                parder_theta_phidot_eq,
                                parder_p_phidot_eq,
                                parder_q_phidot_eq,
                                parder_r_phidot_eq,
                                parder_phi_thetadot_eq,
                                parder_q_thetadot_eq,
                                parder_r_thetadot_eq,
                                parder_phi_psidot_eq,
                                parder_theta_psidot_eq,
                                parder_q_psidot_eq,
                                parder_r_psidot_eq,
                                parder_pd_udot_eq,
                                parder_theta_udot_eq,
                                parder_u_udot_eq,
                                parder_v_udot_eq,
                                parder_w_udot_eq,
                                parder_q_udot_eq,
                                parder_r_udot_eq,
                                parder_pd_vdot_eq,
                                parder_phi_vdot_eq,
                                parder_theta_vdot_eq,
                                parder_u_vdot_eq,
                                parder_v_vdot_eq,
                                parder_w_vdot_eq,
                                parder_p_vdot_eq,
                                parder_r_vdot_eq,
                                parder_pd_wdot_eq,
                                parder_phi_wdot_eq,
                                parder_theta_wdot_eq,
                                parder_u_wdot_eq,
                                parder_v_wdot_eq,
                                parder_w_wdot_eq,
                                parder_p_wdot_eq,
                                parder_q_wdot_eq,
                                parder_pd_pdot_eq,
                                parder_u_pdot_eq,
                                parder_v_pdot_eq,
                                parder_w_pdot_eq,
                                parder_p_pdot_eq,
                                parder_q_pdot_eq,
                                parder_r_pdot_eq,
                                parder_pd_qdot_eq,
                                parder_u_qdot_eq,
                                parder_v_qdot_eq,
                                parder_w_qdot_eq,
                                parder_p_qdot_eq,
                                parder_q_qdot_eq,
                                parder_r_qdot_eq,
                                parder_pd_rdot_eq,
                                parder_u_rdot_eq,
                                parder_v_rdot_eq,
                                parder_w_rdot_eq,
                                parder_p_rdot_eq,
                                parder_q_rdot_eq,
                                parder_r_rdot_eq      
                               ]
                               , dtype=float
                              )
        #Partial derivatives of state respect to input (B) evaluated at the equilibrium point
        parder_u_eq = np.array(
                               [
                                parder_deltat_udot_eq,
                                parder_deltar_vdot_eq,
                                parder_deltae_wdot_eq,
                                parder_deltaa_pdot_eq,
                                parder_deltar_pdot_eq,
                                parder_deltat_pdot_eq,
                                parder_deltae_qdot_eq,
                                parder_deltat_qdot_eq,
                                parder_deltaa_rdot_eq,
                                parder_deltar_rdot_eq,
                                parder_deltat_rdot_eq
                               ]
                               , dtype=float
                              )
        #Update control model dictionary
        self.cm_dict.update(prev_parder_pd_revprop_eq = parder_pd_revprop_eq)
        self.cm_dict.update(prev_parder_u_revprop_eq = parder_u_revprop_eq)
        self.cm_dict.update(prev_parder_deltat_revprop_eq  = parder_deltat_revprop_eq)
        self.cm_dict.update(prev_parder_pd_tau_eq = parder_pd_tau_eq)
        self.cm_dict.update(prev_parder_u_tau_eq = parder_u_tau_eq)
        self.cm_dict.update(prev_parder_deltat_tau_eq = parder_deltat_tau_eq)
        return parder_x_eq, parder_u_eq

''' ANALYTIC NON-LINEAR CONTROL MODEL '''
class ANLCM(BaseControlModel):
    #Analytic Non-Linear Control Model (ANLCM)

    def _init(self):
        #Initialize dictionaries
        self.cm_dict   = {} #control model dictionary
        self.phys_dict = {} #physical variables dictionary
        #Initialize ANLCM
        x_cm    = np.zeros(CM_STATE_LEN) #state
        x_sp_cm = np.zeros(CM_STATE_LEN) #state setpoint
        u_cm    = np.zeros(CM_INPUT_LEN) #input
        u_sp_cm = np.zeros(CM_INPUT_LEN) #state setpoint
        #Update control model dictionary
        self.cm_dict.update(x_cm = x_cm)
        self.cm_dict.update(x_sp_cm = x_sp_cm)
        self.cm_dict.update(u_cm = u_cm)
        self.cm_dict.update(u_sp_cm = u_sp_cm)

    def run(self, cm2act_in, cm2csv_in, eq2cm_out, rx2cm_out, sp2cm_out, event_start, event_end):
        self._init() #initialize ANLCM
        event_start.wait() #wait for simulation start event
        while True:
            try:
                if event_end.is_set():
                    #Close pipes
                    cm2act_in.close()
                    cm2csv_in.close()
                    rx2cm_out.close()
                    sp2cm_out.close()
                    break
                else:
                    rxdata = rx2cm_out.recv() #receive RX telemetry
                    if (np.any(rxdata[:,0] >= self.t)):
                        spdata = sp2cm_out.recv() #receive setpoint
                        self._preprocess(rxdata, spdata) #preprocess piped-in data
                        self._build() #build ANLCM model
                        actdata = self._build_pipe_data() #build pipe data
                        self._pipe(cm2act_in, cm2csv_in, actdata) #pipe out to actuation and CSV logging
                        self.t += self.dt
            except:
                raise RuntimeError('.'.join((__name__, sys._getframe().f_code.co_name)))

    def _preprocess(self, rxdata, spdata):
        i      = np.where(rxdata[:,0] >= self.t)[0][0] #first RX telemetry frame index
        rxdata = rxdata[i,:] #first RX telemetry frame
        rxtime = rxdata[0]
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
        aacm_dict = assumptions_acm(deltat, pd)
        sigmaf       = aacm_dict['sigmaf']
        deltaf       = aacm_dict['deltaf']
        deltam       = aacm_dict['deltam']
        rho          = aacm_dict['rho']
        grav         = aacm_dict['grav']
        stall        = aacm_dict['stall']
        hmacb        = aacm_dict['hmacb']
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
        #Analytic non-linear control model parameters
        params_cm = {}
        params_cm.update(phi = phi)
        params_cm.update(theta = theta)
        params_cm.update(psi = psi)
        params_cm.update(alphadot = alphadot)
        params_cm.update(sigmara = sigmara)
        params_cm.update(sigmala = sigmala)
        params_cm.update(sigmae = sigmae)
        params_cm.update(sigmaf = sigmaf)
        params_cm.update(sigmar = sigmar)
        params_cm.update(hmacb = hmacb)
        params_cm.update(stall = stall)
        params_cm.update(Iyy = Iyy)
        params_cm.update(mass = mass)
        params_cm.update(grav = grav)
        params_cm.update(rho = rho)
        params_cm.update(revprop = revprop)
        params_cm.update(Gamma = Gamma)
        params_cm.update(Gamma1 = Gamma1)
        params_cm.update(Gamma2 = Gamma2)
        params_cm.update(Gamma3 = Gamma3)
        params_cm.update(Gamma4 = Gamma4)
        params_cm.update(Gamma5 = Gamma5)
        params_cm.update(Gamma6 = Gamma6)
        params_cm.update(Gamma7 = Gamma7)
        params_cm.update(Gamma8 = Gamma8)
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
        #State setpoint
        x_sp_cm = np.array(spdata[:CM_STATE_LEN])
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
        #Input setpoint
        u_sp_cm = np.array(spdata[CM_STATE_LEN:])
        #Update control model dictionary
        self.cm_dict.update(x_cm = x_cm)
        self.cm_dict.update(x_sp_cm = x_sp_cm)
        self.cm_dict.update(u_cm = u_cm)
        self.cm_dict.update(u_sp_cm = u_sp_cm)
        self.cm_dict.update(params_cm = params_cm)
        #Update physical variables dictionary
        self.phys_dict.update(rxtime = rxtime)
        self.phys_dict.update(phi = phi)
        self.phys_dict.update(theta = theta)
        self.phys_dict.update(psi = psi)
        self.phys_dict.update(alphadot = alphadot)
        self.phys_dict.update(sigmara = sigmara)
        self.phys_dict.update(deltara = deltara)
        self.phys_dict.update(sigmala = sigmala)
        self.phys_dict.update(deltala = deltala)
        self.phys_dict.update(deltaa = deltaa)
        self.phys_dict.update(sigmae = sigmae)
        self.phys_dict.update(deltae = deltae)
        self.phys_dict.update(sigmar = sigmar)
        self.phys_dict.update(deltar = deltar)
        self.phys_dict.update(deltat = deltat)
        self.phys_dict.update(hmacb = hmacb)
        self.phys_dict.update(qbar = qbar)
        self.phys_dict.update(Ixx = Ixx)
        self.phys_dict.update(Ixy = Ixy)
        self.phys_dict.update(Ixz = Ixz)
        self.phys_dict.update(Iyy = Iyy)
        self.phys_dict.update(Iyz = Iyz)
        self.phys_dict.update(Izz = Izz)
        self.phys_dict.update(mass = mass)
        self.phys_dict.update(Gamma = Gamma)
        self.phys_dict.update(Gamma1 = Gamma1)
        self.phys_dict.update(Gamma2 = Gamma2)
        self.phys_dict.update(Gamma3 = Gamma3)
        self.phys_dict.update(Gamma4 = Gamma4)
        self.phys_dict.update(Gamma5 = Gamma5)
        self.phys_dict.update(Gamma6 = Gamma6)
        self.phys_dict.update(Gamma7 = Gamma7)
        self.phys_dict.update(Gamma8 = Gamma8)
        self.phys_dict.update(sigmaf = sigmaf)
        self.phys_dict.update(deltaf = deltaf)
        self.phys_dict.update(deltam = deltam)
        self.phys_dict.update(revprop = revprop)
        self.phys_dict.update(rho = rho)
        self.phys_dict.update(grav = grav)
        self.phys_dict.update(stall = stall)

    def _build(self):
        params_cm   = self.cm_dict['params_cm']
        #Analytic non-linear control model
        cm_sys      = ctl.iosys.NonlinearIOSystem(self._update_anl, inputs=self.inputs_str, outputs=self.states_str, states=self.states_str, params=params_cm, name='ANL')
        #Update control model dictionary
        self.cm_dict.update(cm_sys = cm_sys)

    def _build_pipe_data(self):
        #Actuation pipe data
        rxtime  = self.phys_dict['rxtime']
        x_cm    = self.cm_dict['x_cm']
        x_sp_cm = self.cm_dict['x_sp_cm']
        u_cm    = self.cm_dict['u_cm']
        u_sp_cm = self.cm_dict['u_sp_cm']
        cm_sys  = self.cm_dict['cm_sys']
        actdata = (cm_sys, x_cm, x_sp_cm, u_cm, u_sp_cm)
        #CSV pipe data
        self.csvdata[0]  = rxtime
        self.csvdata[1:] = x_cm.squeeze()
        return actdata

    def _pipe(self, cm2act_in, cm2csv_in, actdata):
        if (ACT_TYPE == 0) or (ACT_TYPE == 'random'): #random control
            pass
        else: 
            cm2act_in.send(actdata)
        cm2csv_in.send(self.csvdata)

    def _update_anl(self, time, x_cm, u_cm, params_cm):
        #Parameters
        phi      = params_cm['phi']
        theta    = params_cm['theta']
        psi      = params_cm['psi']
        alphadot = params_cm['alphadot']
        sigmara  = params_cm['sigmara']
        sigmala  = params_cm['sigmala']
        sigmae   = params_cm['sigmae']
        sigmaf   = params_cm['sigmaf']
        sigmar   = params_cm['sigmar']
        hmacb    = params_cm['hmacb']
        stall    = params_cm['stall']
        Iyy      = params_cm['Iyy']
        mass     = params_cm['mass']
        grav     = params_cm['grav']
        rho      = params_cm['rho']
        revprop  = params_cm['revprop']
        Gamma    = params_cm['Gamma']
        Gamma1   = params_cm['Gamma1']
        Gamma2   = params_cm['Gamma2']
        Gamma3   = params_cm['Gamma3']
        Gamma4   = params_cm['Gamma4']
        Gamma5   = params_cm['Gamma5']
        Gamma6   = params_cm['Gamma6']
        Gamma7   = params_cm['Gamma7']
        Gamma8   = params_cm['Gamma8']
        #State
        pn    = x_cm[0]
        pe    = x_cm[1]
        pd    = x_cm[2]
        phi   = x_cm[3]
        theta = x_cm[4]
        psi   = x_cm[5]
        u     = x_cm[6]
        v     = x_cm[7]
        w     = x_cm[8]
        p     = x_cm[9]
        q     = x_cm[10]
        r     = x_cm[11]
        #Input
        deltaa = u_cm[0]
        deltae = u_cm[1]
        deltaf = u_cm[2]
        deltar = u_cm[3]
        deltat = u_cm[4]
        deltam = u_cm[5]
        #Rotations
        euler   = np.array([phi, theta, psi], dtype=float)
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
        CP         = TP_eq
        T          = thrust_eng(CT, rho, revprop)
        P          = power_eng(CP, rho, revprop)
        Va         = Va_acm(u, v, w)
        Vauw       = Vauw_acm(u, w)
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
        m          = SW * CW * (qbar * (Cm1 + Cm2 + Cm3 + Cm4) + qbaruw * Cm5 + qbarind * Cm6) + H + r
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
        xdot_cm[3:6]   = np.array([[1, sin_phi * tan_theta, cos_phi * tan_theta], [0, cos_phi, -sin_phi], [0, sin_phi / cos_theta, cos_phi / cos_theta]], dtype=float) @ np.array([p, q, r], dtype=float) #attitude in vehicle frame (phidot, thetadot, psidot)
        xdot_cm[6:9]  = np.array([[0, -w, v], [w, 0, -u], [-v, u, 0]], dtype=float) @ np.array([p, q, r]) + np.array([fx, fy, fz], dtype=float) / mass #linear velocity in body frame (udot, vdot, wdot)
        xdot_cm[9:12] = np.array([Gamma1*p*q-Gamma2*q*r, Gamma5*p*r-Gamma6*(p**2-r**2), Gamma7*p*q-Gamma1*q*r], dtype=float) + np.array([[Gamma3, 0, Gamma4], [0, 1/Iyy, 0], [Gamma4, 0, Gamma8]], dtype=float) @ np.array([l, m, n], dtype=float) #angular velocity in body frame (pdot, qdot, rdot)
        return xdot_cm 

''' LINEARIZED ANALYTIC NON-LINEAR CONTROL MODEL '''
class LANLCM(ANLCM):
    #Linearized Analytic Non-Linear Control Model (LANLCM)

    def _init(self):
        #Initialize dictionaries
        self.cm_dict   = {} #control model dictionary
        self.phys_dict = {} #physical variables dictionary
        #Initialize ALCM
        x_cm    = np.zeros(CM_STATE_LEN) #state
        x_sp_cm = np.zeros(CM_STATE_LEN) #state setpoint
        x_er_cm = np.zeros(CM_STATE_LEN) #state error
        u_cm    = np.zeros(CM_INPUT_LEN) #input
        u_sp_cm = np.zeros(CM_INPUT_LEN) #state setpoint
        u_er_cm = np.zeros(CM_INPUT_LEN) #input error
        #Update control model dictionary
        self.cm_dict.update(x_cm = x_cm)
        self.cm_dict.update(x_sp_cm = x_sp_cm)
        self.cm_dict.update(x_er_cm = x_er_cm)
        self.cm_dict.update(u_cm = u_cm)
        self.cm_dict.update(u_sp_cm = u_sp_cm)
        self.cm_dict.update(u_er_cm = u_er_cm)

    def _preprocess(self, rxdata, spdata):
        i      = np.where(rxdata[:,0] >= self.t)[0][0] #first RX telemetry frame index
        rxdata = rxdata[i,:] #first RX telemetry frame
        rxtime = rxdata[0]
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
        #Assumptions of analytic control models
        aacm_dict = assumptions_acm(deltat, pd)
        sigmaf    = aacm_dict['sigmaf']
        deltaf    = aacm_dict['deltaf']
        deltam    = aacm_dict['deltam']
        rho       = aacm_dict['rho']
        grav      = aacm_dict['grav']
        stall     = aacm_dict['stall']
        hmacb     = aacm_dict['hmacb']
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
        #Analytic non-linear control model parameters
        params_cm = {}
        params_cm.update(phi = phi)
        params_cm.update(theta = theta)
        params_cm.update(psi = psi)
        params_cm.update(alphadot = alphadot)
        params_cm.update(sigmara = sigmara)
        params_cm.update(sigmala = sigmala)
        params_cm.update(sigmae = sigmae)
        params_cm.update(sigmaf = sigmaf)
        params_cm.update(sigmar = sigmar)
        params_cm.update(hmacb = hmacb)
        params_cm.update(stall = stall)
        params_cm.update(Iyy = Iyy)
        params_cm.update(mass = mass)
        params_cm.update(grav = grav)
        params_cm.update(rho = rho)
        params_cm.update(revprop = revprop)
        params_cm.update(Gamma = Gamma)
        params_cm.update(Gamma1 = Gamma1)
        params_cm.update(Gamma2 = Gamma2)
        params_cm.update(Gamma3 = Gamma3)
        params_cm.update(Gamma4 = Gamma4)
        params_cm.update(Gamma5 = Gamma5)
        params_cm.update(Gamma6 = Gamma6)
        params_cm.update(Gamma7 = Gamma7)
        params_cm.update(Gamma8 = Gamma8)
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
        #State setpoint
        x_sp_cm = np.array(spdata[:CM_STATE_LEN])
        #State error
        x_er_cm = x_cm - x_sp_cm
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
        #Input setpoint
        u_sp_cm = np.array(spdata[CM_STATE_LEN:])
        #Input error
        u_er_cm = u_cm - u_sp_cm
        #Update control model dictionary
        self.cm_dict.update(x_cm = x_cm)
        self.cm_dict.update(x_sp_cm = x_sp_cm)
        self.cm_dict.update(x_er_cm = x_er_cm)
        self.cm_dict.update(u_cm = u_cm)
        self.cm_dict.update(u_sp_cm = u_sp_cm)
        self.cm_dict.update(u_er_cm = u_er_cm)
        self.cm_dict.update(params_cm = params_cm)
        #Update physical variables dictionary
        self.phys_dict.update(rxtime = rxtime)
        self.phys_dict.update(phi = phi)
        self.phys_dict.update(theta = theta)
        self.phys_dict.update(psi = psi)
        self.phys_dict.update(alphadot = alphadot)
        self.phys_dict.update(sigmara = sigmara)
        self.phys_dict.update(deltara = deltara)
        self.phys_dict.update(sigmala = sigmala)
        self.phys_dict.update(deltala = deltala)
        self.phys_dict.update(deltaa = deltaa)
        self.phys_dict.update(sigmae = sigmae)
        self.phys_dict.update(deltae = deltae)
        self.phys_dict.update(sigmar = sigmar)
        self.phys_dict.update(deltar = deltar)
        self.phys_dict.update(deltat = deltat)
        self.phys_dict.update(hmacb = hmacb)
        self.phys_dict.update(qbar = qbar)
        self.phys_dict.update(Ixx = Ixx)
        self.phys_dict.update(Ixy = Ixy)
        self.phys_dict.update(Ixz = Ixz)
        self.phys_dict.update(Iyy = Iyy)
        self.phys_dict.update(Iyz = Iyz)
        self.phys_dict.update(Izz = Izz)
        self.phys_dict.update(mass = mass)
        self.phys_dict.update(Gamma = Gamma)
        self.phys_dict.update(Gamma1 = Gamma1)
        self.phys_dict.update(Gamma2 = Gamma2)
        self.phys_dict.update(Gamma3 = Gamma3)
        self.phys_dict.update(Gamma4 = Gamma4)
        self.phys_dict.update(Gamma5 = Gamma5)
        self.phys_dict.update(Gamma6 = Gamma6)
        self.phys_dict.update(Gamma7 = Gamma7)
        self.phys_dict.update(Gamma8 = Gamma8)
        self.phys_dict.update(sigmaf = sigmaf)
        self.phys_dict.update(deltaf = deltaf)
        self.phys_dict.update(deltam = deltam)
        self.phys_dict.update(revprop = revprop)
        self.phys_dict.update(rho = rho)
        self.phys_dict.update(grav = grav)
        self.phys_dict.update(stall = stall)

    def _build(self):
        x_sp_cm   = self.cm_dict['x_sp_cm']
        u_sp_cm   = self.cm_dict['u_sp_cm']
        params_cm = self.cm_dict['params_cm']
        rxtime    = self.phys_dict['rxtime']
        #Analytic non-linear control model
        _cm_sys = ctl.iosys.NonlinearIOSystem(self._update_anl, inputs=self.inputs_str, outputs=self.states_str, states=self.states_str, params=params_cm, name='ANL')
        #Linearized analytic non-linear control model
        cm_sys = ctl.iosys.linearize(_cm_sys, xeq=x_sp_cm.squeeze(), ueq=u_sp_cm.squeeze(), params=params_cm, name='LANL')
        #Update control model dictionary
        self.cm_dict.update(cm_sys = cm_sys)

    def _build_pipe_data(self):
        #Actuation pipe data
        x_cm    = self.cm_dict['x_cm']
        x_sp_cm = self.cm_dict['x_sp_cm']
        x_er_cm = self.cm_dict['x_er_cm']
        u_cm    = self.cm_dict['u_cm']
        u_sp_cm = self.cm_dict['u_sp_cm']
        u_er_cm = self.cm_dict['u_er_cm']
        cm_sys  = self.cm_dict['cm_sys']
        rxtime  = self.phys_dict['rxtime']
        actdata = (cm_sys, x_cm, x_sp_cm, x_er_cm, u_cm, u_sp_cm, u_er_cm)
        #CSV pipe data
        self.csvdata[0]  = rxtime
        self.csvdata[1:] = x_cm.squeeze()
        return actdata

''' CONTROL MODEL MODULE '''
class ControlModel():

    def __init__(self):
        self.ALCM   = ALCM()
        self.ANLCM  = ANLCM()
        self.LANLCM = LANLCM()
