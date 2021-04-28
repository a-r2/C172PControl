import multiprocessing as mp
import numpy as np
import math

from settings import *
from scipy import interpolate
from utils import *

''' PARAMETERS '''
BW_FT                 = 35.8000 #wing span
CBARW_FT              = 4.9000 #wing chord
PROP_DIAM_IN          = 75.0000 #propeller diameter
SH_SQFT               = 21.9000 #horizontal tail area
SV_SQFT               = 16.5000 #vertical tail area
SW_SQFT               = 174.0000 #wing area
SPIRAL_PROPWASH_COEFF = 0.2500 #coefficient due to the propeller induced velocity

#Drag coefficient due to ground effect
#Column 0: h_b_mac_ft
kCDge = np.array(   
                    [
                    [0.0000, 0.4800],
                    [0.1000, 0.5150],
                    [0.1500, 0.6290],
                    [0.2000, 0.7090],
                    [0.3000, 0.8150],
                    [0.4000, 0.8820],
                    [0.5000, 0.9280],
                    [0.6000, 0.9620],
                    [0.7000, 0.9880],
                    [0.8000, 1.0000],
                    [0.9000, 1.0000],
                    [1.0000, 1.0000],
                    [1.1000, 1.0000]
                    ]
                )
kCDge_interp = interpolate.interp1d(kCDge[:,0], kCDge[:,1], bounds_error=False, fill_value=(kCDge[0,1], kCDge[-1,1]))

#Drag coefficient due to flaps position
#Column 0: flaps_pos_deg
CD2 = np.array(
                [
                [0.0000, 0.0000],
                [10.0000, 0.0070],
                [20.0000, 0.0120],
                [30.0000, 0.0180]
                ]
              )
CD2_interp = interpolate.interp1d(CD2[:,0], CD2[:,1], bounds_error=False, fill_value=(CD2[0,1], CD2[-1,1]))

#Drag coefficient due to angle of attack and flaps position
#Column 0: alpha_rad | Row 0: flaps_pos_deg
CD3 = np.array(   
                [
                [np.NaN, 0.0000, 10.0000, 20.0000, 30.0000],
                [-0.0873, 0.0041, 0.0000, 0.0005, 0.0014],
                [-0.0698, 0.0013, 0.0004, 0.0025, 0.0041],
                [-0.0524, 0.0001, 0.0023, 0.0059, 0.0084],
                [-0.0349, 0.0003, 0.0057, 0.0108, 0.0141],
                [-0.0175, 0.0020, 0.0105, 0.0172, 0.0212],
                [0.0000 , 0.0052, 0.0168, 0.0251, 0.0299],
                [0.0175 , 0.0099, 0.0248, 0.0346, 0.0402],
                [0.0349 , 0.0162, 0.0342, 0.0457, 0.0521],
                [0.0524 , 0.0240, 0.0452, 0.0583, 0.0655],
                [0.0698 , 0.0334, 0.0577, 0.0724, 0.0804],
                [0.0873 , 0.0442, 0.0718, 0.0881, 0.0968],
                [0.1047 , 0.0566, 0.0874, 0.1053, 0.1148],
                [0.1222 , 0.0706, 0.1045, 0.124 , 0.1343],
                [0.1396 , 0.0860, 0.1232, 0.1442, 0.1554],
                [0.1571 , 0.0962, 0.1353, 0.1573, 0.1690],
                [0.1745 , 0.1069, 0.1479, 0.1708, 0.1830],
                [0.1920 , 0.1180, 0.1610, 0.1849, 0.1975],
                [0.2094 , 0.1298, 0.1746, 0.1995, 0.2126],
                [0.2269 , 0.1424, 0.1892, 0.2151, 0.2286],
                [0.2443 , 0.1565, 0.2054, 0.2323, 0.2464],
                [0.3491 , 0.2537, 0.3298, 0.3755, 0.3983],
                [0.5236 , 0.4500, 0.5850, 0.6660, 0.7065],
                [0.6981 , 0.7000, 0.9100, 1.0360, 1.0990],
                [0.8727 , 1.0000, 1.3000, 1.4800, 1.5700],
                [1.0472 , 1.3500, 1.7550, 1.9980, 2.1195],
                [1.2217 , 1.5000, 1.9500, 2.2200, 2.3550],
                [1.3963 , 1.5700, 2.0410, 2.3236, 2.4649],
                [1.5710 , 1.6000, 2.0800, 2.3680, 2.5120]
                ]
              )
CD3_trans = CD3.transpose()
CD3_interp = interpolate.interp2d(CD3_trans[0,1:], CD3_trans[1:,0], CD3_trans[1:,1:])

#Side force coefficient due to side-slip angle and flaps position
#Column 0: beta_rad | Row 0: flaps_pos_deg
CY1 = np.array(
                [
                [np.NaN, 0.0000, 30.0000],
                [-0.3490, 0.1370, 0.1060],
                [0.0000, 0.0000, 0.0000],
                [0.3490, -0.1370, -0.1060]
                ]
              )
CY1_trans = CY1.transpose()
CY1_interp = interpolate.interp2d(CY1_trans[0,1:], CY1_trans[1:,0], CY1_trans[1:,1:])

#Lift coefficient due to ground effect
#Column 0: h_b_mac_ft
kCLge = np.array(   
                    [
                    [0.0000, 1.2030],
                    [0.1000, 1.1270],
                    [0.1500, 1.0900],
                    [0.2000, 1.0730],
                    [0.3000, 1.0460],
                    [0.4000, 1.0280],
                    [0.5000, 1.0190],
                    [0.6000, 1.0130],
                    [0.7000, 1.0080],
                    [0.8000, 1.0060],
                    [0.9000, 1.0030],
                    [1.0000, 1.0020],
                    [1.1000, 1.0000]
                    ]
                )
kCLge_interp = interpolate.interp1d(kCLge[:,0], kCLge[:,1], bounds_error=False, fill_value=(kCLge[0,1], kCLge[-1,1]))

#Lift coefficient due to alpha and aerodynamic hysteresis
#Column 0: alpha_rad | Row 0: stall_hyst_norm
CL1 = np.array(   
                [
                [np.NaN, 0.0000, 1.0000],
                [-0.0900, -0.2200, -0.2200],
                [0.0000, 0.2500, 0.2500],
                [0.0900, 0.7300, 0.7300],
                [0.1000, 0.8300, 0.7800],
                [0.1200, 0.9200, 0.7900],
                [0.1400, 1.0200, 0.8100],
                [0.1600, 1.0800, 0.8200],
                [0.1700, 1.1300, 0.8300],
                [0.1900, 1.1900, 0.8500],
                [0.2100, 1.2500, 0.8600],
                [0.2400, 1.3500, 0.8800],
                [0.2600, 1.4400, 0.9000],
                [0.2800, 1.4700, 0.9200],
                [0.3000, 1.4300, 0.9500],
                [0.3200, 1.3800, 0.9900],
                [0.3400, 1.3000, 1.0500],
                [0.3600, 1.1500, 1.1500],
                [0.5200, 1.4700, 1.4700],
                [0.7000, 1.6500, 1.6500],
                [0.8700, 1.4700, 1.4700],
                [1.0500, 1.1700, 1.1700],
                [1.5700, 0.0100, 0.0100]
                ]
              )
CL1_trans = CL1.transpose()
CL1_interp = interpolate.interp2d(CL1_trans[0,1:], CL1_trans[1:,0], CL1_trans[1:,1:])

#Lift coefficient due to flaps position
#Column 0: flaps_pos_deg
CL2 = np.array(
                [
                [0.0000, 0.0000],
                [10.0000, 0.2000],
                [20.0000, 0.3000],
                [30.0000, 0.3500]
                ]
              )
CL2_interp = interpolate.interp1d(CL2[:,0], CL2[:,1], bounds_error=False, fill_value=(CL2[0,1], CL2[-1,1]))

#Roll moment coefficient due to alpha wing
#Column 0: alpha_rad
Cl1 = np.array(
                [
                [0.2790, 1.0000],
                [0.2970, 3.5000]
                ]
              )
Cl1_interp = interpolate.interp1d(Cl1[:,0], Cl1[:,1], bounds_error=False, fill_value=(Cl1[0,1], Cl1[-1,1]))

#Roll moment coefficient due to flaps position
#Column 0: flaps_pos_deg
Cl31 = np.array(
                [
                [0.0000, 0.0798],
                [30.000, 0.1246]
                ]
              )
Cl31_interp = interpolate.interp1d(Cl31[:,0], Cl31[:,1], bounds_error=False, fill_value=(Cl31[0,1], Cl31[-1,1]))

#Roll moment coefficient due to flaps position (stall)
#Column 0: alpha_rad | Row 0: r_rad_sec
Cl32 = np.array(
                [
                [np.NaN, -0.1500, -0.1000, 0.0000, 0.1000, 0.1500],
                [0.2970, 35.0000, 30.0000, 1.0000, 30.0000, 35.0000],
                [0.5000, 5.0000, 5.0000, 1.0000, 5.0000, 5.0000]
                ]
              )
Cl32_trans = Cl32.transpose()
Cl32_interp = interpolate.interp2d(Cl32_trans[0,1:], Cl32_trans[1:,0], Cl32_trans[1:,1:])

#Roll moment coefficient due to flaps position
#Column 0: alpha_rad | Row 0: r_rad_sec
Cl33 = np.array(
                [
                [np.NaN, -0.1500, -0.1000, 0.0000, 0.1000, 0.1500],
                [0.2790, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000],
                [0.2970, 35.0000, 30.0000, 1.0000, 30.0000, 35.0000],
                [0.5000, 5.0000, 5.0000, 1.0000, 5.0000, 5.0000]
                ]
              )
Cl33_trans = Cl33.transpose()
Cl33_interp = interpolate.interp2d(Cl33_trans[0,1:], Cl33_trans[1:,0], Cl33_trans[1:,1:])

#Roll moment coefficient due to flaps position
#Column 0: alpha_rad | Row 0: stall_hyst_norm 
Cl4 = np.array(
                [
                [np.NaN, 0.0000, 1.0000],
                [0.2790, 1.0000, 0.3000],
                [0.2970, 0.3000, 0.3000],
                [0.6110, -0.1000, -0.1000]
                ]
              )
Cl4_trans = Cl4.transpose()
Cl4_interp = interpolate.interp2d(Cl4_trans[0,1:], Cl4_trans[1:,0], Cl4_trans[1:,1:])

#Pitch moment coefficient due to qbar_psf
#Column 0: qbar_psf 
Cm1 = np.array(
                [
                [13.6000, 0.0900],
                [21.2000, 0.0400]
                ]
              )
Cm1_interp = interpolate.interp1d(Cm1[:,0], Cm1[:,1], bounds_error=False, fill_value=(Cm1[0,1], Cm1[-1,1]))

#Pitch moment coefficient due to alpha_deg
#Column 0: alpha_deg 
Cm2 = np.array(
                [
                [20.0000, 1.0000],
                [25.0000, 0.6000],
                [35.0000, 0.4000],
                [45.0000, 0.5000],
                [55.0000, 0.4000],
                [65.0000, 0.2000],
                [90.0000, 0.1000]
                ]
              )
Cm2_interp = interpolate.interp1d(Cm2[:,0], Cm2[:,1], bounds_error=False, fill_value=(Cm2[0,1], Cm2[-1,1]))
    
#Pitch moment coefficient due to elev_pos_rad and alpha_deg
#Column 0: elev_pos_rad | Row 0: alpha_deg
Cm5 = np.array(
                [
                [np.NaN, 18.0000, 25.0000, 35.0000, 45.0000, 55.0000, 65.0000, 90.0000],
                [-0.4900, 1.0000, 0.5000, 0.2000, 0.1000, 0.1000, 0.1000, 0.1000],
                [0.0000, 1.0000, 0.6000, 0.3000, 0.1500, 0.1000, 0.1000, 0.1000],
                [0.4000, 1.0000, 0.9000, 0.8000, 0.7000, 0.6000, 0.5000, 0.4000]
                ]
              )
Cm5_trans = Cm5.transpose()
Cm5_interp = interpolate.interp2d(Cm5_trans[0,1:], Cm5_trans[1:,0], Cm5_trans[1:,1:])

#Pitch moment coefficient due to flaps_pos_deg
#Column 0: flaps_pos_deg
Cm6 = np.array(
                [
                [0.0000, 0.0000],
                [10.0000, -0.0654],
                [20.0000, -0.0981],
                [30.0000, -0.1140]
                ]
              )
Cm6_interp = interpolate.interp1d(Cm6[:,0], Cm6[:,1], bounds_error=False, fill_value=(Cm6[0,1], Cm6[-1,1]))
    
#Yaw moment coefficient due to beta_rad 
#Column 0: beta_rad 
Cn1 = np.array(
                [
                [-0.3490, -0.0205],
                [0.0000, 0.0000],
                [0.3490, 0.0205]
                ]
              )
Cn1_interp = interpolate.interp1d(Cn1[:,0], Cn1[:,1], bounds_error=False, fill_value=(Cn1[0,1], Cn1[-1,1]))

#Yaw moment coefficient due to r_rad_sec
#Column 0: r_rad_sec | Row 0: alpha_rad 
Cn4 = np.array(
                [
                [np.NaN, 0.2790, 0.4000],
                [-15.0000, 0.0000, 0.0000],
                [-5.0000, 0.0000, 0.0000],
                [-3.0000, 0.0000, -0.2500],
                [-1.0000, 0.0000, 0.0000],
                [0.0000, 0.0000, 0.0000],
                [1.0000, 0.0000, 0.0000],
                [3.0000, 0.0000, 0.2500],
                [5.0000, 0.0000, 0.0000],
                [15.0000, 0.0000, 0.0000]
                ]
              )
Cn4_trans = Cn4.transpose()
Cn4_interp = interpolate.interp2d(Cn4_trans[0,1:], Cn4_trans[1:,0], Cn4_trans[1:,1:])

#Yaw moment coefficient due to alpha_rad and beta_rad
#Column 0: alpha_rad | Row 0: beta_rad 
Cn5 = np.array(
                [
                [np.NaN, -0.3500, 0.0000, 0.3500],
                [0.0000, -0.0216, -0.0216, -0.0216],
                [0.0700, -0.0390, -0.0786, -0.0390],
                [0.0940, -0.0250, -0.0504, -0.0250]
                ]
              )
Cn5_trans = Cn5.transpose()
Cn5_interp = interpolate.interp2d(Cn5_trans[0,1:], Cn5_trans[1:,0], Cn5_trans[1:,1:])

#Multiplier of thrust coefficient due to advance_ratio
#Column 0: advance_ratio
CT = np.array(
                [
                [0.0000, 0.0680],
                [0.1000, 0.0680],
                [0.2000, 0.0670],
                [0.3000, 0.0660],
                [0.4000, 0.0640],
                [0.5000, 0.0620],
                [0.6000, 0.0590],
                [0.7000, 0.0540],
                [0.8000, 0.0430],
                [0.9000, 0.0310],
                [1.0000, 0.0190],
                [1.1000, 0.0080],
                [1.2000, -0.0010],        
                [1.3000, -0.0080],        
                [1.4000, -0.0190],        
                [1.5000, -0.0290],
                [1.6000, -0.0400],
                [1.7000, -0.0500],
                [1.8000, -0.0570],
                [1.9000, -0.0610],
                [2.0000, -0.0640],
                [2.1000, -0.0660],
                [2.2000, -0.0670],
                [2.3000, -0.0680],
                [5.0000, -0.0680]
                ]
             )
CT_interp = interpolate.interp1d(CT[:,0], CT[:,1], bounds_error=False, fill_value=(CT[0,1], CT[-1,1]))

#Multiplier of power coefficient due to advance_ratio
#Column 0: advance_ratio
CP = np.array(
                [
                [0.0000, 0.0580],
                [0.1000, 0.0620],
                [0.2000, 0.0600],
                [0.3000, 0.0580],
                [0.4000, 0.0520],
                [0.5000, 0.0457],
                [0.6000, 0.0436],
                [0.7000, 0.0420],
                [0.8000, 0.0372],
                [0.9000, 0.0299],
                [1.0000, 0.0202],
                [1.1000, -0.0111],
                [1.2000, -0.0075],        
                [1.3000, -0.0111],        
                [1.4000, -0.0202],        
                [1.5000, -0.0280],
                [1.6000, -0.0346],
                [1.7000, -0.0389],
                [1.8000, -0.0421],
                [1.9000, -0.0436],
                [2.0000, -0.0445],
                [2.1000, -0.0445],
                [2.2000, -0.0442],
                [2.3000, -0.0431],
                [2.4000, -0.0421],
                [5.0000, -0.0413]
                ]
             )
CP_interp = interpolate.interp1d(CP[:,0], CP[:,1], bounds_error=False, fill_value=(CP[0,1], CP[-1,1]))

''' PARAMETERS CONVERSION '''
PROP_DIAM_M = in_to_m(PROP_DIAM_IN)

''' LONGITUDINAL AERODYNAMIC FORCE (-DRAG) '''
def fxaero1(qbar_psf):
    #Longitudinal aerodynamic force contribution (-drag) offset
    return - qbar_psf * SW_SQFT * 0.0270

def fxaero2(qbar_psf, h_b_mac_ft, flaps_pos_deg):
    #Longitudinal aerodynamic force contribution (-drag) due to ground effect and flaps position
    return - qbar_psf * SW_SQFT * kCDge_interp(h_b_mac_ft) * CD2_interp(flaps_pos_deg)

def fxaero3(qbar_psf, h_b_mac_ft, alpha_rad, flaps_pos_deg):
    #Longitudinal aerodynamic force contribution (-drag) due to ground effect, angle of attack and flaps position
    return - qbar_psf * SW_SQFT * kCDge_interp(h_b_mac_ft) * CD3_interp(alpha_rad, flaps_pos_deg)

def fxaero4(qbar_psf, beta_rad):
    #Longitudinal aerodynamic force contribution (-drag) due to side-slip angle
    return - qbar_psf * SW_SQFT * abs(beta_rad) * 0.1500

''' TRANSVERSAL AERODYNAMIC FORCE '''
def fyaero1(qbar_psf, beta_rad, flaps_pos_deg):
    #Transversal aerodynamic force contribution due to side-slip angle and flaps position
    return qbar_psf * SW_SQFT * CY1_interp(beta_rad, flaps_pos_deg)

def fyaero2(qbar_psf, rudder_pos_rad):
    #Transversal aerodynamic force contribution due to rudder position
    return qbar_psf * SW_SQFT * rudder_pos_rad * 0.1500

''' VERTICAL AERODYNAMIC FORCE (-LIFT) '''
def fzaero1(qbar_psf, h_b_mac_ft, alpha_rad, stall_hyst_norm):
    #Vertical aerodynamic force contribution (-lift) due to ground effect, angle of attack and stall state
    return - qbar_psf * SW_SQFT * kCLge_interp(h_b_mac_ft) * CL1_interp(alpha_rad, stall_hyst_norm)

def fzaero2(qbar_psf, h_b_mac_ft, flaps_pos_deg):
    #Vertical aerodynamic force contribution (-lift) due to ground effect and flaps position
    return - qbar_psf * SW_SQFT * kCLge_interp(h_b_mac_ft) * CL2_interp(flaps_pos_deg)

def fzaero3(qbar_psf, elev_pos_rad):
    #Vertical aerodynamic force contribution (-lift) due to elevators position
    return - qbar_psf * SW_SQFT * elev_pos_rad * 0.4300

def fzaero4(qbar_psf, q_rad_sec, ci2vel):
    #Vertical aerodynamic force contribution (-lift) due to pitch velocity
    return - qbar_psf * SW_SQFT * q_rad_sec * ci2vel * 3.9000

def fzaero5(qbarUW_psf, alphadot_rad_sec, ci2vel):
    #Vertical aerodynamic force contribution (-lift) due to rate of change of angle of attack
    return - qbarUW_psf * SW_SQFT * alphadot_rad_sec * ci2vel * 1.7000

''' LONGITUDINAL AERODYNAMIC MOMENT '''
def mxaero1(qbar_psf, beta_rad, alpha_rad):
    #Longitudinal aerodynamic moment contribution due to side-slip angle and angle of attack
    return qbar_psf * SW_SQFT * BW_FT * beta_rad * -0.0920 * Cl1_interp(alpha_rad)

def mxaero2(qbar_psf, bi2vel, p_rad_sec):
    #Longitudinal aerodynamic moment contribution due to longitudinal angular velocity
    return qbar_psf * SW_SQFT * BW_FT * bi2vel * p_rad_sec * -0.4840

def mxaero3(qbar_psf, bi2vel, r_rad_sec, flaps_pos_deg, alpha_rad, stall_hyst_norm):
    #Longitudinal aerodynamic moment contribution due to vertical angular velocity, flaps position, angle of attack and stall state
    if stall_hyst_norm:
        return qbar_psf * SW_SQFT * BW_FT * bi2vel * r_rad_sec * Cl31_interp(flaps_pos_deg) * Cl32_interp(alpha_rad, r_rad_sec)
    else:
        return qbar_psf * SW_SQFT * BW_FT * bi2vel * r_rad_sec * Cl31_interp(flaps_pos_deg) * Cl33_interp(alpha_rad, r_rad_sec)

def mxaero4(qbar_psf, left_aileron_pos_rad, right_aileron_pos_rad, alpha_rad, stall_hyst_norm):
    #Longitudinal aerodynamic moment contribution due to ailerons position
    return qbar_psf * SW_SQFT * BW_FT * averaged_ailerons(left_aileron_pos_rad, right_aileron_pos_rad) * 0.2290 * Cl4_interp(alpha_rad, stall_hyst_norm)

def mxaero5(qbar_psf, rudder_pos_rad):
    #Longitudinal aerodynamic moment due to rudder position
    return qbar_psf * SW_SQFT * BW_FT * rudder_pos_rad * 0.0147

''' TRANSVERSAL AERODYNAMIC MOMENT '''
def myaero1(qbar_psf):
    #Transversal aerodynamic moment contribution offset
    return qbar_psf * SW_SQFT * CBARW_FT * Cm1_interp(qbar_psf)

def myaero2(qbar_psf, alpha_deg, alpha_rad):
    #Transversal aerodynamic moment contribution due to angle of attack
    return qbar_psf * SW_SQFT * CBARW_FT * math.sin(alpha_rad) * -1.8000 * Cm2_interp(alpha_deg)

def myaero3(qbar_psf, ci2vel, q_rad_sec):
    #Transversal aerodynamic moment contribution due to transversal angular velocity
    return qbar_psf * SW_SQFT * CBARW_FT * ci2vel * q_rad_sec * -12.4000

def maeroy4(qbar_psf, flaps_pos_deg):
    #Transversal aerodynamic moment contribution due to flaps position
    return qbar_psf * SW_SQFT * CBARW_FT * Cm6_interp(flaps_pos_deg) * 0.7000

def maeroy5(qbarUW_psf, ci2vel, alphadot_rad_sec):
    #Transversal aerodynamic moment contribution due to rate of change of angle of attack
    return qbarUW_psf  * SW_SQFT * CBARW_FT * ci2vel * alphadot_rad_sec * -7.2700

def maeroy6(qbar_induced_psf, elev_pos_rad, alpha_deg):
    #Transversal aerodynamic moment contribution due to elevator position and angle of attack
    return qbar_induced_psf * SW_SQFT * CBARW_FT * elev_pos_rad * -1.2800 * Cm5_interp(elev_pos_rad, alpha_deg)


''' VERTICAL AERODYNAMIC MOMENT '''
def mzaero1(qbar_psf, beta_rad):
    #Vertical aerodynamic moment contribution due to side-slip angle
    return qbar_psf * SW_SQFT * BW_FT * Cn1_interp(beta_rad)

def mzaero2(qbar_psf, bi2vel, r_rad_sec):
    #Vertical aerodynamic moment contribution due to vertical angular velocity 
    return qbar_psf * SW_SQFT * BW_FT * bi2vel * r_rad_sec * -0.0937

def mzaero3(qbar_psf, bi2vel, r_rad_sec, alpha_rad):
    #Vertical aerodynamic moment contribution due to vertical angular velocity and angle of attack
    return qbar_psf * SW_SQFT * BW_FT * bi2vel * Cn4_interp(r_rad_sec, alpha_rad)

def mzaero4(qbar_psf, left_aileron_pos_rad, right_aileron_pos_rad, alpha_rad, beta_rad):
    #Vertical aerodynamic moment contribution due to aileron position, angle of attack and side-slip angle
    return qbar_psf * SW_SQFT * BW_FT * averaged_ailerons(left_aileron_pos_rad, right_aileron_pos_rad) * Cn5_interp(alpha_rad, beta_rad)

def mzaero5(qbar_induced_psf, rudder_pos_rad):
    #Vertical aerodynamic moment contribution due to rudder position
    return qbar_induced_psf * SW_SQFT * BW_FT * rudder_pos_rad * -0.0645

def mzaero6(qbar_propwash_psf):
    #Vertical aerodynamic moment contribution due to spiralling propwash
    return qbar_propwash_psf * SW_SQFT * BW_FT * -0.0500 * SPIRAL_PROPWASH_COEFF

''' PROPULSIVE FORCE '''
def fxthrust(advance_ratio, density, rpm_prop):
    #Engine (160 HP) thrust
    return CT_interp(advance_ratio) * slugft3_to_kgm3(density) * (rpm_to_rps(rpm_prop) ** 2) * (PROP_DIAM_M ** 4)

''' GRAVITATIONAL FORCE '''
def fgravity(quat, mass, gravity):
    #Gravitational forces
    euler = attquat_to_euler(quat)
    return vehicle_to_body(euler[0], euler[1], euler[2]).rotate(mass * gravity * np.array([0, 0, 1]))

''' MISC ''' 

def engine_power(advance_ratio, density, rpm_prop):
    #Engine (160 HP) power
    return CP_interp(advance_ratio) * slugft3_to_kgm3(density) * (rpm_to_rps(rpm_prop) ** 3) * (PROP_DIAM_M ** 5)

''' MODEL '''
STATE_LEN = 13 #[pn_dot, pe_dot, pd_dot, q0_dot, q1_dot, q2_dot, q3_dot, u_dot, v_dot, w_dot, p_dot, q_dot, r_dot]
INPUT_LEN = 5 #[delta_a, delta_e, delta_f, delta_r, delta_t]

class Model():
    def __init__(self, rx2nlm_out, rx2lm_out, nlm2act_in, lm2act_in, nlm2csv_in, lm2csv_in, eq2lm_out, event_start):
        self.csvnlm = np.zeros((MODEL_HZ, STATE_LEN + 1)) #array for storing non-linear model
        self.csvlm = np.zeros((MODEL_HZ, STATE_LEN + 1)) #array for storing linear model
        self.proc1 = mp.Process(target=self.non_linear, args=(rx2nlm_out, nlm2act_in, nlm2csv_in, event_start), daemon=True) #process for calculating non-linear model
        self.proc2 = mp.Process(target=self.linear, args=(rx2lm_out, lm2act_in, lm2csv_in, event_start), daemon=True) #process for calculating linear model
        self.proc1.start()
        self.proc2.start()
    def non_linear(self, rx2nlm_out, nlm2act_in, nlm2csv_in, event_start):
        #Analytic non-linear model
        event_start.wait() #wait for simulation start event
        while True:
            rxdata      = rx2nlm_out.recv() #receive RX telemetry
            framescount = rxdata.shape[0]
            for i in range(framescount):                
                time  = rxdata[i,0]
                phi   = rxdata[i,16]
                theta = rxdata[i,18]
                psi   = rxdata[i,20]
                u     = rxdata[i,30]
                v     = rxdata[i,31]
                w     = rxdata[i,32]
                p     = rxdata[i,42]
                q     = rxdata[i,43]
                r     = rxdata[i,44]
                fx    = rxdata[i,59]
                fy    = rxdata[i,64]
                fz    = rxdata[i,69]
                l     = rxdata[i,74]
                m     = rxdata[i,79]
                n     = rxdata[i,84]
            
                x_dot = np.zeros(STATE_LEN)
                (q0, q1, q2, q3) = euler_to_attquat(np.array([phi, theta, psi]))
                x_dot[0:3] = body_to_vehicle(phi, theta, psi) * np.array([u, v, w])
                x_dot[3:6] = np.array([r*v-q*w, p*w-r*u, q*u-p*v]) + np.array([fx, fy, fz]) / m
                x_dot[6:10] = 0.5 * np.array([[0, -p, -q, -r], [p, 0, r, -q], [q, -r, 0, p], [r, q, -p, 0]]) * np.array([q0, q1, q2, q3])
                x_dot[10:13] = np.array([gamma1*p*q-gamma2*q*r, gamma5*p*r-gamma6*(p**2-r**2), gamma7*p*q-gamma1*q*r]) + np.array([gamma3*l+gamma4*n, m/Iyy, gamma4*l+gamma8*n])

                self.csvnlm[i,:] = [time, x_dot]

            nlm2csv_in.send(self.csvnlm[:framescount,:]) #send calculated non-linear model to store in CSV
            self.csvnlm = np.empty((MODEL_HZ, STATE_LEN + 1)) #empty array 

    def linear(self, rx2lm_out, lm2act_in, lm2csv_in, eq2lm_out, event_start):
        #Analytic linear model
        event_start.wait() #wait for simulation start event
        while True:
            rxdata      = rx2lm_out.recv() #receive RX telemetry
            eqdata      = eq2lm_out.recv() #receive equilibrium point
            framescount = rxdata.shape[0]
            for i in range(framescount):                
                time  = rxdata[i,0]
                phi   = rxdata[i,16]
                theta = rxdata[i,18]
                psi   = rxdata[i,20]
                u     = rxdata[i,30]
                v     = rxdata[i,31]
                w     = rxdata[i,32]
                p     = rxdata[i,42]
                q     = rxdata[i,43]
                r     = rxdata[i,44]
                fx    = rxdata[i,59]
                fy    = rxdata[i,64]
                fz    = rxdata[i,69]
                l     = rxdata[i,74]
                m     = rxdata[i,79]
                n     = rxdata[i,84]

                pn_eq    = eqdata[i,0]
                pe_eq    = eqdata[i,1]
                pd_eq    = eqdata[i,2]
                phi_eq   = eqdata[i,3]
                theta_eq = eqdata[i,4]
                psi_eq   = eqdata[i,5]
                u_eq     = eqdata[i,6]
                v_eq     = eqdata[i,7]
                w_eq     = eqdata[i,8]
                p_eq     = eqdata[i,9]
                q_eq     = eqdata[i,10]
                r_eq     = eqdata[i,11]
                (q0_eq, q1_eq, q2_eq, q3_eq) = euler_to_attquat(np.array([phi_eq, theta_eq, psi_eq]))

                x_dot = np.zeros(STATE_LEN)
                A     = np.zeros((STATE_LEN, STATE_LEN))
                B     = np.zeros((STATE_LEN, INPUT_LEN))
                (q0, q1, q2, q3) = euler_to_attquat(np.array([phi, theta, psi]))
                A[1,:] = [0, 0, 0, 2*(q0_eq*u_eq-q3_eq*v_eq+q2_eq*w_eq), 2*(q1_eq*u_eq+q2_eq*v_eq+q3_eq*w_eq), 2*(-q2_eq*u_eq+q1_eq*v_eq+q0_eq*w_eq), 2*(-q3_eq*u_eq-q0_eq*v_eq+q1_eq*w_eq), (q0_eq**2+q1_eq**2-q2_eq**2-q3_eq**2), 2*(q1_eq*q2_eq-q0_eq*q3_eq), 2*(q1_eq*q3_eq+q0_eq*q2_eq), 0, 0, 0]
                A[2,:] = [0, 0, 0, 2*(q3_eq*u_eq+q0_eq*v_eq-q1_eq*w_eq), 2*(q2_eq*u_eq-q1_eq*v_eq-q0_eq*w_eq), 2*(q1_eq*u_eq+q2_eq*v_eq+q3_eq*w_eq), 2*(q0_eq*u_eq-q3_eq*v_eq+q2_eq*w_eq), 2*(q1_eq*q2_eq+q0_eq*q3_eq), (q0_eq**2+q2_eq**2-q1_eq**2-q3_eq**2), 2*(q2_eq*q3_eq-q0_eq*q1_eq), 0, 0, 0]
                A[3,:] = [0, 0, 0, 2*(-q2_eq*u_eq+q1_eq*v_eq+q0_eq*w_eq), 2*(q3_eq*u_eq-q0_eq*v_eq-q1_eq*w_eq), 2*(-q0_eq*u_eq+q3_eq*v_eq-q2_eq*w_eq), 2*(q1_eq*u_eq+q2_eq*v_eq+q3_eq*w_eq), 2*(q1_eq*q3_eq-q0_eq*q2_eq), 2*(q2_eq*q3_eq+q0_eq*q1_eq), (q0_eq**2+q3_eq**2-q1_eq**2-q2_eq**2), 0, 0, 0]
                A[4,:] = [0, 0, 0, 0, -0.5*p_eq, -0.5*q_eq, -0.5*r_eq, 0, 0, 0, -0.5*q1_eq, -0.5*q2_eq, -0.5*q3_eq]
                A[5,:] = [0, 0, 0, 0.5*p_eq, 0, 0.5*r_eq, -0.5*q_eq, 0, 0, 0, 0.5*q0_eq, -0.5*q3_eq, 0.5*q2_eq]
                A[6,:] = [0, 0, 0, 0.5*q_eq, -0.5*r_eq, 0, 0.5*p_eq, 0, 0, 0, 0.5*q3_eq, 0.5*q0_eq, -0.5*q1_eq]
                A[7,:] = [0, 0, 0, 0.5*r_eq, 0.5*q_eq, -0.5*p_eq, 0, 0, 0, 0, -0.5*q2_eq, 0.5*q1_eq, 0.5*q0_eq]

                self.csvlm[i,:] = [time, x_dot]

            lm2csv_in.send(self.csvlm[:framescount,:]) #send calculated linear model to store in CSV
            self.csvlm = np.empty((MODEL_HZ, STATE_LEN + 1)) #empty array 
