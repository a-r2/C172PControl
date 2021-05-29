import multiprocessing as mp
import numpy as np
from math import atan2, sin, sqrt

from settings import *
from scipy import interpolate
from utils import *

''' FLIGHTGEAR MODEL PARAMETERS '''
BW_IMP            = 35.8000 #wing span [ft]
CW_IMP            = 4.9000 #wing chord [ft]
N_MAX             = 45.0000 #maximum propeller revolutions per second [RPS]
N_MIN             = 10.0000 #minimum propeller revolutions per second [RPS]
D_PROP_IMP        = 75.0000 #propeller diameter [in]
STH_IMP           = 21.9000 #horizontal tail area [ft^2]
STV_IMP           = 16.5000 #vertical tail area [ft^2]
SW_IMP            = 174.0000 #wing area [ft^2]
C_SPIRAL_PROPWASH = 0.2500 #coefficient due to the propeller induced velocity
DELTA_M_MIN       = 0.1000 #minimum delta_m to maintain the engine running [-]

''' MODEL PARAMETERS CONVERSION '''
BW_SI      = ft_to_m(BW_IMP) #wing span [m]
CW_SI      = ft_to_m(CW_IMP) #chord span [m]
D_PROP_IMP = in_to_ft(D_PROP_IMP) #propeller diameter [ft]
D_PROP_SI  = in_to_m(D_PROP_IMP) #propeller diameter [m]
STH_SI     = ft_to_m(sqrt(STH_IMP)) ** 2 #horizontal tail area [m^2]
STV_SI     = ft_to_m(sqrt(STV_IMP)) ** 2 #vertical tail area [m^2]
SW_SI      = ft_to_m(sqrt(SW_IMP)) ** 2 #wing area [m^2]

A_PROP_SI  = propeller_area(D_PROP_SI) #propeller area [m^2]
A_PROP_IMP = propeller_area(m_to_ft(D_PROP_SI)) #propeller area [ft^2]

''' DYNAMIC COEFFICIENTS TABLES '''
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

''' AERODYNAMIC DRAG FORCE COEFFICIENTS '''
def CD1():
    #Aerodynamic drag coefficient 1 
    return 0.0270

def CD2(h_b_mac_ft, flaps_pos_deg):
    #Aerodynamic drag coefficient 2
    return kCDge_interp(h_b_mac_ft) * CD2_interp(flaps_pos_deg)

def CD3(h_b_mac_ft, alpha_rad, flaps_pos_deg):
    #Aerodynamic drag coefficient 3
    return kCDge_interp(h_b_mac_ft) * CD3_interp(alpha_rad, flaps_pos_deg)

def CD4(beta_rad):
    #Aerodynamic drag coefficient 4
    return abs(beta_rad) * 0.1500

''' AERODYNAMIC CROSSWIND FORCE COEFFICIENTS '''
def CC1(beta_rad, flaps_pos_deg):
    #Aerodynamic crosswind coefficient 1
    return CY1_interp(beta_rad, flaps_pos_deg)

def CC2(rudder_pos_rad):
    #Aerodynamic crosswind coefficient 2
    return rudder_pos_rad * 0.1500

''' AERODYNAMIC LIFT FORCE COEFFICIENTS '''
def CL1(h_b_mac_ft, alpha_rad, stall_hyst_norm):
    #Aerodynamic lift coefficient 1
    return kCLge_interp(h_b_mac_ft) * CL1_interp(alpha_rad, stall_hyst_norm)

def CL2(h_b_mac_ft, flaps_pos_deg):
    #Aerodynamic lift coefficient 2
    return kCLge_interp(h_b_mac_ft) * CL2_interp(flaps_pos_deg)

def CL3(elev_pos_rad):
    #Aerodynamic lift coefficient 3
    return elev_pos_rad * 0.4300

def CL4(q_rad_sec, ci2vel):
    #Aerodynamic lift coefficient 4
    return q_rad_sec * ci2vel * 3.9000

def CL5(alphadot_rad_sec, ci2vel):
    #Aerodynamic lift coefficient 5
    return alphadot_rad_sec * ci2vel * 1.7000

''' AERODYNAMIC ROLL MOMENT COEFFICIENTS '''
def Cl1(beta_rad, alpha_rad):
    #Aerodynamic roll coefficient 1
    return beta_rad * -0.0920 * Cl1_interp(alpha_rad)

def Cl2(bi2vel, p_rad_sec):
    #Aerodynamic roll coefficient 2
    return bi2vel * p_rad_sec * -0.4840

def Cl3(bi2vel, r_rad_sec, flaps_pos_deg, alpha_rad, stall_hyst_norm):
    #Aerodynamic roll coefficient 3
    if stall_hyst_norm:
        return bi2vel * r_rad_sec * Cl31_interp(flaps_pos_deg) * Cl32_interp(alpha_rad, r_rad_sec)
    else:
        return bi2vel * r_rad_sec * Cl31_interp(flaps_pos_deg) * Cl33_interp(alpha_rad, r_rad_sec)

def Cl4(left_aileron_pos_rad, right_aileron_pos_rad, alpha_rad, stall_hyst_norm):
    #Aerodynamic roll coefficient 4
    return averaged_ailerons(left_aileron_pos_rad, right_aileron_pos_rad) * 0.2290 * Cl4_interp(alpha_rad, stall_hyst_norm)

def Cl5(rudder_pos_rad):
    #Aerodynamic roll coefficient 5
    return rudder_pos_rad * 0.0147

''' AERODYNAMIC PITCH MOMENT COEFFICIENTS '''
def Cm1(qbar_psf):
    #Aerodynamic pitch coefficient 1
    return Cm1_interp(qbar_psf)

def Cm2(alpha_deg, alpha_rad):
    #Aerodynamic pitch coefficient 2
    return sin(alpha_rad) * -1.8000 * Cm2_interp(alpha_deg)

def Cm3(ci2vel, q_rad_sec):
    #Aerodynamic pitch coefficient 3
    return ci2vel * q_rad_sec * -12.4000

def Cm4(flaps_pos_deg):
    #Aerodynamic pitch coefficient 4
    return Cm6_interp(flaps_pos_deg) * 0.7000

def Cm5(ci2vel, alphadot_rad_sec):
    #Aerodynamic pitch coefficient 5
    return ci2vel * alphadot_rad_sec * -7.2700

def Cm6(elev_pos_rad, alpha_deg):
    #Aerodynamic pitch coefficient 6
    return elev_pos_rad * -1.2800 * Cm5_interp(elev_pos_rad, alpha_deg)

''' AERODYNAMIC YAW MOMENT COEFFICIENTS '''
def Cn1(beta_rad):
    #Aerodynamic yaw coefficient 1
    return Cn1_interp(beta_rad)

def Cn2(bi2vel, r_rad_sec):
    #Aerodynamic yaw coefficient 2
    return bi2vel * r_rad_sec * -0.0937

def Cn3(bi2vel, r_rad_sec, alpha_rad):
    #Aerodynamic yaw coefficient 3
    return bi2vel * Cn4_interp(r_rad_sec, alpha_rad)

def Cn4(left_aileron_pos_rad, right_aileron_pos_rad, alpha_rad, beta_rad):
    #Aerodynamic yaw coefficient 4
    return averaged_ailerons(left_aileron_pos_rad, right_aileron_pos_rad) * Cn5_interp(alpha_rad, beta_rad)

def Cn5(rudder_pos_rad):
    #Aerodynamic yaw coefficient 5
    return rudder_pos_rad * -0.0645

def Cn6():
    #Aerodynamic yaw coefficient 6
    return -0.0500 * C_SPIRAL_PROPWASH

''' PROPULSIVE FORCE '''
def thrust_eng(advance_ratio, density, rpm_prop):
    #Engine (160 HP) thrust
    return CT_interp(advance_ratio) * slugft3_to_kgm3(density) * (rpm_to_rps(rpm_prop) ** 2) * (PROP_DIAM_M ** 4)

''' GRAVITATIONAL FORCE '''
def gravity_body(quat, mass, gravity):
    #Gravitational force in body frame
    euler = attquat_to_euler(quat)
    return vehicle_to_body(euler[0], euler[1], euler[2]).rotate(mass * gravity * np.array([0, 0, 1]))

''' PROPULSION POWER ''' 
def power_eng(advance_ratio, density, rpm_prop):
    #Engine (160 HP) power
    return CP_interp(advance_ratio) * slugft3_to_kgm3(density) * (rpm_to_rps(rpm_prop) ** 3) * (PROP_DIAM_M ** 5)

''' CONTROL MODEL PARAMETERS '''
STATE_LEN = 13 #[pn, pe, pd, q0, q1, q2, q3, u, v, w, p, q, r]
INPUT_LEN = 5 #[deltaa, deltae, deltaf, deltar, deltat]

''' ASSUMPTIONS FOR THE ANALYTIC LINEAR CONTROL MODEL'''
G0_IMP = 32.17405 #gravitational acceleration [ft/s^2]

''' MISC FUNCTIONS FOR THE ANALYTIC CONTROL MODELS'''
def alpha_acm(u, w):
    #Angle of attack (alpha) as defined in the analytic control models
    return atan2(w, u)

def beta_acm(v, Vauw):
    #Side-slip angle (beta) as defined in the analytic control models
    return atan2(v, Vauw)

def J_acm(u, n):
    #Advance ratio (J) as defined in the analytic control models
    if n == 0:
        return 0
    else:
        return u / (n * PROP_DIAM_M)

def Va_acm(u, v, w):
    #Aerodynamic velocity (Va) as defined in the analytic control models
    return sqrt((u ** 2) + (v ** 2) + (w ** 2))

def Vauw_acm(u, w):
    #Aerodynamic velocity from components u and w (Vauw) as defined in the analytic control models
    return sqrt((u ** 2) + (w ** 2))

def Vind2_acm(u, rho, T):
    #Squared propulsion induced velocity (Vind^2) as defined in the analytic control models
    return u * abs(u) + ((2 * T) / (rho * PROP_AREA_SI))

def Vind_acm(u, rho, T):
    #Propulsion induced velocity (Vind) as defined in the analytic control models
    Vind2 = Vind2_acm(u, rho, T)
    if Vind2 >= 0:
        return sqrt(Vind2)
    else:
        return - sqrt(- Vind2)

def Vprop_acm(u, Vind):
    #Propeller induced velocity (Vprop) as defined in the analytic control models
    return Vind - u

def qbar_acm(rho, Va):
    #Dynamic pressure (q) as defined in the analytic control models
    return 0.5 * rho * (Va ** 2)

def qbaruw_acm(rho, Vauw):
    #Dynamic pressure from components u and w (quw) as defined in the analytic control models
    return 0.5 * rho * (Vauw ** 2)

def qbarind_acm(rho, Vind):
    #Propulsion induced dynamic pressure (qind) as defined in the analytic control models
    return 0.5 * rho * (Vind ** 2)

def qbarprop_acm(rho, Vprop):
    #Propeller induced dynamic pressure (qprop) as defined in the analytic control models
    return 0.5 * rho * (Vprop ** 2)

def n_acm(deltat, deltam):
    #Propeller RPS (Revolutions Per Second) law as defined in the analytic control models
    if deltam >= DELTA_M_MIN
        return (N_MAX - N_MIN) * deltat + N_MIN
    else:
        return 0

''' MISC PARTIAL DERIVATIVES '''
def parder_deltat_n():
    #Partial derivative of n with respect to deltat
    return N_MAX - N_MIN

def parder_pd_rho(pd):
    #Partial derivative of rho with respect to pd
    return parder_pd_barometric_density(pd)

def parder_pd_Vind(u, rho, T, parder_pd_rho):
    #Partial derivative of Vind with respect to pd
    Vind2      = Vind2_acm(u, rho, T)
    Vind2_sign = (np.sign(Vind2) >= 0)
    return - Vind2_sign * parder_pd_rho * ((T * (rho * PROP_AREA_SI * u * abs(u) + 2 * T)) / ((rho ** 3) * (PROP_AREA_SI ** 2) * (Vind ** 3)))

def parder_u_Vind(u, rho, T):
    #Partial derivative of Vind with respect to u
    Vind2      = Vind2_acm(u, rho, T)
    Vind2_sign = (np.sign(Vind2) >= 0)
    return Vind2_sign * (u ** 2) * (rho * PROP_AREA_SI * u * abs(u) + 2 * T) / (rho * PROP_AREA_SI * abs(u) * (Vind ** 3))

def parder_deltat_Vind(u, rho, T, parder_deltat_Ft):
    #Partial derivative of Vind with respect to deltat
    Vind2      = Vind2_acm(u, rho, T)
    Vind2_sign = (np.sign(Vind2) >= 0)
    return Vind2_sign * parder_deltat_Ft * (rho * PROP_AREA_SI * u * abs(u) + 2 * T) / ((rho ** 2) * (PROP_AREA_SI ** 2) * (Vind ** 3))

def parder_pd_Vprop(parder_pd_Vind):
    #Partial derivative of Vprop with respect to pd
    return parder_pd_Vind

def parder_u_Vprop(parder_u_Vind):
    #Partial derivative of Vprop with respect to u
    return parder_u_Vind - 1

def parder_deltat_Vprop(parder_deltat_Vind):
    #Partial derivative of Vprop with respect to deltat
    return parder_deltat_Vind

def parder_u_alpha(w, Vauw):
    #Partial derivative of alpha with respect to u
    return - w / (Vauw ** 2)

def parder_w_alpha(w, Vauw):
    #Partial derivative of alpha with respect to w
    return u / (Vauw ** 2)

def parder_u_beta(u, v, Va, Vauw):
    #Partial derivative of beta with respect to u
    return - (u * v) / (Vauw * (Va ** 2))

def parder_v_beta(Va, Vauw):
    #Partial derivative of beta with respect to v
    return Vauw / (Va ** 2)

def parder_w_beta(v, w, Va, Vauw):
    #Partial derivative of beta with respect to w
    return - (v * w) / (Vauw * (Va ** 2))

def parder_pd_q(Va, parder_pd_rho):
    #Partial derivative of q with respect to pd
    return 0.5 * parder_pd_rho * (Va ** 2)

def parder_u_q(u, rho):
    #Partial derivative of q with respect to u
    return rho * u

def parder_v_q(v, rho):
    #Partial derivative of q with respect to v
    return rho * v

def parder_w_q(w, rho):
    #Partial derivative of q with respect to w
    return rho * w

def parder_pd_quw(Vauw, parder_pd_rho):
    #Partial derivative of quw with respect to pd
    return 0.5 * parder_pd_rho * (Vauw ** 2)

def parder_u_quw(parder_u_q):
    #Partial derivative of quw with respect to u
    return parder_u_q

def parder_w_quw(parder_w_q):
    #Partial derivative of quw with respect to w
    return parder_w_q

def parder_pd_qind(rho, Vind, parder_pd_rho, parder_pd_Vind):
    #Partial derivative of qind with respect to pd
    return 0.5 * Vind * (parder_pd_rho * Vind + 2 * rho * parder_pd_Vind)

def parder_u_qind(rho, Vind, parder_u_Vind):
    #Partial derivative of qind with respect to u
    return rho * Vind * parder_u_Vind

def parder_deltat_qind(rho, Vind, parder_deltat_Vind):
    #Partial derivative of qind with respect to deltat
    return rho * Vind * parder_deltat_Vind

def parder_pd_qprop(rho, Vprop, parder_pd_rho, parder_pd_Vind):
    #Partial derivative of qprop with respect to pd
    return 0.5 * Vprop * (parder_pd_rho * Vprop + 2 * rho * parder_pd_Vprop)

def parder_u_qprop(rho, Vprop, parder_u_Vprop):
    #Partial derivative of qprop with respect to u
    return rho * Vprop * parder_u_Vprop

def parder_deltat_qprop(rho, Vprop, parder_deltat_Vprop):
    #Partial derivative of qprop with respect to deltat
    return rho * Vprop * parder_deltat_Vprop

def parder_pd_T(J, n, parder_pd_rho):
    #Partial derivative of T with respect to pd
    return CT_interp(J) * (n ** 2) * (PROP_DIAM_M ** 4) * parder_pd_rho

def parder_u_T(rho, J, n, parder_J_CT):
    #Partial derivative of T with respect to u
    return rho * n * (PROP_DIAM_M ** 3) * parder_J_CT_interp(J)

def parder_deltat_T(rho, J, n, parder_J_CT, parder_deltat_n):
    #Partial derivative of T with respect to deltat
    return rho * (PROP_DIAM_M ** 3) * (2 * CT_interp(J) * n * PROP_DIAM_M - u * parder_J_CT_interp(J)) * parder_deltat_n

''' CONTROL MODEL CLASS '''
class ControlModel():
    def __init__(self, rx2nlm_out, rx2lm_out, nlm2act_in, lm2act_in, nlm2csv_in, lm2csv_in, eq2lm_out, event_start):
        self.csvnlm = np.zeros((MODEL_HZ, STATE_LEN + 1)) #array for storing non-linear model
        self.csvlm = np.zeros((MODEL_HZ, STATE_LEN + 1)) #array for storing linear model
        self.proc1 = mp.Process(target=self.non_linear, args=(rx2nlm_out, nlm2act_in, nlm2csv_in, event_start), daemon=True) #process for calculating non-linear model
        self.proc2 = mp.Process(target=self.linear, args=(rx2lm_out, lm2act_in, lm2csv_in, event_start), daemon=True) #process for calculating linear model
        self.proc1.start()
        self.proc2.start()
    def non_linear(self, rx2nlm_out, nlm2act_in, nlm2csv_in, event_start):
        #Analytic non-linear control model
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
            
                #Equilibrium point
                pn_eq     = eqdata[i,0]
                pe_eq     = eqdata[i,1]
                parder_eq = eqdata[i,2]
                phi_eq    = eqdata[i,3]
                theta_eq  = eqdata[i,4]
                psi_eq    = eqdata[i,5]
                u_eq      = eqdata[i,6]
                v_eq      = eqdata[i,7]
                w_eq      = eqdata[i,8]
                p_eq      = eqdata[i,9]
                q_eq      = eqdata[i,10]
                r_eq      = eqdata[i,11]
                (q0_eq, q1_eq, q2_eq, q3_eq) = euler_to_attquat(np.array([phi_eq, theta_eq, psi_eq]))

                dx = np.zeros(STATE_LEN)
                (q0, q1, q2, q3) = euler_to_attquat(np.array([phi, theta, psi]))

                dx[0:3] = body_to_vehicle(phi, theta, psi) * np.array([u, v, w])
                dx[3:6] = np.array([[0, -w, v], [w, 0, -u], [-v, u, 0]]) + np.array([fx, fy, fz]) / mass
                dx[6:10] = 0.5 * np.array([[0, -p, -q, -r], [p, 0, r, -q], [q, -r, 0, p], [r, q, -p, 0]]) * np.array([q0, q1, q2, q3])
                dx[10:13] = np.array([Gamma1*p*q-Gamma2*q*r, Gamma5*p*r-Gamma6*(p**2-r**2), Gamma7*p*q-Gamma1*q*r]) + np.array([[Gamma3, 0, Gamma4], [0, 1/Iyy, 0], [Gamma4, 0, Gamma8]]) * np.array([l, m, n])

                self.csvnlm[i,:] = [time, dx]

            nlm2csv_in.send(self.csvnlm[:framescount,:]) #send calculated non-linear model to store in CSV
            self.csvnlm = np.empty((MODEL_HZ, STATE_LEN + 1)) #empty array 

    def linear(self, rx2lm_out, lm2act_in, lm2csv_in, eq2lm_out, event_start):
        #Analytic linear control model
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

                #Equilibrium point
                pn_eq     = eqdata[i,0]
                pe_eq     = eqdata[i,1]
                parder_eq = eqdata[i,2]
                phi_eq    = eqdata[i,3]
                theta_eq  = eqdata[i,4]
                psi_eq    = eqdata[i,5]
                u_eq      = eqdata[i,6]
                v_eq      = eqdata[i,7]
                w_eq      = eqdata[i,8]
                p_eq      = eqdata[i,9]
                q_eq      = eqdata[i,10]
                r_eq      = eqdata[i,11]
                (q0_eq, q1_eq, q2_eq, q3_eq) = euler_to_attquat(np.array([phi_eq, theta_eq, psi_eq]))

                dx = np.zeros(STATE_LEN)
                A  = np.zeros((STATE_LEN, STATE_LEN))
                B  = np.zeros((STATE_LEN, INPUT_LEN))
                (q0, q1, q2, q3) = euler_to_attquat(np.array([phi, theta, psi]))

                #Misc partial derivatives evaluated in the equilibrium point
                parder_pd_rho_eq  = parder_pd_rho(parder_eq)
                parder_pd_qbar_eq = parder_pd_qbar(Va_eq, parder_pd_rho_eq)
                parder_pd_Ft_eq   = parder_pd_T(J_eq, n_eq, parder_pd_rho_eq)

                #Partial derivatives of \dot{p_n} evaluated in the equilibrium point
                parder_q0_dotpn_eq = 2 * (q0_eq * u_eq - q3_eq * v_eq + q2_eq * w_eq)
                parder_q1_dotpn_eq = 2 * (q1_eq * u_eq + q2_eq * v_eq + q3_eq * w_eq)
                parder_q2_dotpn_eq = 2 * (- q2_eq * u_eq + q1_eq * v_eq + q0_eq * w_eq)
                parder_q3_dotpn_eq = 2 * (- q3_eq * u_eq - q0_eq * v_eq + q1_eq * w_eq)
                parder_u_dotpn_eq  = q0_eq^2 + q1_eq^2 - q2_eq^2 - q3_eq^2
                parder_v_dotpn_eq  = 2 * (q1_eq * q2_eq - q0_eq * q3_eq)
                parder_w_dotpn_eq  = 2 * (q0_eq * q2_eq + q1_eq * q3_eq)

                #Partial derivatives of \dot{p_e} evaluated in the equilibrium point
                parder_q0_dotpe_eq = 2 * (q3_eq * u_eq + q0_eq * v_eq - q1_eq * w_eq)
                parder_q1_dotpe_eq = 2 * (q2_eq * u_eq - q1_eq * v_eq - q0_eq * w_eq)
                parder_q2_dotpe_eq = 2 * (q1_eq * u_eq + q2_eq * v_eq + q3_eq * w_eq)
                parder_q3_dotpe_eq = 2 * (q0_eq * u_eq - q3_eq * v_eq + q2_eq * w_eq)
                parder_u_dotpe_eq  = 2 * (q0_eq * q3_eq + q1_eq * q2_eq)
                parder_v_dotpe_eq  = q0_eq^2 + q2_eq^2 - q1_eq^2 - q3_eq^2
                parder_w_dotpe_eq  = 2 * (q2_eq * q3_eq - q0_eq * q1_eq)

                #Partial derivatives of \dot{p_d} evaluated in the equilibrium point
                parder_q0_dotpd_eq = 2 * (- q2_eq * u_eq + q1_eq * v_eq + q0_eq * w_eq)
                parder_q1_dotpd_eq = 2 * (q3_eq * u_eq + q0_eq * v_eq - q1_eq * w_eq)
                parder_q2_dotpd_eq = 2 * (- q0_eq * u_eq + q3_eq * v_eq - q2_eq * w_eq)
                parder_q3_dotpd_eq = 2 * (q1_eq * u_eq + q2_eq * v_eq + q3_eq * w_eq)
                parder_u_dotpd_eq  = 2 * (q1_eq * q3_eq - q0_eq * q2_eq)
                parder_v_dotpd_eq  = 2 * (q0_eq * q1_eq + q2_eq * q3_eq)
                parder_w_dotpd_eq  = q0_eq^2 + q3_eq^2 - q1_eq^2 - q2_eq^2

                #Partial derivatives of \dot{q_0} evaluated in the equilibrium point
                parder_q1_dotq0_eq = - 0.5 * p_eq
                parder_q2_dotq0_eq = - 0.5 * q_eq
                parder_q3_dotq0_eq = - 0.5 * r_eq
                parder_p_dotq0_eq  = - 0.5 * q1_eq
                parder_q_dotq0_eq  = - 0.5 * q2_eq
                parder_r_dotq0_eq  = - 0.5 * q3_eq

                #Partial derivatives of \dot{q_1} evaluated in the equilibrium point
                parder_q0_dotq1_eq = 0.5 * p_eq
                parder_q2_dotq1_eq = 0.5 * r_eq
                parder_q3_dotq1_eq = - 0.5 * q_eq
                parder_p_dotq1_eq  = 0.5 * q0_eq
                parder_q_dotq1_eq  = - 0.5 * q3_eq
                parder_r_dotq1_eq  = 0.5 * q2_eq

                #Partial derivatives of \dot{q_2} evaluated in the equilibrium point
                parder_q0_dotq2_eq = 0.5 * q_eq
                parder_q1_dotq2_eq = - 0.5 * r_eq
                parder_q3_dotq2_eq = 0.5 * p_eq
                parder_p_dotq2_eq  = 0.5 * q3_eq
                parder_q_dotq2_eq  = 0.5 * q0_eq
                parder_r_dotq2_eq  = - 0.5 * q1_eq

                #Partial derivatives of \dot{q_3} evaluated in the equilibrium point
                parder_q0_dotq3_eq = 0.5 * r_eq
                parder_q1_dotq3_eq = 0.5 * q_eq
                parder_q2_dotq3_eq = - 0.5 * p_eq
                parder_p_dotq3_eq  = - 0.5 * q2_eq
                parder_q_dotq3_eq  = 0.5 * q1_eq
                parder_r_dotq3_eq  = 0.5 * q0_eq

                #Partial derivatives of \dot{u} evaluated in the equilibrium point
                parder_pd_dotu_eq     = (1 / mass) * (- parder_pd_qbar_eq * SW_SI * (CFx1_eq + CFx2_eq + CFx3_eq + CFx4_eq) + parder_pd_Ft_eq)
                parder_q0_dotu_eq     = - 2 * G0_SI * q2_eq
                parder_q1_dotu_eq     = 2 * G0_SI * q3_eq
                parder_q2_dotu_eq     = - 2 * G0_SI * q0_eq
                parder_q3_dotu_eq     = 2 * G0_SI * q1_eq
                parder_u_dotu_eq      = (1 / mass) * (- SW_SI * (parder_u_qbar_eq * (CFx1_eq + CFx2_eq + CFx3_eq + CFx4_eq) + qbar_eq * (parder_u_CFx3_eq + parder_u_CFx4_eq)) + parder_u_Ft_eq)
                parder_v_dotu_eq      = r_eq - (SW_SI / mass) * (parder_v_qbar_eq * (CFx1_eq + CFx2_eq + CFx3_eq + CFx4_eq) + qbar_eq * parder_v_CFx4_eq)
                parder_w_dotu_eq      = - q_eq - (SW_SI / mass) * (parder_w_qbar_eq * (CFx1_eq + CFx2_eq + CFx3_eq + CFx4_eq) + qbar_eq * (parder_w_CFx3_eq + parder_w_CFx4_eq))
                parder_q_dotu_eq      = - w_eq
                parder_r_dotu_eq      = v_eq
                parder_deltaf_dotu_eq = - ((qbar_eq * SW_SI) / mass) * (parder_deltaf_CFx2_eq + parder_deltaf_CFx3_eq)
                parder_deltat_dotu_eq = parder_deltat_Ft_eq / mass

                #Partial derivatives of \dot{v} evaluated in the equilibrium point
                parder_pd_dotv_eq     = (1 / mass) * (parder_pd_qbar_eq * SW_SI * (CFy1_eq + CFy2_eq))
                parder_q0_dotv_eq     = 2 * G0_SI * q1_eq
                parder_q1_dotv_eq     = 2 * G0_SI * q0_eq
                parder_q2_dotv_eq     = 2 * G0_SI * q3_eq
                parder_q3_dotv_eq     = 2 * G0_SI * q2_eq
                parder_u_dotv_eq      = - r_eq + (SW_SI / mass) * (parder_u_qbar_eq * (CFy1_eq + CFy2_eq) + qbar_eq * parder_u_CFy1_eq)
                parder_v_dotv         = (SW_SI / mass) * (parder_v_qbar_eq * (CFy1_eq + CFy2_eq) + qbar_eq * parder_v_CFy1_eq)
                parder_w_dotv_eq      = p_eq + (SW_SI / mass) * (parder_w_qbar_eq * (CFy1_eq + CFy2_eq) + qbar_eq * parder_w_CFy1_eq)
                parder_p_dotv_eq      = w_eq
                parder_r_dotv_eq      = - u_eq
                parder_deltaf_dotv_eq = ((qbar_eq * SW_SI) / mass) * parder_deltaf_CFy1_eq
                parder_deltar_dotv_eq = ((qbar_eq * SW_SI) / mass) * parder_deltar_CFy2_eq

                #Partial derivatives of \dot{w} evaluated in the equilibrium point
                parder_pd_dotw_eq     = - (SW_SI / mass) * (parder_pd_qbar_eq * (CFz1_eq + CFz2_eq + CFz3_eq + CFz4_eq) + parder_pd_quw_eq * CFz5_eq)
                parder_q0_dotw_eq     = 2 * G0_SI * q0_eq
                parder_q1_dotw_eq     = - 2 * G0_SI * q1_eq
                parder_q2_dotw_eq     = - 2 * G0_SI * q2_eq
                parder_q3_dotw_eq     = 2 * G0_SI * q3_eq
                parder_u_dotw_eq      = q_eq - (SW_SI / mass) * (parder_u_qbar_eq * (CFz1_eq + CFz2_eq + CFz3_eq + CFz4_eq) + qbar_eq * (parder_u_CFz1_eq + parder_u_CFz4_eq) + parder_u_quw_eq * CFz5_eq + quw_eq * parder_u_CFz5_eq)
                parder_v_dotw_eq      = - p_eq - (SW_SI / mass) * (parder_v_qbar_eq * (CFz1_eq + CFz2_eq + CFz3_eq + CFz4_eq) + qbar_eq * parder_v_CFz4_eq + quw_eq * CFz5_eq)
                parder_w_dotw_eq      = - (SW_SI / mass) * (parder_w_qbar_eq * (CFz1_eq + CFz2_eq + CFz3_eq + CFz4_eq) + qbar_eq * (parder_w_CFz1_eq + parder_w_CFz4_eq) + parder_w_quw_eq * CFz5_eq + quw_eq * parder_w_CFz5_eq)
                parder_p_dotw_eq      = - v_eq
                parder_q_dotw_eq      = u_eq - ((qbar_eq * SW_SFQT) / mass) * parder_q_CFz4
                parder_deltaf_dotw_eq = - ((qbar_eq * SW_SI) / mass) * parder_deltaf_CFz2_eq
                parder_deltae_dotw_eq = - ((qbar_eq * SW_SI) / mass) * parder_deltae_CFz3_eq

                #Partial derivatives of \dot{p} evaluated in the equilibrium point
                parder_pd_dotp_eq     = SW_SI * BW_SI * (parder_pd_qbar_eq * (Gamma3 * (CMx1_eq + CMx2_eq + CMx3_eq + CMx4_eq + CMx5_eq) + Gamma4 * (CMz1_eq + CMz2_eq + CMz3_eq + CMz4_eq)) + Gamma4 * (parder_pd_qind_eq * CMz5_eq + parder_pd_qprop_eq * CMz6_eq))
                parder_u_dotp_eq      = SW_SI * BW_SI * (parder_u_qbar_eq * (Gamma3 * (CMx1_eq + CMx2_eq + CMx3_eq + CMx4_eq + CMx5_eq) + Gamma4 * (CMz1_eq + CMz2_eq + CMz3_eq + CMz4_eq)) + qbar_eq * (Gamma3 * (parder_u_CMx1_eq + parder_u_CMx2_eq + parder_u_CMx3_eq + parder_u_CMx4_eq) + Gamma4 * (parder_u_CMz1_eq + parder_u_CMz2_eq + parder_u_CMz3_eq + parder_u_CMz4_eq)) + Gamma4 * (parder_u_qind_eq * CMz5_eq + parder_u_qprop_eq * CMz6_eq))
                parder_v_dotp_eq      = SW_SI * BW_SI * (parder_v_qbar_eq * (Gamma3 * (CMx1_eq + CMx2_eq + CMx3_eq + CMx4_eq + CMx5_eq) + Gamma4 * (CMz1_eq + CMz2_eq + CMz3_eq + CMz4_eq)) + qbar_eq * (Gamma3 * (parder_v_CMx1_eq + parder_v_CMx2_eq + parder_v_CMx3_eq) + Gamma4 * (parder_v_CMz1_eq + parder_v_CMz2_eq + parder_v_CMz3_eq + parder_v_CMz4_eq)))
                parder_w_dotp_eq      = SW_SI * BW_SI * (parder_w_qbar_eq * (Gamma3 * (CMx1_eq + CMx2_eq + CMx3_eq + CMx4_eq + CMx5_eq) + Gamma4 * (CMz1_eq + CMz2_eq + CMz3_eq + CMz4_eq)) + qbar_eq * (Gamma3 * (parder_w_CMx1_eq + parder_w_CMx2_eq + parder_w_CMx3_eq + parder_w_CMx4_eq) + Gamma4 * (parder_w_CMz1_eq + parder_w_CMz2_eq + parder_w_CMz3_eq + parder_w_CMz4_eq)))
                parder_p_dotp_eq      = Gamma1 * q_eq + SW_SI * BW_SI * qbar_eq * Gamma3 * parder_p_CMx2_eq
                parder_q_dotp_eq      = Gamma1 * p_eq - Gamma2 * r_eq
                parder_r_dotp_eq      = - Gamma2 * q_eq + SW_SI * BW_SI * qbar_eq * (Gamma3 * parder_r_CMx3_eq + Gamma4 * (parder_p_CMz2_eq + parder_p_CMz3_eq))
                parder_deltaa_dotp_eq = SW_SI * BW_SI * qbar_eq * (Gamma3 * parder_deltaa_CMx4_eq + Gamma4 * parder_deltaa_CMz4_eq)
                parder_deltaf_dotp_eq = SW_SI * BW_SI * qbar_eq * Gamma3 * parder_deltaf_CMx3_eq
                parder_deltar_dotp_eq = SW_SI * BW_SI * (Gamma3 * qbar_eq * parder_deltar_CMx5_eq + Gamma4 * qind_eq * parder_deltar_CMz5_eq)

                #Partial derivatives of \dot{q} evaluated in the equilibrium point
                parder_pd_dotq_eq     = ((SW_SI * CW_SI) / Iyy) * (parder_pd_qbar_eq * (CMy1_eq + CMy2_eq + CMy3_eq + CMy4_eq) + qbar_eq * parder_pd_CMy1_eq + parder_pd_quw_eq * CMy5_eq + parder_pd_qind_eq * CMy6_eq)
                parder_u_dotq_eq      = ((SW_SI * CW_SI) / Iyy) * (parder_u_qbar_eq * (CMy1_eq + CMy2_eq + CMy3_eq + CMy4_eq) + qbar_eq * (parder_u_CMy2_eq + parder_u_CMy3_eq) + parder_u_quw_eq * CMy5_eq + quw_eq * parder_u_CMy5_eq + parder_u_qind_eq * CMy6_eq + qind_eq * parder_u_CMy6_eq)
                parder_v_dotq_eq      = ((SW_SI * CW_SI) / Iyy) * (parder_v_qbar_eq * (CMy1_eq + CMy2_eq + CMy3_eq + CMy4_eq) + qbar_eq * parder_v_CMy3_eq + quw_eq * parder_v_CMy5_eq)
                parder_w_dotq_eq      = ((SW_SI * CW_SI) / Iyy) * (parder_w_qbar_eq * (CMy1_eq + CMy2_eq + CMy3_eq + CMy4_eq) + qbar_eq * (parder_w_CMy2_eq + parder_w_CMy3_eq) + parder_w_quw_eq * CMy5_eq + quw_eq * parder_w_CMy5_eq + qind_eq * parder_w_CMy6_eq)
                parder_p_dotq_eq      = Gamma5 * r_eq - 2 * Gamma6 * p_eq
                parder_q_dotq_eq      = ((SW_SI * CW_SI * qbar_eq) / Iyy) * parder_q_CMy3_eq
                parder_r_dotq_eq      = Gamma5 * p_eq + 2 * Gamma6 * r_eq
                parder_deltaf_dotq_eq = ((SW_SI * CW_SI * qbar_eq) / Iyy) * parder_deltaf_CMy4_eq
                parder_deltae_dotq_eq = ((SW_SI * CW_SI * q_ind) / Iyy) * parder_deltae_CMy6_eq

                #Partial derivatives of \dot{r} evaluated in the equilibrium point
                parder_pd_dotr_eq     = SW_SI * BW_SI * (parder_pd_qbar_eq * (Gamma4 * (CMx1_eq + CMx2_eq + CMx3_eq + CMx4_eq + CMx5_eq) + Gamma8 * (CMz1_eq + CMz2_eq + CMz3_eq + CMz4_eq)) + Gamma8 * (parder_pd_qind_eq * CMz5_eq + parder_pd_qprop_eq * CMz6_eq))
                parder_u_dotr_eq      = SW_SI * BW_SI * (parder_u_qbar_eq * (Gamma4 * (CMx1_eq + CMx2_eq + CMx3_eq + CMx4_eq + CMx5_eq) + Gamma8 * (CMz1_eq + CMz2_eq + CMz3_eq + CMz4_eq)) + qbar_eq * (Gamma4 * (parder_u_CMx1_eq + parder_u_CMx2_eq + parder_u_CMx3_eq + parder_u_CMx4_eq) + Gamma8 * (parder_u_CMz1_eq + parder_u_CMz2_eq + parder_u_CMz3_eq + parder_u_CMz4_eq)) + Gamma8 * (parder_u_qind_eq * CMz5_eq + parder_u_qprop_eq * CMz6_eq))
                parder_v_dotr_eq      = SW_SI * BW_SI * (parder_v_qbar_eq * (Gamma4 * (CMx1_eq + CMx2_eq + CMx3_eq + CMx4_eq + CMx5_eq) + Gamma8 * (CMz1_eq + CMz2_eq + CMz3_eq + CMz4_eq)) + qbar_eq * (Gamma4 * (parder_v_CMx1_eq + parder_v_CMx2_eq + parder_v_CMx3_eq) + Gamma8 * (parder_v_CMz1_eq + parder_v_CMz2_eq + parder_v_CMz3_eq + parder_v_CMz4_eq)))
                parder_w_dotr_eq      = SW_SI * BW_SI * (parder_w_qbar_eq * (Gamma4 * (CMx1_eq + CMx2_eq + CMx3_eq + CMx4_eq + CMx5_eq) + Gamma8 * (CMz1_eq + CMz2_eq + CMz3_eq + CMz4_eq)) + qbar_eq * (Gamma4 * (parder_w_CMx1_eq + parder_w_CMx2_eq + parder_w_CMx3_eq + parder_w_CMx4_eq) + Gamma8 * (parder_w_CMz1_eq + parder_w_CMz2_eq + parder_w_CMz3_eq + parder_w_CMz4_eq)))
                parder_p_dotr_eq      = Gamma7 * q_eq + SW_SI * BW_SI * qbar_eq * Gamma4 * parder_p_CMx2_eq
                parder_q_dotr_eq      = Gamma7 * p_eq - Gamma1 * r_eq
                parder_r_dotr_eq      = - Gamma1 * q_eq + SW_SI * BW_SI * qbar_eq * (Gamma4 * parder_r_CMx3_eq + Gamma8 * (parder_r_CMz2_eq + parder_r_CMz3_eq))
                parder_deltaa_dotr_eq = SW_SI * BW_SI * qbar_eq * (Gamma4 * parder_deltaa_CMx4_eq + Gamma8 * parder_deltaa_CMz4_eq)
                parder_deltaf_dotr_eq = SW_SI * BW_SI * qbar_eq * Gamma4 * parder_deltaf_CMx3_eq
                parder_deltar_dotr_eq = SW_SI * BW_SI * (Gamma4 * qbar_eq * parder_deltar_CMx5_eq + Gamma8 * qind_eq * parder_deltar_CMz5_eq)

                self.csvlm[i,:] = [time, dx]

            lm2csv_in.send(self.csvlm[:framescount,:]) #send calculated linear model to store in CSV
            self.csvlm = np.empty((MODEL_HZ, STATE_LEN + 1)) #empty array 
'''
