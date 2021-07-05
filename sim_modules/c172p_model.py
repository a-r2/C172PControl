import control
import numpy as np
from math import atan2, sin, sqrt
from operator import itemgetter
from scipy import interpolate
import sys

from constants import *
from settings import *
from utils import *

''' FLIGHTGEAR MODEL PARAMETERS '''
BW_IMP       = 35.8000 #wing span [ft]
C_SPIRALPROP = 0.2500 #coefficient due to the induced velocity by the propeller spiral
CW_IMP       = 4.9000 #wing chord [ft]
D_PROP_IMP   = 75.0000 #propeller diameter [in]
DELTAM_MIN   = 0.1000 #minimum delta_m to maintain the engine running [-]
N_RPM_MAX    = 2700.0000 #maximum propeller revolutions per minute [RPM]
N_RPM_MIN    = 600.0000 #minimum propeller revolutions per minute [RPM]
STH_IMP      = 21.9000 #horizontal tail area [ft^2]
STV_IMP      = 16.5000 #vertical tail area [ft^2]
SW_IMP       = 174.0000 #wing area [ft^2]

''' AERODYNAMIC ACTUATORS DEFLECTION '''
# ((Min. deflect. [deg], Max. deflec. [deg]), (Min. norm. deflect. [-], Max. norm. deflect. [-]))
DELTAA_RANGE = ((-20, 15), (-1, 1)) #negative: left, positive: right 
DELTAE_RANGE = ((-28, 23), (-1, 1))
DELTAF_RANGE = ((0, 30), (0, 1))
DELTAR_RANGE = ((-16, 16), (-1, 1))

''' MODEL PARAMETERS CONVERSION '''
BW_SI      = ft_to_m(BW_IMP) #wing span [m]
CW_SI      = ft_to_m(CW_IMP) #chord span [m]
D_PROP_IMP = in_to_ft(D_PROP_IMP) #propeller diameter [ft]
D_PROP_SI  = ft_to_m(D_PROP_IMP) #propeller diameter [m]
N_RPS_MAX  = rpm_to_rps(N_RPM_MAX) #maximum propeller revolutions per second [RPS]
N_RPS_MIN  = rpm_to_rps(N_RPM_MIN) #minimum propeller revolutions per second [RPS]
STH_SI     = ft_to_m(sqrt(STH_IMP)) ** 2 #horizontal tail area [m^2]
STV_SI     = ft_to_m(sqrt(STV_IMP)) ** 2 #vertical tail area [m^2]
SW_SI      = ft_to_m(sqrt(SW_IMP)) ** 2 #wing area [m^2]

A_PROP_SI  = propeller_area(D_PROP_SI) #propeller area [m^2]
A_PROP_IMP = propeller_area(m_to_ft(D_PROP_SI)) #propeller area [ft^2]

''' MOMENTS OF INERTIA '''
def mominert_gamma(Ixx, Ixz, Izz):
    return Ixx * Izz - (Ixz ** 2)

def mominert_gamma1(Ixx, Ixz, Iyy, Izz, Gamma):
    return (Ixz * (Ixx - Iyy + Izz)) / Gamma

def mominert_gamma2(Ixz, Iyy, Izz, Gamma):
    return (Izz * (Izz - Iyy) + (Ixz ** 2)) / Gamma

def mominert_gamma3(Izz, Gamma):
    return Izz / Gamma

def mominert_gamma4(Ixz, Gamma):
    return Ixz / Gamma

def mominert_gamma5(Ixx, Iyy, Izz):
    return (Izz - Ixx) / Iyy

def mominert_gamma6(Ixz, Iyy):
    return Ixz / Iyy

def mominert_gamma7(Ixx, Iyy, Ixz, Gamma):
    return (Ixx * (Ixx - Iyy) + (Ixz ** 2)) / Gamma

def mominert_gamma8(Ixx, Gamma):
    return Ixx / Gamma

''' LOOK-UP TABLES DERIVATIVES '''
def interp_table_1d(table_1d):
    #Interpolation function corresponding to 1D look-up table
    table_1d_interp = interpolate.interp1d(table_1d[:,0], table_1d[:,1], bounds_error=False, fill_value=(table_1d[0,1], table_1d[-1,1]))

    return table_1d_interp

def interp_table_2d(table_2d):
    #Interpolation function corresponding to 2D look-up table
    table_2d_trans   = table_2d.transpose()
    table_2d_interp  = interpolate.interp2d(table_2d_trans[0,1:], table_2d_trans[1:,0], table_2d_trans[1:,1:])
    table_2d_interp1 = (lambda x, y: np.squeeze(table_2d_interp(x,y))) #eliminate singleton dimension

    return table_2d_interp1

def parder_table_1d(table_1d):
    #Derivative of 1D look-up table
    parder_table_1d         = np.zeros((table_1d.shape[0] + 1, 2))
    parder_table_1d[0,0]    = table_1d[0,0]
    parder_table_1d[-1,0]   = table_1d[-1,0]
    parder_table_1d[1:-1,0] = table_1d[:-1,0] + 0.5 * np.diff(table_1d[:,0])
    parder_table_1d[0,1:-1] = table_1d[0,:-1] + 0.5 * np.diff(table_1d[0,:])
    parder_table_1d[1:-1,1] = np.diff(table_1d[:,1]) / np.diff(table_1d[:,0])
    parder_table_1d_interp  = interpolate.interp1d(parder_table_1d[:,0], parder_table_1d[:,1], bounds_error=False, fill_value=0)

    return parder_table_1d_interp

def parder_table_2d(table_2d):
    #Partial derivatives of 2D look-up table
    parder_x_table_2d         = np.zeros((table_2d.shape[0] + 1, table_2d.shape[1] + 1))
    parder_x_table_2d[0,0]    = np.NaN
    parder_x_table_2d[1,0]    = table_2d[1,0]
    parder_x_table_2d[-1,0]   = table_2d[-1,0]
    parder_x_table_2d[0,1]    = table_2d[0,1]
    parder_x_table_2d[0,-1]   = table_2d[0,-1]
    parder_x_table_2d[2:-1,0] = table_2d[1:-1,0] + 0.5 * np.diff(table_2d[1:,0])
    parder_x_table_2d[0,2:-1] = table_2d[0,1:-1] + 0.5 * np.diff(table_2d[0,1:])
    for j in range(table_2d.shape[1] - 1):
        parder_x_table_2d[2:-1,j+1] = np.diff(table_2d[1:,j+1]) / np.diff(table_2d[1:,0])
    parder_x_table_2d_trans  = parder_x_table_2d.transpose()
    parder_x_table_2d_interp = interpolate.interp2d(parder_x_table_2d_trans[0,1:], parder_x_table_2d_trans[1:,0], parder_x_table_2d_trans[1:,1:], bounds_error=False, fill_value=0)
    parder_x_table_2d_interp1 = (lambda x, y: np.squeeze(parder_x_table_2d_interp(x,y))) #eliminate singleton dimension

    parder_y_table_2d         = np.zeros((table_2d.shape[0] + 1, table_2d.shape[1] + 1))
    parder_y_table_2d[0,0]    = np.NaN
    parder_y_table_2d[1,0]    = table_2d[1,0]
    parder_y_table_2d[-1,0]   = table_2d[-1,0]
    parder_y_table_2d[0,1]    = table_2d[0,1]
    parder_y_table_2d[0,-1]   = table_2d[0,-1]
    parder_y_table_2d[2:-1,0] = table_2d[1:-1,0] + 0.5 * np.diff(table_2d[1:,0])
    parder_y_table_2d[0,2:-1] = table_2d[0,1:-1] + 0.5 * np.diff(table_2d[0,1:])
    for i in range(table_2d.shape[0] - 1):
        parder_y_table_2d[i+1,2:-1] = np.diff(table_2d[i+1,1:]) / np.diff(table_2d[0,1:])
    parder_y_table_2d_trans  = parder_y_table_2d.transpose()
    parder_y_table_2d_interp = interpolate.interp2d(parder_y_table_2d_trans[0,1:], parder_y_table_2d_trans[1:,0], parder_y_table_2d_trans[1:,1:], bounds_error=False, fill_value=0)
    parder_y_table_2d_interp1 = (lambda x, y: np.squeeze(parder_y_table_2d_interp(x,y))) #eliminate singleton dimension

    return parder_x_table_2d_interp1, parder_y_table_2d_interp1
    
''' DYNAMIC COEFFICIENTS TABLES '''
#Drag coefficient due to ground effect
#Column 0: hmacb [-]
kCDge_FG = np.array(   
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
kCDge        = kCDge_FG.copy()
kCDge_interp = interp_table_1d(kCDge)

#Drag coefficient due to flaps position
#Column 0: sigmaf_deg [deg]
TD2_FG = np.array(
                  [
                   [0.0000, 0.0000],
                   [10.0000, 0.0070],
                   [20.0000, 0.0120],
                   [30.0000, 0.0180]
                  ]
                 )
TD2                      = TD2_FG.copy()
TD2[:,0]                 = deg_to_rad(TD2[:,0])
TD2_interp               = interp_table_1d(TD2)
parder_deltaf_TD2_interp = parder_table_1d(TD2)

#Drag coefficient due to angle of attack and flaps position
#Column 0: alpha_rad [rad] | Row 0: sigmaf_deg [deg]
TD3_FG = np.array(   
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
TD3                                               = TD3_FG.copy()
TD3[0,1:]                                         = deg_to_rad(TD3[0,1:])
TD3_interp                                        = interp_table_2d(TD3)
parder_alpha_TD3_interp, parder_deltaf_TD3_interp = parder_table_2d(TD3)

#Side force coefficient due to side-slip angle and flaps position
#Column 0: beta_rad [rad] | Row 0: sigmaf_deg [deg]
TC1_FG = np.array(
                  [
                   [np.NaN, 0.0000, 30.0000],
                   [-0.3490, 0.1370, 0.1060],
                   [0.0000, 0.0000, 0.0000],
                   [0.3490, -0.1370, -0.1060]
                  ]
                 )
TC1                                              = TC1_FG.copy()
TC1[0,1:]                                        = deg_to_rad(TC1[0,1:])
TC1_interp                                       = interp_table_2d(TC1)
parder_beta_TC1_interp, parder_deltaf_TC1_interp = parder_table_2d(TC1)

#Lift coefficient due to ground effect
#Column 0: hmacb [-]
kCLge_FG = np.array(   
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
kCLge        = kCLge_FG.copy()
kCLge_interp = interp_table_1d(kCLge)

#Lift coefficient due to alpha and aerodynamic hysteresis
#Column 0: alpha_rad [rad] | Row 0: stall [-]
TL1_FG = np.array(   
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
TL1                        = TL1_FG.copy()
TL1_interp                 = interp_table_2d(TL1)
parder_alpha_TL1_interp, _ = parder_table_2d(TL1)

#Lift coefficient due to flaps position
#Column 0: sigmaf_deg [deg]
TL2_FG = np.array(
                  [
                   [0.0000, 0.0000],
                   [10.0000, 0.2000],
                   [20.0000, 0.3000],
                   [30.0000, 0.3500]
                  ]
                 )
TL2                      = TL2_FG.copy()
TL2[:,0]                 = deg_to_rad(TL2[:,0])
TL2_interp               = interp_table_1d(TL2)
parder_deltaf_TL2_interp = parder_table_1d(TL2)

#Roll moment coefficient due to alpha wing
#Column 0: alpha_rad [rad]
Tl1_FG = np.array(
                  [
                   [0.2790, 1.0000],
                   [0.2970, 3.5000]
                  ]
                 )
Tl1                     = Tl1_FG.copy()
Tl1_interp              = interp_table_1d(Tl1)
parder_alpha_Tl1_interp = parder_table_1d(Tl1)

#Roll moment coefficient due to flaps position
#Column 0: sigmaf_deg [deg]
Tl31_FG = np.array(
                   [
                    [0.0000, 0.0798],
                    [30.000, 0.1246]
                   ]
                  )
Tl31                      = Tl31_FG.copy()
Tl31[:,0]                 = deg_to_rad(Tl31[:,0])
Tl31_interp               = interp_table_1d(Tl31)
parder_deltaf_Tl31_interp = parder_table_1d(Tl31)

#Roll moment coefficient due to flaps position (stall)
#Column 0: alpha_rad [rad] | Row 0: r [rad/s]
Tl32_FG = np.array(
                   [
                    [np.NaN, -0.1500, -0.1000, 0.0000, 0.1000, 0.1500],
                    [0.2970, 35.0000, 30.0000, 1.0000, 30.0000, 35.0000],
                    [0.5000, 5.0000, 5.0000, 1.0000, 5.0000, 5.0000]
                   ]
                  )
Tl32                                           = Tl32_FG.copy()
Tl32_interp                                    = interp_table_2d(Tl32)
parder_alpha_Tl32_interp, parder_r_Tl32_interp = parder_table_2d(Tl32)

#Roll moment coefficient due to flaps position
#Column 0: alpha_rad [rad] | Row 0: r [rad/s]
Tl33_FG = np.array(
                   [
                    [np.NaN, -0.1500, -0.1000, 0.0000, 0.1000, 0.1500],
                    [0.2790, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000],
                    [0.2970, 35.0000, 30.0000, 1.0000, 30.0000, 35.0000],
                    [0.5000, 5.0000, 5.0000, 1.0000, 5.0000, 5.0000]
                   ]
                  )
Tl33                                           = Tl33_FG.copy()
Tl33_interp                                    = interp_table_2d(Tl33)
parder_alpha_Tl33_interp, parder_r_Tl33_interp = parder_table_2d(Tl33)

#Roll moment coefficient due to flaps position
#Column 0: alpha_rad [rad] | Row 0: stall [-]
Tl4_FG = np.array(
                  [
                   [np.NaN, 0.0000, 1.0000],
                   [0.2790, 1.0000, 0.3000],
                   [0.2970, 0.3000, 0.3000],
                   [0.6110, -0.1000, -0.1000]
                  ]
                 )
Tl4                        = Tl4_FG.copy()
Tl4_interp                 = interp_table_2d(Tl4)
parder_alpha_Tl4_interp, _ = parder_table_2d(Tl4)

#Pitch moment coefficient due to qbar_psf
#Column 0: qbar_psf [psf]
Tm1_FG = np.array(
                  [
                   [13.6000, 0.0900],
                   [21.2000, 0.0400]
                  ]
                 )
Tm1                    = Tm1_FG.copy()
Tm1[:,0]               = psf_to_pa(Tm1[:,0])
Tm1_interp             = interp_table_1d(Tm1)
parder_qbar_Tm1_interp = parder_table_1d(Tm1)

#Pitch moment coefficient due to alpha_deg
#Column 0: alpha_deg [deg]
Tm2_FG = np.array(
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
Tm2                     = Tm2_FG.copy()
Tm2[:,0]                = deg_to_rad(Tm2[:,0])
Tm2_interp              = interp_table_1d(Tm2)
parder_alpha_Tm2_interp = parder_table_1d(Tm2)

#Pitch moment coefficient due to sigmaf_deg
#Column 0: sigmaf_deg [deg]
Tm4_FG = np.array(
                  [
                   [0.0000, 0.0000],
                   [10.0000, -0.0654],
                   [20.0000, -0.0981],
                   [30.0000, -0.1140]
                  ]
                 )
Tm4                      = Tm4_FG.copy()
Tm4[:,0]                 = deg_to_rad(Tm4[:,0])
Tm4_interp               = interp_table_1d(Tm4)
parder_deltaf_Tm4_interp = parder_table_1d(Tm4)

#Pitch moment coefficient due to sigmae_rad and alpha_deg
#Column 0: sigmae_rad [rad] | Row 0: alpha_deg [deg]
Tm5_FG = np.array(
                  [
                   [np.NaN, 18.0000, 25.0000, 35.0000, 45.0000, 55.0000, 65.0000, 90.0000],
                   [-0.4900, 1.0000, 0.5000, 0.2000, 0.1000, 0.1000, 0.1000, 0.1000],
                   [0.0000, 1.0000, 0.6000, 0.3000, 0.1500, 0.1000, 0.1000, 0.1000],
                   [0.4000, 1.0000, 0.9000, 0.8000, 0.7000, 0.6000, 0.5000, 0.4000]
                  ]
                 )
Tm5                                               = Tm5_FG.copy()
Tm5[0,1:]                                         = deg_to_rad(Tm5[0,1:])
Tm5_interp                                        = interp_table_2d(Tm5)
parder_deltae_Tm5_interp, parder_alpha_Tm5_interp = parder_table_2d(Tm5)

#Yaw moment coefficient due to beta_rad 
#Column 0: beta_rad [rad]
Tn1_FG = np.array(
                  [
                   [-0.3490, -0.0205],
                   [0.0000, 0.0000],
                   [0.3490, 0.0205]
                  ]
                 )
Tn1                    = Tn1_FG.copy()
Tn1_interp             = interp_table_1d(Tn1)
parder_beta_Tn1_interp = parder_table_1d(Tn1)

#Yaw moment coefficient due to r
#Column 0: r [rad/s] | Row 0: alpha_rad [rad]
Tn3_FG = np.array(
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
Tn3                                          = Tn3_FG.copy()
Tn3_interp                                   = interp_table_2d(Tn3)
parder_r_Tn3_interp, parder_alpha_Tn3_interp = parder_table_2d(Tn3)

#Yaw moment coefficient due to alpha_rad and beta_rad
#Column 0: alpha_rad [rad] | Row 0: beta_rad [rad]
Tn4_FG = np.array(
                  [
                   [np.NaN, -0.3500, 0.0000, 0.3500],
                   [0.0000, -0.0216, -0.0216, -0.0216],
                   [0.0700, -0.0390, -0.0786, -0.0390],
                   [0.0940, -0.0250, -0.0504, -0.0250]
                  ]
                 )
Tn4                                             = Tn4_FG.copy()
Tn4_interp                                      = interp_table_2d(Tn4)
parder_alpha_Tn4_interp, parder_beta_Tn4_interp = parder_table_2d(Tn4)

#Multiplier of thrust coefficient due to J
#Column 0: J [-]
CT_FG = np.array(
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
CT                 = CT_FG.copy()
CT_interp          = interp_table_1d(CT)
parder_J_CT_interp = parder_table_1d(CT)

#Multiplier of power coefficient due to J
#Column 0: J [-]
CP_FG = np.array(
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
CP        = CP_FG.copy()
CP_interp = interp_table_1d(CP)

''' AERODYNAMIC DRAG FORCE COEFFICIENTS '''
def aerocoeff_CD1():
    #Aerodynamic drag coefficient 1 
    return 0.0270

def aerocoeff_CD2(hmacb, sigmaf_rad):
    #Aerodynamic drag coefficient 2
    return kCDge_interp(hmacb) * TD2_interp(sigmaf_rad)

def aerocoeff_CD3(hmacb, alpha_rad, sigmaf_rad):
    #Aerodynamic drag coefficient 3
    return kCDge_interp(hmacb) * TD3_interp(alpha_rad, sigmaf_rad)

def aerocoeff_CD4(beta_rad):
    #Aerodynamic drag coefficient 4
    return 0.1500 * abs(beta_rad) 

''' AERODYNAMIC CROSSWIND FORCE COEFFICIENTS '''
def aerocoeff_CC1(beta_rad, sigmaf_rad):
    #Aerodynamic crosswind coefficient 1
    return TC1_interp(beta_rad, sigmaf_rad)

def aerocoeff_CC2(sigmar_rad):
    #Aerodynamic crosswind coefficient 2
    return 0.1500 * sigmar_rad 

''' AERODYNAMIC LIFT FORCE COEFFICIENTS '''
def aerocoeff_CL1(hmacb, alpha_rad, stall):
    #Aerodynamic lift coefficient 1
    return kCLge_interp(hmacb) * TL1_interp(alpha_rad, stall)

def aerocoeff_CL2(hmacb, sigmaf_rad):
    #Aerodynamic lift coefficient 2
    return kCLge_interp(hmacb) * TL2_interp(sigmaf_rad)

def aerocoeff_CL3(sigmae_rad):
    #Aerodynamic lift coefficient 3
    return 0.4300 * sigmae_rad

def aerocoeff_CL4(Cw2Va, q):
    #Aerodynamic lift coefficient 4
    return 3.9000 * Cw2Va * q

def aerocoeff_CL5(Cw2Va, alphadot_rads):
    #Aerodynamic lift coefficient 5
    return 1.7000 * Cw2Va  * alphadot_rads

''' AERODYNAMIC ROLL MOMENT COEFFICIENTS '''
def aerocoeff_Cl1(beta_rad, alpha_rad):
    #Aerodynamic roll coefficient 1
    return - 0.0920 * beta_rad * Tl1_interp(alpha_rad)

def aerocoeff_Cl2(Bw2Va, p):
    #Aerodynamic roll coefficient 2
    return - 0.4840 * Bw2Va * p 

def aerocoeff_Cl3(Bw2Va, r, sigmaf_rad, alpha_rad, stall):
    #Aerodynamic roll coefficient 3
    if stall:
        return Bw2Va * r * Tl31_interp(sigmaf_rad) * Tl32_interp(alpha_rad, r)
    else:
        return Bw2Va * r * Tl31_interp(sigmaf_rad) * Tl33_interp(alpha_rad, r)

def aerocoeff_Cl4(sigmala_rad, sigmara_rad, alpha_rad, stall):
    #Aerodynamic roll coefficient 4
    return 0.2290 * averaged_ailerons(sigmala_rad, sigmara_rad) * Tl4_interp(alpha_rad, stall)

def aerocoeff_Cl5(sigmar_rad):
    #Aerodynamic roll coefficient 5
    return 0.0147 * sigmar_rad

''' AERODYNAMIC PITCH MOMENT COEFFICIENTS '''
def aerocoeff_Cm1(qbar):
    #Aerodynamic pitch coefficient 1
    return Tm1_interp(qbar)

def aerocoeff_Cm2(alpha_rad):
    #Aerodynamic pitch coefficient 2
    return - 1.8000 * sin(alpha_rad) * Tm2_interp(alpha_rad)

def aerocoeff_Cm3(Cw2Va, q):
    #Aerodynamic pitch coefficient 3
    return - 12.4000 * Cw2Va * q

def aerocoeff_Cm4(sigmaf_rad):
    #Aerodynamic pitch coefficient 4
    return 0.7000 * Tm4_interp(sigmaf_rad)

def aerocoeff_Cm5(Cw2Va, alphadot_rads):
    #Aerodynamic pitch coefficient 5
    return - 7.2700 * Cw2Va * alphadot_rads

def aerocoeff_Cm6(sigmae_rad, alpha_rad):
    #Aerodynamic pitch coefficient 6
    return - 1.2800 * sigmae_rad * Tm5_interp(sigmae_rad, alpha_rad)

''' AERODYNAMIC YAW MOMENT COEFFICIENTS '''
def aerocoeff_Cn1(beta_rad):
    #Aerodynamic yaw coefficient 1
    return Tn1_interp(beta_rad)

def aerocoeff_Cn2(Bw2Va, r):
    #Aerodynamic yaw coefficient 2
    return - 0.0937 * Bw2Va * r

def aerocoeff_Cn3(Bw2Va, r, alpha_rad):
    #Aerodynamic yaw coefficient 3
    return Bw2Va * Tn3_interp(r, alpha_rad)

def aerocoeff_Cn4(sigmala_rad, sigmara_rad, alpha_rad, beta_rad):
    #Aerodynamic yaw coefficient 4
    return averaged_ailerons(sigmala_rad, sigmara_rad) * Tn4_interp(alpha_rad, beta_rad)

def aerocoeff_Cn5(sigmar_rad):
    #Aerodynamic yaw coefficient 5
    return - 0.0645 * sigmar_rad
def aerocoeff_Cn6():
    #Aerodynamic yaw coefficient 6
    return - 0.0500 * C_SPIRALPROP

''' PROPULSIVE FORCE '''
def thrust_eng(J, rho, rpsprop):
    #Engine (160 HP) thrust
    return CT_interp(J) * rho * (rpsprop ** 2) * (D_PROP_SI ** 4)

''' GRAVITATIONAL FORCE '''
def gravity_body(quat, mass, grav):
    #Gravitational force in body frame
    euler = attquat_to_euler(quat)
    return vehicle_to_body(euler[0], euler[1], euler[2]).rotate(mass * grav * np.array([0, 0, 1]))

''' PROPULSION POWER ''' 
def power_eng(J, rho, rpsprop):
    #Engine (160 HP) power
    return CP_interp(J) * rho * (rpsprop ** 3) * (D_PROP_SI ** 5)

''' MISC FUNCTIONS FOR THE ANALYTIC CONTROL MODELS'''
def acm_assumptions(deltat, pd):
    #Assumptions of analytic control models
    sigmaf_rad = 0
    deltaf     = 0
    deltam     = 1
    n          = n_acm(deltat, deltam)
    rho        = barometric_density(pd)
    grav       = G0_SI
    stall      = 0
    return sigmaf_rad, deltaf, deltam, n, rho, grav, stall

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
        return u / (n * D_PROP_SI + 1e-6)

def Va_acm(u, v, w):
    #Aerodynamic velocity (Va) as defined in the analytic control models
    return sqrt((u ** 2) + (v ** 2) + (w ** 2))

def Vauw_acm(u, w):
    #Aerodynamic velocity from components u and w (Vauw) as defined in the analytic control models
    return sqrt((u ** 2) + (w ** 2))

def Vind2_acm(u, rho, T):
    #Squared propulsion induced velocity (Vind^2) as defined in the analytic control models
    return u * abs(u) + ((2 * T) / (rho * A_PROP_SI))

def Vind_acm(u, rho, T, Vind2):
    #Propulsion induced velocity (Vind) as defined in the analytic control models
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
    #Propulsion induced dynamic pressure (qbarind_sp) as defined in the analytic control models
    return 0.5 * rho * (Vind ** 2)

def qbarprop_acm(rho, Vprop):
    #Propeller induced dynamic pressure (qbarprop) as defined in the analytic control models
    return 0.5 * rho * (Vprop ** 2)

def n_acm(deltat, deltam):
    #Propeller RPS (Revolutions Per Second) law as defined in the analytic control models
    if deltam >= DELTAM_MIN:
        return (N_RPS_MAX - N_RPS_MIN) * deltat + N_RPS_MIN
    else:
        return 0

''' PARTIAL DERIVATIVES FOR ALCM'''
def parder_deltat_n_alcm():
    #Partial derivative of n respect to deltat
    return N_RPS_MAX - N_RPS_MIN

def parder_pd_rho_alcm(pd):
    #Partial derivative of rho respect to pd
    return parder_pd_barometric_density(pd)

def parder_u_Va_alcm(u, Va):
    #Partial derivative of Va respect to u
    return u / (Va + 1e-6)

def parder_v_Va_alcm(v, Va):
    #Partial derivative of Va respect to v
    return v / (Va + 1e-6)

def parder_w_Va_alcm(w, Va):
    #Partial derivative of Va respect to w
    return w / (Va + 1e-6)

def parder_u_Bw2Va_alcm(u, Va, parder_u_Va):
    #Partial derivative of Va respect to u
    return - 0.5000 * BW_SI * parder_u_Va / ((Va ** 3) + 1e-6)

def parder_v_Bw2Va_alcm(v, Va, parder_v_Va):
    #Partial derivative of Va respect to v
    return - 0.5000 * BW_SI * parder_v_Va / ((Va ** 3) + 1e-6)

def parder_w_Bw2Va_alcm(w, Va, parder_w_Va):
    #Partial derivative of Va respect to w
    return - 0.5000 * BW_SI * parder_w_Va / ((Va ** 3) + 1e-6)

def parder_u_Cw2Va_alcm(u, Va, parder_u_Va):
    #Partial derivative of Va respect to u
    return - 0.5000 * CW_SI * parder_u_Va / ((Va ** 3) + 1e-6)

def parder_v_Cw2Va_alcm(v, Va, parder_v_Va):
    #Partial derivative of Va respect to v
    return - 0.5000 * CW_SI * parder_v_Va / ((Va ** 3) + 1e-6)

def parder_w_Cw2Va_alcm(w, Va, parder_w_Va):
    #Partial derivative of Va respect to w
    return - 0.5000 * CW_SI * parder_w_Va / ((Va ** 3) + 1e-6)

def parder_pd_Vind_alcm(u, rho, T, Vind2, parder_pd_rho):
    #Partial derivative of Vind respect to pd
    Vind2_sign = float(np.sign(Vind2) >= 0)
    return - Vind2_sign * parder_pd_rho * ((T * (rho * A_PROP_SI * u * abs(u) + 2 * T)) / ((rho ** 3) * (A_PROP_SI ** 2) * (sqrt(Vind2) ** 3) + 1e-6))

def parder_u_Vind_alcm(u, rho, T, Vind2):
    #Partial derivative of Vind respect to u
    Vind2_sign = float(np.sign(Vind2) >= 0)
    return Vind2_sign * (u ** 2) * (rho * A_PROP_SI * u * abs(u) + 2 * T) / (rho * A_PROP_SI * abs(u) * (sqrt(Vind2) ** 3) + 1e-6)

def parder_deltat_Vind_alcm(u, rho, T, Vind2, parder_deltat_T):
    #Partial derivative of Vind respect to deltat
    Vind2_sign = float(np.sign(Vind2) >= 0)
    return Vind2_sign * parder_deltat_T * (rho * A_PROP_SI * u * abs(u) + 2 * T) / ((rho ** 2) * (A_PROP_SI ** 2) * (sqrt(Vind2) ** 3) + 1e-6)

def parder_pd_Vprop(parder_pd_Vind):
    #Partial derivative of Vprop respect to pd
    return parder_pd_Vind

def parder_u_Vprop_alcm(parder_u_Vind):
    #Partial derivative of Vprop respect to u
    return parder_u_Vind - 1

def parder_deltat_Vprop_alcm(parder_deltat_Vind):
    #Partial derivative of Vprop respect to deltat
    return parder_deltat_Vind

def parder_u_alpha_alcm(w, Vauw):
    #Partial derivative of alpha respect to u
    return - w / ((Vauw ** 2) + 1e-6)

def parder_w_alpha_alcm(u, Vauw):
    #Partial derivative of alpha respect to w
    return u / ((Vauw ** 2) + 1e-6)

def parder_u_beta_alcm(u, v, Va, Vauw):
    #Partial derivative of beta respect to u
    return - (u * v) / (Vauw * (Va ** 2) + 1e-6)

def parder_v_beta_alcm(Va, Vauw):
    #Partial derivative of beta respect to v
    return Vauw / ((Va ** 2) + 1e-6)

def parder_w_beta_alcm(v, w, Va, Vauw):
    #Partial derivative of beta respect to w
    return - (v * w) / (Vauw * (Va ** 2) + 1e-6)

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

def parder_pd_qbaruw_alcm(Vauw, parder_pd_rho):
    #Partial derivative of qbaruw respect to pd
    return 0.5 * parder_pd_rho * (Vauw ** 2)

def parder_u_qbaruw_alcm(parder_u_qbar):
    #Partial derivative of qbaruw respect to u
    return parder_u_qbar

def parder_w_qbaruw_alcm(parder_w_qbar):
    #Partial derivative of qbaruw respect to w
    return parder_w_qbar

def parder_pd_qbarind_alcm(rho, Vind, parder_pd_rho, parder_pd_Vind):
    #Partial derivative of qbarind_sp respect to pd
    return 0.5 * Vind * (parder_pd_rho * Vind + 2 * rho * parder_pd_Vind)

def parder_u_qbarind_alcm(rho, Vind, parder_u_Vind):
    #Partial derivative of qbarind_sp respect to u
    return rho * Vind * parder_u_Vind

def parder_deltat_qbarind_alcm(rho, Vind, parder_deltat_Vind):
    #Partial derivative of qbarind_sp respect to deltat
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

def parder_pd_T_alcm(J, n, parder_pd_rho):
    #Partial derivative of T respect to pd
    return CT_interp(J) * (n ** 2) * (D_PROP_SI ** 4) * parder_pd_rho

def parder_u_T_alcm(rho, J, n, parder_J_CT):
    #Partial derivative of T respect to u
    return rho * n * (D_PROP_SI ** 3) * parder_J_CT_interp(J)

def parder_deltat_T_alcm(u, rho, J, n, parder_J_CT, parder_deltat_n):
    #Partial derivative of T respect to deltat
    return rho * (D_PROP_SI ** 3) * (2 * CT_interp(J) * n * D_PROP_SI - u * parder_J_CT_interp(J)) * parder_deltat_n

''' DYNAMIC COEFFICIENTS PARTIAL DERIVATIVES'''
def parder_u_CD3_alcm(hmacb, parder_alpha_TD3, parder_u_alpha):
    #Partial derivative of CD3 respect to u
    return kCDge_interp(hmacb) * parder_alpha_TD3 * parder_u_alpha

def parder_w_CD3_alcm(hmacb, parder_alpha_TD3, parder_w_alpha):
    #Partial derivative of CD3 respect to w
    return kCDge_interp(hmacb) * parder_alpha_TD3 * parder_w_alpha

def parder_u_CD4_alcm(beta_rad, parder_u_beta):
    #Partial derivative of CD4 respect to u
    return 0 if beta_rad == 0 else 0.1500 * (beta_rad / abs(beta_rad)) * parder_u_beta

def parder_v_CD4_alcm(beta_rad, parder_v_beta):
    #Partial derivative of CD4 respect to v
    return 0 if beta_rad == 0 else 0.1500 * (beta_rad / abs(beta_rad)) * parder_v_beta

def parder_w_CD4_alcm(beta_rad, parder_w_beta):
    #Partial derivative of CD4 respect to w
    return 0 if beta_rad == 0 else 0.1500 * (beta_rad / abs(beta_rad)) * parder_w_beta

def parder_u_CC1_alcm(parder_beta_TC1, parder_u_beta):
    #Partial derivative of CC1 respect to u
    return parder_beta_TC1 * parder_u_beta

def parder_v_CC1_alcm(parder_beta_TC1, parder_v_beta):
    #Partial derivative of CC1 respect to v
    return parder_beta_TC1 * parder_v_beta

def parder_w_CC1_alcm(parder_beta_TC1, parder_w_beta):
    #Partial derivative of CC1 respect to w
    return parder_beta_TC1 * parder_w_beta

def parder_deltar_CC2_alcm():
    #Partial derivative of CC2 respect to deltar
    return 0.1500

def parder_u_CL1_alcm(hmacb, parder_alpha_TL1, parder_u_alpha):
    #Partial derivative of CL1 respect to u
    return kCLge_interp(hmacb) * parder_alpha_TL1 * parder_u_alpha

def parder_w_CL1_alcm(hmacb, parder_alpha_TL1, parder_w_alpha):
    #Partial derivative of CL1 respect to w
    return kCLge_interp(hmacb) * parder_alpha_TL1 * parder_w_alpha

def parder_deltae_CL3_alcm():
    #Partial derivative of CL3 respect to deltae
    return 0.4300

def parder_u_CL4_alcm(q, parder_u_Cw2Va):
    #Partial derivative of CL4 respect to u
    return  3.9000 * q * parder_u_Cw2Va

def parder_v_CL4_alcm(q, parder_v_Cw2Va):
    #Partial derivative of CL4 respect to v
    return  3.9000 * q * parder_v_Cw2Va

def parder_w_CL4_alcm(q, parder_w_Cw2Va):
    #Partial derivative of CL4 respect to w
    return  3.9000 * q * parder_w_Cw2Va

def parder_q_CL4_alcm(Cw2Va):
    #Partial derivative of CL4 respect to q
    return 3.9000 * Cw2Va

def parder_u_CL5_alcm(alphadot_rads, parder_u_Cw2Va):
    #Partial derivative of CL5 respect to u
    return  1.7000 * alphadot_rads  * parder_u_Cw2Va

def parder_v_CL5_alcm(alphadot_rads, parder_v_Cw2Va):
    #Partial derivative of CL5 respect to v
    return  1.7000 * alphadot_rads  * parder_v_Cw2Va

def parder_w_CL5_alcm(alphadot_rads, parder_w_Cw2Va):
    #Partial derivative of CL5 respect to w
    return  1.7000 * alphadot_rads  * parder_w_Cw2Va

def parder_u_Cl1_alcm(beta_rad, alpha_rad, parder_u_beta, parder_alpha_Tl1, parder_u_alpha):
    #Partial derivative of Cl1 respect to u
    return - 0.0920 * (parder_u_beta * Tl1_interp(alpha_rad) + beta_rad * parder_alpha_Tl1 * parder_u_alpha)

def parder_v_Cl1_alcm(beta_rad, alpha_rad, parder_v_beta):
    #Partial derivative of Cl1 respect to v
    return - 0.0920 * (parder_v_beta * Tl1_interp(alpha_rad))

def parder_w_Cl1_alcm(beta_rad, alpha_rad, parder_w_beta, parder_alpha_Tl1, parder_w_alpha):
    #Partial derivative of Cl1 respect to w
    return - 0.0920 * (parder_w_beta * Tl1_interp(alpha_rad) + beta_rad * parder_alpha_Tl1 * parder_w_alpha)

def parder_u_Cl2_alcm(p, parder_u_Bw2Va):
    #Partial derivative of Cl2 respect to u
    return - 0.4840 * parder_u_Bw2Va * p

def parder_v_Cl2_alcm(p, parder_v_Bw2Va):
    #Partial derivative of Cl2 respect to v
    return - 0.4840 * parder_v_Bw2Va * p

def parder_w_Cl2(p, parder_w_Bw2Va):
    #Partial derivative of Cl2 respect to w
    return - 0.4840 * parder_w_Bw2Va * p

def parder_p_Cl2(Bw2Va):
    #Partial derivative of Cl2 respect to w
    return - 0.4840 * Bw2Va

def parder_u_Cl3(Bw2Va, r, sigmaf_rad, alpha_rad, stall, parder_u_Bw2Va, parder_alpha_Tl32, parder_alpha_Tl33, parder_u_alpha):
    #Partial derivative of Cl3 respect to u
    if stall:
        return r * Tl31_interp(sigmaf_rad) * (parder_u_Bw2Va * Tl32_interp(alpha_rad, r) + Bw2Va * parder_alpha_Tl32 * parder_u_alpha)

    else:
        return r * Tl31_interp(sigmaf_rad) * (parder_u_Bw2Va * Tl33_interp(alpha_rad, r) + Bw2Va * parder_alpha_Tl33 * parder_u_alpha)

def parder_v_Cl3(Bw2Va, r, sigmaf_rad, alpha_rad, stall, parder_v_Bw2Va, parder_alpha_Tl32, parder_alpha_Tl33):
    #Partial derivative of Cl3 respect to v
    if stall:
        return r * Tl31_interp(sigmaf_rad) * parder_v_Bw2Va * Tl32_interp(alpha_rad, r)

    else:
        return r * Tl31_interp(sigmaf_rad) * parder_v_Bw2Va * Tl33_interp(alpha_rad, r)

def parder_w_Cl3_alcm(Bw2Va, r, sigmaf_rad, alpha_rad, stall, parder_w_Bw2Va, parder_alpha_Tl32, parder_alpha_Tl33, parder_w_alpha):
    #Partial derivative of Cl3 respect to w
    if stall:
        return r * Tl31_interp(sigmaf_rad) * (parder_w_Bw2Va * Tl32_interp(alpha_rad, r) + Bw2Va * parder_alpha_Tl32 * parder_w_alpha)

    else:
        return r * Tl31_interp(sigmaf_rad) * (parder_w_Bw2Va * Tl33_interp(alpha_rad, r) + Bw2Va * parder_alpha_Tl33 * parder_w_alpha)

def parder_r_Cl3_alcm(Bw2Va, r, sigmaf_rad, alpha_rad, stall, parder_r_Tl32, parder_r_Tl33):
    #Partial derivative of Cl3 respect to r
    if stall:
        return Bw2Va * Tl31_interp(sigmaf_rad) * (Tl32_interp(alpha_rad, r) + r * parder_r_Tl32)
    else:
        return Bw2Va * Tl31_interp(sigmaf_rad) * (Tl32_interp(alpha_rad, r) + r * parder_r_Tl33)

def parder_u_Cl4_alcm(sigmala_rad, sigmara_rad, parder_alpha_Tl4, parder_u_alpha):
    #Partial derivative of Cl4 respect to u
    return 0.2290 * averaged_ailerons(sigmala_rad, sigmara_rad) * parder_alpha_Tl4 * parder_u_alpha

def parder_w_Cl4_alcm(sigmala_rad, sigmara_rad, parder_alpha_Tl4_sp, parder_w_alpha_sp):
    #Partial derivative of Cl4 respect to w
    return 0.2290 * averaged_ailerons(sigmala_rad, sigmara_rad) * parder_alpha_Tl4_sp * parder_w_alpha_sp

def parder_deltaa_Cl4_alcm(alpha_rad, stall):
    #Partial derivative of Cl4 respect to deltaa
    return 0.2290 * deg_to_rad(0.5 * (DELTAA_RANGE[0][1] - DELTAA_RANGE[0][0])) * Tl4_interp(alpha_rad, stall)

def parder_deltar_Cl5_alcm():
    #Partial derivative of Cl5 respect to deltar
    return 0.0147

def parder_pd_Cm1_alcm(parder_qbar_Tm1, parder_pd_qbar):
    #Partial derivative of Cm1 respect to pd
    return  parder_qbar_Tm1 * parder_pd_qbar

def parder_u_Cm2_alcm(alpha_rad, parder_alpha_Tm2, parder_u_alpha):
    #Partial derivative of Cm2 respect to u
    return - 1.8000 * (cos(alpha_rad) * Tm2_interp(alpha_rad) + sin(alpha_rad) * parder_alpha_Tm2 * parder_u_alpha)

def parder_w_Cm2_alcm(alpha_rad, parder_alpha_Tm2, parder_w_alpha):
    #Partial derivative of Cm2 respect to w
    return - 1.8000 * (cos(alpha_rad) * Tm2_interp(alpha_rad) + sin(alpha_rad) * parder_alpha_Tm2 * parder_w_alpha)

def parder_u_Cm3_alcm(q, parder_u_Cw2Va):
    #Partial derivative of Cm3 respect to u
    return - 12.4000 * q * parder_u_Cw2Va  

def parder_v_Cm3_alcm(q, parder_v_Cw2Va):
    #Partial derivative of Cm3 respect to v
    return - 12.4000 * q * parder_v_Cw2Va  

def parder_w_Cm3_alcm(q, parder_w_Cw2Va):
    #Partial derivative of Cm3 respect to w
    return - 12.4000 * q * parder_w_Cw2Va  

def parder_q_Cm3_alcm(q, Cw2Va):
    #Partial derivative of Cm3 respect to q
    return - 12.4000 * Cw2Va

def parder_u_Cm5_alcm(alphadot_rads, parder_u_Cw2Va):
    #Partial derivative of Cm5 respect to u
    return - 7.2700 * alphadot_rads  * parder_u_Cw2Va  

def parder_v_Cm5_alcm(alphadot_rads, parder_v_Cw2Va):
    #Partial derivative of Cm5 respect to v
    return - 7.2700 * alphadot_rads * parder_v_Cw2Va  

def parder_w_Cm5_alcm(alphadot_rads, parder_w_Cw2Va):
    #Partial derivative of Cm5 respect to w
    return - 7.2700  * alphadot_rads * parder_w_Cw2Va  

def parder_u_Cm6_alcm(sigmae_rad, parder_alpha_Tm5, parder_u_alpha):
    #Partial derivative of Cm6 respect to u
    return - 1.2800 * sigmae_rad * parder_alpha_Tm5 * parder_u_alpha

def parder_w_Cm6_alcm(sigmae_rad, parder_alpha_Tm5, parder_w_alpha):
    #Partial derivative of Cm6 respect to w
    return - 1.2800 * sigmae_rad * parder_alpha_Tm5 * parder_w_alpha

def parder_deltae_Cm6_alcm(alpha_rad, sigmae_rad, parder_deltae_Tm5):
    #Partial derivative of Cm6 respect to deltae
    return - 1.2800 * (Tm5_interp(sigmae_rad, alpha_rad) + parder_deltae_Tm5)

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

def parder_u_Cn3_alcm(Bw2Va, r, alpha_rad, parder_u_Bw2Va, parder_alpha_Tn3, parder_u_alpha):
    #Partial derivative of Cn3 respect to u
    return parder_u_Bw2Va * Tn3_interp(r, alpha_rad) + Bw2Va * parder_alpha_Tn3 * parder_u_alpha

def parder_v_Cn3_alcm(Bw2Va, r, alpha_rad, parder_v_Bw2Va):
    #Partial derivative of Cn3 respect to v
    return parder_v_Bw2Va * Tn3_interp(r, alpha_rad)

def parder_w_Cn3_alcm(Bw2Va, r, alpha_rad, parder_w_Bw2Va, parder_alpha_Tn3, parder_w_alpha):
    #Partial derivative of Cn3 respect to w
    return parder_w_Bw2Va * Tn3_interp(r, alpha_rad) + Bw2Va * parder_alpha_Tn3 * parder_w_alpha

def parder_r_Cn3_alcm(Bw2Va, parder_r_Tn3):
    #Partial derivative of Cn3 respect to r
    return Bw2Va * parder_r_Tn3

def parder_u_Cn4_alcm(sigmala_rad, sigmara_rad, parder_alpha_Tn4, parder_beta_Tn4, parder_u_alpha, parder_u_beta):
    #Partial derivative of Cn4 respect to u
    return averaged_ailerons(sigmala_rad, sigmara_rad) * (parder_alpha_Tn4 * parder_u_alpha + parder_beta_Tn4 * parder_u_beta)

def parder_v_Cn4_alcm(sigmala_rad, sigmara_rad, parder_beta_Tn4, parder_v_beta):
    #Partial derivative of Cn4 respect to v
    return averaged_ailerons(sigmala_rad, sigmara_rad) * parder_beta_Tn4 * parder_v_beta

def parder_w_Cn4_alcm(sigmala_rad, sigmara_rad, parder_alpha_Tn4, parder_beta_Tn4, parder_w_alpha, parder_w_beta):
    #Partial derivative of Cn4 respect to w
    return averaged_ailerons(sigmala_rad, sigmara_rad) * (parder_alpha_Tn4 * parder_w_alpha + parder_beta_Tn4 * parder_w_beta)

def parder_deltaa_Cn4_alcm(alpha_rad, beta_rad):
    #Partial derivative of Cn4 respect to deltaa
    return deg_to_rad(0.5 * (DELTAA_RANGE[0][1] - DELTAA_RANGE[0][0])) * Tn4_interp(alpha_rad, beta_rad)

def parder_deltar_Cn5_alcm():
    #Partial derivative of Cn5 respect to deltar
    return - 0.0645 

''' CONTROL MODEL CLASS '''
class ControlModel():

    def __init__(self):
        self.t          = TELEM_WAIT
        self.dt         = 1 / CM_HZ
        self.inputs_str = CM_INPUT_STR
        self.states_str = CM_STATE_STR
        self.csvdata    = np.zeros(CM_STATE_LEN + 1) #array for storing csv actuation

    def alcm(self, cm2act_in, cm2csv_in, rx2cm_out, sp2cm_out, event_start, event_end):
        #Analytic linear control model
        alcm_dict, phys_dict = self._init_alcm() #initialize alcm
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
                        #Preprocess piped-in data
                        rxtime, alcm_dict, phys_dict = self._preprocess_alcm(alcm_dict, phys_dict, rxdata, spdata)
                        #Step control model
                        alcm_dict = self._build_alcm(alcm_dict, phys_dict)
                        alcm_dict = self._step_alcm(alcm_dict)
                        #Pipe out to actuation and CSV logging
                        actdata = self._build_pipe_data_alcm(alcm_dict, rxtime)
                        self._pipe_alcm(cm2act_in, cm2csv_in, actdata)
                        #Update loop variables
                        alcm_dict = self._loop_update_alcm(alcm_dict)
            except:
                raise RuntimeError('.'.join((__name__, sys._getframe().f_code.co_name)))

    def anlcm(self, cm2act_in, cm2csv_in, rx2cm_out, sp2cm_out, event_start, event_end):
        #Analytic non-linear control model
        anlcm_dict, phys_dict = self._init_anlcm() #initialize anlcm
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
                        #Preprocess piped-in data
                        rxtime, anlcm_dict, phys_dict = self._preprocess_anlcm(anlcm_dict, phys_dict, rxdata, spdata)
                        #Step control model
                        anlcm_dict = self._build_anlcm(anlcm_dict)
                        anlcm_dict = self._step_anlcm(anlcm_dict)
                        #Pipe out to actuation and CSV logging
                        actdata = self._build_pipe_data_anlcm(anlcm_dict, rxtime)
                        self._pipe_anlcm(cm2act_in, cm2csv_in, actdata)
                        #Update loop variables
                        anlcm_dict = self._loop_update_anlcm(anlcm_dict)
            except:
                raise RuntimeError('.'.join((__name__, sys._getframe().f_code.co_name)))

    def _preprocess_anlcm(self, anlcm_dict, phys_dict, rxdata, spdata):
        i      = np.where(rxdata[:,0] >= self.t)[0][0] #first RX telemetry frame index
        rxdata = rxdata[i,:] #first RX telemetry frame
        rxtime = rxdata[0]
        #RX telemetry
        longd         = rxdata[2]
        latd          = rxdata[3]
        hqfe_km       = rxdata[15]
        phi_rad       = rxdata[20]
        theta_rad     = rxdata[22]
        psi_rad       = rxdata[24]
        u             = rxdata[34]
        v             = rxdata[35]
        w             = rxdata[36]
        p             = rxdata[46]
        q             = rxdata[47]
        r             = rxdata[48]
        alphadot_rads = rxdata[53]
        sigmara_rad   = rxdata[93] 
        deltara       = rxdata[94]
        sigmala_rad   = rxdata[96]
        deltala       = rxdata[97]
        sigmae_rad    = rxdata[99]
        deltae        = rxdata[100]
        sigmar_rad    = rxdata[105]
        deltar        = rxdata[106]
        deltat        = rxdata[107]
        hmacb         = rxdata[111]
        qbar          = rxdata[112]
        Ixx           = rxdata[120]
        Ixy           = rxdata[121]
        Ixz           = rxdata[122]
        Iyy           = rxdata[123]
        Iyz           = rxdata[124]
        Izz           = rxdata[125]
        mass          = rxdata[126]
        # Conversions
        pn             = latd
        pe             = longd
        pd             = - km_to_m(hqfe_km)
        u              = ft_to_m(u)
        v              = ft_to_m(v)
        w              = ft_to_m(w)
        deltaa         = deltaa_acm(deltala, deltara) 
        qbar           = psf_to_pa(qbar) 
        Ixx            = slugft2_to_kgm2(Ixx)
        Ixy            = slugft2_to_kgm2(Ixy)
        Ixz            = slugft2_to_kgm2(Ixz)
        Iyy            = slugft2_to_kgm2(Iyy)
        Iyz            = slugft2_to_kgm2(Iyz)
        Izz            = slugft2_to_kgm2(Izz)
        mass           = slug_to_kg(mass)
        euler          = np.array([phi_rad, theta_rad, psi_rad])
        q0, q1, q2, q3 = euler_to_attquat(euler)
        #Assumptions of analytic control models
        sigmaf_rad, deltaf, deltam, n, rho, grav, stall = acm_assumptions(deltat, pd)
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
        params_anl = {}
        params_anl.update(phi_rad = phi_rad)
        params_anl.update(theta_rad = theta_rad)
        params_anl.update(psi_rad = psi_rad)
        params_anl.update(alphadot_rads = alphadot_rads)
        params_anl.update(sigmara_rad = sigmara_rad)
        params_anl.update(sigmala_rad = sigmala_rad)
        params_anl.update(sigmae_rad = sigmae_rad)
        params_anl.update(sigmaf_rad = sigmaf_rad)
        params_anl.update(sigmar_rad = sigmar_rad)
        params_anl.update(hmacb = hmacb)
        params_anl.update(stall = stall)
        params_anl.update(Iyy = Iyy)
        params_anl.update(mass = mass)
        params_anl.update(grav = grav)
        params_anl.update(rho = rho)
        params_anl.update(n = n)
        params_anl.update(Gamma = Gamma)
        params_anl.update(Gamma1 = Gamma1)
        params_anl.update(Gamma2 = Gamma2)
        params_anl.update(Gamma3 = Gamma3)
        params_anl.update(Gamma4 = Gamma4)
        params_anl.update(Gamma5 = Gamma5)
        params_anl.update(Gamma6 = Gamma6)
        params_anl.update(Gamma7 = Gamma7)
        params_anl.update(Gamma8 = Gamma8)
        #State
        x_anl = np.row_stack(
                             [
                              pn,
                              pe,
                              pd,
                              q0,
                              q1,
                              q2,
                              q3,
                              u,
                              v,
                              w,
                              p,
                              q,
                              r
                             ]
                            )
        #State setpoint
        x_sp_anl = np.row_stack(spdata[:CM_STATE_LEN])
        #Input
        u_anl = np.row_stack(
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
        u_sp_anl = np.row_stack(spdata[CM_STATE_LEN:])
        #Update ALCM dictionary
        anlcm_dict.update(x_anl = x_anl)
        anlcm_dict.update(x_sp_anl = x_sp_anl)
        anlcm_dict.update(u_anl = u_anl)
        anlcm_dict.update(u_sp_anl = u_sp_anl)
        anlcm_dict.update(params_anl = params_anl)
        #Construct physical variables dictionary
        phys_dict.update(phi_rad = phi_rad)
        phys_dict.update(theta_rad = theta_rad)
        phys_dict.update(psi_rad = psi_rad)
        phys_dict.update(alphadot_rads = alphadot_rads)
        phys_dict.update(sigmara_rad = sigmara_rad)
        phys_dict.update(deltara = deltara)
        phys_dict.update(sigmala_rad = sigmala_rad)
        phys_dict.update(deltala = deltala)
        phys_dict.update(deltaa = deltaa)
        phys_dict.update(sigmae_rad = sigmae_rad)
        phys_dict.update(deltae = deltae)
        phys_dict.update(sigmar_rad = sigmar_rad)
        phys_dict.update(deltar = deltar)
        phys_dict.update(deltat = deltat)
        phys_dict.update(hmacb = hmacb)
        phys_dict.update(qbar = qbar)
        phys_dict.update(Ixx = Ixx)
        phys_dict.update(Ixy = Ixy)
        phys_dict.update(Ixz = Ixz)
        phys_dict.update(Iyy = Iyy)
        phys_dict.update(Iyz = Iyz)
        phys_dict.update(Izz = Izz)
        phys_dict.update(mass = mass)
        phys_dict.update(Gamma = Gamma)
        phys_dict.update(Gamma1 = Gamma1)
        phys_dict.update(Gamma2 = Gamma2)
        phys_dict.update(Gamma3 = Gamma3)
        phys_dict.update(Gamma4 = Gamma4)
        phys_dict.update(Gamma5 = Gamma5)
        phys_dict.update(Gamma6 = Gamma6)
        phys_dict.update(Gamma7 = Gamma7)
        phys_dict.update(Gamma8 = Gamma8)
        phys_dict.update(sigmaf_rad = sigmaf_rad)
        phys_dict.update(deltaf = deltaf)
        phys_dict.update(deltam = deltam)
        phys_dict.update(n = n)
        phys_dict.update(rho = rho)
        phys_dict.update(grav = grav)
        phys_dict.update(stall = stall)
        phys_dict.update(q0 = q0)
        phys_dict.update(q1 = q1)
        phys_dict.update(q2 = q2)
        phys_dict.update(q3 = q3)
        return rxtime, anlcm_dict, phys_dict

    def _build_anlcm(self, anlcm_dict):
        params_anl   = anlcm_dict['params_anl']
        #Analytic linear control model
        anl_sys      = control.iosys.NonlinearIOSystem(self._update_anl, inputs=self.inputs_str, outputs=self.states_str, states=self.states_str, params=params_anl, name='ANL')
        #Update ANLCM dictionary
        anlcm_dict.update(anl_sys = anl_sys)
        return anlcm_dict

    def _step_anlcm(self, anlcm_dict):
        #Analytic non-linear control model
        prev_t_anl   = anlcm_dict['prev_t_anl']
        prev_u_anl   = anlcm_dict['prev_u_anl']
        x_anl        = anlcm_dict['x_anl']
        u_anl        = anlcm_dict['u_anl']
        anl_sys      = anlcm_dict['anl_sys']
        times        = np.array([prev_t_anl, self.t + self.dt])
        t_anl, y_anl = control.input_output_response(anl_sys, times, np.concatenate((prev_u_anl, u_anl),1), x_anl)
        t_anl        = t_anl[1]
        y_anl        = y_anl[:,1]
        #Update ANLCM dictionary
        anlcm_dict.update(t_anl = t_anl)
        anlcm_dict.update(y_anl = y_anl)
        return anlcm_dict

    def _build_pipe_data_anlcm(self, anlcm_dict, rxtime):
        #Actuation pipe data
        x_anl    = anlcm_dict['x_anl']
        x_sp_anl = anlcm_dict['x_sp_anl']
        u_anl    = anlcm_dict['u_anl']
        u_sp_anl = anlcm_dict['u_sp_anl']
        anl_sys  = anlcm_dict['anl_sys']
        actdata = (x_anl, x_sp_anl, u_anl, u_sp_anl, anl_sys)
        #CSV pipe data
        y_anl            = anlcm_dict['y_anl']
        self.csvdata[0]  = rxtime
        self.csvdata[1:] = y_anl
        return actdata

    def _pipe_anlcm(self, cm2act_in, cm2csv_in, actdata):
        if (ACT_TYPE == 0) or (ACT_TYPE == 'random'): #random control
            pass
        else: 
            cm2act_in.send(actdata)
        cm2csv_in.send(self.csvdata)

    def _loop_update_anlcm(self, anlcm_dict):
        t_anl = anlcm_dict['t_anl']
        u_anl = anlcm_dict['u_anl']
        anlcm_dict.update(prev_t_anl = t_anl)
        anlcm_dict.update(prev_u_anl = u_anl)
        self.t = self.t + self.dt
        return anlcm_dict

    def _update_anl(self, time, x_anl, u_anl, params_anl):
        #Parameters
        phi_rad       = params_anl['phi_rad']
        theta_rad     = params_anl['theta_rad']
        psi_rad       = params_anl['psi_rad']
        alphadot_rads = params_anl['alphadot_rads']
        sigmara_rad   = params_anl['sigmara_rad']
        sigmala_rad   = params_anl['sigmala_rad']
        sigmae_rad    = params_anl['sigmae_rad']
        sigmaf_rad    = params_anl['sigmaf_rad']
        sigmar_rad    = params_anl['sigmar_rad']
        hmacb         = params_anl['hmacb']
        stall         = params_anl['stall']
        Iyy           = params_anl['Iyy']
        mass          = params_anl['mass']
        grav          = params_anl['grav']
        rho           = params_anl['rho']
        n             = params_anl['n']
        Gamma         = params_anl['Gamma']
        Gamma1        = params_anl['Gamma1']
        Gamma2        = params_anl['Gamma2']
        Gamma3        = params_anl['Gamma3']
        Gamma4        = params_anl['Gamma4']
        Gamma5        = params_anl['Gamma5']
        Gamma6        = params_anl['Gamma6']
        Gamma7        = params_anl['Gamma7']
        Gamma8        = params_anl['Gamma8']
        #State
        pn = x_anl[0]
        pe = x_anl[1]
        pd = x_anl[2]
        q0 = x_anl[3]
        q1 = x_anl[4]
        q2 = x_anl[5]
        q3 = x_anl[6]
        u  = x_anl[7]
        v  = x_anl[8]
        w  = x_anl[9]
        p  = x_anl[10]
        q  = x_anl[11]
        r  = x_anl[12]
        #Input
        deltaa = u_anl[0]
        deltae = u_anl[1]
        deltaf = u_anl[2]
        deltar = u_anl[3]
        deltat = u_anl[4]
        deltam = u_anl[5]
        #Rotations
        euler = np.array([phi_rad, theta_rad, psi_rad])
        qbv   = body_to_vehicle(phi_rad, theta_rad, psi_rad)
        quat  = euler_to_attquat(euler)
        #Physical variables
        Vauw      = Vauw_acm(u, w)
        Va        = Va_acm(u, v, w)
        alpha_rad = alpha_acm(u, w)
        beta      = beta_acm(v, Vauw)
        J         = J_acm(u, n)
        T         = thrust_eng(J, rho, n)
        Vind2     = Vind2_acm(u, rho, T)
        Vind      = Vind_acm(u, rho, T, Vind2)
        Vprop     = Vprop_acm(u, Vind)
        qbar      = qbar_acm(rho, Va)
        qbaruw    = qbaruw_acm(rho, Vauw)
        qbarind   = qbarind_acm(rho, Vind)
        qbarprop  = qbarprop_acm(rho, Vprop)
        Bw2Va     = BW_SI / (2 * Va)
        Cw2Va     = CW_SI / (2 * Va)
        #Dynamic tables
        CD1 = aerocoeff_CD1()
        CD2 = aerocoeff_CD2(hmacb, sigmaf_rad)
        CD3 = aerocoeff_CD3(hmacb, alpha_rad, sigmaf_rad)
        CD4 = aerocoeff_CD4(beta)
        CC1 = aerocoeff_CC1(beta, sigmaf_rad)
        CC2 = aerocoeff_CC2(sigmar_rad)
        CL1 = aerocoeff_CL1(hmacb, alpha_rad, stall)
        CL2 = aerocoeff_CL2(hmacb, sigmaf_rad)
        CL3 = aerocoeff_CL3(sigmae_rad)
        CL4 = aerocoeff_CL4(Cw2Va, q)
        CL5 = aerocoeff_CL5(Cw2Va, alphadot_rads)
        Cl1 = aerocoeff_Cl1(beta, alpha_rad)
        Cl2 = aerocoeff_Cl2(Bw2Va, p)
        Cl3 = aerocoeff_Cl3(Bw2Va, r, sigmaf_rad, alpha_rad, stall)
        Cl4 = aerocoeff_Cl4(sigmala_rad, sigmara_rad, alpha_rad, stall)
        Cl5 = aerocoeff_Cl5(sigmar_rad)
        Cm1 = aerocoeff_Cm1(qbar)
        Cm2 = aerocoeff_Cm2(alpha_rad)
        Cm3 = aerocoeff_Cm3(Cw2Va, q)
        Cm4 = aerocoeff_Cm4(sigmaf_rad)
        Cm5 = aerocoeff_Cm5(Cw2Va, alphadot_rads)
        Cm6 = aerocoeff_Cm6(sigmae_rad, alpha_rad)
        Cn1 = aerocoeff_Cn1(beta)
        Cn2 = aerocoeff_Cn2(Bw2Va, r)
        Cn3 = aerocoeff_Cn3(Bw2Va, r, alpha_rad)
        Cn4 = aerocoeff_Cn4(sigmala_rad, sigmara_rad, alpha_rad, beta)
        Cn5 = aerocoeff_Cn5(sigmar_rad)
        Cn6 = aerocoeff_Cn6()
        #Dynamics
        D          = qbar * SW_SI * (CD1 + CD2 + CD3 + CD4) 
        C          = qbar * SW_SI * (CC1 + CC2)
        L          = SW_SI * (qbar * (CL1 + CL2 + CL3 + CL4) + qbaruw * CL5)
        gx, gy, gz = gravity_body(quat, mass, grav)
        fx         = - D + gx + T
        fy         = C + gy
        fz         = - L + gz
        l          = qbar * SW_SI * BW_SI * (Cl1 + Cl2 + Cl3 + Cl4 + Cl5)
        m          = SW_SI * CW_SI * (qbar * (Cm1 + Cm2 + Cm3 + Cm4) + qbaruw * Cm5 + qbarind * Cm6)
        n          = SW_SI * BW_SI * (qbar * (Cn1 + Cn2 + Cn3 + Cn4) + qbarind * Cn5 + qbarprop * Cn6)
        #State derivative respect to time
        xdot_anl        = np.zeros(CM_STATE_LEN) #initialize xdot
        xdot_anl[0:3]   = qbv.rotation_matrix @ np.array([u, v, w]) #position in NED frame (pn, pe, pd)
        xdot_anl[3:7]   = 0.5 * np.array([[0, -p, -q, -r], [p, 0, r, -q], [q, -r, 0, p], [r, q, -p, 0]]) @ np.array([q0, q1, q2, q3]) #attitude in vehicle (q0, q1, q2, q3)
        xdot_anl[7:10]  = np.array([[0, -w, v], [w, 0, -u], [-v, u, 0]]) @ np.array([p, q, r]) + np.array([fx, fy, fz]) / mass #linear velocity in body frame (u, v, w)
        xdot_anl[10:13] = np.array([Gamma1*p*q-Gamma2*q*r, Gamma5*p*r-Gamma6*(p**2-r**2), Gamma7*p*q-Gamma1*q*r]) + np.array([[Gamma3, 0, Gamma4], [0, 1/Iyy, 0], [Gamma4, 0, Gamma8]]) @ np.array([l, m, n]) #angular velocity in body frame (p, q, r)
        return xdot_anl 

    def _init_alcm(self):
        #Initialize dictionaries
        alcm_dict = {}
        phys_dict = {}
        #Create states and inputs error strings tuple
        self.inputs_er_str = [item + '_er' for item in self.inputs_str]
        self.states_er_str = [item + '_er' for item in self.states_str]
        #Initialize ALCM
        xdot_al = np.zeros((CM_STATE_LEN, 1)) #state derivative respect to time
        x_al    = xdot_al #state
        x_sp_al = xdot_al #state setpoint
        x_er_al = x_al #state error
        u_al    = np.zeros((CM_INPUT_LEN, 1)) #input
        u_sp_al = u_al #state setpoint
        u_er_al = u_al #input error
        A       = np.zeros((CM_STATE_LEN, CM_STATE_LEN)) #state-state jacobian
        B       = np.zeros((CM_STATE_LEN, CM_INPUT_LEN)) #state-input jacobian
        C       = np.eye(CM_STATE_LEN) #output-state jacobian (y = x)
        D       = np.zeros((CM_STATE_LEN, CM_INPUT_LEN)) #output-input jacobian
        #Initialize loop variables
        prev_t_al    = TELEM_WAIT
        prev_u_er_al = u_er_al
        #Update ALCM dictionary
        alcm_dict.update(prev_t_al = prev_t_al)
        alcm_dict.update(xdot_al = xdot_al)
        alcm_dict.update(x_al = x_al)
        alcm_dict.update(x_sp_al = x_sp_al)
        alcm_dict.update(x_er_al = x_er_al)
        alcm_dict.update(u_al = u_al)
        alcm_dict.update(u_sp_al = u_sp_al)
        alcm_dict.update(u_er_al = u_er_al)
        alcm_dict.update(prev_u_er_al = prev_u_er_al)
        alcm_dict.update(A = A)
        alcm_dict.update(B = B)
        alcm_dict.update(C = C)
        alcm_dict.update(D = D)
        return alcm_dict, phys_dict

    def _init_anlcm(self):
        #Initialize dictionaries
        anlcm_dict = {}
        phys_dict  = {}
        #Initialize ANLCM
        xdot_anl = np.zeros((CM_STATE_LEN, 1)) #state derivative respect to time
        x_anl    = xdot_anl #state
        x_sp_anl = xdot_anl #state setpoint
        u_anl    = np.zeros((CM_INPUT_LEN, 1)) #input
        u_sp_anl = u_anl #state setpoint
        #Initialize loop variables
        prev_t_anl = TELEM_WAIT
        prev_u_anl = u_anl
        #Update ANLCM dictionary
        anlcm_dict.update(prev_t_anl = prev_t_anl)
        anlcm_dict.update(xdot_anl = xdot_anl)
        anlcm_dict.update(x_anl = x_anl)
        anlcm_dict.update(x_sp_anl = x_sp_anl)
        anlcm_dict.update(u_anl = u_anl)
        anlcm_dict.update(prev_u_anl = prev_u_anl)
        anlcm_dict.update(u_sp_anl = u_sp_anl)
        return anlcm_dict, phys_dict

    def _preprocess_alcm(self, alcm_dict, phys_dict, rxdata, spdata):
        i      = np.where(rxdata[:,0] >= self.t)[0][0] #first RX telemetry frame index
        rxdata = rxdata[i,:] #first RX telemetry frame
        rxtime = rxdata[0]
        #RX telemetry
        longd         = rxdata[2]
        latd          = rxdata[3]
        hqfe_km       = rxdata[15]
        phi_rad       = rxdata[20]
        theta_rad     = rxdata[22]
        psi_rad       = rxdata[24]
        u             = rxdata[34]
        v             = rxdata[35]
        w             = rxdata[36]
        p             = rxdata[46]
        q             = rxdata[47]
        r             = rxdata[48]
        alphadot_rads = rxdata[53]
        sigmara_rad   = rxdata[93] 
        deltara       = rxdata[94]
        sigmala_rad   = rxdata[96]
        deltala       = rxdata[97]
        sigmae_rad    = rxdata[99]
        deltae        = rxdata[100]
        sigmar_rad    = rxdata[105]
        deltar        = rxdata[106]
        deltat        = rxdata[107]
        hmacb         = rxdata[111]
        Ixx           = rxdata[120]
        Ixy           = rxdata[121]
        Ixz           = rxdata[122]
        Iyy           = rxdata[123]
        Iyz           = rxdata[124]
        Izz           = rxdata[125]
        mass          = rxdata[126]
        #Setpoint
        pn_sp     = spdata[0]
        pe_sp     = spdata[1]
        pd_sp     = spdata[2]
        q0_sp     = spdata[3]
        q1_sp     = spdata[4]
        q2_sp     = spdata[5]
        q3_sp     = spdata[6]
        u_sp      = spdata[7]
        v_sp      = spdata[8]
        w_sp      = spdata[9]
        p_sp      = spdata[10]
        q_sp      = spdata[11]
        r_sp      = spdata[12]
        deltaa_sp = spdata[13]
        deltae_sp = spdata[14]
        deltaf_sp = spdata[15]
        deltar_sp = spdata[16]
        deltat_sp = spdata[17]
        deltam_sp = spdata[18]
        #Conversions
        pn             = latd
        pe             = longd
        pd             = - km_to_m(hqfe_km)
        u              = ft_to_m(u)
        v              = ft_to_m(v)
        w              = ft_to_m(w)
        deltaa         = deltaa_acm(deltala, deltara) 
        Ixx            = slugft2_to_kgm2(Ixx)
        Ixy            = slugft2_to_kgm2(Ixy)
        Ixz            = slugft2_to_kgm2(Ixz)
        Iyy            = slugft2_to_kgm2(Iyy)
        Iyz            = slugft2_to_kgm2(Iyz)
        Izz            = slugft2_to_kgm2(Izz)
        mass           = slug_to_kg(mass)
        euler          = np.array([phi_rad, theta_rad, psi_rad])
        q0, q1, q2, q3 = euler_to_attquat(euler)
        #Assumptions of analytic control models
        sigmaf_rad, deltaf, deltam, n, rho_sp, grav, stall = acm_assumptions(deltat, pd_sp)
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
        #Physical variables evaluated at the setpoint
        Vauw_sp     = Vauw_acm(u_sp, w_sp)
        Va_sp       = Va_acm(u_sp, v_sp, w_sp)
        alpha_sp    = alpha_acm(u_sp, w_sp)
        beta_sp     = beta_acm(v_sp, Vauw_sp)
        J_sp        = J_acm(u_sp, n)
        T           = thrust_eng(J_sp, rho_sp, n)
        Vind2_sp    = Vind2_acm(u_sp, rho_sp, T)
        Vind_sp     = Vind_acm(u_sp, rho_sp, T, Vind2_sp)
        Vprop_sp    = Vprop_acm(u_sp, Vind_sp)
        qbar_sp     = qbar_acm(rho_sp, Va_sp)
        qbaruw_sp   = qbaruw_acm(rho_sp, Vauw_sp)
        qbarind_sp  = qbarind_acm(rho_sp, Vind_sp)
        qbarprop_sp = qbarprop_acm(rho_sp, Vprop_sp)
        Bw2Va_sp    = BW_SI / (2 * Va_sp)
        Cw2Va_sp    = CW_SI / (2 * Va_sp)
        #State
        x_al = np.row_stack(
                            [
                             pn,
                             pe,
                             pd,
                             q0,
                             q1,
                             q2,
                             q3,
                             u,
                             v,
                             w,
                             p,
                             q,
                             r
                            ]
                           )
        #State setpoint
        x_sp_al = np.row_stack(spdata[:CM_STATE_LEN])
        #State error
        x_er_al = x_al - x_sp_al
        #Input
        u_al = np.row_stack(
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
        u_sp_al = np.row_stack(spdata[CM_STATE_LEN:])
        #Input error
        u_er_al = u_al - u_sp_al
        #Update ALCM dictionary
        alcm_dict.update(x_al = x_al)
        alcm_dict.update(x_sp_al = x_sp_al)
        alcm_dict.update(x_er_al = x_er_al)
        alcm_dict.update(u_al = u_al)
        alcm_dict.update(u_sp_al = u_sp_al)
        alcm_dict.update(u_er_al = u_er_al)
        #Construct physical variables dictionary
        phys_dict.update(phi_rad = phi_rad)
        phys_dict.update(theta_rad = theta_rad)
        phys_dict.update(psi_rad = psi_rad)
        phys_dict.update(alphadot_rads = alphadot_rads)
        phys_dict.update(sigmara_rad = sigmara_rad)
        phys_dict.update(deltara = deltara)
        phys_dict.update(sigmala_rad = sigmala_rad)
        phys_dict.update(deltala = deltala)
        phys_dict.update(deltaa = deltaa)
        phys_dict.update(sigmae_rad = sigmae_rad)
        phys_dict.update(deltae = deltae)
        phys_dict.update(sigmar_rad = sigmar_rad)
        phys_dict.update(deltar = deltar)
        phys_dict.update(deltat = deltat)
        phys_dict.update(hmacb = hmacb)
        phys_dict.update(Ixx = Ixx)
        phys_dict.update(Ixy = Ixy)
        phys_dict.update(Ixz = Ixz)
        phys_dict.update(Iyy = Iyy)
        phys_dict.update(Iyz = Iyz)
        phys_dict.update(Izz = Izz)
        phys_dict.update(mass = mass)
        phys_dict.update(Gamma = Gamma)
        phys_dict.update(Gamma1 = Gamma1)
        phys_dict.update(Gamma2 = Gamma2)
        phys_dict.update(Gamma3 = Gamma3)
        phys_dict.update(Gamma4 = Gamma4)
        phys_dict.update(Gamma5 = Gamma5)
        phys_dict.update(Gamma6 = Gamma6)
        phys_dict.update(Gamma7 = Gamma7)
        phys_dict.update(Gamma8 = Gamma8)
        phys_dict.update(Vauw_sp = Vauw_sp)
        phys_dict.update(Va_sp = Va_sp)
        phys_dict.update(alpha_sp = alpha_sp)
        phys_dict.update(beta_sp = beta_sp)
        phys_dict.update(J_sp = J_sp)
        phys_dict.update(T = T)
        phys_dict.update(Vind2_sp = Vind2_sp)
        phys_dict.update(Vind_sp = Vind_sp)
        phys_dict.update(Vprop_sp = Vprop_sp)
        phys_dict.update(qbar_sp = qbar_sp)
        phys_dict.update(qbaruw_sp = qbaruw_sp)
        phys_dict.update(qbarind_sp = qbarind_sp)
        phys_dict.update(qbarprop_sp = qbarprop_sp)
        phys_dict.update(Bw2Va_sp = Bw2Va_sp)
        phys_dict.update(Cw2Va_sp = Cw2Va_sp)
        phys_dict.update(sigmaf_rad = sigmaf_rad)
        phys_dict.update(deltaf = deltaf)
        phys_dict.update(deltam = deltam)
        phys_dict.update(n = n)
        phys_dict.update(rho_sp = rho_sp)
        phys_dict.update(grav = grav)
        phys_dict.update(stall = stall)
        phys_dict.update(q0 = q0)
        phys_dict.update(q1 = q1)
        phys_dict.update(q2 = q2)
        phys_dict.update(q3 = q3)
        return rxtime, alcm_dict, phys_dict

    def _update_A_alcm(self, A, parder_x_sp):
        #Partial derivatives of state respect to state variables evaluated at the setpoint
        parder_q0_pndot_sp = parder_x_sp[0]
        parder_q1_pndot_sp = parder_x_sp[1]
        parder_q2_pndot_sp = parder_x_sp[2]
        parder_q3_pndot_sp = parder_x_sp[3]
        parder_u_pndot_sp  = parder_x_sp[4]
        parder_v_pndot_sp  = parder_x_sp[5]
        parder_w_pndot_sp  = parder_x_sp[6]
        parder_q0_pedot_sp = parder_x_sp[7]
        parder_q1_pedot_sp = parder_x_sp[8]
        parder_q2_pedot_sp = parder_x_sp[9]
        parder_q3_pedot_sp = parder_x_sp[10]
        parder_u_pedot_sp  = parder_x_sp[11]
        parder_v_pedot_sp  = parder_x_sp[12]
        parder_w_pedot_sp  = parder_x_sp[13]
        parder_q0_pddot_sp = parder_x_sp[14]
        parder_q1_pddot_sp = parder_x_sp[15]
        parder_q2_pddot_sp = parder_x_sp[16]
        parder_q3_pddot_sp = parder_x_sp[17]
        parder_u_pddot_sp  = parder_x_sp[18]
        parder_v_pddot_sp  = parder_x_sp[19]
        parder_w_pddot_sp  = parder_x_sp[20]
        parder_q1_q0dot_sp = parder_x_sp[21]
        parder_q2_q0dot_sp = parder_x_sp[22]
        parder_q3_q0dot_sp = parder_x_sp[23]
        parder_p_q0dot_sp  = parder_x_sp[24]
        parder_q_q0dot_sp  = parder_x_sp[25]
        parder_r_q0dot_sp  = parder_x_sp[26]
        parder_q0_q1dot_sp = parder_x_sp[27]
        parder_q2_q1dot_sp = parder_x_sp[28]
        parder_q3_q1dot_sp = parder_x_sp[29]
        parder_p_q1dot_sp  = parder_x_sp[30]
        parder_q_q1dot_sp  = parder_x_sp[31]
        parder_r_q1dot_sp  = parder_x_sp[32]
        parder_q0_q2dot_sp = parder_x_sp[33]
        parder_q1_q2dot_sp = parder_x_sp[34]
        parder_q3_q2dot_sp = parder_x_sp[35]
        parder_p_q2dot_sp  = parder_x_sp[36]
        parder_q_q2dot_sp  = parder_x_sp[37]
        parder_r_q2dot_sp  = parder_x_sp[38]
        parder_q0_q3dot_sp = parder_x_sp[39]
        parder_q1_q3dot_sp = parder_x_sp[40]
        parder_q2_q3dot_sp = parder_x_sp[41]
        parder_p_q3dot_sp  = parder_x_sp[42]
        parder_q_q3dot_sp  = parder_x_sp[43]
        parder_r_q3dot_sp  = parder_x_sp[44]
        parder_pd_udot_sp  = parder_x_sp[45]
        parder_q0_udot_sp  = parder_x_sp[46]
        parder_q1_udot_sp  = parder_x_sp[47]
        parder_q2_udot_sp  = parder_x_sp[48]
        parder_q3_udot_sp  = parder_x_sp[49]
        parder_u_udot_sp   = parder_x_sp[50]
        parder_v_udot_sp   = parder_x_sp[51]
        parder_w_udot_sp   = parder_x_sp[52]
        parder_q_udot_sp   = parder_x_sp[53]
        parder_r_udot_sp   = parder_x_sp[54]
        parder_pd_vdot_sp  = parder_x_sp[55]
        parder_q0_vdot_sp  = parder_x_sp[56]
        parder_q1_vdot_sp  = parder_x_sp[57]
        parder_q2_vdot_sp  = parder_x_sp[58]
        parder_q3_vdot_sp  = parder_x_sp[59]
        parder_u_vdot_sp   = parder_x_sp[60]
        parder_v_vdot_sp   = parder_x_sp[61]
        parder_w_vdot_sp   = parder_x_sp[62]
        parder_p_vdot_sp   = parder_x_sp[63]
        parder_r_vdot_sp   = parder_x_sp[64]
        parder_pd_wdot_sp  = parder_x_sp[65]
        parder_q0_wdot_sp  = parder_x_sp[66]
        parder_q1_wdot_sp  = parder_x_sp[67]
        parder_q2_wdot_sp  = parder_x_sp[68]
        parder_q3_wdot_sp  = parder_x_sp[69]
        parder_u_wdot_sp   = parder_x_sp[70]
        parder_v_wdot_sp   = parder_x_sp[71]
        parder_w_wdot_sp   = parder_x_sp[72]
        parder_p_wdot_sp   = parder_x_sp[73]
        parder_q_wdot_sp   = parder_x_sp[74]
        parder_pd_pdot_sp  = parder_x_sp[75]
        parder_u_pdot_sp   = parder_x_sp[76]
        parder_v_pdot_sp   = parder_x_sp[77]
        parder_w_pdot_sp   = parder_x_sp[78]
        parder_p_pdot_sp   = parder_x_sp[79]
        parder_q_pdot_sp   = parder_x_sp[80]
        parder_r_pdot_sp   = parder_x_sp[81]
        parder_pd_qdot_sp  = parder_x_sp[82]
        parder_u_qdot_sp   = parder_x_sp[83]
        parder_v_qdot_sp   = parder_x_sp[84]
        parder_w_qdot_sp   = parder_x_sp[85]
        parder_p_qdot_sp   = parder_x_sp[86]
        parder_q_qdot_sp   = parder_x_sp[87]
        parder_r_qdot_sp   = parder_x_sp[88]
        parder_pd_rdot_sp  = parder_x_sp[89]
        parder_u_rdot_sp   = parder_x_sp[90]
        parder_v_rdot_sp   = parder_x_sp[91]
        parder_w_rdot_sp   = parder_x_sp[92]
        parder_p_rdot_sp   = parder_x_sp[93]
        parder_q_rdot_sp   = parder_x_sp[94]
        parder_r_rdot_sp   = parder_x_sp[95]
        #Partial derivatives of pndot respect to state variables
        A[0,3] = parder_q0_pndot_sp
        A[0,4] = parder_q1_pndot_sp
        A[0,5] = parder_q2_pndot_sp
        A[0,6] = parder_q3_pndot_sp
        A[0,7] = parder_u_pndot_sp
        A[0,8] = parder_v_pndot_sp
        A[0,9] = parder_w_pndot_sp
        #Partial derivatives of pedot respect to state variables
        A[1,3] = parder_q0_pedot_sp
        A[1,4] = parder_q1_pedot_sp
        A[1,5] = parder_q2_pedot_sp
        A[1,6] = parder_q3_pedot_sp
        A[1,7] = parder_u_pedot_sp
        A[1,8] = parder_v_pedot_sp
        A[1,9] = parder_w_pedot_sp
        #Partial derivatives of pddot respect to state variables
        A[2,3] = parder_q0_pddot_sp
        A[2,4] = parder_q1_pddot_sp
        A[2,5] = parder_q2_pddot_sp
        A[2,6] = parder_q3_pddot_sp
        A[2,7] = parder_u_pddot_sp
        A[2,8] = parder_v_pddot_sp
        A[2,9] = parder_w_pddot_sp
        #Partial derivatives of q0dot respect to state variables
        A[3,4]  = parder_q1_q0dot_sp
        A[3,5]  = parder_q2_q0dot_sp
        A[3,6]  = parder_q3_q0dot_sp
        A[3,10] = parder_p_q0dot_sp
        A[3,11] = parder_q_q0dot_sp
        A[3,12] = parder_r_q0dot_sp
        #Partial derivatives of q1dot respect to state variables
        A[4,3]  = parder_q0_q1dot_sp
        A[4,5]  = parder_q2_q1dot_sp
        A[4,6]  = parder_q3_q1dot_sp
        A[4,10] = parder_p_q1dot_sp
        A[4,11] = parder_q_q1dot_sp
        A[4,12] = parder_r_q1dot_sp
        #Partial derivatives of q2dot respect to state variables
        A[5,3]  = parder_q0_q2dot_sp
        A[5,4]  = parder_q1_q2dot_sp
        A[5,6]  = parder_q3_q2dot_sp
        A[5,10] = parder_p_q2dot_sp
        A[5,11] = parder_q_q2dot_sp
        A[5,12] = parder_r_q2dot_sp
        #Partial derivatives of q3dot respect to state variables
        A[6,3]  = parder_q0_q3dot_sp
        A[6,4]  = parder_q1_q3dot_sp
        A[6,5]  = parder_q2_q3dot_sp
        A[6,10] = parder_p_q3dot_sp
        A[6,11] = parder_q_q3dot_sp
        A[6,12] = parder_r_q3dot_sp
        #Partial derivatives of udot respect to state variables
        A[7,2]  = parder_pd_udot_sp
        A[7,3]  = parder_q0_udot_sp
        A[7,4]  = parder_q1_udot_sp
        A[7,5]  = parder_q2_udot_sp
        A[7,6]  = parder_q3_udot_sp
        A[7,7]  = parder_u_udot_sp
        A[7,8]  = parder_v_udot_sp
        A[7,9]  = parder_w_udot_sp
        A[7,11] = parder_q_udot_sp
        A[7,12] = parder_r_udot_sp
        #Partial derivatives of vdot respect to state variables
        A[8,2]  = parder_pd_vdot_sp
        A[8,3]  = parder_q0_vdot_sp
        A[8,4]  = parder_q1_vdot_sp
        A[8,5]  = parder_q2_vdot_sp
        A[8,6]  = parder_q3_vdot_sp
        A[8,7]  = parder_u_vdot_sp
        A[8,8]  = parder_v_vdot_sp
        A[8,9]  = parder_w_vdot_sp
        A[8,10] = parder_p_vdot_sp
        A[8,12] = parder_r_vdot_sp
        #Partial derivatives of wdot respect to state variables
        A[9,2]  = parder_pd_wdot_sp
        A[9,3]  = parder_q0_wdot_sp
        A[9,4]  = parder_q1_wdot_sp
        A[9,5]  = parder_q2_wdot_sp
        A[9,6]  = parder_q3_wdot_sp
        A[9,7]  = parder_u_wdot_sp
        A[9,8]  = parder_v_wdot_sp
        A[9,9]  = parder_w_wdot_sp
        A[9,10] = parder_p_wdot_sp
        A[9,11] = parder_q_wdot_sp
        #Partial derivatives of pdot respect to state variables
        A[10,2]  = parder_pd_pdot_sp
        A[10,7]  = parder_u_pdot_sp
        A[10,8]  = parder_v_pdot_sp
        A[10,9]  = parder_w_pdot_sp
        A[10,10] = parder_p_pdot_sp
        A[10,11] = parder_q_pdot_sp
        A[10,12] = parder_r_pdot_sp
        #Partial derivatives of qdot respect to state variables
        A[11,2]  = parder_pd_qdot_sp
        A[11,7]  = parder_u_qdot_sp
        A[11,8]  = parder_v_qdot_sp
        A[11,9]  = parder_w_qdot_sp
        A[11,10] = parder_p_qdot_sp
        A[11,11] = parder_q_qdot_sp
        A[11,12] = parder_r_qdot_sp
        #Partial derivatives of rdot respect to state variables
        A[12,2]  = parder_pd_rdot_sp
        A[12,7]  = parder_u_rdot_sp
        A[12,8]  = parder_v_rdot_sp
        A[12,9]  = parder_w_rdot_sp
        A[12,10] = parder_p_rdot_sp
        A[12,11] = parder_q_rdot_sp
        A[12,12] = parder_r_rdot_sp
        return A

    def _update_B_alcm(self, B, parder_delta_sp):
        #B is reduced by 2 columns corresponding to deltaf and deltam because they are assumed constant in the analytic linear control model
        #State partial derivatives respect to input variables evaluated at the setpoint
        parder_deltat_udot_sp = parder_delta_sp[0]
        parder_deltar_vdot_sp = parder_delta_sp[1]
        parder_deltae_wdot_sp = parder_delta_sp[2]
        parder_deltaa_pdot_sp = parder_delta_sp[3]
        parder_deltar_pdot_sp = parder_delta_sp[4]
        parder_deltae_qdot_sp = parder_delta_sp[5]
        parder_deltaa_rdot_sp = parder_delta_sp[6]
        parder_deltar_rdot_sp = parder_delta_sp[7]
        #Partial derivatives of udot respect to input variables
        B[7,4] = parder_deltat_udot_sp
        #Partial derivatives of vdot respect to input variables
        B[8,3] = parder_deltar_vdot_sp
        #Partial derivatives of wdot respect to input variables
        B[9,1] = parder_deltae_wdot_sp
        #Partial derivatives of pdot respect to input variables
        B[10,0] = parder_deltaa_pdot_sp
        B[10,3] = parder_deltar_pdot_sp
        #Partial derivatives of qdot respect to input variables
        B[11,1] = parder_deltae_qdot_sp
        #Partial derivatives of rdot respect to input variables
        B[12,0] = parder_deltaa_rdot_sp
        B[12,3] = parder_deltar_rdot_sp
        return B

    def _build_alcm(self, alcm_dict, phys_dict):
        #Retrieve data
        x_er_al = alcm_dict['x_er_al']
        u_er_al = alcm_dict['u_er_al']
        A       = alcm_dict['A']
        B       = alcm_dict['B']
        C       = alcm_dict['C']
        D       = alcm_dict['D']
        #Compute partial derivatives
        parder_x_sp, parder_delta_sp = self._parder_alcm(alcm_dict, phys_dict)
        #Update state space
        A = self._update_A_alcm(A, parder_x_sp)
        B = self._update_B_alcm(B, parder_delta_sp)
        #Analytic linear control model
        xdot_er_al  = A @ x_er_al + B @ u_er_al
        al_ss       = control.StateSpace(A, B, C, D) #state space
        al_sys      = control.iosys.LinearIOSystem(al_ss, inputs=self.inputs_er_str, outputs=self.states_er_str, states=self.states_er_str, name='AL') #linear input-output system
        #Update ALCM dictionary
        alcm_dict.update(xdot_er_al = xdot_er_al)
        alcm_dict.update(al_sys = al_sys)
        return alcm_dict

    def _step_alcm(self, alcm_dict):
        prev_t_al     = alcm_dict['prev_t_al']
        prev_u_er_al  = alcm_dict['prev_u_er_al']
        x_er_al       = alcm_dict['x_er_al']
        u_er_al       = alcm_dict['u_er_al']
        al_sys        = alcm_dict['al_sys']
        times         = np.array([prev_t_al, self.t + self.dt])
        t_al, y_er_al = control.input_output_response(al_sys, times, np.concatenate((prev_u_er_al, u_er_al),1), x_er_al)
        t_al          = t_al[1]
        y_er_al       = y_er_al[:,1]
        #Update ALCM dictionary
        alcm_dict.update(t_al = t_al)
        alcm_dict.update(y_er_al = y_er_al)
        return alcm_dict

    def _build_pipe_data_alcm(self, alcm_dict, rxtime):
        #Actuation pipe data
        x_al    = alcm_dict['x_al']
        x_sp_al = alcm_dict['x_sp_al']
        x_er_al = alcm_dict['x_er_al']
        u_al    = alcm_dict['u_al']
        u_sp_al = alcm_dict['u_sp_al']
        u_er_al = alcm_dict['u_er_al']
        al_sys  = alcm_dict['al_sys']
        actdata = (x_al, x_sp_al, x_er_al, u_al, u_sp_al, u_er_al, al_sys)
        #CSV pipe data
        y_er_al          = alcm_dict['y_er_al']
        self.csvdata[0]  = rxtime
        self.csvdata[1:] = y_er_al + x_sp_al.squeeze()
        return actdata

    def _pipe_alcm(self, cm2act_in, cm2csv_in, actdata):
        if (ACT_TYPE == 0) or (ACT_TYPE == 'random'): #random control
            pass
        else: 
            cm2act_in.send(actdata)
        cm2csv_in.send(self.csvdata)

    def _loop_update_alcm(self, alcm_dict):
        t_al    = alcm_dict['t_al']
        u_er_al = alcm_dict['u_er_al']
        alcm_dict.update(prev_t_al = t_al)
        alcm_dict.update(prev_u_er_al = u_er_al)
        self.t = self.t + self.dt
        return alcm_dict

    def _parder_alcm(self, alcm_dict, phys_dict):
        #State setpoint
        x_sp_al   = alcm_dict['x_sp_al']
        pn_sp     = x_sp_al[0]
        pe_sp     = x_sp_al[1]
        pd_sp     = x_sp_al[2]
        q0_sp     = x_sp_al[3]
        q1_sp     = x_sp_al[4]
        q2_sp     = x_sp_al[5]
        q3_sp     = x_sp_al[6]
        u_sp      = x_sp_al[7]
        v_sp      = x_sp_al[8]
        w_sp      = x_sp_al[9]
        p_sp      = x_sp_al[10]
        q_sp      = x_sp_al[11]
        r_sp      = x_sp_al[12]
        #Input setpoint
        u_sp_al   = alcm_dict['u_sp_al']
        deltaa_sp = u_sp_al[0]
        deltae_sp = u_sp_al[1]
        deltaf_sp = u_sp_al[2]
        deltar_sp = u_sp_al[3]
        deltat_sp = u_sp_al[4]
        deltam_sp = u_sp_al[5]
        #Physical variables
        phi_rad       = phys_dict['phi_rad']
        theta_rad     = phys_dict['theta_rad']
        psi_rad       = phys_dict['psi_rad']
        alphadot_rads = phys_dict['alphadot_rads']
        sigmara_rad   = phys_dict['sigmara_rad']
        deltara       = phys_dict['deltara']
        sigmala_rad   = phys_dict['sigmala_rad']
        deltala       = phys_dict['deltala']
        sigmae_rad    = phys_dict['sigmae_rad']
        deltae        = phys_dict['deltae']
        sigmar_rad    = phys_dict['sigmar_rad']
        deltar        = phys_dict['deltar']
        deltat        = phys_dict['deltat']
        hmacb         = phys_dict['hmacb']
        Ixx           = phys_dict['Ixx']
        Ixy           = phys_dict['Ixy']
        Ixz           = phys_dict['Ixz']
        Iyy           = phys_dict['Iyy']
        Iyz           = phys_dict['Iyz']
        Izz           = phys_dict['Izz']
        mass          = phys_dict['mass']
        Gamma         = phys_dict['Gamma']
        Gamma1        = phys_dict['Gamma1']
        Gamma2        = phys_dict['Gamma2']
        Gamma3        = phys_dict['Gamma3']
        Gamma4        = phys_dict['Gamma4']
        Gamma5        = phys_dict['Gamma5']
        Gamma6        = phys_dict['Gamma6']
        Gamma7        = phys_dict['Gamma7']
        Gamma8        = phys_dict['Gamma8']
        Vauw_sp       = phys_dict['Vauw_sp']
        Va_sp         = phys_dict['Va_sp']
        alpha_sp      = phys_dict['alpha_sp']
        beta_sp       = phys_dict['beta_sp']
        J_sp          = phys_dict['J_sp']
        T             = phys_dict['T']
        Vind2_sp      = phys_dict['Vind2_sp']
        Vind_sp       = phys_dict['Vind_sp']
        Vprop_sp      = phys_dict['Vprop_sp']
        qbar_sp       = phys_dict['qbar_sp']
        qbaruw_sp     = phys_dict['qbaruw_sp']
        qbarind_sp    = phys_dict['qbarind_sp']
        qbarprop_sp   = phys_dict['qbarprop_sp']
        Bw2Va_sp      = phys_dict['Bw2Va_sp']
        Cw2Va_sp      = phys_dict['Cw2Va_sp']
        sigmaf_rad    = phys_dict['sigmaf_rad']
        deltaf        = phys_dict['deltaf']
        deltam        = phys_dict['deltam']
        n             = phys_dict['n']
        rho_sp        = phys_dict['rho_sp']
        grav          = phys_dict['grav']
        stall         = phys_dict['stall']
        #Dynamics tables evaluated at the setpoint
        CD1_sp = aerocoeff_CD1()
        CD2_sp = aerocoeff_CD2(hmacb, sigmaf_rad)
        CD3_sp = aerocoeff_CD3(hmacb, alpha_sp, sigmaf_rad)
        CD4_sp = aerocoeff_CD4(beta_sp)
        CC1_sp = aerocoeff_CC1(beta_sp, sigmaf_rad)
        CC2_sp = aerocoeff_CC2(sigmar_rad)
        CL1_sp = aerocoeff_CL1(hmacb, alpha_sp, stall)
        CL2_sp = aerocoeff_CL2(hmacb, sigmaf_rad)
        CL3_sp = aerocoeff_CL3(sigmae_rad)
        CL4_sp = aerocoeff_CL4(Cw2Va_sp, q_sp)
        CL5_sp = aerocoeff_CL5(Cw2Va_sp, alphadot_rads)
        Cl1_sp = aerocoeff_Cl1(beta_sp, alpha_sp)
        Cl2_sp = aerocoeff_Cl2(Bw2Va_sp, p_sp)
        Cl3_sp = aerocoeff_Cl3(Bw2Va_sp, r_sp, sigmaf_rad, alpha_sp, stall)
        Cl4_sp = aerocoeff_Cl4(sigmala_rad, sigmara_rad, alpha_sp, stall)
        Cl5_sp = aerocoeff_Cl5(sigmar_rad)
        Cm1_sp = aerocoeff_Cm1(qbar_sp)
        Cm2_sp = aerocoeff_Cm2(alpha_sp)
        Cm3_sp = aerocoeff_Cm3(Cw2Va_sp, q_sp)
        Cm4_sp = aerocoeff_Cm4(sigmaf_rad)
        Cm5_sp = aerocoeff_Cm5(Cw2Va_sp, alphadot_rads)
        Cm6_sp = aerocoeff_Cm6(sigmae_rad, alpha_sp)
        Cn1_sp = aerocoeff_Cn1(beta_sp)
        Cn2_sp = aerocoeff_Cn2(Bw2Va_sp, r_sp)
        Cn3_sp = aerocoeff_Cn3(Bw2Va_sp, r_sp, alpha_sp)
        Cn4_sp = aerocoeff_Cn4(sigmala_rad, sigmara_rad, alpha_sp, beta_sp)
        Cn5_sp = aerocoeff_Cn5(sigmar_rad)
        Cn6_sp = aerocoeff_Cn6()
        #Derivatives of dynamic tables respect to physical variables evaluated at the setpoint 
        parder_alpha_TD3_sp  = parder_alpha_TD3_interp(alpha_sp, sigmaf_rad)
        parder_beta_TC1_sp   = parder_beta_TC1_interp(beta_sp, sigmaf_rad)
        parder_alpha_TL1_sp  = parder_alpha_TL1_interp(alpha_sp, stall)
        parder_alpha_Tl1_sp  = parder_alpha_Tl1_interp(alpha_sp)
        parder_alpha_Tl32_sp = parder_alpha_Tl32_interp(alpha_sp, r_sp)
        parder_r_Tl32_sp     = parder_r_Tl32_interp(alpha_sp, r_sp)
        parder_alpha_Tl33_sp = parder_alpha_Tl33_interp(alpha_sp, r_sp)
        parder_r_Tl33_sp     = parder_r_Tl33_interp(alpha_sp, r_sp)
        parder_alpha_Tl4_sp  = parder_alpha_Tl4_interp(alpha_sp, stall)
        parder_qbar_Tm1_sp   = parder_qbar_Tm1_interp(qbar_sp)
        parder_alpha_Tm2_sp  = parder_alpha_Tm2_interp(alpha_sp)
        parder_deltae_Tm5_sp = parder_deltae_Tm5_interp(deltae_sp, alpha_sp)
        parder_alpha_Tm5_sp  = parder_alpha_Tm5_interp(deltae_sp, alpha_sp)
        parder_beta_Tn1_sp   = parder_beta_Tn1_interp(beta_sp)
        parder_r_Tn3_sp      = parder_r_Tn3_interp(r_sp, alpha_sp)
        parder_alpha_Tn3_sp  = parder_alpha_Tn3_interp(alpha_sp, r_sp)
        parder_alpha_Tn4_sp  = parder_alpha_Tn4_interp(alpha_sp, beta_sp)
        parder_beta_Tn4_sp   = parder_beta_Tn4_interp(alpha_sp, beta_sp)
        parder_J_CT_sp       = parder_J_CT_interp(J_sp)
        #Derivatives of physical variables respect to state variables evaluated at the setpoint
        parder_pd_rho_sp          = parder_pd_rho_alcm(pd_sp)
        parder_deltat_n_sp        = parder_deltat_n_alcm()
        parder_u_alpha_sp         = parder_u_alpha_alcm(w_sp, Vauw_sp)
        parder_w_alpha_sp         = parder_w_alpha_alcm(u_sp, Vauw_sp)
        parder_u_beta_sp          = parder_u_beta_alcm(u_sp, v_sp, Va_sp, Vauw_sp)
        parder_v_beta_sp          = parder_v_beta_alcm(Va_sp, Vauw_sp)
        parder_w_beta_sp          = parder_w_beta_alcm(v_sp, w_sp, Va_sp, Vauw_sp)
        parder_pd_Vind_sp         = parder_pd_Vind_alcm(u_sp, rho_sp, T, Vind2_sp, parder_pd_rho_sp)
        parder_u_Vind_sp          = parder_u_Vind_alcm(u_sp, rho_sp, T, Vind2_sp)
        parder_deltat_T_sp        = parder_deltat_T_alcm(u_sp, rho_sp, J_sp, n, parder_J_CT_sp, parder_deltat_n_sp)
        parder_deltat_Vind_sp     = parder_deltat_Vind_alcm(u_sp, rho_sp, T, Vind2_sp, parder_deltat_T_sp)
        parder_pd_Vprop_sp        = parder_pd_Vprop(parder_pd_Vind_sp)
        parder_u_Vprop_sp         = parder_u_Vprop_alcm(parder_u_Vind_sp)
        parder_deltat_Vprop_sp    = parder_deltat_Vprop_alcm(parder_deltat_Vind_sp)
        parder_pd_qbar_sp         = parder_pd_qbar_alcm(Va_sp, parder_pd_rho_sp)
        parder_u_qbar_sp          = parder_u_qbar_alcm(u_sp, rho_sp)
        parder_v_qbar_sp          = parder_v_qbar_alcm(v_sp, rho_sp)
        parder_w_qbar_sp          = parder_w_qbar_alcm(w_sp, rho_sp)
        parder_pd_qbaruw_sp       = parder_pd_qbaruw_alcm(Vauw_sp, parder_pd_rho_sp)
        parder_u_qbaruw_sp        = parder_u_qbaruw_alcm(parder_u_qbar_sp)
        parder_w_qbaruw_sp        = parder_w_qbaruw_alcm(parder_w_qbar_sp)
        parder_pd_qbarind_sp      = parder_pd_qbarind_alcm(rho_sp, Vind_sp, parder_pd_rho_sp, parder_pd_Vind_sp)
        parder_u_qbarind_sp       = parder_u_qbarind_alcm(rho_sp, Vind_sp, parder_u_Vind_sp)
        parder_deltat_qbarind_sp  = parder_deltat_qbarind_alcm(rho_sp, Vind_sp, parder_deltat_Vind_sp)
        parder_pd_qbarprop_sp     = parder_pd_qbarprop_alcm(rho_sp, Vprop_sp, parder_pd_rho_sp, parder_pd_Vprop_sp)
        parder_deltat_qbarprop_sp = parder_deltat_qbarprop_alcm(rho_sp, Vprop_sp, parder_deltat_Vprop_sp)
        parder_u_qbarprop_sp      = parder_u_qbarprop_alcm(rho_sp, Vprop_sp, parder_u_Vprop_sp)
        parder_pd_T_sp            = parder_pd_T_alcm(J_sp, n, parder_pd_rho_sp)
        parder_u_T_sp             = parder_u_T_alcm(rho_sp, J_sp, n, parder_J_CT_sp)
        parder_u_Va_sp            = parder_u_Va_alcm(u_sp, Va_sp)
        parder_v_Va_sp            = parder_v_Va_alcm(v_sp, Va_sp)
        parder_w_Va_sp            = parder_w_Va_alcm(w_sp, Va_sp)
        parder_u_Bw2Va_sp         = parder_u_Bw2Va_alcm(u_sp, Va_sp, parder_u_Va_sp)
        parder_v_Bw2Va_sp         = parder_v_Bw2Va_alcm(v_sp, Va_sp, parder_v_Va_sp)
        parder_w_Bw2Va_sp         = parder_w_Bw2Va_alcm(w_sp, Va_sp, parder_w_Va_sp)
        parder_u_Cw2Va_sp         = parder_u_Cw2Va_alcm(u_sp, Va_sp, parder_u_Va_sp)
        parder_v_Cw2Va_sp         = parder_v_Cw2Va_alcm(v_sp, Va_sp, parder_v_Va_sp)
        parder_w_Cw2Va_sp         = parder_w_Cw2Va_alcm(w_sp, Va_sp, parder_w_Va_sp)
        #Partial derivatives of dynamics coefficients respect to state variables evaluated at the setpoint 
        parder_u_CD3_sp      = parder_u_CD3_alcm(hmacb, parder_alpha_TD3_sp, parder_u_alpha_sp)
        parder_w_CD3_sp      = parder_w_CD3_alcm(hmacb, parder_alpha_TD3_sp, parder_w_alpha_sp)
        parder_u_CD4_sp      = parder_u_CD4_alcm(beta_sp, parder_u_beta_sp)
        parder_v_CD4_sp      = parder_v_CD4_alcm(beta_sp, parder_v_beta_sp)
        parder_w_CD4_sp      = parder_w_CD4_alcm(beta_sp, parder_w_beta_sp)
        parder_u_CC1_sp      = parder_u_CC1_alcm(parder_beta_TC1_sp, parder_u_beta_sp)
        parder_v_CC1_sp      = parder_v_CC1_alcm(parder_beta_TC1_sp, parder_v_beta_sp)
        parder_w_CC1_sp      = parder_w_CC1_alcm(parder_beta_TC1_sp, parder_w_beta_sp)
        parder_deltar_CC2_sp = parder_deltar_CC2_alcm()
        parder_u_CL1_sp      = parder_u_CL1_alcm(hmacb, parder_alpha_TL1_sp, parder_u_alpha_sp)
        parder_w_CL1_sp      = parder_w_CL1_alcm(hmacb, parder_alpha_TL1_sp, parder_w_alpha_sp)
        parder_deltae_CL3_sp = parder_deltae_CL3_alcm()
        parder_u_CL4_sp      = parder_u_CL4_alcm(q_sp, parder_u_Cw2Va_sp)
        parder_v_CL4_sp      = parder_v_CL4_alcm(q_sp, parder_v_Cw2Va_sp)
        parder_w_CL4_sp      = parder_w_CL4_alcm(q_sp, parder_w_Cw2Va_sp)
        parder_q_CL4_sp      = parder_q_CL4_alcm(Cw2Va_sp)
        parder_u_CL5_sp      = parder_u_CL5_alcm(alphadot_rads, parder_u_Cw2Va_sp)
        parder_v_CL5_sp      = parder_v_CL5_alcm(alphadot_rads, parder_v_Cw2Va_sp)
        parder_w_CL5_sp      = parder_w_CL5_alcm(alphadot_rads, parder_w_Cw2Va_sp)
        parder_u_Cl1_sp      = parder_u_Cl1_alcm(beta_sp, alpha_sp, parder_u_beta_sp, parder_alpha_Tl1_sp, parder_u_alpha_sp)
        parder_v_Cl1_sp      = parder_v_Cl1_alcm(beta_sp, alpha_sp, parder_v_beta_sp)
        parder_w_Cl1_sp      = parder_w_Cl1_alcm(beta_sp, alpha_sp, parder_w_beta_sp, parder_alpha_Tl1_sp, parder_w_alpha_sp)
        parder_u_Cl2_sp      = parder_u_Cl2_alcm(p_sp, parder_u_Bw2Va_sp)
        parder_v_Cl2_sp      = parder_v_Cl2_alcm(p_sp, parder_v_Bw2Va_sp)
        parder_w_Cl2_sp      = parder_w_Cl2(p_sp, parder_w_Bw2Va_sp)
        parder_p_Cl2_sp      = parder_p_Cl2(Bw2Va_sp)
        parder_u_Cl3_sp      = parder_u_Cl3(Bw2Va_sp, r_sp, sigmaf_rad, alpha_sp, stall, parder_u_Bw2Va_sp, parder_alpha_Tl32_sp, parder_alpha_Tl33_sp, parder_u_alpha_sp)
        parder_v_Cl3_sp      = parder_v_Cl3(Bw2Va_sp, r_sp, sigmaf_rad, alpha_sp, stall, parder_v_Bw2Va_sp, parder_alpha_Tl32_sp, parder_alpha_Tl33_sp)
        parder_w_Cl3_sp      = parder_w_Cl3_alcm(Bw2Va_sp, r_sp, sigmaf_rad, alpha_sp, stall, parder_w_Bw2Va_sp, parder_alpha_Tl32_sp, parder_alpha_Tl33_sp, parder_w_alpha_sp)
        parder_r_Cl3_sp      = parder_r_Cl3_alcm(Bw2Va_sp, r_sp, sigmaf_rad, alpha_sp, stall, parder_r_Tl32_sp, parder_r_Tl33_sp)
        parder_u_Cl4_sp      = parder_u_Cl4_alcm(sigmala_rad, sigmara_rad, parder_alpha_Tl4_sp, parder_u_alpha_sp)
        parder_w_Cl4_sp      = parder_w_Cl4_alcm(sigmala_rad, sigmara_rad, parder_alpha_Tl4_sp, parder_u_alpha_sp)
        parder_deltaa_Cl4_sp = parder_deltaa_Cl4_alcm(alpha_sp, stall)
        parder_deltar_Cl5_sp = parder_deltar_Cl5_alcm()
        parder_pd_Cm1_sp     = parder_pd_Cm1_alcm(parder_qbar_Tm1_sp, parder_pd_qbar_sp)
        parder_u_Cm2_sp      = parder_u_Cm2_alcm(alpha_sp, parder_alpha_Tm2_sp, parder_u_alpha_sp)
        parder_w_Cm2_sp      = parder_w_Cm2_alcm(alpha_sp, parder_alpha_Tm2_sp, parder_w_alpha_sp)
        parder_u_Cm3_sp      = parder_u_Cm3_alcm(q_sp, parder_u_Cw2Va_sp)
        parder_v_Cm3_sp      = parder_v_Cm3_alcm(q_sp, parder_v_Cw2Va_sp)
        parder_w_Cm3_sp      = parder_w_Cm3_alcm(q_sp, parder_w_Cw2Va_sp)
        parder_q_Cm3_sp      = parder_q_Cm3_alcm(q_sp, Cw2Va_sp)
        parder_u_Cm5_sp      = parder_u_Cm5_alcm(alphadot_rads, parder_u_Cw2Va_sp)
        parder_v_Cm5_sp      = parder_v_Cm5_alcm(alphadot_rads, parder_v_Cw2Va_sp)
        parder_w_Cm5_sp      = parder_w_Cm5_alcm(alphadot_rads, parder_w_Cw2Va_sp)
        parder_u_Cm6_sp      = parder_u_Cm6_alcm(sigmae_rad, parder_alpha_Tm5_sp, parder_u_alpha_sp)
        parder_w_Cm6_sp      = parder_w_Cm6_alcm(sigmae_rad, parder_alpha_Tm5_sp, parder_w_alpha_sp)
        parder_deltae_Cm6_sp = parder_deltae_Cm6_alcm(alpha_sp, sigmae_rad, parder_deltae_Tm5_sp)
        parder_u_Cn1_sp      = parder_u_Cn1_alcm(parder_beta_Tn1_sp, parder_u_beta_sp)
        parder_v_Cn1_sp      = parder_v_Cn1_alcm(parder_beta_Tn1_sp, parder_v_beta_sp)
        parder_w_Cn1_sp      = parder_w_Cn1_alcm(parder_beta_Tn1_sp, parder_w_beta_sp)
        parder_u_Cn2_sp      = parder_u_Cn2_alcm(r_sp, parder_u_Bw2Va_sp)
        parder_v_Cn2_sp      = parder_v_Cn2_alcm(r_sp, parder_v_Bw2Va_sp)
        parder_w_Cn2_sp      = parder_w_Cn2_alcm(r_sp, parder_w_Bw2Va_sp)
        parder_r_Cn2_sp      = parder_r_Cn2_alcm(Bw2Va_sp)
        parder_u_Cn3_sp      = parder_u_Cn3_alcm(Bw2Va_sp, r_sp, alpha_sp, parder_u_Bw2Va_sp, parder_alpha_Tn3_sp, parder_u_alpha_sp)
        parder_v_Cn3_sp      = parder_v_Cn3_alcm(Bw2Va_sp, r_sp, alpha_sp, parder_v_Bw2Va_sp)
        parder_w_Cn3_sp      = parder_w_Cn3_alcm(Bw2Va_sp, r_sp, alpha_sp, parder_w_Bw2Va_sp, parder_alpha_Tn3_sp, parder_w_alpha_sp)
        parder_r_Cn3_sp      = parder_r_Cn3_alcm(Bw2Va_sp, parder_r_Tn3_sp)
        parder_u_Cn4_sp      = parder_u_Cn4_alcm(sigmala_rad, sigmara_rad, parder_alpha_Tn4_sp, parder_beta_Tn4_sp, parder_u_alpha_sp, parder_u_beta_sp)
        parder_v_Cn4_sp      = parder_v_Cn4_alcm(sigmala_rad, sigmara_rad, parder_beta_Tn4_sp, parder_v_beta_sp)
        parder_w_Cn4_sp      = parder_w_Cn4_alcm(sigmala_rad, sigmara_rad, parder_alpha_Tn4_sp, parder_beta_Tn4_sp, parder_w_alpha_sp, parder_w_beta_sp)
        parder_deltaa_Cn4_sp = parder_deltaa_Cn4_alcm(alpha_sp, beta_sp)
        parder_deltar_Cn5_sp = parder_deltar_Cn5_alcm()
        #Partial derivatives of pndot respect to state variables evaluated at the setpoint
        parder_q0_pndot_sp = 2 * (q0_sp * u_sp - q3_sp * v_sp + q2_sp * w_sp)
        parder_q1_pndot_sp = 2 * (q1_sp * u_sp + q2_sp * v_sp + q3_sp * w_sp)
        parder_q2_pndot_sp = 2 * (- q2_sp * u_sp + q1_sp * v_sp + q0_sp * w_sp)
        parder_q3_pndot_sp = 2 * (- q3_sp * u_sp - q0_sp * v_sp + q1_sp * w_sp)
        parder_u_pndot_sp  = q0_sp ** 2 + q1_sp ** 2 - q2_sp ** 2 - q3_sp ** 2
        parder_v_pndot_sp  = 2 * (q1_sp * q2_sp - q0_sp * q3_sp)
        parder_w_pndot_sp  = 2 * (q0_sp * q2_sp + q1_sp * q3_sp)
        #Partial derivatives of pedot respect to state variables evaluated at the setpoint
        parder_q0_pedot_sp = 2 * (q3_sp * u_sp + q0_sp * v_sp - q1_sp * w_sp)
        parder_q1_pedot_sp = 2 * (q2_sp * u_sp - q1_sp * v_sp - q0_sp * w_sp)
        parder_q2_pedot_sp = 2 * (q1_sp * u_sp + q2_sp * v_sp + q3_sp * w_sp)
        parder_q3_pedot_sp = 2 * (q0_sp * u_sp - q3_sp * v_sp + q2_sp * w_sp)
        parder_u_pedot_sp  = 2 * (q0_sp * q3_sp + q1_sp * q2_sp)
        parder_v_pedot_sp  = q0_sp ** 2 + q2_sp ** 2 - q1_sp ** 2 - q3_sp ** 2
        parder_w_pedot_sp  = 2 * (q2_sp * q3_sp - q0_sp * q1_sp)
        #Partial derivatives of pddot respect to state variables evaluated at the setpoint
        parder_q0_pddot_sp = 2 * (- q2_sp * u_sp + q1_sp * v_sp + q0_sp * w_sp)
        parder_q1_pddot_sp = 2 * (q3_sp * u_sp + q0_sp * v_sp - q1_sp * w_sp)
        parder_q2_pddot_sp = 2 * (- q0_sp * u_sp + q3_sp * v_sp - q2_sp * w_sp)
        parder_q3_pddot_sp = 2 * (q1_sp * u_sp + q2_sp * v_sp + q3_sp * w_sp)
        parder_u_pddot_sp  = 2 * (q1_sp * q3_sp - q0_sp * q2_sp)
        parder_v_pddot_sp  = 2 * (q0_sp * q1_sp + q2_sp * q3_sp)
        parder_w_pddot_sp  = q0_sp ** 2 + q3_sp ** 2 - q1_sp ** 2 - q2_sp ** 2
        #Partial derivatives of q0dot respect to state variables evaluated at the setpoint
        parder_q1_q0dot_sp = - 0.5 * p_sp
        parder_q2_q0dot_sp = - 0.5 * q_sp
        parder_q3_q0dot_sp = - 0.5 * r_sp
        parder_p_q0dot_sp  = - 0.5 * q1_sp
        parder_q_q0dot_sp  = - 0.5 * q2_sp
        parder_r_q0dot_sp  = - 0.5 * q3_sp
        #Partial derivatives of q1dot respect to state variables evaluated at the setpoint
        parder_q0_q1dot_sp = 0.5 * p_sp
        parder_q2_q1dot_sp = 0.5 * r_sp
        parder_q3_q1dot_sp = - 0.5 * q_sp
        parder_p_q1dot_sp  = 0.5 * q0_sp
        parder_q_q1dot_sp  = - 0.5 * q3_sp
        parder_r_q1dot_sp  = 0.5 * q2_sp
        #Partial derivatives of q2dot respect to state variables evaluated at the setpoint
        parder_q0_q2dot_sp = 0.5 * q_sp
        parder_q1_q2dot_sp = - 0.5 * r_sp
        parder_q3_q2dot_sp = 0.5 * p_sp
        parder_p_q2dot_sp  = 0.5 * q3_sp
        parder_q_q2dot_sp  = 0.5 * q0_sp
        parder_r_q2dot_sp  = - 0.5 * q1_sp
        #Partial derivatives of q3dot respect to state variables evaluated at the setpoint
        parder_q0_q3dot_sp = 0.5 * r_sp
        parder_q1_q3dot_sp = 0.5 * q_sp
        parder_q2_q3dot_sp = - 0.5 * p_sp
        parder_p_q3dot_sp  = - 0.5 * q2_sp
        parder_q_q3dot_sp  = 0.5 * q1_sp
        parder_r_q3dot_sp  = 0.5 * q0_sp
        #Partial derivatives of udot respect to state variables evaluated at the setpoint
        parder_pd_udot_sp     = (1 / mass) * (- parder_pd_qbar_sp * SW_SI * (CD1_sp + CD2_sp + CD3_sp + CD4_sp) + parder_pd_T_sp)
        parder_q0_udot_sp     = - 2 * G0_SI * q2_sp
        parder_q1_udot_sp     = 2 * G0_SI * q3_sp
        parder_q2_udot_sp     = - 2 * G0_SI * q0_sp
        parder_q3_udot_sp     = 2 * G0_SI * q1_sp
        parder_u_udot_sp      = (1 / mass) * (- SW_SI * (parder_u_qbar_sp * (CD1_sp + CD2_sp + CD3_sp + CD4_sp) + qbar_sp * (parder_u_CD3_sp + parder_u_CD4_sp)) + parder_u_T_sp)
        parder_v_udot_sp      = r_sp - (SW_SI / mass) * (parder_v_qbar_sp * (CD1_sp + CD2_sp + CD3_sp + CD4_sp) + qbar_sp * parder_v_CD4_sp)
        parder_w_udot_sp      = - q_sp - (SW_SI / mass) * (parder_w_qbar_sp * (CD1_sp + CD2_sp + CD3_sp + CD4_sp) + qbar_sp * (parder_w_CD3_sp + parder_w_CD4_sp))
        parder_q_udot_sp      = - w_sp
        parder_r_udot_sp      = v_sp
        parder_deltat_udot_sp = parder_deltat_T_sp / mass
        #Partial derivatives of vdot respect to state variables evaluated at the setpoint
        parder_pd_vdot_sp     = (1 / mass) * (parder_pd_qbar_sp * SW_SI * (CC1_sp + CC2_sp))
        parder_q0_vdot_sp     = 2 * G0_SI * q1_sp
        parder_q1_vdot_sp     = 2 * G0_SI * q0_sp
        parder_q2_vdot_sp     = 2 * G0_SI * q3_sp
        parder_q3_vdot_sp     = 2 * G0_SI * q2_sp
        parder_u_vdot_sp      = - r_sp + (SW_SI / mass) * (parder_u_qbar_sp * (CC1_sp + CC2_sp) + qbar_sp * parder_u_CC1_sp)
        parder_v_vdot_sp      = (SW_SI / mass) * (parder_v_qbar_sp * (CC1_sp + CC2_sp) + qbar_sp * parder_v_CC1_sp)
        parder_w_vdot_sp      = p_sp + (SW_SI / mass) * (parder_w_qbar_sp * (CC1_sp + CC2_sp) + qbar_sp * parder_w_CC1_sp)
        parder_p_vdot_sp      = w_sp
        parder_r_vdot_sp      = - u_sp
        parder_deltar_vdot_sp = ((qbar_sp * SW_SI) / mass) * parder_deltar_CC2_sp
        #Partial derivatives of wdot respect to state variables evaluated at the setpoint
        parder_pd_wdot_sp     = - (SW_SI / mass) * (parder_pd_qbar_sp * (CL1_sp + CL2_sp + CL3_sp + CL4_sp) + parder_pd_qbaruw_sp * CL5_sp)
        parder_q0_wdot_sp     = 2 * G0_SI * q0_sp
        parder_q1_wdot_sp     = - 2 * G0_SI * q1_sp
        parder_q2_wdot_sp     = - 2 * G0_SI * q2_sp
        parder_q3_wdot_sp     = 2 * G0_SI * q3_sp
        parder_u_wdot_sp      = q_sp - (SW_SI / mass) * (parder_u_qbar_sp * (CL1_sp + CL2_sp + CL3_sp + CL4_sp) + qbar_sp * (parder_u_CL1_sp + parder_u_CL4_sp) + parder_u_qbaruw_sp * CL5_sp + qbaruw_sp * parder_u_CL5_sp)
        parder_v_wdot_sp      = - p_sp - (SW_SI / mass) * (parder_v_qbar_sp * (CL1_sp + CL2_sp + CL3_sp + CL4_sp) + qbar_sp * parder_v_CL4_sp + qbaruw_sp * CL5_sp)
        parder_w_wdot_sp      = - (SW_SI / mass) * (parder_w_qbar_sp * (CL1_sp + CL2_sp + CL3_sp + CL4_sp) + qbar_sp * (parder_w_CL1_sp + parder_w_CL4_sp) + parder_w_qbaruw_sp * CL5_sp + qbaruw_sp * parder_w_CL5_sp)
        parder_p_wdot_sp      = - v_sp
        parder_q_wdot_sp      = u_sp - ((qbar_sp * SW_SI) / mass) * parder_q_CL4_sp
        parder_deltae_wdot_sp = - ((qbar_sp * SW_SI) / mass) * parder_deltae_CL3_sp
        #Partial derivatives of pdot respect to state variables evaluated at the setpoint
        parder_pd_pdot_sp     = SW_SI * BW_SI * (parder_pd_qbar_sp * (Gamma3 * (Cl1_sp + Cl2_sp + Cl3_sp + Cl4_sp + Cl5_sp) + Gamma4 * (Cn1_sp + Cn2_sp + Cn3_sp + Cn4_sp)) + Gamma4 * (parder_pd_qbarind_sp * Cn5_sp + parder_pd_qbarprop_sp * Cn6_sp))
        parder_u_pdot_sp      = SW_SI * BW_SI * (parder_u_qbar_sp * (Gamma3 * (Cl1_sp + Cl2_sp + Cl3_sp + Cl4_sp + Cl5_sp) + Gamma4 * (Cn1_sp + Cn2_sp + Cn3_sp + Cn4_sp)) + qbar_sp * (Gamma3 * (parder_u_Cl1_sp + parder_u_Cl2_sp + parder_u_Cl3_sp + parder_u_Cl4_sp) + Gamma4 * (parder_u_Cn1_sp + parder_u_Cn2_sp + parder_u_Cn3_sp + parder_u_Cn4_sp)) + Gamma4 * (parder_u_qbarind_sp * Cn5_sp + parder_u_qbarprop_sp * Cn6_sp))
        parder_v_pdot_sp      = SW_SI * BW_SI * (parder_v_qbar_sp * (Gamma3 * (Cl1_sp + Cl2_sp + Cl3_sp + Cl4_sp + Cl5_sp) + Gamma4 * (Cn1_sp + Cn2_sp + Cn3_sp + Cn4_sp)) + qbar_sp * (Gamma3 * (parder_v_Cl1_sp + parder_v_Cl2_sp + parder_v_Cl3_sp) + Gamma4 * (parder_v_Cn1_sp + parder_v_Cn2_sp + parder_v_Cn3_sp + parder_v_Cn4_sp)))
        parder_w_pdot_sp      = SW_SI * BW_SI * (parder_w_qbar_sp * (Gamma3 * (Cl1_sp + Cl2_sp + Cl3_sp + Cl4_sp + Cl5_sp) + Gamma4 * (Cn1_sp + Cn2_sp + Cn3_sp + Cn4_sp)) + qbar_sp * (Gamma3 * (parder_w_Cl1_sp + parder_w_Cl2_sp + parder_w_Cl3_sp + parder_w_Cl4_sp) + Gamma4 * (parder_w_Cn1_sp + parder_w_Cn2_sp + parder_w_Cn3_sp + parder_w_Cn4_sp)))
        parder_p_pdot_sp      = Gamma1 * q_sp + SW_SI * BW_SI * qbar_sp * Gamma3 * parder_p_Cl2_sp
        parder_q_pdot_sp      = Gamma1 * p_sp - Gamma2 * r_sp
        parder_r_pdot_sp      = - Gamma2 * q_sp + SW_SI * BW_SI * qbar_sp * (Gamma3 * parder_r_Cl3_sp + Gamma4 * (parder_r_Cn2_sp + parder_r_Cn3_sp))
        parder_deltaa_pdot_sp = SW_SI * BW_SI * qbar_sp * (Gamma3 * parder_deltaa_Cl4_sp + Gamma4 * parder_deltaa_Cn4_sp)
        parder_deltar_pdot_sp = SW_SI * BW_SI * (Gamma3 * qbar_sp * parder_deltar_Cl5_sp + Gamma4 * qbarind_sp * parder_deltar_Cn5_sp)
        #Partial derivatives of qdot respect to state variables evaluated at the setpoint
        parder_pd_qdot_sp     = ((SW_SI * CW_SI) / Iyy) * (parder_pd_qbar_sp * (Cm1_sp + Cm2_sp + Cm3_sp + Cm4_sp) + qbar_sp * parder_pd_Cm1_sp + parder_pd_qbaruw_sp * Cm5_sp + parder_pd_qbarind_sp * Cm6_sp)
        parder_u_qdot_sp      = ((SW_SI * CW_SI) / Iyy) * (parder_u_qbar_sp * (Cm1_sp + Cm2_sp + Cm3_sp + Cm4_sp) + qbar_sp * (parder_u_Cm2_sp + parder_u_Cm3_sp) + parder_u_qbaruw_sp * Cm5_sp + qbaruw_sp * parder_u_Cm5_sp + parder_u_qbarind_sp * Cm6_sp + qbarind_sp * parder_u_Cm6_sp)
        parder_v_qdot_sp      = ((SW_SI * CW_SI) / Iyy) * (parder_v_qbar_sp * (Cm1_sp + Cm2_sp + Cm3_sp + Cm4_sp) + qbar_sp * parder_v_Cm3_sp + qbaruw_sp * parder_v_Cm5_sp)
        parder_w_qdot_sp      = ((SW_SI * CW_SI) / Iyy) * (parder_w_qbar_sp * (Cm1_sp + Cm2_sp + Cm3_sp + Cm4_sp) + qbar_sp * (parder_w_Cm2_sp + parder_w_Cm3_sp) + parder_w_qbaruw_sp * Cm5_sp + qbaruw_sp * parder_w_Cm5_sp + qbarind_sp * parder_w_Cm6_sp)
        parder_p_qdot_sp      = Gamma5 * r_sp - 2 * Gamma6 * p_sp
        parder_q_qdot_sp      = ((SW_SI * CW_SI * qbar_sp) / Iyy) * parder_q_Cm3_sp
        parder_r_qdot_sp      = Gamma5 * p_sp + 2 * Gamma6 * r_sp
        parder_deltae_qdot_sp = ((SW_SI * CW_SI * qbarind_sp) / Iyy) * parder_deltae_Cm6_sp
        #Partial derivatives of rdot respect to state variables evaluated at the setpoint
        parder_pd_rdot_sp     = SW_SI * BW_SI * (parder_pd_qbar_sp * (Gamma4 * (Cl1_sp + Cl2_sp + Cl3_sp + Cl4_sp + Cl5_sp) + Gamma8 * (Cn1_sp + Cn2_sp + Cn3_sp + Cn4_sp)) + Gamma8 * (parder_pd_qbarind_sp * Cn5_sp + parder_pd_qbarprop_sp * Cn6_sp))
        parder_u_rdot_sp      = SW_SI * BW_SI * (parder_u_qbar_sp * (Gamma4 * (Cl1_sp + Cl2_sp + Cl3_sp + Cl4_sp + Cl5_sp) + Gamma8 * (Cn1_sp + Cn2_sp + Cn3_sp + Cn4_sp)) + qbar_sp * (Gamma4 * (parder_u_Cl1_sp + parder_u_Cl2_sp + parder_u_Cl3_sp + parder_u_Cl4_sp) + Gamma8 * (parder_u_Cn1_sp + parder_u_Cn2_sp + parder_u_Cn3_sp + parder_u_Cn4_sp)) + Gamma8 * (parder_u_qbarind_sp * Cn5_sp + parder_u_qbarprop_sp * Cn6_sp))
        parder_v_rdot_sp      = SW_SI * BW_SI * (parder_v_qbar_sp * (Gamma4 * (Cl1_sp + Cl2_sp + Cl3_sp + Cl4_sp + Cl5_sp) + Gamma8 * (Cn1_sp + Cn2_sp + Cn3_sp + Cn4_sp)) + qbar_sp * (Gamma4 * (parder_v_Cl1_sp + parder_v_Cl2_sp + parder_v_Cl3_sp) + Gamma8 * (parder_v_Cn1_sp + parder_v_Cn2_sp + parder_v_Cn3_sp + parder_v_Cn4_sp)))
        parder_w_rdot_sp      = SW_SI * BW_SI * (parder_w_qbar_sp * (Gamma4 * (Cl1_sp + Cl2_sp + Cl3_sp + Cl4_sp + Cl5_sp) + Gamma8 * (Cn1_sp + Cn2_sp + Cn3_sp + Cn4_sp)) + qbar_sp * (Gamma4 * (parder_w_Cl1_sp + parder_w_Cl2_sp + parder_w_Cl3_sp + parder_w_Cl4_sp) + Gamma8 * (parder_w_Cn1_sp + parder_w_Cn2_sp + parder_w_Cn3_sp + parder_w_Cn4_sp)))
        parder_p_rdot_sp      = Gamma7 * q_sp + SW_SI * BW_SI * qbar_sp * Gamma4 * parder_p_Cl2_sp
        parder_q_rdot_sp      = Gamma7 * p_sp - Gamma1 * r_sp
        parder_r_rdot_sp      = - Gamma1 * q_sp + SW_SI * BW_SI * qbar_sp * (Gamma4 * parder_r_Cl3_sp + Gamma8 * (parder_r_Cn2_sp + parder_r_Cn3_sp))
        parder_deltaa_rdot_sp = SW_SI * BW_SI * qbar_sp * (Gamma4 * parder_deltaa_Cl4_sp + Gamma8 * parder_deltaa_Cn4_sp)
        parder_deltar_rdot_sp = SW_SI * BW_SI * (Gamma4 * qbar_sp * parder_deltar_Cl5_sp + Gamma8 * qbarind_sp * parder_deltar_Cn5_sp)
        #Partial derivatives of state respect to state (A) evaluated at the setpoint
        parder_x_sp = np.array(
                               [ 
                                parder_q0_pndot_sp,
                                parder_q1_pndot_sp,
                                parder_q2_pndot_sp,
                                parder_q3_pndot_sp,
                                parder_u_pndot_sp,
                                parder_v_pndot_sp,
                                parder_w_pndot_sp,
                                parder_q0_pedot_sp,
                                parder_q1_pedot_sp,
                                parder_q2_pedot_sp,
                                parder_q3_pedot_sp,
                                parder_u_pedot_sp,
                                parder_v_pedot_sp,
                                parder_w_pedot_sp,
                                parder_q0_pddot_sp,
                                parder_q1_pddot_sp,
                                parder_q2_pddot_sp,
                                parder_q3_pddot_sp,
                                parder_u_pddot_sp,
                                parder_v_pddot_sp,
                                parder_w_pddot_sp,
                                parder_q1_q0dot_sp,
                                parder_q2_q0dot_sp,
                                parder_q3_q0dot_sp,
                                parder_p_q0dot_sp,
                                parder_q_q0dot_sp,
                                parder_r_q0dot_sp,
                                parder_q0_q1dot_sp,
                                parder_q2_q1dot_sp,
                                parder_q3_q1dot_sp,
                                parder_p_q1dot_sp,
                                parder_q_q1dot_sp,
                                parder_r_q1dot_sp,
                                parder_q0_q2dot_sp,
                                parder_q1_q2dot_sp,
                                parder_q3_q2dot_sp,
                                parder_p_q2dot_sp,
                                parder_q_q2dot_sp,
                                parder_r_q2dot_sp,
                                parder_q0_q3dot_sp,
                                parder_q1_q3dot_sp,
                                parder_q2_q3dot_sp,
                                parder_p_q3dot_sp,
                                parder_q_q3dot_sp,
                                parder_r_q3dot_sp,
                                parder_pd_udot_sp,
                                parder_q0_udot_sp,
                                parder_q1_udot_sp,
                                parder_q2_udot_sp,
                                parder_q3_udot_sp,
                                parder_u_udot_sp,
                                parder_v_udot_sp,
                                parder_w_udot_sp,
                                parder_q_udot_sp,
                                parder_r_udot_sp,
                                parder_pd_vdot_sp,
                                parder_q0_vdot_sp,
                                parder_q1_vdot_sp,
                                parder_q2_vdot_sp,
                                parder_q3_vdot_sp,
                                parder_u_vdot_sp,
                                parder_v_vdot_sp,
                                parder_w_vdot_sp,
                                parder_p_vdot_sp,
                                parder_r_vdot_sp,
                                parder_pd_wdot_sp,
                                parder_q0_wdot_sp,
                                parder_q1_wdot_sp,
                                parder_q2_wdot_sp,
                                parder_q3_wdot_sp,
                                parder_u_wdot_sp,
                                parder_v_wdot_sp,
                                parder_w_wdot_sp,
                                parder_p_wdot_sp,
                                parder_q_wdot_sp,
                                parder_pd_pdot_sp,
                                parder_u_pdot_sp,
                                parder_v_pdot_sp,
                                parder_w_pdot_sp,
                                parder_p_pdot_sp,
                                parder_q_pdot_sp,
                                parder_r_pdot_sp,
                                parder_pd_qdot_sp,
                                parder_u_qdot_sp,
                                parder_v_qdot_sp,
                                parder_w_qdot_sp,
                                parder_p_qdot_sp,
                                parder_q_qdot_sp,
                                parder_r_qdot_sp,
                                parder_pd_rdot_sp,
                                parder_u_rdot_sp,
                                parder_v_rdot_sp,
                                parder_w_rdot_sp,
                                parder_p_rdot_sp,
                                parder_q_rdot_sp,
                                parder_r_rdot_sp
                               ]
                               , dtype=float
                              )
        #Partial derivatives of state respect to input (B) evaluated at the setpoint
        parder_delta_sp = np.array(
                                   [
                                    parder_deltat_udot_sp,
                                    parder_deltar_vdot_sp,
                                    parder_deltae_wdot_sp,
                                    parder_deltaa_pdot_sp,
                                    parder_deltar_pdot_sp,
                                    parder_deltae_qdot_sp,
                                    parder_deltaa_rdot_sp,
                                    parder_deltar_rdot_sp
                                   ]
                                   , dtype=float
                                  )
        return parder_x_sp, parder_delta_sp
