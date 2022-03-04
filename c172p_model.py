from constants import *
from settings import *
from modules.func.utils import *

import control
import numpy as np
from scipy import interpolate

''' FLIGHTGEAR MODEL PARAMETERS '''
BW         = 10.91184  #wing span [m]
C_NSPROP   = 0.25      #yaw moment coefficient due to propeller's spiral
CW         = 1.49352   #wing chord [m]
D_PROP     = 1.905     #propeller diameter [m]
DELTAM_MIN = 0.1       #minimum delta_m to maintain the engine running [-]
I_XX_PROP  = 2.264216  #propeller moment of inertia along longitudinal axis [kg·m^2]
N_MAX      = 45        #maximum propeller revolutions rate [RPS]
N_MIN      = 10        #minimum propeller revolutions rate [RPS]
SW         = 16.165129 #wing area [m^2]
A_PROP     = 2.850230  #propeller area [m^2]

''' AERODYNAMIC ACTUATORS DEFLECTION '''
#DELTA = (Minimum normalized deflection [-], Maximum normalized deflection [-])
DELTAA_RANGE = (-1, 1) #negative: left aileron up / left roll, positive: right aileron up / right roll
DELTAE_RANGE = (-1, 1) #negative: elevator up / up pitch, positive: elevator down / down pitch
DELTAF_RANGE = (0, 1) #negative: rudder left / left yaw, positive: rudder right / right yaw
DELTAR_RANGE = (-1, 1) #negative: left, positive: right
DELTAT_RANGE = (0, 1)
#SIGMA = (Minimum deflection [rad], Maximum deflection [rad])
SIGMAA_RANGE = (-0.349066, 0.261799)
SIGMAE_RANGE = (-0.488692, 0.401426)
SIGMAF_RANGE = (0        , 0.523599)
SIGMAR_RANGE = (-0.279253, 0.279253)

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
#Table corresponding to drag coefficient due to ground effect
#Column 0: hmacb [-]
TDge = np.array(   
                [
                 [0   , 0.48 ],
                 [0.1 , 0.515],
                 [0.15, 0.629],
                 [0.2 , 0.709],
                 [0.3 , 0.815],
                 [0.4 , 0.882],
                 [0.5 , 0.928],
                 [0.6 , 0.962],
                 [0.7 , 0.988],
                 [0.8 , 1    ],
                 [0.9 , 1    ],
                 [1   , 1    ],
                 [1.1 , 1    ]
                ], dtype=float
               )
TDge_interp = interp_table_1d(TDge)

#Table corresponding to aerodynamic coefficient CD2
#Column 0: sigmaf_rad [rad]
TD2 = np.array(
                  [
                   [0       , 0    ],
                   [0.174532, 0.007],
                   [0.349065, 0.012],
                   [0.523598, 0.018]
                  ], dtype=float
                 )
TD2_interp               = interp_table_1d(TD2)
parder_sigmaf_TD2_interp = parder_table_1d(TD2)

#Table corresponding to aerodynamic coefficient CD3
#Column 0: alpha_rad [rad] | Row 0: sigmaf_rad [rad]
TD3 = np.array(   
               [
                [np.NaN , 0       ,0.174533 , 0.349066, 0.523599],
                [-0.0873, 0.0041  , 0       , 0.0005  , 0.0014  ],
                [-0.0698, 0.0013  , 0.0004  , 0.0025  , 0.0041  ],
                [-0.0524, 0.0001  , 0.0023  , 0.0059  , 0.0084  ],
                [-0.0349, 0.0003  , 0.0057  , 0.0108  , 0.0141  ],
                [-0.0175, 0.002   , 0.0105  , 0.0172  , 0.0212  ],
                [0      , 0.0052  , 0.0168  , 0.0251  , 0.0299  ],
                [0.0175 , 0.0099  , 0.0248  , 0.0346  , 0.0402  ],
                [0.0349 , 0.0162  , 0.0342  , 0.0457  , 0.0521  ],
                [0.0524 , 0.024   , 0.0452  , 0.0583  , 0.0655  ],
                [0.0698 , 0.0334  , 0.0577  , 0.0724  , 0.0804  ],
                [0.0873 , 0.0442  , 0.0718  , 0.0881  , 0.0968  ],
                [0.1047 , 0.0566  , 0.0874  , 0.1053  , 0.1148  ],
                [0.1222 , 0.0706  , 0.1045  , 0.124   , 0.1343  ],
                [0.1396 , 0.086   , 0.1232  , 0.1442  , 0.1554  ],
                [0.1571 , 0.0962  , 0.1353  , 0.1573  , 0.169   ],
                [0.1745 , 0.1069  , 0.1479  , 0.1708  , 0.183   ],
                [0.192  , 0.118   , 0.161   , 0.1849  , 0.1975  ],
                [0.2094 , 0.1298  , 0.1746  , 0.1995  , 0.2126  ],
                [0.2269 , 0.1424  , 0.1892  , 0.2151  , 0.2286  ],
                [0.2443 , 0.1565  , 0.2054  , 0.2323  , 0.2464  ],
                [0.3491 , 0.2537  , 0.3298  , 0.3755  , 0.3983  ],
                [0.5236 , 0.45    , 0.585   , 0.666   , 0.7065  ],
                [0.6981 , 0.7     , 0.91    , 1.036   , 1.099   ],
                [0.8727 , 1       , 1.3     , 1.48    , 1.57    ],
                [1.0472 , 1.35    , 1.755   , 1.998   , 2.1195  ],
                [1.2217 , 1.5     , 1.95    , 2.22    , 2.355   ],
                [1.3963 , 1.57    , 2.041   , 2.3236  , 2.4649  ],
                [1.571  , 1.6     , 2.08    , 2.368   , 2.512   ]
               ], dtype=float
              )
TD3_interp                                        = interp_table_2d(TD3)
parder_alpha_TD3_interp, parder_sigmaf_TD3_interp = parder_table_2d(TD3)

#Table corresponding to aerodynamic coefficient CC1
#Column 0: beta_rad [rad] | Row 0: sigmaf_rad [rad]
TC1 = np.array(
               [
                [np.NaN , 0     , 0.523599],
                [-0.3490, 0.137 , 0.1060  ],
                [0      , 0     , 0       ],
                [0.3490 , -0.137, -0.1060 ]
               ], dtype=float
              )
TC1_interp                                       = interp_table_2d(TC1)
parder_beta_TC1_interp, parder_sigmaf_TC1_interp = parder_table_2d(TC1)

#Table corresponding to lift coefficient due to ground effect
#Column 0: hmacb [-]
TLge = np.array(   
                [
                 [0   , 1.203],
                 [0.1 , 1.127],
                 [0.15, 1.09 ],
                 [0.2 , 1.073],
                 [0.3 , 1.046],
                 [0.4 , 1.028],
                 [0.5 , 1.019],
                 [0.6 , 1.013],
                 [0.7 , 1.008],
                 [0.8 , 1.006],
                 [0.9 , 1.003],
                 [1   , 1.002],
                 [1.1 , 1    ]
                ], dtype=float
               )
TLge_interp = interp_table_1d(TLge)

#Table corresponding to aerodynamic coefficient CL1
#Column 0: alpha_rad [rad] | Row 0: stall [-]
TL1 = np.array(   
               [
                [np.NaN, 0    , 1    ],
                [-0.09 , -0.22, -0.22],
                [0     , 0.25 , 0.25 ],
                [0.09  , 0.73 , 0.73 ],
                [0.1   , 0.83 , 0.78 ],
                [0.12  , 0.92 , 0.79 ],
                [0.14  , 1.02 , 0.81 ],
                [0.16  , 1.08 , 0.82 ],
                [0.17  , 1.13 , 0.83 ],
                [0.19  , 1.19 , 0.85 ],
                [0.21  , 1.25 , 0.86 ],
                [0.24  , 1.35 , 0.88 ],
                [0.26  , 1.44 , 0.9  ],
                [0.28  , 1.47 , 0.92 ],
                [0.3   , 1.43 , 0.95 ],
                [0.32  , 1.38 , 0.99 ],
                [0.34  , 1.30 , 1.05 ],
                [0.36  , 1.15 , 1.15 ],
                [0.52  , 1.47 , 1.47 ],
                [0.7   , 1.65 , 1.65 ],
                [0.87  , 1.47 , 1.47 ],
                [1.05  , 1.17 , 1.17 ],
                [1.57  , 0.01 , 0.01 ]
               ], dtype=float
              )
TL1_interp                 = interp_table_2d(TL1)
parder_alpha_TL1_interp, _ = parder_table_2d(TL1)

#Table corresponding to aerodynamic coefficient CL2
#Column 0: sigmaf_rad [rad]
TL2 = np.array(
               [
                [0       , 0   ],
                [0.174533, 0.2 ],
                [0.349066, 0.3 ],
                [0.523599, 0.35]
               ], dtype=float
              )
TL2_interp               = interp_table_1d(TL2)
parder_sigmaf_TL2_interp = parder_table_1d(TL2)

#Table corresponding to aerodynamic coefficient Cl1
#Column 0: alpha_rad [rad]
Tl1 = np.array(
               [
                [0.279, 1  ],
                [0.297, 3.5]
               ], dtype=float
              )
Tl1_interp              = interp_table_1d(Tl1)
parder_alpha_Tl1_interp = parder_table_1d(Tl1)

#Table 1 corresponding to aerodynamic coefficient Cl3
#Column 0: sigmaf_rad [rad]
Tl31 = np.array(
                [
                 [0       , 0.0798],
                 [0.523599, 0.1246]
                ], dtype=float
               )
Tl31_interp               = interp_table_1d(Tl31)
parder_sigmaf_Tl31_interp = parder_table_1d(Tl31)

#Table 2 corresponding to aerodynamic coefficient Cl3
#Column 0: alpha_rad [rad] | Row 0: r [rad/s]
Tl32 = np.array(
                [
                 [np.NaN, -0.15, -0.1, 0, 0.1, 0.15 ],
                 [0.297 , 35   , 30  , 1, 30 , 35   ],
                 [0.500 , 5    , 5   , 1, 5  , 5    ]
                ], dtype=float
               )
Tl32_interp                                    = interp_table_2d(Tl32)
parder_alpha_Tl32_interp, parder_r_Tl32_interp = parder_table_2d(Tl32)

#Table 3 corresponding to aerodynamic coefficient Cl3
#Column 0: alpha_rad [rad] | Row 0: r [rad/s]
Tl33 = np.array(
                [
                 [np.NaN, -0.15, -0.1, 0, 0.1, 0.15],
                 [0.279 , 1    , 1   , 1, 1  , 1   ],
                 [0.297 , 35   , 30  , 1, 30 , 35  ],
                 [0.5   , 5    , 5   , 1, 5  , 5   ]
                ], dtype=float
               )
Tl33_interp                                    = interp_table_2d(Tl33)
parder_alpha_Tl33_interp, parder_r_Tl33_interp = parder_table_2d(Tl33)

#Table corresponding to aerodynamic coefficient Cl4
#Column 0: alpha_rad [rad] | Row 0: stall [-]
Tl4 = np.array(
               [
                [np.NaN, 0   , 1   ],
                [0.279 , 1   , 0.3 ],
                [0.297 , 0.3 , 0.3 ],
                [0.611 , -0.1, -0.1]
               ], dtype=float
              )
Tl4_interp                 = interp_table_2d(Tl4)
parder_alpha_Tl4_interp, _ = parder_table_2d(Tl4)

#Table corresponding to aerodynamic coefficient Cm1
#Column 0: qbar [Pa]
Tm1 = np.array(
               [
                [651.171524 , 0.09],
                [1015.061493, 0.04]
               ], dtype=float
              )
Tm1_interp             = interp_table_1d(Tm1)
parder_qbar_Tm1_interp = parder_table_1d(Tm1)

#Table corresponding to aerodynamic coefficient Cm2
#Column 0: alpha_rad [rad]
Tm2 = np.array(
               [
                [0.349066, 1  ],
                [0.436332, 0.6],
                [0.610865, 0.4],
                [0.785398, 0.5],
                [0.959931, 0.4],
                [1.134464, 0.2],
                [1.570796, 0.1]
               ], dtype=float
              )
Tm2_interp              = interp_table_1d(Tm2)
parder_alpha_Tm2_interp = parder_table_1d(Tm2)

#Table corresponding to aerodynamic coefficient Cm4
#Column 0: sigmaf_rad [rad]
Tm4 = np.array(
               [
                [0       , 0      ],
                [0.174533, -0.0654],
                [0.349066, -0.0981],
                [0.523599, -0.114 ]
               ], dtype=float
              )
Tm4_interp               = interp_table_1d(Tm4)
parder_sigmaf_Tm4_interp = parder_table_1d(Tm4)

#Table corresponding to aerodynamic coefficient Cm5
#Column 0: sigmae_rad [rad] | Row 0: alpha_rad [rad]
Tm5 = np.array(
               [
                [np.NaN, 0.314159, 0.436332, 0.610865, 0.785398, 0.959931, 1.134464, 1.570796],
                [-0.49 , 1       , 0.5     , 0.2     , 0.1     , 0.1     , 0.1     , 0.1     ],
                [0     , 1       , 0.6     , 0.3     , 0.15    , 0.1     , 0.1     , 0.1     ],
                [0.4   , 1       , 0.9     , 0.8     , 0.7     , 0.6     , 0.5     , 0.4     ]
               ], dtype=float
              )
Tm5_interp                                        = interp_table_2d(Tm5)
parder_sigmae_Tm5_interp, parder_alpha_Tm5_interp = parder_table_2d(Tm5)

#Table corresponding to aerodynamic coefficient Cn1
#Column 0: beta_rad [rad]
Tn1 = np.array(
               [
                [-0.3490, -0.0205],
                [0      , 0      ],
                [0.349  , 0.0205 ]
               ], dtype=float
              )
Tn1_interp             = interp_table_1d(Tn1)
parder_beta_Tn1_interp = parder_table_1d(Tn1)

#Table corresponding to aerodynamic coefficient Cn3
#Column 0: r [rad/s] | Row 0: alpha_rad [rad]
Tn3 = np.array(
               [
                [np.NaN, 0.279, 0.4  ],
                [-15   , 0    , 0    ],
                [-5    , 0    , 0    ],
                [-3    , 0    , -0.25],
                [-1    , 0    , 0    ],
                [0     , 0    , 0    ],
                [1     , 0    , 0    ],
                [3     , 0    , 0.25 ],
                [5     , 0    , 0    ],
                [15    , 0    , 0    ]
               ], dtype=float
              )
Tn3_interp                                   = interp_table_2d(Tn3)
parder_r_Tn3_interp, parder_alpha_Tn3_interp = parder_table_2d(Tn3)

#Table corresponding to aerodynamic coefficient Cn4
#Column 0: alpha_rad [rad] | Row 0: beta_rad [rad]
Tn4 = np.array(
               [
                [np.NaN, -0.35  , 0      , 0.35   ],
                [0     , -0.0216, -0.0216, -0.0216],
                [0.07  , -0.039 , -0.0786, -0.039 ],
                [0.094 , -0.025 , -0.0504, -0.025 ]
               ], dtype=float
              )
Tn4_interp                                      = interp_table_2d(Tn4)
parder_alpha_Tn4_interp, parder_beta_Tn4_interp = parder_table_2d(Tn4)

#Table of propulsive coefficient CT
#Column 0: J [-]
TT = np.array(
              [
               [0  , 0.068 ],
               [0.1, 0.068 ],
               [0.2, 0.067 ],
               [0.3, 0.066 ],
               [0.4, 0.064 ],
               [0.5, 0.062 ],
               [0.6, 0.059 ],
               [0.7, 0.054 ],
               [0.8, 0.043 ],
               [0.9, 0.031 ],
               [1  , 0.019 ],
               [1.1, 0.008 ],
               [1.2, -0.001],        
               [1.3, -0.008],        
               [1.4, -0.019],        
               [1.5, -0.029],
               [1.6, -0.040],
               [1.7, -0.050],
               [1.8, -0.057],
               [1.9, -0.061],
               [2  , -0.064],
               [2.1, -0.066],
               [2.2, -0.067],
               [2.3, -0.068],
               [5  , -0.068]
              ], dtype=float
             )
TT_interp          = interp_table_1d(TT)
parder_J_TT_interp = parder_table_1d(TT)

#Table of propulsive coefficient CP
#Column 0: J [-]
TP = np.array(
              [
               [0  , 0.058  ],
               [0.1, 0.062  ],
               [0.2, 0.06   ],
               [0.3, 0.058  ],
               [0.4, 0.052  ],
               [0.5, 0.0457 ],
               [0.6, 0.0436 ],
               [0.7, 0.042  ],
               [0.8, 0.0372 ],
               [0.9, 0.0299 ],
               [1  , 0.0202 ],
               [1.1, -0.0111],
               [1.2, -0.0075],        
               [1.3, -0.0111],        
               [1.4, -0.0202],        
               [1.5, -0.028 ],
               [1.6, -0.0346],
               [1.7, -0.0389],
               [1.8, -0.0421],
               [1.9, -0.0436],
               [2  , -0.0445],
               [2.1, -0.0445],
               [2.2, -0.0442],
               [2.3, -0.0431],
               [2.4, -0.0421],
               [5  , -0.0413]
              ], dtype=float
             )
TP_interp = interp_table_1d(TP)
parder_J_TP_interp = parder_table_1d(TP)

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

''' AERODYNAMIC DRAG FORCE COEFFICIENTS '''
def aerocoeff_CD1():
    #Aerodynamic drag coefficient 1 
    return 0.027

def aerocoeff_CD2(TDge_out, TD2_out):
    #Aerodynamic drag coefficient 2
    return TDge_out * TD2_out

def aerocoeff_CD3(TDge_out, TD3_out):
    #Aerodynamic drag coefficient 3
    return TDge_out * TD3_out

def aerocoeff_CD4(beta_rad):
    #Aerodynamic drag coefficient 4
    return 0.15 * abs(beta_rad) 

''' AERODYNAMIC CROSSWIND FORCE COEFFICIENTS '''
def aerocoeff_CC1(TC1_out):
    #Aerodynamic crosswind coefficient 1
    return TC1_out

def aerocoeff_CC2(sigmar_rad):
    #Aerodynamic crosswind coefficient 2
    return 0.15 * sigmar_rad 

''' AERODYNAMIC LIFT FORCE COEFFICIENTS '''
def aerocoeff_CL1(TLge_out, TL1_out):
    #Aerodynamic lift coefficient 1
    return TLge_out * TL1_out

def aerocoeff_CL2(TLge_out, TL2_out):
    #Aerodynamic lift coefficient 2
    return TLge_out * TL2_out

def aerocoeff_CL3(sigmae_rad):
    #Aerodynamic lift coefficient 3
    return 0.43 * sigmae_rad

def aerocoeff_CL4(Cw2Va, q):
    #Aerodynamic lift coefficient 4
    return 3.9 * Cw2Va * q

def aerocoeff_CL5(Cw2Va, alphadot_rads):
    #Aerodynamic lift coefficient 5
    return 1.7 * Cw2Va  * alphadot_rads

''' AERODYNAMIC ROLL MOMENT COEFFICIENTS '''
def aerocoeff_Cl1(beta_rad, Tl1_out):
    #Aerodynamic roll coefficient 1
    return - 0.092 * beta_rad * Tl1_out

def aerocoeff_Cl2(Bw2Va, p):
    #Aerodynamic roll coefficient 2
    return - 0.484 * Bw2Va * p 

def aerocoeff_Cl3(Bw2Va, r, Tl31_out, Tl32_out, Tl33_out, stall):
    #Aerodynamic roll coefficient 3
    if stall:
        return Bw2Va * r * Tl31_out * Tl32_out
    else:
        return Bw2Va * r * Tl31_out * Tl33_out

def aerocoeff_Cl4(sigmaa_avg, Tl4_out):
    #Aerodynamic roll coefficient 4
    return 0.229 * sigmaa_avg * Tl4_out

def aerocoeff_Cl5(sigmar_rad):
    #Aerodynamic roll coefficient 5
    return 0.0147 * sigmar_rad

''' AERODYNAMIC PITCH MOMENT COEFFICIENTS '''
def aerocoeff_Cm1(Tm1_out):
    #Aerodynamic pitch coefficient 1
    return Tm1_out

def aerocoeff_Cm2(alpha_rad, Tm2_out):
    #Aerodynamic pitch coefficient 2
    return - 1.8 * np.sin(alpha_rad) * Tm2_out

def aerocoeff_Cm3(Cw2Va, q):
    #Aerodynamic pitch coefficient 3
    return - 12.4 * Cw2Va * q

def aerocoeff_Cm4(Tm4_out):
    #Aerodynamic pitch coefficient 4
    return 0.7 * Tm4_out

def aerocoeff_Cm5(Cw2Va, alphadot_rads):
    #Aerodynamic pitch coefficient 5
    return - 7.27 * Cw2Va * alphadot_rads

def aerocoeff_Cm6(sigmae_rad, Tm5_out):
    #Aerodynamic pitch coefficient 6
    return - 1.28 * sigmae_rad * Tm5_out

''' AERODYNAMIC YAW MOMENT COEFFICIENTS '''
def aerocoeff_Cn1(Tn1_out):
    #Aerodynamic yaw coefficient 1
    return Tn1_out

def aerocoeff_Cn2(Bw2Va, r):
    #Aerodynamic yaw coefficient 2
    return - 0.0937 * Bw2Va * r

def aerocoeff_Cn3(Bw2Va, Tn3_out):
    #Aerodynamic yaw coefficient 3
    return Bw2Va * Tn3_out

def aerocoeff_Cn4(sigmaa_avg, Tn4_out):
    #Aerodynamic yaw coefficient 4
    return sigmaa_avg * Tn4_out

def aerocoeff_Cn5(sigmar_rad):
    #Aerodynamic yaw coefficient 5
    return - 0.0645 * sigmar_rad
def aerocoeff_Cn6():
    #Aerodynamic yaw coefficient 6
    return - 0.05 * C_NSPROP

''' PROPULSIVE FORCE '''
def thrust_eng(CT_out, rho, n):
    #Engine (160 HP) thrust
    return CT_out * rho * (n ** 2) * (D_PROP ** 4)

''' PROPULSIVE POWER ''' 
def power_eng(CP_out, rho, n):
    #Engine (160 HP) power
    return CP_out * rho * (n ** 3) * (D_PROP ** 5)

''' GRAVITATIONAL FORCE '''
def gravity_body(euler, mass, grav):
    #Gravitational force in body frame
    phi   = euler[0]
    theta = euler[1]
    return mass * grav * np.array([- np.sin(theta), np.cos(theta) * np.sin(phi), np.cos(theta) * np.cos(phi)])
    #return vehicle_to_body(euler[0], euler[1], euler[2]).rotate(mass * grav * np.array([0, 0, 1]))
