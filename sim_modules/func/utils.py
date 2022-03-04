import numpy as np
from math import asin, atan2, cos, exp, pi, sin, sqrt
from pyquaternion import Quaternion

from constants import *
import settings

''' CONVERSIONS '''
def angles_deg_conversion(rxdata):
    #Radians to degrees conversion

    rxdata[:,39] = rad_to_deg(rxdata[:,39]) #Rate of change of roll angle in degrees per second
    rxdata[:,40] = rad_to_deg(rxdata[:,40]) #Rate of change of pitch angle in degrees per second
    rxdata[:,41] = rad_to_deg(rxdata[:,41]) #Rate of change of yaw angle in degrees per second
    rxdata[:,42] = rad_to_deg(rxdata[:,42]) #Roll velocity (body) in degrees per second
    rxdata[:,43] = rad_to_deg(rxdata[:,43]) #Pitch velocity (body) in degrees per second
    rxdata[:,44] = rad_to_deg(rxdata[:,44]) #Yaw velocity (body) in degrees per second
    rxdata[:,52] = rad_to_deg(rxdata[:,52]) #Roll acceleration (body) in degrees per squared second 
    rxdata[:,53] = rad_to_deg(rxdata[:,53]) #Pitch acceleration (body) in degrees per squared second 
    rxdata[:,54] = rad_to_deg(rxdata[:,54]) #Yaw acceleration (body) in degrees per squared second 

    TELEM_RX_KEYS = list(settings.TELEM_RX_PLOT.keys())
    
    for i in range(len(settings.TELEM_RX_PLOT)):
        unit = settings.TELEM_RX_PLOT[TELEM_RX_KEYS[i]][0][0] #current unit
        if 'rad' in unit:
            settings.TELEM_RX_PLOT[TELEM_RX_KEYS[i]][0][0] = unit.replace('rad', 'º') 
        else:
            continue

    return rxdata

def attquat_to_euler(quat):
    #Attitude quaternion to Euler angles
    return np.array(quat.yaw_pitch_roll[::-1])

def deg_to_rad(degrees):
    #Degrees to radians conversion (unconstrained)
    return np.radians(degrees)

def deg_to_rad_2pi(degrees):
    #Degrees to radians conversion (constrained to [0, 2pi))
    return np.radians(degrees % 360)

def euler_to_attquat(euler):
    #Euler angles to attitude quaternion
    return vehicle_to_body(euler[0], euler[1], euler[2]) 

def ft_to_m(feet):
    #Feet to meters conversion
    return 0.3048 * feet

def imperial_conversion(rxdata):
    #SI units to imperical units conversion

    rxdata[:,2]  = m_to_ft(rxdata[:,2]) #Delta longitude from start in meters
    rxdata[:,3]  = m_to_ft(rxdata[:,3]) #Delta latitude from start in meters
    rxdata[:,4]  = m_to_ft(rxdata[:,4]) #Distance from start in meters
    rxdata[:,10] = km_to_ft(rxdata[:,10]) #Altitude (GNSS) in kilometers
    rxdata[:,12] = km_to_ft(rxdata[:,12]) #Barometric altitude (QFE) in kilometers
    rxdata[:,14] = m_to_ft(rxdata[:,14]) #Barometric altitude (QNH) in meters

    TELEM_RX_KEYS = list(settings.TELEM_RX_PLOT.keys())
    
    for i in range(len(settings.TELEM_RX_PLOT)):
        unit = settings.TELEM_RX_PLOT[TELEM_RX_KEYS[i]][0][0] #current unit
        impunit  = settings.TELEM_RX_PLOT[TELEM_RX_KEYS[i]][0][1][1] #imperial unit
        if unit == impunit:
            continue
        else:
            settings.TELEM_RX_PLOT[TELEM_RX_KEYS[i]][0][0] = unit.replace(unit, impunit) 

    return rxdata
def in_to_m(inches):
    #Inches to meters conversion
    return 0.0254 * inches

def in_to_ft(inches):
    #Inches to feet conversion
    return m_to_ft(in_to_m(inches))

def kg_to_slug(kilograms):
    #Kilograms to slugs conversion
    return kilograms / slug_to_kg(1) 

def kgm3_to_slugft3(kilogramspercubicmetre):
    #Kilograms per cubic metre to slugs per cubic feet conversion
    slugspercubicmetre = kg_to_slug(kilogramspercubicmetre)
    return kilogramspercubicmetre / (m_to_ft(1) ** 3)

def km_to_ft(kilometers):
    #Kilometers to feet conversion
    return m_to_ft(1000 * kilometers)

def km_to_m(kilometers):
    #Kilometers to meters conversion
    return 1000 * kilometers 

def lbs_to_N(pounds):
    #Pounds to Newtons conversions
    return 4.448221628250858 * pounds

def lbsft_to_Nm(poundsfeet):
    #Pounds-feet to Newtons-meters conversion
    newtonsfeet   = lbs_to_N(poundsfeet)
    return newtonsfeet * ft_to_m(1)

def m_to_ft(meters):
    #Meters to feet conversion
    return meters / ft_to_m(1)

def N_to_lbs(newtons):
    #Newtons to pounds conversions
    return 0.22480894244319 * newtons

def rad_to_deg(radians):
    #Radians to degrees conversion (unconstrained)
    return np.degrees(radians)

def rad_to_deg_360(radians):
    #Degrees to radians conversion (constrained to [0, 360))
    return np.degrees(radians % (2 * pi))

def pa_to_psf(pascal):
    #Pascal to pounds per squared foot conversion
    return pascal / psf_to_pa(1)

def propeller_area(propeller_diameter):
    #Propeller area from propeller diameter
    return pi * (0.5 * propeller_diameter) ** 2

def psf_to_pa(poundspersquaredfoot):
    #Pounds per squared foot to pascal conversion
    newtonspersquaredfoot = lbs_to_N(poundspersquaredfoot)
    return newtonspersquaredfoot / (ft_to_m(1) ** 2)

def rpm_to_rps(rpm):
    #Revolutions per minute to revolutions per second conversion
    return rpm / rps_to_rpm(1) 

def rps_to_rpm(rps):
    #Revolutions per second to revolutions per minute conversion
    return 60 * rps

def rxtelem_to_SI(framesarray):
    #RX telemetry frames array from IS units to SI units

    framesarray[:,5]   = ft_to_m(framesarray[:,5]) #North position [m]
    framesarray[:,6]   = ft_to_m(framesarray[:,6]) #East position [m]
    framesarray[:,7]   = ft_to_m(framesarray[:,7]) #Down position [m]
    framesarray[:,10]  = km_to_m(framesarray[:,10]) #Altitude (GNSS) [m]
    framesarray[:,11]  = km_to_m(framesarray[:,11]) #Barometric altitude AGL (QFE) [m]
    framesarray[:,13]  = ft_to_m(framesarray[:,13]) #Terrain elevation ASL [m]
    framesarray[:,20]  = ft_to_m(framesarray[:,20]) #North velocity (NED) [m/s]
    framesarray[:,21]  = ft_to_m(framesarray[:,21]) #East velocity (NED) [m/s]
    framesarray[:,22]  = ft_to_m(framesarray[:,22]) #Down velocity (NED) [m/s]
    framesarray[:,23]  = ft_to_m(framesarray[:,23]) #Longitudinal velocity (body) [m/s]
    framesarray[:,24]  = ft_to_m(framesarray[:,24]) #Transversal velocity (body) [m/s]
    framesarray[:,25]  = ft_to_m(framesarray[:,25]) #Vertical velocity (body) [m/s]
    framesarray[:,26]  = ft_to_m(framesarray[:,26]) #Aerodynamic longitudinal velocity (body) [m/s]
    framesarray[:,27]  = ft_to_m(framesarray[:,27]) #Aerodynamic transversal velocity (body) [m/s]
    framesarray[:,28]  = ft_to_m(framesarray[:,28]) #Aerodynamic vertical velocity (body) [m/s]
    framesarray[:,29]  = ft_to_m(framesarray[:,29]) #North wind velocity (NED) [m/s]
    framesarray[:,30]  = ft_to_m(framesarray[:,30]) #East wind velocity (NED) [m/s]
    framesarray[:,31]  = ft_to_m(framesarray[:,31]) #Down wind velocity (NED) [m/s]
    framesarray[:,43]  = ft_to_m(framesarray[:,43]) #Longitudinal acceleration (body) [m/s^2]
    framesarray[:,44]  = ft_to_m(framesarray[:,44]) #Transversal acceleration (body) [m/s^2]
    framesarray[:,45]  = ft_to_m(framesarray[:,45]) #Vertical acceleration (body) [m/s^2]
    framesarray[:,49]  = lbs_to_N(framesarray[:,49]) #Longitudinal aerodynamic force (body) [N]
    framesarray[:,50]  = lbs_to_N(framesarray[:,50]) #Longitudinal external force (body) [N]
    framesarray[:,51]  = lbs_to_N(framesarray[:,51]) #Longitudinal gear force (body) [N]
    framesarray[:,52]  = lbs_to_N(framesarray[:,52]) #Longitudinal propeller force (body) [N]
    framesarray[:,53]  = lbs_to_N(framesarray[:,53]) #Longitudinal total force (body) [N]
    framesarray[:,54]  = lbs_to_N(framesarray[:,54]) #Transversal aerodynamic force (body) [N]
    framesarray[:,55]  = lbs_to_N(framesarray[:,55]) #Transversal external force (body) [N]
    framesarray[:,56]  = lbs_to_N(framesarray[:,56]) #Transversal gear force (body) [N]
    framesarray[:,57]  = lbs_to_N(framesarray[:,57]) #Transversal propeller force (body) [N]
    framesarray[:,58]  = lbs_to_N(framesarray[:,58]) #Transversal total force (body) [N]
    framesarray[:,59]  = lbs_to_N(framesarray[:,59]) #Vertical aerodynamic force (body) [N]
    framesarray[:,60]  = lbs_to_N(framesarray[:,60]) #Vertical external force (body) [N]
    framesarray[:,61]  = lbs_to_N(framesarray[:,61]) #Vertical gear force (body) [N]
    framesarray[:,62]  = lbs_to_N(framesarray[:,62]) #Vertical propeller force (body) [N]
    framesarray[:,63]  = lbs_to_N(framesarray[:,63]) #Vertical total force (body) [N]
    framesarray[:,64]  = lbsft_to_Nm(framesarray[:,64]) #Aerodynamic roll moment (body) [N·m]
    framesarray[:,65]  = lbsft_to_Nm(framesarray[:,65]) #External roll moment (body) [N·m]
    framesarray[:,66]  = lbsft_to_Nm(framesarray[:,66]) #Gear roll moment (body) [N·m]
    framesarray[:,67]  = lbsft_to_Nm(framesarray[:,67]) #Propeller roll moment (body) [N·m]
    framesarray[:,68]  = lbsft_to_Nm(framesarray[:,68]) #Total roll moment (body) [N·m]
    framesarray[:,69]  = lbsft_to_Nm(framesarray[:,69]) #Aerodynamic pitch moment (body) [N·m]
    framesarray[:,70]  = lbsft_to_Nm(framesarray[:,70]) #External pitch moment (body) [N·m]
    framesarray[:,71]  = lbsft_to_Nm(framesarray[:,71]) #Gear pitch moment (body) [N·m]
    framesarray[:,72]  = lbsft_to_Nm(framesarray[:,72]) #Propeller pitch moment (body) [N·m]
    framesarray[:,73]  = lbsft_to_Nm(framesarray[:,73]) #Total pitch moment (body) [N·m]
    framesarray[:,74]  = lbsft_to_Nm(framesarray[:,74]) #Aerodynamic yaw moment (body) [N·m]
    framesarray[:,75]  = lbsft_to_Nm(framesarray[:,75]) #External yaw moment (body) [N·m]
    framesarray[:,76]  = lbsft_to_Nm(framesarray[:,76]) #Gear yaw moment (body) [N·m]
    framesarray[:,77]  = lbsft_to_Nm(framesarray[:,77]) #Propeller yaw moment (body) [N·m]
    framesarray[:,78]  = lbsft_to_Nm(framesarray[:,78]) #Total yaw moment (body) [N·m]
    framesarray[:,94]  = psf_to_pa(framesarray[:,94]) #Dynamic pressure [Pa]
    framesarray[:,95]  = psf_to_pa(framesarray[:,95]) #Dynamic pressure (UW plane of wind frame) [Pa]
    framesarray[:,96]  = psf_to_pa(framesarray[:,96]) #Dynamic pressure due to propeller induced velocity [Pa]
    framesarray[:,97]  = psf_to_pa(framesarray[:,97]) #Dynamic pressure including propulsion induced velocity [Pa]
    framesarray[:,99]  = slugft3_to_kgm3(framesarray[:,99]) #Air density [kg/m^3]
    framesarray[:,101] = rpm_to_rps(framesarray[:,101]) #Propeller revolutions [rev/s]
    framesarray[:,102] = slugft2_to_kgm2(framesarray[:,102]) #Moment of inertia Ixx [kg/m^2]
    framesarray[:,103] = slugft2_to_kgm2(framesarray[:,103]) #Moment of inertia Ixy [kg/m^2]
    framesarray[:,104] = slugft2_to_kgm2(framesarray[:,104]) #Moment of inertia Ixz [kg/m^2]
    framesarray[:,105] = slugft2_to_kgm2(framesarray[:,105]) #Moment of inertia Iyy [kg/m^2]
    framesarray[:,106] = slugft2_to_kgm2(framesarray[:,106]) #Moment of inertia Iyz [kg/m^2]
    framesarray[:,107] = slugft2_to_kgm2(framesarray[:,107]) #Moment of inertia Izz [kg/m^2]
    framesarray[:,108] = slug_to_kg(framesarray[:,108]) #Mass [kg]
    framesarray[:,109] = ft_to_m(framesarray[:,109]) #Gravitational acceleration [m/s^2]
    framesarray[:,117] = lbs_to_N(framesarray[:,117]) #Aerodynamic drag contribution 1 [N]
    framesarray[:,118] = lbs_to_N(framesarray[:,118]) #Aerodynamic drag contribution 2 [N]
    framesarray[:,119] = lbs_to_N(framesarray[:,119]) #Aerodynamic drag contribution 3 [N]
    framesarray[:,120] = lbs_to_N(framesarray[:,120]) #Aerodynamic drag contribution 4 [N]
    framesarray[:,121] = lbs_to_N(framesarray[:,121]) #Aerodynamic crosswind contribution 1 [N]
    framesarray[:,122] = lbs_to_N(framesarray[:,122]) #Aerodynamic crosswind contribution 2 [N]
    framesarray[:,123] = lbs_to_N(framesarray[:,123]) #Aerodynamic lift contribution 1 [N]
    framesarray[:,124] = lbs_to_N(framesarray[:,124]) #Aerodynamic lift contribution 2 [N]
    framesarray[:,125] = lbs_to_N(framesarray[:,125]) #Aerodynamic lift contribution 3 [N]
    framesarray[:,126] = lbs_to_N(framesarray[:,126]) #Aerodynamic lift contribution 4 [N]
    framesarray[:,127] = lbs_to_N(framesarray[:,127]) #Aerodynamic lift contribution 5 [N]
    framesarray[:,128] = lbsft_to_Nm(framesarray[:,128]) #Aerodynamic roll contribution 1 [N·m]
    framesarray[:,129] = lbsft_to_Nm(framesarray[:,129]) #Aerodynamic roll contribution 2 [N·m]
    framesarray[:,130] = lbsft_to_Nm(framesarray[:,130]) #Aerodynamic roll contribution 3 [N·m]
    framesarray[:,131] = lbsft_to_Nm(framesarray[:,131]) #Aerodynamic roll contribution 4 [N·m]
    framesarray[:,132] = lbsft_to_Nm(framesarray[:,132]) #Aerodynamic roll contribution 5 [N·m]
    framesarray[:,133] = lbsft_to_Nm(framesarray[:,133]) #Aerodynamic pitch contribution 1 [N·m]
    framesarray[:,134] = lbsft_to_Nm(framesarray[:,134]) #Aerodynamic pitch contribution 2 [N·m]
    framesarray[:,135] = lbsft_to_Nm(framesarray[:,135]) #Aerodynamic pitch contribution 3 [N·m]
    framesarray[:,136] = lbsft_to_Nm(framesarray[:,136]) #Aerodynamic pitch contribution 4 [N·m]
    framesarray[:,137] = lbsft_to_Nm(framesarray[:,137]) #Aerodynamic pitch contribution 5 [N·m]
    framesarray[:,138] = lbsft_to_Nm(framesarray[:,138]) #Aerodynamic pitch contribution 6 [N·m]
    framesarray[:,139] = lbsft_to_Nm(framesarray[:,139]) #Aerodynamic yaw contribution 1 [N·m]
    framesarray[:,140] = lbsft_to_Nm(framesarray[:,140]) #Aerodynamic yaw contribution 2 [N·m]
    framesarray[:,141] = lbsft_to_Nm(framesarray[:,141]) #Aerodynamic yaw contribution 3 [N·m]
    framesarray[:,142] = lbsft_to_Nm(framesarray[:,142]) #Aerodynamic yaw contribution 4 [N·m]
    framesarray[:,143] = lbsft_to_Nm(framesarray[:,143]) #Aerodynamic yaw contribution 5 [N·m]
    framesarray[:,144] = lbsft_to_Nm(framesarray[:,144]) #Aerodynamic yaw contribution 6 [N·m]
    framesarray[:,145] = ft_to_m(framesarray[:,145]) #Velocity including the propulsion induced velocity [m/s]
    framesarray[:,146] = ft_to_m(framesarray[:,146]) #Propeller induced velocity [m/s]

    return framesarray

def slug_to_kg(slugs):
    #Slugs to kilograms conversion
    return 14.593903 * slugs 

def slugft2_to_kgm2(slugssquaredfoot):
    #Slugs per squared foot to kilograms squared metre conversion
    kilogramssquaredfoot = slug_to_kg(slugssquaredfoot)
    return kilogramssquaredfoot * ft_to_m(1) ** 2

def slugft3_to_kgm3(slugspercubicfoot):
    #Slugs per cubic feet to kilograms per cubic metre conversion
    kilogramspercubicfoot = slug_to_kg(slugspercubicfoot)
    return kilogramspercubicfoot / (ft_to_m(1) ** 3)

''' FRAME ROTATIONS '''
def body_to_stability(alpha):
    #Body frame to stability frame rotation quaternion
    return Quaternion(axis=[0, 1, 0], radians=alpha)

def body_to_vehicle(phi, theta, psi):
    #Body frame to vehicle frame rotation quaternion
    q_r_X = body_to_vehicle2(phi)
    q_r_Y = vehicle2_to_vehicle1(theta)
    q_r_Z = vehicle1_to_vehicle(psi)
    return q_r_Z * q_r_Y * q_r_X

def body_to_vehicle2(phi):
    #Body frame to vehicle-2 frame rotation quaternion
    return Quaternion(axis=[-1, 0, 0], radians=-phi)

def body_to_wind(alpha, beta):
    #Body frame to wind frame rotation quaternion
    q_r_Y = body_to_stability(alpha)
    q_r_Z = stability_to_wind(beta)
    return q_r_Z * q_r_Y

def stability_to_body(alpha):
    #Stability frame to body frame rotation quaternion
    return Quaternion(axis=[0, 1, 0], radians=-alpha)

def stability_to_wind(beta):
    #Stability frame to wind frame rotation quaternion
    return Quaternion(axis=[0, 0, 1], radians=-beta)

def vehicle_to_body(phi, theta, psi):
    #Vehicle frame to body frame rotation quaternion
    q_r_Z = vehicle_to_vehicle1(psi)
    q_r_Y = vehicle1_to_vehicle2(theta)
    q_r_X = vehicle2_to_body(phi)
    return q_r_X * q_r_Y * q_r_Z

def vehicle_to_vehicle1(psi):
    #Vehicle frame to vehicle-1 frame rotation quaternion
    return Quaternion(axis=[0, 0, -1], radians=psi)

def vehicle1_to_vehicle(psi):
    #Vehicle-1 frame to vehicle frame rotation quaternion
    return Quaternion(axis=[0, 0, -1], radians=-psi)

def vehicle1_to_vehicle2(theta):
    #Vehicle-1 frame to vehicle-2 frame rotation quaternion
    return Quaternion(axis=[0, -1, 0], radians=theta)

def vehicle2_to_vehicle1(theta):
    #Vehicle-2 frame to vehicle-1 frame rotation quaternion
    return Quaternion(axis=[0, -1, 0], radians=-theta)

def vehicle2_to_body(phi):
    #Vehicle-2 frame to body frame rotation quaternion
    return Quaternion(axis=[-1, 0, 0], radians=phi)

def wind_to_body(alpha, beta):
    #Wind frame to body frame rotation quaternion
    q_r_Z = wind_to_stability(beta)
    q_r_Y = stability_to_body(alpha)
    return q_r_Y * q_r_Z

def wind_to_stability(beta):
    #Wind frame to stability frame rotation quaternion
    return Quaternion(axis=[0, 0, 1], radians=beta)

''' OTHER '''
def sigmaa_avg_acm(sigmala, sigmara):
    #Averaged ailerons position 
    return 0.5 * (sigmala - sigmara)

def deltaa_avg_acm(deltala, deltara):
    #Normalized aileron position from left and righ ailerons position
    return 0.5 * (deltala + deltara)

def deltaa_to_deg(deltaa):
    #Ailerons position in degrees from normalized ailerons
    return deltaa * (DELTAA_RANGE[0][1] - DELTAA_RANGE[0][0]) / (DELTAA_RANGE[1][1] - DELTAA_RANGE[1][0])

def deltaa_to_rad(deltaa):
    #Ailerons position in radians from normalized ailerons
    return deg_to_rad(deltaa_to_deg)

def deltae_to_deg(deltae):
    #Elevator position in degrees from normalized aileron
    return deltae * (DELTAE_RANGE[0][1] - DELTAE_RANGE[0][0]) / (DELTAE_RANGE[1][1] - DELTAE_RANGE[1][0])

def deltae_to_rad(deltae):
    #Elevators position in radians from normalized elevators
    return deg_to_rad(deltae_to_deg)

def deltaf_to_deg(deltaf):
    #Flaps position in degrees from normalized flaps
    return deltaf * (DELTAF_RANGE[0][1] - DELTAF_RANGE[0][0]) / (DELTAF_RANGE[1][1] - DELTAF_RANGE[1][0])

def deltaf_to_rad(deltaf):
    #Flaps position in radians from normalized flaps
    return deg_to_rad(deltaf_to_deg)

def deltar_to_deg(deltar):
    #Rudder position in degrees from normalized rudder
    return deltar * (DELTAR_RANGE[0][1] - DELTAR_RANGE[0][0]) / (DELTAR_RANGE[1][1] - DELTAR_RANGE[1][0])

def deltar_to_rad(deltar):
    #Rudder position in radians from normalized rudder
    return deg_to_rad(deltar_to_deg)

def rxdata_to_dict(rxdata):
    #Create dictionary from tuple
    outdict = dict()
    for i in range(TELEM_RX_LEN):
        outdict[TELEM_RX_STR[i]] = rxdata[i]
    return outdict

''' BAROMETRIC ATMOSPHERE '''
#https://en.wikipedia.org/wiki/Barometric_formula
Rg_SI = 8.3144598 #Universal gas constant [N·m/(mol·K)]
G0_SI = 9.80665 #Gravitational acceleration [m/s^2]
Me_SI = 0.0289644 #Molar mass of Earth's air [kg/mol]
#Barometric atmosphere table for calculating barometric density
#Column 0: height above sea level (altitude) [m]
#Colum 1: mass density [kg/m^3]
#Colum 2: standard temperature [K]
#Colum 3: temperature lapse rate [K/m]
BARO_ATM = np.array(   
                        [
                        [0.0000, 1.2250, 288.1500, -0.0065],
                        [11000.0000, 0.36391, 216.6500, 0.0000],
                        [20000.0000, 0.08803, 216.6500, 0.0010],
                        [32000.0000, 0.01322, 228.6500, 0.0028],
                        [47000.0000, 0.00143, 270.6500, 0.0000],
                        [51000.0000, 0.00086, 270.6500, -0.0028],
                        [71000.0000, 0.000064, 214.6500, -0.0020],
                        ]
                   )

def barometric_density(pd):
    #Barometric density as a function of pd
    try:
        layer_ind = np.where(BARO_ATM[:,0] <= (- pd))[0][-1]
    except:
        layer_ind = 0

    h_layer   = BARO_ATM[layer_ind,0] #altitude
    rho_layer = BARO_ATM[layer_ind,1] #mass density
    T_layer   = BARO_ATM[layer_ind,2] #standard temperature
    L_layer   = BARO_ATM[layer_ind,3] #temperature lapse rate

    if L_layer == 0:
        return rho_layer * exp((G0_SI * Me_SI * (pd + h_layer)) / (Rg_SI * T_layer))
    else:
        return rho_layer * (T_layer / (T_layer - L_layer * (pd + h_layer))) ** (1 + ((G0_SI * Me_SI) / (Rg_SI * L_layer)))

def parder_pd_barometric_density(pd):
    #Partial derivative of barometric density with respect to pd 
    try:
        layer_ind = np.where(BARO_ATM[:,0] <= (- pd))[0][-1]
    except:
        layer_ind = 0

    h_layer   = BARO_ATM[layer_ind,0] #altitude
    rho_layer = BARO_ATM[layer_ind,1] #mass density
    T_layer   = BARO_ATM[layer_ind,2] #standard temperature
    L_layer   = BARO_ATM[layer_ind,3] #temperature lapse rate

    if L_layer == 0:
        return ((rho_layer * G0_SI * Me_SI) / (Rg_SI * T_layer)) * exp((G0_SI * Me_SI * (pd + h_layer)) / (Rg_SI * T_layer))
    else:
        return (rho_layer * T_layer * L_layer * (1 + ((G0_SI * Me_SI) / (Rg_SI * L_layer))) * (T_layer / (T_layer - L_layer * (pd + h_layer))) ** ((G0_SI * Me_SI) / (Rg_SI * L_layer))) / ((T_layer - L_layer * (pd + h_layer)) ** 2)
