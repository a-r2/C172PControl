import numpy as np
import settings

from math import cos, sin, asin, atan2
from pyquaternion import Quaternion

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
    newtonsmeters = newtonsfeet * ft_to_m(1)
    return newtonsmeters

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

def psf_to_pa(poundspersquaredfoot):
    #Pounds per squared foot to Pascal conversion
    newtonspersquaredfoot = lbs_to_N(poundspersquaredfoot)
    pascals = newtonspersquaredfoot / (ft_to_m(1) ** 2)
    return pascals 

def rpm_to_rps(rpm):
    #Revolutions per minute to revolutions per second conversion
    return rpm / 60

def SI_conversion(rxdata):
    #Imperial units to SI units conversion

    rxdata[:,10]  = km_to_m(rxdata[:,10]) #Altitude (GNSS) in meters
    rxdata[:,12]  = km_to_m(rxdata[:,12]) #Barometric altitude (QFE) in meters
    rxdata[:,27]  = ft_to_m(rxdata[:,27]) #North velocity (NED) in meters per second
    rxdata[:,28]  = ft_to_m(rxdata[:,28]) #East velocity (NED) in meters per second
    rxdata[:,29]  = ft_to_m(rxdata[:,29]) #Down velocity (NED) in meters per second
    rxdata[:,30]  = ft_to_m(rxdata[:,30]) #Longitudinal velocity (body) in meters per second
    rxdata[:,31]  = ft_to_m(rxdata[:,31]) #Transversal velocity (body) in meters per second
    rxdata[:,32]  = ft_to_m(rxdata[:,32]) #Vertical velocity (body) in meters per second
    rxdata[:,33]  = ft_to_m(rxdata[:,33]) #Longitudinal aerodynamic velocity (body) in meters per second
    rxdata[:,34]  = ft_to_m(rxdata[:,34]) #Transversal aerodynamic velocity (body) in meters per second
    rxdata[:,35]  = ft_to_m(rxdata[:,35]) #Vertical aerodynamic velocity (body) in meters per second
    rxdata[:,36]  = ft_to_m(rxdata[:,36]) #North wind velocity (NED) in meters per second
    rxdata[:,37]  = ft_to_m(rxdata[:,37]) #East wind velocity (NED) in meters per second
    rxdata[:,38]  = ft_to_m(rxdata[:,38]) #Down wind velocity (NED) in meters per second
    rxdata[:,49]  = ft_to_m(rxdata[:,49]) #Longitudinal acceleration (body) in meters per squared second 
    rxdata[:,50]  = ft_to_m(rxdata[:,50]) #Transversal acceleration (body) in meters per squared second 
    rxdata[:,51]  = ft_to_m(rxdata[:,51]) #Vertical acceleration (body) in meters per squared second 
    rxdata[:,55]  = lbs_to_N(rxdata[:,55]) #Longitudinal aerodynamic force (body) in newtons
    rxdata[:,56]  = lbs_to_N(rxdata[:,56]) #Longitudinal external force (body) in newtons
    rxdata[:,57]  = lbs_to_N(rxdata[:,57]) #Longitudinal gear force (body) in newtons
    rxdata[:,58]  = lbs_to_N(rxdata[:,58]) #Longitudinal propeller force (body) in newtons
    rxdata[:,59]  = lbs_to_N(rxdata[:,59]) #Longitudinal total force (body) in newtons
    rxdata[:,60]  = lbs_to_N(rxdata[:,60]) #Transversal aerodynamic force (body) in newtons
    rxdata[:,61]  = lbs_to_N(rxdata[:,61]) #Transversal external force (body) in newtons
    rxdata[:,62]  = lbs_to_N(rxdata[:,62]) #Transversal gear force (body) in newtons
    rxdata[:,63]  = lbs_to_N(rxdata[:,63]) #Transversal propeller force (body) in newtons
    rxdata[:,64]  = lbs_to_N(rxdata[:,64]) #Transversal total force (body) in newtons
    rxdata[:,65]  = lbs_to_N(rxdata[:,65]) #Vertical aerodynamic force (body) in newtons
    rxdata[:,66]  = lbs_to_N(rxdata[:,66]) #Vertical external force (body) in newtons
    rxdata[:,67]  = lbs_to_N(rxdata[:,67]) #Vertical gear force (body) in newtons
    rxdata[:,68]  = lbs_to_N(rxdata[:,68]) #Vertical propeller force (body) in newtons
    rxdata[:,69]  = lbs_to_N(rxdata[:,69]) #Vertical total force (body) in newtons
    rxdata[:,70]  = lbsft_to_Nm(rxdata[:,70]) #Aerodynamic roll moment (body) in newtons-meters
    rxdata[:,71]  = lbsft_to_Nm(rxdata[:,71]) #External roll moment (body) in newtons-meters
    rxdata[:,72]  = lbsft_to_Nm(rxdata[:,72]) #Gear roll moment (body) in newtons-meters
    rxdata[:,73]  = lbsft_to_Nm(rxdata[:,73]) #Propeller roll moment (body) in newtons-meters
    rxdata[:,74]  = lbsft_to_Nm(rxdata[:,74]) #Total roll moment (body) in newtons-meters
    rxdata[:,75]  = lbsft_to_Nm(rxdata[:,75]) #Aerodynamic pitch moment (body) in newtons-meters
    rxdata[:,76]  = lbsft_to_Nm(rxdata[:,76]) #External pitch moment (body) in newtons-meters
    rxdata[:,77]  = lbsft_to_Nm(rxdata[:,77]) #Gear pitch moment (body) in newtons-meters
    rxdata[:,78]  = lbsft_to_Nm(rxdata[:,78]) #Propeller pitch moment (body) in newtons-meters
    rxdata[:,79]  = lbsft_to_Nm(rxdata[:,79]) #Total pitch moment (body) in newtons-meters
    rxdata[:,80]  = lbsft_to_Nm(rxdata[:,80]) #Aerodynamic yaw moment (body) in newtons-meters
    rxdata[:,81]  = lbsft_to_Nm(rxdata[:,81]) #External yaw moment (body) in newtons-meters
    rxdata[:,82]  = lbsft_to_Nm(rxdata[:,82]) #Gear yaw moment (body) in newtons-meters
    rxdata[:,83]  = lbsft_to_Nm(rxdata[:,83]) #Propeller yaw moment (body) in newtons-meters
    rxdata[:,84]  = lbsft_to_Nm(rxdata[:,84]) #Total yaw moment (body) in newtons-meters
    rxdata[:,109] = psf_to_pa(rxdata[:,109]) #Dynamic pressure in pascals
    rxdata[:,110] = psf_to_pa(rxdata[:,110]) #Dynamic pressure (XZ plane) in pascals
    rxdata[:,111] = psf_to_pa(rxdata[:,111]) #Dynamic pressure (propeller) in pascals
    rxdata[:,114] = slugft3_to_kgm3(rxdata[:,114]) #Density in kg per cubic metre

    TELEM_RX_KEYS = list(settings.TELEM_RX_PLOT.keys())
    
    for i in range(len(settings.TELEM_RX_PLOT)):
        unit = settings.TELEM_RX_PLOT[TELEM_RX_KEYS[i]][0][0] #current unit
        siunit  = settings.TELEM_RX_PLOT[TELEM_RX_KEYS[i]][0][1][0] #SI unit
        if unit == siunit:
            continue
        else:
            settings.TELEM_RX_PLOT[TELEM_RX_KEYS[i]][0][0] = unit.replace(unit, siunit) 

    return rxdata

def slug_to_kg(slugs):
    #Slugs to kilograms conversion
    return 14.593903 * slugs 

def slugft2_to_kgm2(slugssquaredfoot):
    #Slugs squared foot to kilograms squared metre conversion
    kilogramssquaredfoot = slug_to_kg(slugssquaredfoot)
    kilogramspercubicmetre = kilogramssquaredfoot * ft_to_m(1) ** 2
    return kilogramspercubicmetre  

def slugft3_to_kgm3(slugspercubicfoot):
    #Slugs per cubic feet to kilograms per cubic metre conversion
    kilogramspercubicfoot = slug_to_kg(slugspercubicfoot)
    kilogramspercubicmetre = kilogramspercubicfoot / (ft_to_m(1) ** 3)
    return kilogramspercubicmetre  

''' FRAME ROTATIONS '''
def body_to_stability(alpha):
    #Body frame to stability frame rotation
    
    q_r = Quaternion(axis=[0, 1, 0], radians=alpha)

    return q_r

def body_to_vehicle(phi, theta, psi):
    #Body frame to vehicle frame rotation
    
    q_r_X = body_to_vehicle2(phi)
    q_r_Y = vehicle2_to_vehicle1(theta)
    q_r_Z = vehicle1_to_vehicle(psi)
    q_r   = q_r_Z * q_r_Y * q_r_X

    return q_r

def body_to_vehicle2(phi):
    #Body frame to vehicle-2 frame rotation

    q_r = Quaternion(axis=[-1, 0, 0], radians=-phi)

    return q_r

def body_to_wind(alpha, beta):
    #Body frame to wind frame rotation

    q_r_Y = body_to_stability(alpha)
    q_r_Z = stability_to_wind(beta)
    q_r   = q_r_Z * q_r_Y

    return q_r

def stability_to_body(alpha):
    #Stability frame to body frame rotation
    
    q_r = Quaternion(axis=[0, 1, 0], radians=-alpha)

    return q_r

def stability_to_wind(beta):
    #Stability frame to wind frame rotation

    q_r = Quaternion(axis=[0, 0, 1], radians=-beta)

    return q_r

def vehicle_to_body(phi, theta, psi):
    #Vehicle frame to body frame rotation
    
    q_r_Z = vehicle_to_vehicle1(psi)
    q_r_Y = vehicle1_to_vehicle2(theta)
    q_r_X = vehicle2_to_body(phi)
    q_r   = q_r_X * q_r_Y * q_r_Z

    return q_r

def vehicle_to_vehicle1(psi):
    #Vehicle frame to vehicle-1 frame rotation

    q_r = Quaternion(axis=[0, 0, -1], radians=psi)

    return q_r

def vehicle1_to_vehicle(psi):
    #Vehicle-1 frame to vehicle frame rotation

    q_r = Quaternion(axis=[0, 0, -1], radians=-psi)

    return q_r

def vehicle1_to_vehicle2(theta):
    #Vehicle-1 frame to vehicle-2 frame rotation

    q_r = Quaternion(axis=[0, -1, 0], radians=theta)

    return q_r

def vehicle2_to_vehicle1(theta):
    #Vehicle-2 frame to vehicle-1 frame rotation

    q_r = Quaternion(axis=[0, -1, 0], radians=-theta)

    return q_r

def vehicle2_to_body(phi):
    #Vehicle-2 frame to body frame rotation

    q_r = Quaternion(axis=[-1, 0, 0], radians=phi)

    return q_r

def wind_to_body(alpha, beta):
    #Wind frame to body frame rotation

    q_r_Z = wind_to_stability(beta)
    q_r_Y = stability_to_body(alpha)
    q_r   = q_r_Y * q_r_Z

    return q_r

def wind_to_stability(beta):
    #Wind frame to stability frame rotation

    q_r = Quaternion(axis=[0, 0, 1], radians=beta)

    return q_r

''' OTHER '''
def averaged_ailerons(left_aileron_pos_rad, right_aileron_pos_rad):
    #Averaged position of ailerons

    return 0.5 * (left_aileron_pos_rad - right_aileron_pos_rad)

