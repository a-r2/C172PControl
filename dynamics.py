import multiprocessing as mp
import numpy as np

from c172p_model import *
from settings import *

DYN_LEN = 10 #[D, gx, T, C, gy, L, gz, l, m, n]

class Dynamics():
    def __init__(self, rx2dyn_out, dyn2csv_in, event_start):
        self.csvdyn = np.zeros((MODEL_HZ, DYN_LEN + 1)) #array for storing dynamics
        self.proc   = mp.Process(target=self.forces_moments_body, args=(rx2dyn_out, dyn2csv_in, event_start), daemon=True) #process for calculating dynamics
        self.proc.start()
    def forces_moments_body(self, rx2dyn_out, dyn2csv_in, event_start):
        #Aerodynamic and propulsive forces and moments applied to the aircraft in body frame
        event_start.wait() #wait for simulation start event
        while True:
            rxdata      = rx2dyn_out.recv() #receive RX telemetry
            framescount = rxdata.shape[0]
            for i in range(framescount):                
                time                  = rxdata[i,0]
                phi                   = rxdata[i,16]
                theta                 = rxdata[i,18]
                psi                   = rxdata[i,20]
                alpha_deg             = rxdata[i,21]
                alpha_rad             = rxdata[i,22]
                beta_rad              = rxdata[i,24]
                p_rad_sec             = rxdata[i,42]
                q_rad_sec             = rxdata[i,43]
                r_rad_sec             = rxdata[i,44]
                alphadot_rad_sec      = rxdata[i,46]
                right_aileron_pos_rad = rxdata[i,86]
                left_aileron_pos_rad  = rxdata[i,89]
                elev_pos_rad          = rxdata[i,92]
                flaps_pos_deg         = rxdata[i,94]
                rudder_pos_rad        = rxdata[i,98]
                bi2vel                = rxdata[i,106]
                ci2vel                = rxdata[i,107]
                h_b_mac_ft            = rxdata[i,108]
                qbar_psf              = rxdata[i,109]
                qbarUW_psf            = rxdata[i,110]
                qbar_propwash_psf     = rxdata[i,111]
                qbar_induced_psf      = rxdata[i,112]
                stall_hyst_norm       = rxdata[i,113]
                density               = rxdata[i,114]
                advance_ratio         = rxdata[i,115]
                rpm_prop              = rxdata[i,116]
                mass                  = rxdata[i,123]
                gravity               = rxdata[i,124]

                quat = euler_to_attquat(np.array([phi, theta, psi]))

                CD1 = aerocoeff_CD1()
                CD2 = aerocoeff_CD2(h_b_mac_ft, flaps_pos_deg)
                CD3 = aerocoeff_CD3(h_b_mac_ft, alpha_rad, flaps_pos_deg)
                CD4 = aerocoeff_CD4(beta_rad)

                CC1 = aerocoeff_CC1(beta_rad, flaps_pos_deg)
                CC2 = aerocoeff_CC2(rudder_pos_rad)

                CL1 = aerocoeff_CL1(h_b_mac_ft, alpha_rad, stall_hyst_norm)
                CL2 = aerocoeff_CL2(h_b_mac_ft, flaps_pos_deg)
                CL3 = aerocoeff_CL3(elev_pos_rad)
                CL4 = aerocoeff_CL4(q_rad_sec, ci2vel)
                CL5 = aerocoeff_CL5(alphadot_rad_sec, ci2vel)

                Cl1 = aerocoeff_Cl1(beta_rad, alpha_rad)
                Cl2 = aerocoeff_Cl2(bi2vel, p_rad_sec) 
                Cl3 = aerocoeff_Cl3(bi2vel, r_rad_sec, flaps_pos_deg, alpha_rad, stall_hyst_norm)
                Cl4 = aerocoeff_Cl4(left_aileron_pos_rad, right_aileron_pos_rad, alpha_rad, stall_hyst_norm)
                Cl5 = aerocoeff_Cl5(rudder_pos_rad)

                Cm1 = aerocoeff_Cm1(qbar_psf)
                Cm2 = aerocoeff_Cm2(alpha_deg, alpha_rad)
                Cm3 = aerocoeff_Cm3(ci2vel, q_rad_sec)
                Cm4 = aerocoeff_Cm4(flaps_pos_deg)
                Cm5 = aerocoeff_Cm5(ci2vel, alphadot_rad_sec)
                Cm6 = aerocoeff_Cm6(elev_pos_rad, alpha_deg)

                Cn1 = aerocoeff_Cn1(beta_rad)
                Cn2 = aerocoeff_Cn2(bi2vel, r_rad_sec)
                Cn3 = aerocoeff_Cn3(bi2vel, r_rad_sec, alpha_rad)
                Cn4 = aerocoeff_Cn4(left_aileron_pos_rad, right_aileron_pos_rad, alpha_rad, beta_rad)
                Cn5 = aerocoeff_Cn5(rudder_pos_rad)
                Cn6 = aerocoeff_Cn6()

                D = qbar_psf * SW_SI * (CD1 + CD2 + CD3 + CD4) 
                C = qbar_psf * SW_SI * (CC1 + CC2)
                L = SW_SI * (qbar_psf * (CL1 + CL2 + CL3 + CL4) + qbarUW_psf * CL5)
                l = qbar_psf * SW_SI * BW_SI * (Cl1 + Cl2 + Cl3 + Cl4 + Cl5)
                m = SW_SI * CW_SI * (qbar_psf * (Cm1 + Cm2 + Cm3 + Cm4) + qbarUW_psf * Cm5 + qbar_induced_psf * Cm6)
                n = SW_SI * BW_SI * (qbar_psf * (Cn1 + Cn2 + Cn3 + Cn4) + qbar_induced_psf * Cn5 + qbar_propwash_psf * Cn6)
                gx, gy, gz = gravity_body(quat, mass, gravity)
                T = thrust_eng(advance_ratio, density, rpm_prop)

                self.csvdyn[i,:] = [time, D, gx, T, C, gy, L, gz, l, m, n]

            dyn2csv_in.send(self.csvdyn[:framescount,:]) #send calculated dynamics to store in CSV
            self.csvdyn = np.empty((MODEL_HZ, DYN_LEN + 1)) #empty array 
