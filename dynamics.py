import multiprocessing as mp
import numpy as np

from c172p_model import *
from settings import *

DYN_LEN = 10 #[Fax, Fgx, Ft, Fay, Fgy, Faz, Fgz, Max, May, Maz]

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

                CD1 = CD1()
                CD2 = CD2(h_b_mac_ft, flaps_pos_deg)
                CD3 = CD3(h_b_mac_ft, alpha_rad, flaps_pos_deg)
                CD4 = CD4(beta_rad)

                CC1 = CC1(beta_rad, flaps_pos_deg)
                CC2 = CC2(rudder_pos_rad)

                CL1 = CL1(h_b_mac_ft, alpha_rad, stall_hyst_norm)
                CL2 = CL2(h_b_mac_ft, flaps_pos_deg)
                CL3 = CL3(elev_pos_rad)
                CL4 = CL4(q_rad_sec, ci2vel)
                CL5 = CL5(alphadot_rad_sec, ci2vel)

                Cl1 = Cl1(beta_rad, alpha_rad)
                Cl2 = Cl2(bi2vel, p_rad_sec) 
                Cl3 = Cl3(bi2vel, r_rad_sec, flaps_pos_deg, alpha_rad, stall_hyst_norm)
                Cl4 = Cl4(left_aileron_pos_rad, right_aileron_pos_rad, alpha_rad, stall_hyst_norm)
                Cl5 = Cl5(rudder_pos_rad)

                Cm1 = Cm1(qbar_psf)
                Cm2 = Cm2(alpha_deg, alpha_rad)
                Cm3 = Cm3(ci2vel, q_rad_sec)
                Cm4 = Cm4(flaps_pos_deg)
                Cm5 = Cm5(ci2vel, alphadot_rad_sec)
                Cm6 = Cm6(elev_pos_rad, alpha_deg)

                Cn1 = Cn1(beta_rad)
                Cn2 = Cn2(bi2vel, r_rad_sec)
                Cn3 = Cn3(bi2vel, r_rad_sec, alpha_rad)
                Cn4 = Cn4(left_aileron_pos_rad, right_aileron_pos_rad, alpha_rad, beta_rad)
                Cn5 = Cn5(rudder_pos_rad)
                Cn6 = Cn6()

                D = qbar_psf * SW_SQFT * (CFx1 + CFx2 + CFx3 + CFx4) 
                C = qbar_psf * SW_SQFT * (CFy1 + CFy2)
                L = SW_SQFT * (qbar_psf * (CFz1 + CFz2 + CFz3 + CFz4) + qbarUW_psf * CFz5)
                l = qbar_psf * SW_SQFT * BW_FT * (CMx1 + CMx2 + CMx3 + CMx4 + CMx5)
                m = SW_SQFT * CBARW_FT * (qbar_psf * (CMy1 + CMy2 + CMy3 + CMy4) + qbarUW_psf * CMy5 + qbar_induced_psf * CMy6)
                n = SW_SQFT * BW_FT * (qbar_psf * (CMz1 + CMz2 + CMz3 + CMz4) + qbar_induced_psf * CMz5 + qbar_propwash_psf * CMz6)
                gx, gy, gz = force_gravity(quat, mass, gravity)
                T = thrust_eng(advance_ratio, density, rpm_prop)

                self.csvdyn[i,:] = [time, D, gx, T, C, gy, L, gz, l, m, n]

            dyn2csv_in.send(self.csvdyn[:framescount,:]) #send calculated dynamics to store in CSV
            self.csvdyn = np.empty((MODEL_HZ, DYN_LEN + 1)) #empty array 
