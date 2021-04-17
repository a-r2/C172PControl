import multiprocessing as mp
import numpy as np

from c172p_model import *
from settings import *

DYN_LEN = 6 #[D, S, L, l, m, n]

class Aerodynamics():
    def __init__(self, rx2dyn_out, dyn2csv_in, event_start):
        self.csvdyn = np.zeros((MODEL_HZ, DYN_LEN + 1)) #array for storing dynamics
        self.proc = mp.Process(target=self.forces_moments_wind, args=(rx2dyn_out, dyn2csv_in, event_start), daemon=True) #process for calculating actuation 
        self.proc.start()
    def forces_moments_wind(self, rx2dyn_out, dyn2csv_in, event_start):
        #Aerodynamic forces and moments applied to the aircraft in wind frame
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

                D1 = D0(qbar_psf)
                D2 = DDf(qbar_psf, h_b_mac_ft, flaps_pos_deg)
                D3 = Dwbh(qbar_psf, h_b_mac_ft, alpha_rad, flaps_pos_deg)
                D4 = DDe(qbar_psf, elev_pos_rad)
                D5 = Dbeta(qbar_psf, beta_rad)

                S1 = Yb(qbar_psf, beta_rad, flaps_pos_deg)
                S2 = Ydr(qbar_psf, rudder_pos_rad)

                L1 = Lwbh(qbar_psf, h_b_mac_ft, alpha_rad, stall_hyst_norm)
                L2 = LDf(qbar_psf, h_b_mac_ft, flaps_pos_deg)
                L3 = LDe(qbar_psf, elev_pos_rad)
                L4 = Ladot(qbarUW_psf, alphadot_rad_sec, ci2vel)
                L5 = Lq(qbar_psf, q_rad_sec, ci2vel)

                l1 = lb(qbar_psf, beta_rad, alpha_rad)
                l2 = lp(qbar_psf, beta_rad, bi2vel, p_rad_sec) 
                l3 = lr(qbar_psf, bi2vel, r_rad_sec, flaps_pos_deg, alpha_rad, stall_hyst_norm)
                l4 = lDa(qbar_psf, left_aileron_pos_rad, right_aileron_pos_rad, alpha_rad, stall_hyst_norm)
                l5 = ldr(qbar_psf, rudder_pos_rad)

                m1 = m0(qbar_psf)
                m2 = malpha(qbar_psf, alpha_deg, alpha_rad)
                m3 = mq(qbar_psf, ci2vel, q_rad_sec)
                m4 = madot(qbarUW_psf, ci2vel, alphadot_rad_sec)
                m5 = mde(qbar_induced_psf, elev_pos_rad, alpha_deg)
                m6 = mdf(qbar_psf, flaps_pos_deg)

                n1 = nb(qbar_psf, beta_rad)
                n2 = nspw(qbar_propwash_psf)
                n3 = nr(qbar_psf, bi2vel, r_rad_sec)
                n4 = nrf(qbar_psf, bi2vel, r_rad_sec, alpha_rad)
                n5 = nda(qbar_psf, left_aileron_pos_rad, right_aileron_pos_rad, alpha_rad, beta_rad)
                n6 = ndr(qbar_induced_psf, rudder_pos_rad)

                D = D1 + D2 + D3 + D4 + D5
                S = S1 + S2
                L = L1 + L2 + L3 + L4 + L5
                l = l1 + l2 + l3 + l4 + l5
                m = m1 + m2 + m3 + m4 + m5 + m6
                n = n1 + n2 + n3 + n4 + n5 + n6
            
                self.csvdyn[i,:] = [time, D, S, L, l, m, n]

            dyn2csv_in.send(self.csvdyn[:framescount,:]) #send calculated dynamics to store in CSV
            self.csvdyn = np.empty((MODEL_HZ, DYN_LEN + 1)) #empty array 
