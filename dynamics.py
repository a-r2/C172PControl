import multiprocessing as mp
import numpy as np

from c172p_model import *
from settings import *

DYN_LEN = 7 #[D, S, L, T, l, m, n]

class Dynamics():
    def __init__(self, rx2dyn_out, dyn2csv_in, event_start):
        self.csvdyn = np.zeros((MODEL_HZ, DYN_LEN + 1)) #array for storing dynamics
        self.proc = mp.Process(target=self.forces_moments_body, args=(rx2dyn_out, dyn2csv_in, event_start), daemon=True) #process for calculating dynamics
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

                fxaero1 = fxaero1(qbar_psf)
                fxaero2 = fxaero2(qbar_psf, h_b_mac_ft, flaps_pos_deg)
                fxaero3 = fxaero3(qbar_psf, h_b_mac_ft, alpha_rad, flaps_pos_deg)
                fxaero4 = fxaero4(qbar_psf, beta_rad)

                fyaero1 = fyaero1(qbar_psf, beta_rad, flaps_pos_deg)
                fyaero2 = fyaero2(qbar_psf, rudder_pos_rad)

                fzaero1 = fzaero1(qbar_psf, h_b_mac_ft, alpha_rad, stall_hyst_norm)
                fzaero2 = fzaero2(qbar_psf, h_b_mac_ft, flaps_pos_deg)
                fzaero3 = fzaero3(qbar_psf, elev_pos_rad)
                fzaero4 = fzaero4(qbarUW_psf, alphadot_rad_sec, ci2vel)
                fzaero5 = fzaero5(qbar_psf, q_rad_sec, ci2vel)

                fxthrust = fxthrust(advance_ratio, density, rpm_prop)

                mxaero1 = mxaero1(qbar_psf, beta_rad, alpha_rad)
                mxaero2 = mxaero2(qbar_psf, bi2vel, p_rad_sec) 
                mxaero3 = mxaero3(qbar_psf, bi2vel, r_rad_sec, flaps_pos_deg, alpha_rad, stall_hyst_norm)
                mxaero4 = mxaero4(qbar_psf, left_aileron_pos_rad, right_aileron_pos_rad, alpha_rad, stall_hyst_norm)
                mxaero5 = mxaero5(qbar_psf, rudder_pos_rad)

                myaero1 = myaero1(qbar_psf)
                myaero2 = myaero2(qbar_psf, alpha_deg, alpha_rad)
                myaero3 = myaero3(qbar_psf, ci2vel, q_rad_sec)
                myaero4 = myaero4(qbarUW_psf, ci2vel, alphadot_rad_sec)
                myaero5 = myaero5(qbar_induced_psf, elev_pos_rad, alpha_deg)
                myaero6 = myaero6(qbar_psf, flaps_pos_deg)

                mzaero1 = mzaero1(qbar_psf, beta_rad)
                mzaero2 = mzaero2(qbar_propwash_psf)
                mzaero3 = mzaero3(qbar_psf, bi2vel, r_rad_sec)
                mzaero4 = mzaero4(qbar_psf, bi2vel, r_rad_sec, alpha_rad)
                mzaero5 = mzaero5(qbar_psf, left_aileron_pos_rad, right_aileron_pos_rad, alpha_rad, beta_rad)
                mzaero6 = mzaero6(qbar_induced_psf, rudder_pos_rad)

                fxaero = fxaero1 + fxaero2 + fxaero3 + fxaero4 + fxthrust
                fyaero = fyaero1 + fyaero2
                fzaero = fzaero1 + fzaero2 + fzaero3 + fzaero4 + fzaero5 + fzaero6
                mxaero = mxaero1 + mxaero2 + mxaero3 + mxaero4
                myaero = myaero1 + myaero2 + myaero3 + myaero4 + myaero5 + myaero6
                mzaero = mzaero1 + mzaero2 + mzaero3 + mzaero4 + mzaero5 + mzaero6

                D = - fxaero
                Y = fyaero
                L = - fzaero
                T = fxthrust
                l = mxaero
                m = myaero
                n = mzaero
            
                self.csvdyn[i,:] = [time, D, S, L, T, l, m, n]

            dyn2csv_in.send(self.csvdyn[:framescount,:]) #send calculated dynamics to store in CSV
            self.csvdyn = np.empty((MODEL_HZ, DYN_LEN + 1)) #empty array 
