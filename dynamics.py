import multiprocessing as mp
import numpy as np

from constants import *
from c172p_model import *
from settings import *

class Dynamics():
    def __init__(self, dyn2csv_in, rx2dyn_out, event_start):
        self.csvdyn = np.zeros((MODEL_HZ, DYN_LEN + 1)) #array for storing dynamics
        self.proc   = mp.Process(target=self.forces_moments_body, args=(dyn2csv_in, rx2dyn_out, event_start), daemon=True) #process for calculating dynamics
        self.proc.start()
    def forces_moments_body(self, dyn2csv_in, rx2dyn_out, event_start):
        #Aerodynamic and propulsive forces and moments applied to the aircraft in body frame
        event_start.wait() #wait for simulation start event
        while True:
            rxdata      = rx2dyn_out.recv() #receive RX telemetry
            framescount = rxdata.shape[0]
            for i in range(framescount):                

                t_sim         = rxdata[i,0]
                phi_rad       = rxdata[i,20]
                theta_rad     = rxdata[i,22]
                psi_rad       = rxdata[i,24]
                alpha_rad     = rxdata[i,26]
                beta_rad      = rxdata[i,28]
                p             = rxdata[i,46]
                q             = rxdata[i,47]
                r             = rxdata[i,48]
                alphadot_rads = rxdata[i,53]
                sigmara_rad   = rxdata[i,93] 
                sigmala_rad   = rxdata[i,96]
                sigmae_rad    = rxdata[i,99]
                sigmaf_rad    = rxdata[i,102]
                sigmar_rad    = rxdata[i,105]
                Bw2Va         = rxdata[i,109]
                Cw2Va         = rxdata[i,110]
                hmacb         = rxdata[i,111]
                qbar          = rxdata[i,112]
                qbaruw        = rxdata[i,113]
                qbarprop      = rxdata[i,114]
                qbarind       = rxdata[i,115]
                stall         = rxdata[i,116]
                rho           = rxdata[i,117]
                J             = rxdata[i,118]
                rpmprop       = rxdata[i,119]
                mass          = rxdata[i,126]
                grav          = rxdata[i,127]

                euler = np.array([phi_rad, theta_rad, psi_rad])
                quat  = euler_to_attquat(euler)

                # Conversions
                qbar     = psf_to_pa(qbar) 
                qbaruw   = psf_to_pa(qbaruw) 
                qbarprop = psf_to_pa(qbarprop) 
                qbarind  = psf_to_pa(qbarind) 
                rho      = slugft3_to_kgm3(rho)
                rpsprop  = rpm_to_rps(rpmprop)
                mass     = slug_to_kg(mass)
                grav     = ft_to_m(grav)

                CD1 = aerocoeff_CD1()
                CD2 = aerocoeff_CD2(hmacb, sigmaf_rad)
                CD3 = aerocoeff_CD3(hmacb, alpha_rad, sigmaf_rad)
                CD4 = aerocoeff_CD4(beta_rad)
                CC1 = aerocoeff_CC1(beta_rad, sigmaf_rad)
                CC2 = aerocoeff_CC2(sigmar_rad)
                CL1 = aerocoeff_CL1(hmacb, alpha_rad, stall)
                CL2 = aerocoeff_CL2(hmacb, sigmaf_rad)
                CL3 = aerocoeff_CL3(sigmae_rad)
                CL4 = aerocoeff_CL4(Cw2Va, q)
                CL5 = aerocoeff_CL5(Cw2Va, alphadot_rads)
                Cl1 = aerocoeff_Cl1(beta_rad, alpha_rad)
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
                Cn1 = aerocoeff_Cn1(beta_rad)
                Cn2 = aerocoeff_Cn2(Bw2Va, r)
                Cn3 = aerocoeff_Cn3(Bw2Va, r, alpha_rad)
                Cn4 = aerocoeff_Cn4(sigmala_rad, sigmara_rad, alpha_rad, beta_rad)
                Cn5 = aerocoeff_Cn5(sigmar_rad)
                Cn6 = aerocoeff_Cn6()

                D = qbar * SW_SI * (CD1 + CD2 + CD3 + CD4) 
                C = qbar * SW_SI * (CC1 + CC2)
                L = SW_SI * (qbar * (CL1 + CL2 + CL3 + CL4) + qbaruw * CL5)
                l = qbar * SW_SI * BW_SI * (Cl1 + Cl2 + Cl3 + Cl4 + Cl5)
                m = SW_SI * CW_SI * (qbar * (Cm1 + Cm2 + Cm3 + Cm4) + qbaruw * Cm5 + qbarind * Cm6)
                n = SW_SI * BW_SI * (qbar * (Cn1 + Cn2 + Cn3 + Cn4) + qbarind * Cn5 + qbarprop * Cn6)
                gx, gy, gz = gravity_body(quat, mass, grav)
                T = thrust_eng(J, rho, rpsprop)

                DCL = wind_to_body(alpha_rad, beta_rad).rotate([-D, C, -L])

                self.csvdyn[i,:] = [t_sim, D, C, L, T, gx, gy, gz, l, m, n]

            dyn2csv_in.send(self.csvdyn[:framescount,:]) #send dynamics to CSV
            self.csvdyn = np.empty((MODEL_HZ, DYN_LEN + 1)) #empty array 
