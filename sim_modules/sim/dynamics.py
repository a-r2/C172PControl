from constants import *
from settings import *
from c172p_model import *

import numpy as np

''' DYNAMICS MODULE '''
class Dynamics():

    def __init__(self):
        self.csvdyn = np.zeros((MODEL_HZ, DYN_LEN + 1)) #array for storing dynamics

    def run(self, dyn2csv_in, rx2dyn_out, event_start, event_end):
        #Aerodynamic and propulsive forces and moments applied to the aircraft in body frame
        event_start.wait() #wait for simulation start event
        while True:
            if event_end.is_set():
                #Close pipes
                dyn2csv_in.close()
                rx2dyn_out.close()
                break
            else:
                rxdata      = rx2dyn_out.recv() #receive RX telemetry
                framescount = rxdata.shape[0]
                for i in range(framescount):                
                    #RX telemetry
                    t_sim    = rxdata[i,0]
                    phi      = rxdata[i,14]
                    theta    = rxdata[i,15]
                    psi      = rxdata[i,16]
                    alpha    = rxdata[i,17]
                    beta     = rxdata[i,18]
                    p        = rxdata[i,35]
                    q        = rxdata[i,36]
                    r        = rxdata[i,37]
                    alphadot = rxdata[i,41]
                    sigmara  = rxdata[i,79] 
                    sigmala  = rxdata[i,81]
                    sigmae   = rxdata[i,83]
                    sigmaf   = rxdata[i,85]
                    sigmar   = rxdata[i,87]
                    Bw2Va    = rxdata[i,91]
                    Cw2Va    = rxdata[i,92]
                    hmacb    = rxdata[i,93]
                    qbar     = rxdata[i,94]
                    qbaruw   = rxdata[i,95]
                    qbarprop = rxdata[i,96]
                    qbarind  = rxdata[i,97]
                    stall    = rxdata[i,98]
                    rho      = rxdata[i,99]
                    J        = rxdata[i,100]
                    revprop  = rxdata[i,101]
                    Ixx      = rxdata[i,102]
                    Ixy      = rxdata[i,103]
                    Ixz      = rxdata[i,104]
                    Iyy      = rxdata[i,105]
                    Iyz      = rxdata[i,106]
                    Izz      = rxdata[i,107]
                    mass     = rxdata[i,108]
                    grav     = rxdata[i,109]
                    #Conversions
                    sigmaa_avg = sigmaa_avg_acm(sigmala, sigmara)
                    euler      = np.array([phi, theta, psi], dtype=float)
                    #Dynamics tables
                    TDge_out = TDge_interp(hmacb)
                    TD2_out  = TD2_interp(sigmaf)
                    TD3_out  = TD3_interp(alpha, sigmaf)
                    TC1_out  = TC1_interp(beta, sigmaf)
                    TLge_out = TLge_interp(hmacb)
                    TL1_out  = TL1_interp(alpha, stall)
                    TL2_out  = TL2_interp(sigmaf)
                    Tl1_out  = Tl1_interp(alpha)
                    Tl31_out = Tl31_interp(sigmaf)
                    Tl32_out = Tl32_interp(alpha, r)
                    Tl33_out = Tl33_interp(alpha, r)
                    Tl4_out  = Tl4_interp(alpha, stall)
                    Tm1_out  = Tm1_interp(qbar)
                    Tm2_out  = Tm2_interp(alpha)
                    Tm4_out  = Tm4_interp(sigmaf)
                    Tm5_out  = Tm5_interp(sigmae, alpha)
                    Tn1_out  = Tn1_interp(beta)
                    Tn3_out  = Tn3_interp(r, alpha)
                    Tn4_out  = Tn4_interp(alpha, beta)
                    TT_out   = TT_interp(J)
                    #Dynamics coefficients
                    CT  = TT_out
                    CD1 = aerocoeff_CD1()
                    CD2 = aerocoeff_CD2(TDge_out, TD2_out)
                    CD3 = aerocoeff_CD3(TDge_out, TD3_out)
                    CD4 = aerocoeff_CD4(beta)
                    CC1 = aerocoeff_CC1(TC1_out)
                    CC2 = aerocoeff_CC2(sigmar)
                    CL1 = aerocoeff_CL1(TLge_out, TL1_out)
                    CL2 = aerocoeff_CL2(TLge_out, TL2_out)
                    CL3 = aerocoeff_CL3(sigmae)
                    CL4 = aerocoeff_CL4(Cw2Va, q)
                    CL5 = aerocoeff_CL5(Cw2Va, alphadot)
                    Cl1 = aerocoeff_Cl1(beta, Tl1_out)
                    Cl2 = aerocoeff_Cl2(Bw2Va, p)
                    Cl3 = aerocoeff_Cl3(Bw2Va, r, Tl31_out, Tl32_out, Tl33_out, stall)
                    Cl4 = aerocoeff_Cl4(sigmaa_avg, Tl4_out)
                    Cl5 = aerocoeff_Cl5(sigmar)
                    Cm1 = aerocoeff_Cm1(Tm1_out)
                    Cm2 = aerocoeff_Cm2(alpha, Tm2_out)
                    Cm3 = aerocoeff_Cm3(Cw2Va, q)
                    Cm4 = aerocoeff_Cm4(Tm4_out)
                    Cm5 = aerocoeff_Cm5(Cw2Va, alphadot)
                    Cm6 = aerocoeff_Cm6(sigmae, Tm5_out)
                    Cn1 = aerocoeff_Cn1(Tn1_out)
                    Cn2 = aerocoeff_Cn2(Bw2Va, r)
                    Cn3 = aerocoeff_Cn3(Bw2Va, Tn3_out)
                    Cn4 = aerocoeff_Cn4(sigmaa_avg, Tn4_out)
                    Cn5 = aerocoeff_Cn5(sigmar)
                    Cn6 = aerocoeff_Cn6()
                    #Dynamics equations
                    D = qbar * SW * (CD1 + CD2 + CD3 + CD4) 
                    C = qbar * SW * (CC1 + CC2)
                    L = SW * (qbar * (CL1 + CL2 + CL3 + CL4) + qbaruw * CL5)
                    T = thrust_eng(CT, rho, revprop)
                    gx, gy, gz = gravity_body(euler, mass, grav)
                    l = qbar * SW * BW * (Cl1 + Cl2 + Cl3 + Cl4 + Cl5)
                    m = SW * CW * (qbar * (Cm1 + Cm2 + Cm3 + Cm4) + qbaruw * Cm5 + qbarind * Cm6)
                    n = SW * BW * (qbar * (Cn1 + Cn2 + Cn3 + Cn4) + qbarind * Cn5 + qbarprop * Cn6)

                    self.csvdyn[i,:] = [t_sim, D, C, L, T, gx, gy, gz, l, m, n, CD1, CD2, CD3, CD4, CC1, CC2, CL1, CL2, CL3, CL4, CL5, Cl1, Cl2, Cl3, Cl4, Cl5, Cm1, Cm2, Cm3, Cm4, Cm5, Cm6, Cn1, Cn2, Cn3, Cn4, Cn5, Cn6]

                dyn2csv_in.send(self.csvdyn[:framescount,:]) #send dynamics to CSV
                self.csvdyn = np.empty((MODEL_HZ, DYN_LEN + 1)) #empty array 
