from constants import *
from settings import *
from modules.func.utils import *
from modules.sim.control_models import *

import matplotlib as mpl
import matplotlib.pyplot as plt

from c172p_model import *

''' PLOTTING PARAMETERS '''
mpl.rcParams['axes.labelsize']    = 14
mpl.rcParams['axes.titlesize']    = 16
mpl.rcParams['figure.autolayout'] = True
mpl.rcParams['figure.titlesize']  = 16
mpl.rcParams['lines.linestyle']   = '-'
mpl.rcParams['lines.linewidth']   = 1
mpl.rcParams['lines.marker']      = 'o'
mpl.rcParams['lines.markersize']  = 6
mpl.rcParams['xtick.labelsize']   = 12
mpl.rcParams['ytick.labelsize']   = 12

''' PLOTTING FUNCTION '''
def plot(rxdata, txdata, dyndata, eqdata, spdata):
    #RX telemetry
    t_sim_rx    = rxdata[:,0]
    dt_sim      = rxdata[:,1]
    long_delta  = rxdata[:,2]
    lat_delta   = rxdata[:,3]
    dist_delta  = rxdata[:,4]
    pn_ecef     = rxdata[:,5]
    pe_ecef     = rxdata[:,6]
    pd_ecef     = rxdata[:,7]
    long_gnss   = rxdata[:,8]
    lat_gnss    = rxdata[:,9]
    h_gnss      = rxdata[:,10]
    h_qfe       = rxdata[:,11]
    h_qnh       = rxdata[:,12]
    h_terr      = rxdata[:,13]
    phi_rx      = rxdata[:,14]
    theta_rx    = rxdata[:,15]
    psi_rx      = rxdata[:,16]
    alpha       = rxdata[:,17]
    beta        = rxdata[:,18]
    gamma       = rxdata[:,19]
    vn          = rxdata[:,20]
    ve          = rxdata[:,21]
    vd          = rxdata[:,22]
    u_rx        = rxdata[:,23]
    v_rx        = rxdata[:,24]
    w_rx        = rxdata[:,25]
    u_aero      = rxdata[:,26]
    v_aero      = rxdata[:,27]
    w_aero      = rxdata[:,28]
    wn          = rxdata[:,29]
    we          = rxdata[:,30]
    wd          = rxdata[:,31]
    phidot      = rxdata[:,32]
    thetadot    = rxdata[:,33]
    psidot      = rxdata[:,34]
    p_rx        = rxdata[:,35]
    q_rx        = rxdata[:,36]
    r_rx        = rxdata[:,37]
    p_aero      = rxdata[:,38]
    q_aero      = rxdata[:,39]
    r_aero      = rxdata[:,40]
    alphadot    = rxdata[:,41]
    betadot     = rxdata[:,42]
    udot        = rxdata[:,43]
    vdot        = rxdata[:,44]
    wdot        = rxdata[:,45]
    pdot        = rxdata[:,46]
    qdot        = rxdata[:,47]
    rdot        = rxdata[:,48]
    fx_aero     = rxdata[:,49]
    fx_ext      = rxdata[:,50]
    fx_gear     = rxdata[:,51]
    fx_prop     = rxdata[:,52]
    fx          = rxdata[:,53]
    fy_aero     = rxdata[:,54]
    fy_ext      = rxdata[:,55]
    fy_gear     = rxdata[:,56]
    fy_prop     = rxdata[:,57]
    fy          = rxdata[:,58]
    fz_aero     = rxdata[:,59]
    fz_ext      = rxdata[:,60]
    fz_gear     = rxdata[:,61]
    fz_prop     = rxdata[:,62]
    fz          = rxdata[:,63]
    l_aero      = rxdata[:,64]
    l_ext       = rxdata[:,65]
    l_gear      = rxdata[:,66]
    l_prop      = rxdata[:,67]
    l           = rxdata[:,68]
    m_aero      = rxdata[:,69]
    m_ext       = rxdata[:,70]
    m_gear      = rxdata[:,71]
    m_prop      = rxdata[:,72]
    m           = rxdata[:,73]
    n_aero      = rxdata[:,74]
    n_ext       = rxdata[:,75]
    n_gear      = rxdata[:,76]
    n_prop      = rxdata[:,77]
    n           = rxdata[:,78]
    sigmara_rx  = rxdata[:,79]
    deltara_rx  = rxdata[:,80]
    sigmala_rx  = rxdata[:,81]
    deltala_rx  = rxdata[:,82]
    sigmae_rx   = rxdata[:,83]
    deltae_rx   = rxdata[:,84]
    sigmaf_rx   = rxdata[:,85]
    deltaf_rx   = rxdata[:,86]
    sigmar_rx   = rxdata[:,87]
    deltar_rx   = rxdata[:,88]
    deltat_rx   = rxdata[:,89]
    deltam_rx   = rxdata[:,90]
    Bw2Va       = rxdata[:,91]
    Cw2Va       = rxdata[:,92]
    hmacb       = rxdata[:,93]
    qbar_rx     = rxdata[:,94]
    qbaruw_rx   = rxdata[:,95]
    qbarprop_rx = rxdata[:,96]
    qbarind_rx  = rxdata[:,97]
    stall       = rxdata[:,98]
    rho         = rxdata[:,99]
    J           = rxdata[:,100]
    revprop_rx  = rxdata[:,101]
    Ixx         = rxdata[:,102]
    Ixy         = rxdata[:,103]
    Ixz         = rxdata[:,104]
    Iyy         = rxdata[:,105]
    Iyz         = rxdata[:,106]
    Izz         = rxdata[:,107]
    mass        = rxdata[:,108]
    grav        = rxdata[:,109]
    up_down     = rxdata[:,110]
    wow1        = rxdata[:,111]
    wow2        = rxdata[:,112]
    wow3        = rxdata[:,113]
    wc1         = rxdata[:,114]
    wc2         = rxdata[:,115]
    wc3         = rxdata[:,116]
    D1_rx       = rxdata[:,117]
    D2_rx       = rxdata[:,118]
    D3_rx       = rxdata[:,119]
    D4_rx       = rxdata[:,120]
    C1_rx       = rxdata[:,121]
    C2_rx       = rxdata[:,122]
    L1_rx       = rxdata[:,123]
    L2_rx       = rxdata[:,124]
    L3_rx       = rxdata[:,125]
    L4_rx       = rxdata[:,126]
    L5_rx       = rxdata[:,127]
    l1_rx       = rxdata[:,128]
    l2_rx       = rxdata[:,129]
    l3_rx       = rxdata[:,130]
    l4_rx       = rxdata[:,131]
    l5_rx       = rxdata[:,132]
    m1_rx       = rxdata[:,133]
    m2_rx       = rxdata[:,134]
    m3_rx       = rxdata[:,135]
    m4_rx       = rxdata[:,136]
    m5_rx       = rxdata[:,137]
    m6_rx       = rxdata[:,138]
    n1_rx       = rxdata[:,139]
    n2_rx       = rxdata[:,140]
    n3_rx       = rxdata[:,141]
    n4_rx       = rxdata[:,142]
    n5_rx       = rxdata[:,143]
    n6_rx       = rxdata[:,144]
    Vprop_rx    = rxdata[:,145]
    Vind_rx     = rxdata[:,146]
    #TX telemetry
    t_sim_tx       = txdata[:,0]
    deltaa_tx      = txdata[:,1]
    deltae_tx      = txdata[:,2]
    deltaf_tx      = txdata[:,3]
    deltar_tx      = txdata[:,4]
    deltat_tx      = txdata[:,5]
    deltam_tx      = txdata[:,6]
    deltaa_trim_tx = txdata[:,7]
    deltae_trim_tx = txdata[:,8]
    deltar_trim_tx = txdata[:,9]
    #Dynamics
    t_sim_dyn = dyndata[:,0]
    D         = dyndata[:,1]
    C         = dyndata[:,2]
    L         = dyndata[:,3]
    T         = dyndata[:,4]
    gx        = dyndata[:,5]
    gy        = dyndata[:,6]
    gz        = dyndata[:,7]
    l_dyn     = dyndata[:,8]
    m_dyn     = dyndata[:,9]
    n_dyn     = dyndata[:,10]
    CD1_dyn   = dyndata[:,11]
    CD2_dyn   = dyndata[:,12]
    CD3_dyn   = dyndata[:,13]
    CD4_dyn   = dyndata[:,14]
    CC1_dyn   = dyndata[:,15]
    CC2_dyn   = dyndata[:,16]
    CL1_dyn   = dyndata[:,17]
    CL2_dyn   = dyndata[:,18]
    CL3_dyn   = dyndata[:,19]
    CL4_dyn   = dyndata[:,20]
    CL5_dyn   = dyndata[:,21]
    Cl1_dyn   = dyndata[:,22]
    Cl2_dyn   = dyndata[:,23]
    Cl3_dyn   = dyndata[:,24]
    Cl4_dyn   = dyndata[:,25]
    Cl5_dyn   = dyndata[:,26]
    Cm1_dyn   = dyndata[:,27]
    Cm2_dyn   = dyndata[:,28]
    Cm3_dyn   = dyndata[:,29]
    Cm4_dyn   = dyndata[:,30]
    Cm5_dyn   = dyndata[:,31]
    Cm6_dyn   = dyndata[:,32]
    Cn1_dyn   = dyndata[:,33]
    Cn2_dyn   = dyndata[:,34]
    Cn3_dyn   = dyndata[:,35]
    Cn4_dyn   = dyndata[:,36]
    Cn5_dyn   = dyndata[:,37]
    Cn6_dyn   = dyndata[:,38]
    #Equilibrium point
    t_sim_eq  = eqdata[:,0]
    pn_eq     = eqdata[:,1]
    pe_eq     = eqdata[:,2]
    pd_eq     = eqdata[:,3]
    phi_eq    = eqdata[:,4]
    theta_eq  = eqdata[:,5]
    psi_eq    = eqdata[:,6]
    u_eq      = eqdata[:,7]
    v_eq      = eqdata[:,8]
    w_eq      = eqdata[:,9]
    p_eq      = eqdata[:,10]
    q_eq      = eqdata[:,11]
    r_eq      = eqdata[:,12]
    deltaa_eq = eqdata[:,13]
    deltae_eq = eqdata[:,14]
    deltaf_eq = eqdata[:,15]
    deltar_eq = eqdata[:,16]
    deltat_eq = eqdata[:,17]
    deltam_eq = eqdata[:,18]
    #Setpoint
    t_sim_sp  = spdata[:,0]
    pn_sp     = spdata[:,1]
    pe_sp     = spdata[:,2]
    pd_sp     = spdata[:,3]
    phi_sp    = spdata[:,4]
    theta_sp  = spdata[:,5]
    psi_sp    = spdata[:,6]
    u_sp      = spdata[:,7]
    v_sp      = spdata[:,8]
    w_sp      = spdata[:,9]
    p_sp      = spdata[:,10]
    q_sp      = spdata[:,11]
    r_sp      = spdata[:,12]
    deltaa_sp = spdata[:,13]
    deltae_sp = spdata[:,14]
    deltaf_sp = spdata[:,15]
    deltar_sp = spdata[:,16]
    deltat_sp = spdata[:,17]
    deltam_sp = spdata[:,18]
    #Aerodynamic contributions
    D1_dyn = qbar_rx * SW * CD1_dyn
    D2_dyn = qbar_rx * SW * CD2_dyn
    D3_dyn = qbar_rx * SW * CD3_dyn
    D4_dyn = qbar_rx * SW * CD4_dyn
    C1_dyn = qbar_rx * SW * CC1_dyn
    C2_dyn = qbar_rx * SW * CC2_dyn
    L1_dyn = qbar_rx * SW * CL1_dyn
    L2_dyn = qbar_rx * SW * CL2_dyn
    L3_dyn = qbar_rx * SW * CL3_dyn
    L4_dyn = qbar_rx * SW * CL4_dyn
    L5_dyn = qbaruw_rx * SW * CL5_dyn
    l1_dyn = qbar_rx * SW * BW * Cl1_dyn
    l2_dyn = qbar_rx * SW * BW * Cl2_dyn
    l3_dyn = qbar_rx * SW * BW * Cl3_dyn
    l4_dyn = qbar_rx * SW * BW * Cl4_dyn
    l5_dyn = qbar_rx * SW * BW * Cl5_dyn
    m1_dyn = qbar_rx * SW * CW * Cm1_dyn
    m2_dyn = qbar_rx * SW * CW * Cm2_dyn
    m3_dyn = qbar_rx * SW * CW * Cm3_dyn
    m4_dyn = qbar_rx * SW * CW * Cm4_dyn
    m5_dyn = qbaruw_rx * SW * CW * Cm5_dyn
    m6_dyn = qbarind_rx * SW * CW * Cm6_dyn
    n1_dyn = qbar_rx * SW * BW * Cn1_dyn
    n2_dyn = qbar_rx * SW * BW * Cn2_dyn
    n3_dyn = qbar_rx * SW * BW * Cn3_dyn
    n4_dyn = qbar_rx * SW * BW * Cn4_dyn
    n5_dyn = qbarind_rx * SW * BW * Cn5_dyn
    n6_dyn = qbarprop_rx * SW * BW * Cn6_dyn
    #Global aileron actuation from left and right aileron actuations
    deltaa_rx = np.zeros(rxdata.shape[0])
    for i in range(rxdata.shape[0]):
        deltaa_rx[i] = deltaa_avg_acm(deltala_rx[i], deltara_rx[i])
    #Control model
    t_sim_cm   = t_sim_rx
    revprop_cm = np.zeros(rxdata.shape[0])
    for i in range(rxdata.shape[0]):
        if i == 0:
            #revprop_cm[i] = revprop_rx[i]
            revprop_cm[i] = revprop_acm(deltat_rx[i])
        else:
            #revprop_cm[i] = min(N_MAX, revprop_cm[i-1] + (N_MAX - N_MIN) * (deltat_rx[i] - deltat_rx[i-1]))
            revprop_cm[i] = revprop_acm(deltat_rx[i])
    pn_er      = np.zeros(rxdata.shape[0])
    pe_er      = np.zeros(rxdata.shape[0])
    pd_er      = np.zeros(rxdata.shape[0])
    phi_er     = np.zeros(rxdata.shape[0])
    theta_er   = np.zeros(rxdata.shape[0])
    psi_er     = np.zeros(rxdata.shape[0])
    u_er       = np.zeros(rxdata.shape[0])
    v_er       = np.zeros(rxdata.shape[0])
    w_er       = np.zeros(rxdata.shape[0])
    p_er       = np.zeros(rxdata.shape[0])
    q_er       = np.zeros(rxdata.shape[0])
    r_er       = np.zeros(rxdata.shape[0])
    deltaa_er  = np.zeros(rxdata.shape[0])
    deltae_er  = np.zeros(rxdata.shape[0])
    deltaf_er  = np.zeros(rxdata.shape[0])
    deltar_er  = np.zeros(rxdata.shape[0])
    deltat_er  = np.zeros(rxdata.shape[0])
    deltam_er  = np.zeros(rxdata.shape[0])
    j = 0
    for i in range(rxdata.shape[0]):
        if t_sim_eq[j] == t_sim_cm[i]:
            pn_er[i]     = pn_ecef[i] - pn_eq[j]
            pe_er[i]     = pe_ecef[i] - pe_eq[j]
            pd_er[i]     = pd_ecef[i] - pd_eq[j]
            phi_er[i]    = phi_rx[i] - phi_eq[j]
            theta_er[i]  = theta_rx[i] - theta_eq[j]
            psi_er[i]    = psi_rx[i] - psi_eq[j]
            u_er[i]      = u_rx[i] - u_eq[j]
            v_er[i]      = v_rx[i] - v_eq[j]
            w_er[i]      = w_rx[i] - w_eq[j]
            p_er[i]      = p_rx[i] - p_eq[j]
            q_er[i]      = q_rx[i] - q_eq[j]
            r_er[i]      = r_rx[i] - r_eq[j]
            deltaa_er[i] = deltaa_rx[i] - deltaa_eq[j]
            deltae_er[i] = deltae_rx[i] - deltae_eq[j]
            deltaf_er[i] = deltaf_rx[i] - deltaf_eq[j]
            deltar_er[i] = deltar_rx[i] - deltar_eq[j]
            deltat_er[i] = deltat_rx[i] - deltat_eq[j]
            deltam_er[i] = deltam_rx[i] - deltam_eq[j]
            if j < (eqdata.shape[0] - 1):
                j += 1
            else:
                j = eqdata.shape[0] - 1
        else:
            pn_er[i]     = pn_er[i-1]
            pe_er[i]     = pe_er[i-1]
            pd_er[i]     = pd_er[i-1]
            phi_er[i]    = phi_er[i-1]
            theta_er[i]  = theta_er[i-1]
            psi_er[i]    = psi_er[i-1]
            u_er[i]      = u_er[i-1]
            v_er[i]      = v_er[i-1]
            w_er[i]      = w_er[i-1]
            p_er[i]      = p_er[i-1]
            q_er[i]      = q_er[i-1]
            r_er[i]      = r_er[i-1]
            deltaa_er[i] = deltaa_er[i-1]
            deltae_er[i] = deltae_er[i-1]
            deltaf_er[i] = deltaf_er[i-1]
            deltar_er[i] = deltar_er[i-1]
            deltat_er[i] = deltat_er[i-1]
            deltam_er[i] = deltam_er[i-1]

    ########################################################################
    r_cg = in_to_m(np.array([-39.06, 0, -36.5]))
    r_prop = in_to_m(np.array([-39.06, 0, -36.5]))
    vlocal = np.zeros((dyndata.shape[0], 3))
    Thrust = np.zeros(dyndata.shape[0])
    Va_cm  = np.zeros(dyndata.shape[0])
    Vauw_cm  = np.zeros(dyndata.shape[0])
    qbar_cm  = np.zeros(dyndata.shape[0])
    qbaruw_cm  = np.zeros(dyndata.shape[0])
    qbarprop_cm  = np.zeros(dyndata.shape[0])
    qbarind_cm  = np.zeros(dyndata.shape[0])
    Vprop2  = np.zeros(dyndata.shape[0])
    Vprop2_cm  = np.zeros(dyndata.shape[0])
    Vprop  = np.zeros(dyndata.shape[0])
    Vprop_cm  = np.zeros(dyndata.shape[0])
    Vind   = np.zeros(dyndata.shape[0])
    Vind_cm   = np.zeros(dyndata.shape[0])
    tanVel = np.zeros(dyndata.shape[0])
    angle  = np.zeros(dyndata.shape[0])
    factor = np.zeros(dyndata.shape[0])
    p_factor = np.zeros((dyndata.shape[0],3))
    omega = np.zeros(dyndata.shape[0])
    Power = np.zeros(dyndata.shape[0])
    Torque = np.zeros(dyndata.shape[0])
    ExcessTorque = np.zeros(dyndata.shape[0])
    #revprop_cm = np.zeros(rxdata.shape[0])
    H = np.zeros((dyndata.shape[0], 3))
    r_act = np.zeros((dyndata.shape[0],3))
    Moment = np.zeros((dyndata.shape[0], 3))
    for i in range(min(rxdata.shape[0], dyndata.shape[0])):
        vlocal[i,:] = np.array([u_rx[i], v_rx[i], w_rx[i]]) + np.array([p_rx[i], q_rx[i], r_rx[i]]) * (r_prop - r_cg)
        Thrust[i] = thrust_eng(TT_interp(J[i]), rho[i], revprop_rx[i])
        Vprop2[i]  = vlocal[i,0] * abs(vlocal[i,0]) + ((2 * Thrust[i]) / (rho[i] * A_PROP))
        if Vprop2[i] >= 0:
            Vprop[i] = 0.5 * (- vlocal[i,0] + np.sqrt(Vprop2[i]))
        else:
            Vprop[i] = 0.5 * (- vlocal[i,0] - np.sqrt(- Vprop2[i]))
        tanVel[i] = np.linalg.norm([vlocal[i,1], vlocal[i,2]])
        angle[i]  = np.arctan2(tanVel[i], Vprop[i] + vlocal[i,0])
        factor[i] = 15 * angle[i] / tanVel[i]
        r_act[i,1] = r_prop[1] + factor[i] * vlocal[i,2]
        r_act[i,2] = r_prop[2] + factor[i] * vlocal[i,1]
        omega[i] = 2 * np.pi * revprop_rx[i]
        H[i,0] = slugft2_to_kgm2(1.67) * omega[i]
        Power[i] = power_eng(TP_interp(J[i]), rho[i], revprop_rx[i])
        Torque[i] = - Power[i] / max(0.01, omega[i])
        p_factor[i,1] = (r_act[i,2] - r_prop[2]) * Thrust[i] / 12
        p_factor[i,2] = (r_act[i,1] - r_prop[1]) * Thrust[i] / 12
        Moment[i,0] = Torque[i] #torque effect
        Moment[i,1] = H[i,0] * r_rx[i] #gyroscopic effect Y
        Moment[i,2] = - H[i,0] * q_rx[i] #gyroscopic effect Z
        Va_cm[i]         = Va_acm(u_rx[i], v_rx[i], w_rx[i])
        Vauw_cm[i]       = Vauw_acm(u_rx[i], w_rx[i])
        Vprop2_cm[i] = Vprop2_acm(u_rx[i], rho[i], Thrust[i])
        Vprop_cm[i] = Vprop_acm(u_rx[i], Vprop2_cm[i])
        Vind_cm[i] = Vind_acm(u_rx[i], Vprop_cm[i])
        qbar_cm[i]       = qbar_acm(rho[i], Va_cm[i])
        qbaruw_cm[i]     = qbaruw_acm(rho[i], Vauw_cm[i])
        qbarind_cm[i]    = qbarind_acm(rho[i], Vind_cm[i])
        qbarprop_cm[i]   = qbarprop_acm(rho[i], Vprop_cm[i])
        if omega[i] > 0.01:
            ExcessTorque[i] = Power[i] / omega[i]
        else:
            ExcessTorque[i] = Power[i]
        if i == 0:
            revprop_cm[i] = revprop_rx[i]
            #revprop_cm[i] = 0
        else:
            revprop_cm[i] = revprop_rx[i-1] + (2.0 * np.pi * ExcessTorque[i] / slugft2_to_kgm2(1.67)) * 0.001
            #revprop_cm[i] = revprop_rx[i] - (revprop_rx[i-1] + (2.0 * np.pi * ExcessTorque[i] / slugft2_to_kgm2(1.67)) * 0.001)
            #revprop_cm[i] = (Power[i] - Power[i-1] - 2 * np.pi * revprop_cm[i-1] * Torque[i]) / abs((2 * np.pi * (2 * Torque[i] - Torque[i-1])))
    #Moment[i,0] = Torque[i] #torque effect
    #Moment[i,1] = H[i,0] * r_rx[i] #gyroscopic effect Y
    #Moment[i,2] = - H[i,0] * q_rx[i] #gyroscopic effect Z
    ########################################################################

    fxyz_aero_dyn = np.zeros((dyndata.shape[0], 3))
    fxyz_dyn      = np.zeros((dyndata.shape[0], 3))
    lmn_aero_dyn  = np.zeros((dyndata.shape[0], 3))
    lmn_prop_dyn  = np.zeros((dyndata.shape[0], 3))
    lmn_dyn       = np.zeros((dyndata.shape[0], 3))
    for i in range(min(rxdata.shape[0], dyndata.shape[0])):
        fxyz_aero_dyn[i,:] = wind_to_body(alpha[i], beta[i]).rotate([-D[i], C[i], -L[i]])
        fxyz_dyn[i,:]      = fxyz_aero_dyn[i,:] + np.array([T[i], 0, 0])
        #fxyz_dyn[i,:]      = fxyz_aero_dyn[i,:] + np.array([gx[i], gy[i], gz[i]]) + np.array([T[i], 0, 0])
        #Moment[i,:] = wind_to_body(alpha[i], beta[i]).rotate(Moment[i,:])
        lmn_aero_dyn[i,:]  = [l_dyn[i], m_dyn[i], n_dyn[i]]
        #lmn_prop_dyn[i,:]  = np.cross(dr[i,:],H[i,:]) / 12
        lmn_prop_dyn[i,:]  = Moment[i,:]
        #lmn_dyn[i,:]       = lmn_aero_dyn[i,:] + lmn_prop_dyn[i,:]
        lmn_dyn[i,:]       = Moment[i,:]
    fx_aero_dyn = fxyz_aero_dyn[:,0]
    fy_aero_dyn = fxyz_aero_dyn[:,1]
    fz_aero_dyn = fxyz_aero_dyn[:,2]
    fx_dyn      = fxyz_dyn[:,0]
    fy_dyn      = fxyz_dyn[:,1]
    fz_dyn      = fxyz_dyn[:,2]
    l_aero_dyn  = lmn_aero_dyn[:,0]
    m_aero_dyn  = lmn_aero_dyn[:,1]
    n_aero_dyn  = lmn_aero_dyn[:,2]
    l_prop_dyn  = lmn_prop_dyn[:,0]
    m_prop_dyn  = lmn_prop_dyn[:,1]
    n_prop_dyn  = lmn_prop_dyn[:,2]
    l_dyn       = lmn_dyn[:,0]
    m_dyn       = lmn_dyn[:,1]
    n_dyn       = lmn_dyn[:,2]

    labels = list()
    for key, value in TELEM_RX_PLOT.items():
        labels.append(key + ' ' + value[1] + ' [' + value[0] + ']')
    for key, value in TELEM_TX_PLOT.items():
        labels.append(key + ' ' + value[1] + ' [' + value[0] + ']')

    if 1 in PLOTS:

        fig = plt.figure('Aircraft relative position in ECEF frame')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aircraft relative position in ECEF frame')
        plt.subplot(3,1,1)
        plt.plot(t_sim_rx, long_delta)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[2])
        plt.subplot(3,1,2)
        plt.plot(t_sim_rx, lat_delta)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[3])
        plt.subplot(3,1,3)
        plt.plot(t_sim_rx, dist_delta)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[4])
        plt.show()
        
    if 2 in PLOTS:

        fig = plt.figure('Aircraft position in ECEF frame')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aircraft position in ECEF frame')
        plt.subplot(3,1,1)
        plt.plot(t_sim_rx, pn_ecef)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[5])
        plt.subplot(3,1,2)
        plt.plot(t_sim_rx, pe_ecef)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[6])
        plt.subplot(3,1,3)
        plt.plot(t_sim_rx, pd_ecef)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[7])
        plt.show()

    if 3 in PLOTS:

        fig = plt.figure('Aircraft GNSS position in ECEF frame')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aircraft GNSS position in ECEF frame')
        plt.subplot(3,1,1)
        plt.plot(t_sim_rx, long_gnss)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[8])
        plt.subplot(3,1,2)
        plt.plot(t_sim_rx, lat_gnss)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[9])
        plt.subplot(3,1,3)
        plt.plot(t_sim_rx, h_gnss)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[10])
        plt.show()

    if 4 in PLOTS:

        fig = plt.figure('Aircraft barometric altitude')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aircraft barometric altitude')
        plt.subplot(3,1,1)
        plt.plot(t_sim_rx, h_qfe)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[11])
        plt.subplot(2,1,2)
        plt.plot(t_sim_rx, h_qnh)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[12])
        plt.subplot(3,1,3)
        plt.plot(t_sim_rx, h_terr)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[13])
        plt.show()
        
    if 5 in PLOTS:

        fig = plt.figure('Aircraft attitude in body frame')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aircraft attitude in body frame')
        plt.subplot(3,1,1)
        plt.plot(t_sim_rx, phi_rx)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[14])
        plt.subplot(3,1,2)
        plt.plot(t_sim_rx, theta_rx)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[15])
        plt.subplot(3,1,3)
        plt.plot(t_sim_rx, psi_rx)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[16])
        plt.show()

    if 6 in PLOTS:

        fig = plt.figure('Wind angles')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the wind angles')
        plt.subplot(2,1,1)
        plt.plot(t_sim_rx, alpha)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[17])
        plt.subplot(2,1,2)
        plt.plot(t_sim_rx, beta)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[18])
        plt.show()

    if 7 in PLOTS:

        fig = plt.figure('Aircraft path angle')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aircraft path angle')
        plt.subplot(1,1,1)
        plt.plot(t_sim_rx, gamma)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[19])
        plt.show()

    if 8 in PLOTS:

        fig = plt.figure('Aircraft linear velocity in NED frame')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aircraft linear velocity in NED frame')
        plt.subplot(3,1,1)
        plt.plot(t_sim_rx, vn)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[20])
        plt.subplot(3,1,2)
        plt.plot(t_sim_rx, ve)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[21])
        plt.subplot(3,1,3)
        plt.plot(t_sim_rx, vd)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[22])
        plt.show()

    if 9 in PLOTS:

        fig = plt.figure('Aircraft linear velocity in body frame')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aircraft linear velocity in body frame')
        plt.subplot(3,1,1)
        plt.plot(t_sim_rx, u_rx)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[23])
        plt.subplot(3,1,2)
        plt.plot(t_sim_rx, v_rx)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[24])
        plt.subplot(3,1,3)
        plt.plot(t_sim_rx, w_rx)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[25])
        plt.show()

    if 10 in PLOTS:

        fig = plt.figure('Aircraft linear velocity in wind frame')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aircraft linear velocity in wind frame')
        plt.subplot(3,1,1)
        plt.plot(t_sim_rx, u_aero)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[26])
        plt.subplot(3,1,2)
        plt.plot(t_sim_rx, v_aero)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[27])
        plt.subplot(3,1,3)
        plt.plot(t_sim_rx, w_aero)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[28])
        plt.show()

    if 11 in PLOTS:

        fig = plt.figure('Wind linear velocity in NED frame')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the wind linear velocity in NED frame')
        plt.subplot(3,1,1)
        plt.plot(t_sim_rx, wn)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[29])
        plt.subplot(3,1,2)
        plt.plot(t_sim_rx, we)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[30])
        plt.subplot(3,1,3)
        plt.plot(t_sim_rx, wd)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[31])
        plt.show()

    if 12 in PLOTS:

        fig = plt.figure('Aircraft attitude rate of change in body frame')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the rate of change of the aircraft attitude in body frame')
        plt.subplot(3,1,1)
        plt.plot(t_sim_rx, phidot)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[32])
        plt.subplot(3,1,2)
        plt.plot(t_sim_rx, thetadot)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[33])
        plt.subplot(3,1,3)
        plt.plot(t_sim_rx, psidot)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[34])
        plt.show()

    if 13 in PLOTS:

        fig = plt.figure('Aircraft rotational velocity in body frame')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aircraft rotational velocity in body frame')
        plt.subplot(3,1,1)
        plt.plot(t_sim_rx, p_rx)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[35])
        plt.subplot(3,1,2)
        plt.plot(t_sim_rx, q_rx)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[36])
        plt.subplot(3,1,3)
        plt.plot(t_sim_rx, r_rx)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[37])
        plt.show()

    if 14 in PLOTS:

        fig = plt.figure('Aircraft aerodynamic rotational velocity in body frame')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aircraft aerodynamic rotational velocity in body frame')
        plt.subplot(3,1,1)
        plt.plot(t_sim_rx, p_aero)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[38])
        plt.subplot(3,1,2)
        plt.plot(t_sim_rx, q_aero)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[39])
        plt.subplot(3,1,3)
        plt.plot(t_sim_rx, r_aero)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[40])
        plt.show()

    if 15 in PLOTS:

        fig = plt.figure('Wind angles rate of change')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the rate of change of the wind angles')
        plt.subplot(2,1,1)
        plt.plot(t_sim_rx, alphadot)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[41])
        plt.subplot(2,1,2)
        plt.plot(t_sim_rx, betadot)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[42])
        plt.show()

    if 16 in PLOTS:

        fig = plt.figure('Aircraft linear acceleration in body frame')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aircraft linear acceleration in body frame')
        plt.subplot(3,1,1)
        plt.plot(t_sim_rx, udot)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[43])
        plt.subplot(3,1,2)
        plt.plot(t_sim_rx, vdot)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[44])
        plt.subplot(3,1,3)
        plt.plot(t_sim_rx, wdot)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[45])
        plt.show()

    if 17 in PLOTS:

        fig = plt.figure('Aircraft rotational acceleration in body frame')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aircraft rotational acceleration in body frame')
        plt.subplot(3,1,1)
        plt.plot(t_sim_rx, pdot)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[46])
        plt.subplot(3,1,2)
        plt.plot(t_sim_rx, qdot)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[47])
        plt.subplot(3,1,3)
        plt.plot(t_sim_rx, rdot)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[48])
        plt.show()
    
    if 18 in PLOTS:

        fig = plt.figure('Aircraft aerodynamic force in body frame')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aircraft aerodynamic force in body frame')
        plt.subplot(3,1,1)
        plt.plot(t_sim_rx, fx_aero, t_sim_dyn, fx_aero_dyn, '.')
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[49])
        plt.subplot(3,1,2)
        plt.plot(t_sim_rx, fy_aero, t_sim_dyn, fy_aero_dyn, '.')
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[54])
        plt.subplot(3,1,3)
        plt.plot(t_sim_rx, fz_aero, t_sim_dyn, fz_aero_dyn, '.')
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[59])
        plt.show()
    
    if 19 in PLOTS:

        fig = plt.figure('Aircraft external force in body frame')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aircraft external force in body frame')
        plt.subplot(3,1,1)
        plt.plot(t_sim_rx, fx_ext)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[50])
        plt.subplot(3,1,2)
        plt.plot(t_sim_rx, fy_ext)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[55])
        plt.subplot(3,1,3)
        plt.plot(t_sim_rx, fz_ext)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[60])
        plt.show()
    
    if 20 in PLOTS:

        fig = plt.figure('Aircraft gear force in body frame')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aircraft gear force in body frame')
        plt.subplot(3,1,1)
        plt.plot(t_sim_rx, fx_gear)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[51])
        plt.subplot(3,1,2)
        plt.plot(t_sim_rx, fy_gear)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[56])
        plt.subplot(3,1,3)
        plt.plot(t_sim_rx, fz_gear)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[61])
        plt.show()
    
    if 21 in PLOTS:

        fig = plt.figure('Aircraft propeller force in body frame')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aircraft propeller force in body frame')
        plt.subplot(3,1,1)
        plt.plot(t_sim_rx, fx_prop, t_sim_dyn, T, '.')
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[52])
        plt.subplot(3,1,2)
        plt.plot(t_sim_rx, fy_prop)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[57])
        plt.subplot(3,1,3)
        plt.plot(t_sim_rx, fz_prop)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[62])
        plt.show()
    
    if 22 in PLOTS:

        fig = plt.figure('Aircraft total force in body frame')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aircraft total force in body frame')
        plt.subplot(3,1,1)
        plt.plot(t_sim_rx, fx, t_sim_dyn, fx_dyn, '.')
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[53])
        plt.subplot(3,1,2)
        plt.plot(t_sim_rx, fy, t_sim_dyn, fy_dyn, '.')
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[58])
        plt.subplot(3,1,3)
        plt.plot(t_sim_rx, fz, t_sim_dyn, fz_dyn, '.')
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[63])
        plt.show()
    
    if 23 in PLOTS:

        fig = plt.figure('Aircraft aerodynamic moment in body frame')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aircraft aerodynamic moment in body frame')
        plt.subplot(3,1,1)
        plt.plot(t_sim_rx, l_aero, t_sim_dyn, l_aero_dyn, '.')
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[64])
        plt.subplot(3,1,2)
        plt.plot(t_sim_rx, m_aero, t_sim_dyn, m_aero_dyn, '.')
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[69])
        plt.subplot(3,1,3)
        plt.plot(t_sim_rx, n_aero, t_sim_dyn, n_aero_dyn, '.')
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[74])
        plt.show()
    
    if 24 in PLOTS:

        fig = plt.figure('Aircraft external moment in body frame')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aircraft external moment in body frame')
        plt.subplot(3,1,1)
        plt.plot(t_sim_rx, l_ext)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[65])
        plt.subplot(3,1,2)
        plt.plot(t_sim_rx, m_ext)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[70])
        plt.subplot(3,1,3)
        plt.plot(t_sim_rx, n_ext)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[75])
        plt.show()
    
    if 25 in PLOTS:

        fig = plt.figure('Aircraft gear moment in body frame')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aircraft gear moment in body frame')
        plt.subplot(3,1,1)
        plt.plot(t_sim_rx, l_gear)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[66])
        plt.subplot(3,1,2)
        plt.plot(t_sim_rx, m_gear)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[71])
        plt.subplot(3,1,3)
        plt.plot(t_sim_rx, n_gear)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[76])
        plt.show()
    
    if 26 in PLOTS:

        fig = plt.figure('Aircraft propeller moment in body frame')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aircraft propeller moment in body frame')
        plt.subplot(3,1,1)
        plt.plot(t_sim_rx, l_prop, t_sim_dyn, l_prop_dyn, '.')
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[67])
        plt.subplot(3,1,2)
        plt.plot(t_sim_rx, m_prop, t_sim_dyn, m_prop_dyn, '.')
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[72])
        plt.subplot(3,1,3)
        plt.plot(t_sim_rx, n_prop, t_sim_dyn, n_prop_dyn, '.')
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[77])
        plt.show()
    
    if 27 in PLOTS:

        fig = plt.figure('Aircraft total moment in body frame')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aircraft total moment in body frame')
        plt.subplot(3,1,1)
        plt.plot(t_sim_rx, l, t_sim_dyn, l_dyn, '.')
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[68])
        plt.subplot(3,1,2)
        plt.plot(t_sim_rx, m, t_sim_dyn, m_dyn, '.')
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[73])
        plt.subplot(3,1,3)
        plt.plot(t_sim_rx, n, t_sim_dyn, n_dyn, '.')
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[78])
        plt.show()
    
    if 28 in PLOTS:

        fig = plt.figure('Aerodynamic actuators position')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aerodynamic actuators position')
        plt.subplot(5,1,1)
        plt.plot(t_sim_rx, sigmara_rx)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[79])
        plt.subplot(5,1,2)
        plt.plot(t_sim_rx, sigmala_rx)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[81])
        plt.subplot(5,1,3)
        plt.plot(t_sim_rx, sigmae_rx)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[83])
        plt.subplot(5,1,4)
        plt.plot(t_sim_rx, sigmaf_rx)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[85])
        plt.subplot(5,1,5)
        plt.plot(t_sim_rx, sigmar_rx)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[87])
        plt.show()

    if 29 in PLOTS:

        fig = plt.figure('Aerodynamic actuators normalized position')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aerodynamic actuators normalized position')
        plt.subplot(5,1,1)
        plt.plot(t_sim_rx, deltara_rx)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[80])
        plt.subplot(5,1,2)
        plt.plot(t_sim_rx, deltala_rx)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[82])
        plt.subplot(5,1,3)
        plt.plot(t_sim_rx, deltae_rx)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[84])
        plt.subplot(5,1,4)
        plt.plot(t_sim_rx, deltaf_rx)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[86])
        plt.subplot(5,1,5)
        plt.plot(t_sim_rx, deltar_rx)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[88])
        plt.show()

    if 30 in PLOTS:

        fig = plt.figure('Propulsive actuators normalized position')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the propulsive actuators normalized position')
        plt.subplot(2,1,1)
        plt.plot(t_sim_rx, deltat_rx)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[89])
        plt.subplot(2,1,2)
        plt.plot(t_sim_rx, deltam_rx)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[90])
        plt.show()

    if 31 in PLOTS:

        fig = plt.figure('Aerodynamic ratios')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aerodynamic ratios')
        plt.subplot(3,1,1)
        plt.plot(t_sim_rx, Bw2Va)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[91])
        plt.subplot(3,1,2)
        plt.plot(t_sim_rx, Cw2Va)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[92])
        plt.subplot(3,1,3)
        plt.plot(t_sim_rx, hmacb)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[93])
        plt.show()

    if 32 in PLOTS:

        fig = plt.figure('Dynamic pressure')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the dynamic pressure')
        plt.subplot(2,2,1)
        plt.plot(t_sim_rx, qbar_rx, t_sim_cm, qbar_cm, '.')
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[94])
        plt.subplot(2,2,2)
        plt.plot(t_sim_rx, qbaruw_rx, t_sim_cm, qbaruw_cm, '.')
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[95])
        plt.subplot(2,2,3)
        plt.plot(t_sim_rx, qbarprop_rx, t_sim_cm, qbarprop_cm, '.')
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[96])
        plt.subplot(2,2,4)
        plt.plot(t_sim_rx, qbarind_rx, t_sim_cm, qbarind_cm, '.')
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[97])
        plt.show()

    if 33 in PLOTS:

        fig = plt.figure('Normalized stall hysteresis')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the normalized stall hysteresis')
        plt.subplot(1,1,1)
        plt.plot(t_sim_rx, stall)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[98])
        plt.show()

    if 34 in PLOTS:

        fig = plt.figure('Air density')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the air density')
        plt.subplot(1,1,1)
        plt.plot(t_sim_rx, rho)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[99])
        plt.show()

    if 35 in PLOTS:

        fig = plt.figure('Advance ratio')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the advance ratio')
        plt.subplot(1,1,1)
        plt.plot(t_sim_rx, J)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[100])
        plt.show()

    if 36 in PLOTS:

        fig = plt.figure('Propeller revolutions')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the propeller revolutions per minute')
        plt.subplot(1,1,1)
        plt.plot(t_sim_rx, revprop_rx, t_sim_cm, revprop_cm, '.')
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[101])
        plt.show()

    if 37 in PLOTS:

        fig = plt.figure('Moments of inertia')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the moments of inertia')
        plt.subplot(3,2,1)
        plt.plot(t_sim_rx, Ixx)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[102])
        plt.subplot(3,2,2)
        plt.plot(t_sim_rx, Ixy)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[103])
        plt.subplot(3,2,3)
        plt.plot(t_sim_rx, Ixz)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[104])
        plt.subplot(3,2,4)
        plt.plot(t_sim_rx, Iyy)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[105])
        plt.subplot(3,2,5)
        plt.plot(t_sim_rx, Iyz)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[106])
        plt.subplot(3,2,6)
        plt.plot(t_sim_rx, Izz)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[107])
        plt.show()

    if 38 in PLOTS:

        fig = plt.figure('Mass')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the mass')
        plt.subplot(1,1,1)
        plt.plot(t_sim_rx, mass)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[108])
        plt.show()

    if 39 in PLOTS:

        fig = plt.figure('Gravity acceleration')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the gravity acceleration')
        plt.subplot(1,1,1)
        plt.plot(t_sim_rx, grav)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[109])
        plt.show()

    if 40 in PLOTS:

        fig = plt.figure('Upside-down state')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the upside-down state')
        plt.subplot(1,1,1)
        plt.plot(t_sim_rx, up_down)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[110])
        plt.show()

    if 41 in PLOTS:

        fig = plt.figure('Weight-on-wheel states')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the weight-on-wheel states')
        plt.subplot(3,1,1)
        plt.plot(t_sim_rx, wow1)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[111])
        plt.subplot(3,1,2)
        plt.plot(t_sim_rx, wow2)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[112])
        plt.subplot(3,1,3)
        plt.plot(t_sim_rx, wow3)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[113])
        plt.show()

    if 42 in PLOTS:

        fig = plt.figure('Wheels contact')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the wheels contact states')
        plt.subplot(3,1,1)
        plt.plot(t_sim_rx, wc1)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[114])
        plt.subplot(3,1,2)
        plt.plot(t_sim_rx, wc2)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[115])
        plt.subplot(3,1,3)
        plt.plot(t_sim_rx, wc3)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[116])
        plt.show()

    if 43 in PLOTS:

        fig = plt.figure('Aerodynamic drag coefficients')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aerodynamic drag coefficients')
        plt.subplot(2,2,1)
        plt.plot(t_sim_rx, D1_rx, t_sim_dyn, D1_dyn, '.')
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[117])
        plt.subplot(2,2,2)
        plt.plot(t_sim_rx, D2_rx, t_sim_dyn, D2_dyn, '.')
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[118])
        plt.subplot(2,2,3)
        plt.plot(t_sim_rx, D3_rx, t_sim_dyn, D3_dyn, '.')
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[119])
        plt.subplot(2,2,4)
        plt.plot(t_sim_rx, D4_rx, t_sim_dyn, D4_dyn, '.')
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[120])
        plt.show()

    if 44 in PLOTS:

        fig = plt.figure('Aerodynamic crosswind coefficients')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aerodynamic crosswind coefficients')
        plt.subplot(2,1,1)
        plt.plot(t_sim_rx, C1_rx, t_sim_dyn, C1_dyn, '.')
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[121])
        plt.subplot(2,1,2)
        plt.plot(t_sim_rx, C2_rx, t_sim_dyn, C2_dyn, '.')
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[122])
        plt.show()

    if 45 in PLOTS:

        fig = plt.figure('Aerodynamic lift coefficients')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aerodynamic lift coefficients')
        plt.subplot(3,2,1)
        plt.plot(t_sim_rx, L1_rx, t_sim_dyn, L1_dyn, '.')
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[123])
        plt.subplot(3,2,2)
        plt.plot(t_sim_rx, L2_rx, t_sim_dyn, L2_dyn, '.')
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[124])
        plt.subplot(3,2,3)
        plt.plot(t_sim_rx, L3_rx, t_sim_dyn, L3_dyn, '.')
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[125])
        plt.subplot(3,2,4)
        plt.plot(t_sim_rx, L4_rx, t_sim_dyn, L4_dyn, '.')
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[126])
        plt.subplot(3,2,5)
        plt.plot(t_sim_rx, L5_rx, t_sim_dyn, L5_dyn, '.')
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[127])
        plt.show()

    if 46 in PLOTS:

        fig = plt.figure('Aerodynamic roll coefficients')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aerodynamic roll coefficients')
        plt.subplot(3,2,1)
        plt.plot(t_sim_rx, l1_rx, t_sim_dyn, l1_dyn, '.')
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[128])
        plt.subplot(3,2,2)
        plt.plot(t_sim_rx, l2_rx, t_sim_dyn, l2_dyn, '.')
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[129])
        plt.subplot(3,2,3)
        plt.plot(t_sim_rx, l3_rx, t_sim_dyn, l3_dyn, '.')
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[130])
        plt.subplot(3,2,4)
        plt.plot(t_sim_rx, l4_rx, t_sim_dyn, l4_dyn, '.')
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[131])
        plt.subplot(3,2,5)
        plt.plot(t_sim_rx, l5_rx, t_sim_dyn, l5_dyn, '.')
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[132])
        plt.show()

    if 47 in PLOTS:

        fig = plt.figure('Aerodynamic pitch coefficients')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aerodynamic pitch coefficients')
        plt.subplot(3,2,1)
        plt.plot(t_sim_rx, m1_rx, t_sim_dyn, m1_dyn, '.')
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[133])
        plt.subplot(3,2,2)
        plt.plot(t_sim_rx, m2_rx, t_sim_dyn, m2_dyn, '.')
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[134])
        plt.subplot(3,2,3)
        plt.plot(t_sim_rx, m3_rx, t_sim_dyn, m3_dyn, '.')
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[135])
        plt.subplot(3,2,4)
        plt.plot(t_sim_rx, m4_rx, t_sim_dyn, m4_dyn, '.')
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[136])
        plt.subplot(3,2,5)
        plt.plot(t_sim_rx, m5_rx, t_sim_dyn, m5_dyn, '.')
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[137])
        plt.subplot(3,2,6)
        plt.plot(t_sim_rx, m5_rx, t_sim_dyn, m5_dyn, '.')
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[138])
        plt.show()

    if 48 in PLOTS:

        fig = plt.figure('Aerodynamic yaw coefficients')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aerodynamic yaw coefficients')
        plt.subplot(3,2,1)
        plt.plot(t_sim_rx, n1_rx, t_sim_dyn, n1_dyn, '.')
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[139])
        plt.subplot(3,2,2)
        plt.plot(t_sim_rx, n2_rx, t_sim_dyn, n2_dyn, '.')
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[140])
        plt.subplot(3,2,3)
        plt.plot(t_sim_rx, n3_rx, t_sim_dyn, n3_dyn, '.')
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[141])
        plt.subplot(3,2,4)
        plt.plot(t_sim_rx, n4_rx, t_sim_dyn, n4_dyn, '.')
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[142])
        plt.subplot(3,2,5)
        plt.plot(t_sim_rx, n5_rx, t_sim_dyn, n5_dyn, '.')
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[143])
        plt.subplot(3,2,6)
        plt.plot(t_sim_rx, n5_rx, t_sim_dyn, n5_dyn, '.')
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[144])
        plt.show()

    if 49 in PLOTS:

        fig = plt.figure('Propulsion-induced velocities')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the propulsion-induced velocities')
        plt.subplot(2,1,1)
        plt.plot(t_sim_rx, Vprop_rx, t_sim_dyn, Vprop_cm, '.')
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[145])
        plt.subplot(2,1,2)
        plt.plot(t_sim_rx, Vind_rx, t_sim_dyn, Vind_cm, '.')
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[146])
        plt.show()

    if 50 in PLOTS:

        if (len(t_sim_rx) != len(deltaa_tx)) or (len(t_sim_rx) != len(deltae_tx)) or (len(t_sim_rx) != len(deltaf_tx)) or (len(t_sim_rx) != len(deltar_tx)): 
            t_sim_rx = t_sim_rx[:-1]

        fig = plt.figure('Aerodynamic actuators commands')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aerodynamic actuators commands')
        plt.subplot(4,1,1)
        plt.plot(t_sim_rx, deltaa_tx)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[147])
        plt.subplot(4,1,2)
        plt.plot(t_sim_rx, deltae_tx)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[148])
        plt.subplot(4,1,3)
        plt.plot(t_sim_rx, deltaf_tx)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[149])
        plt.subplot(4,1,4)
        plt.plot(t_sim_rx, deltar_tx)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[150])
        plt.show()

    if 51 in PLOTS:

        if (len(t_sim_rx) != len(deltat_tx)) or (len(t_sim_rx) != len(deltam_tx)): 
            t_sim_rx = t_sim_rx[:-1]

        fig = plt.figure('Propulsive actuators commands')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the propulsive actuators commands')
        plt.subplot(2,1,1)
        plt.plot(t_sim_rx, deltat_tx)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[151])
        plt.subplot(2,1,2)
        plt.plot(t_sim_rx, deltam_tx)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[152])
        plt.show()

    if 52 in PLOTS:

        if (len(t_sim_rx) != len(deltaa_trim_tx)) or (len(t_sim_rx) != len(deltae_trim_tx)) or (len(t_sim_rx) != len(deltar_trim_tx)): 
            t_sim_rx = t_sim_rx[:-1]

        fig = plt.figure('Aerodynamic actuators trim commands')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aerodynamic actuators trim commands')
        plt.subplot(3,1,1)
        plt.plot(t_sim_rx, deltaa_trim_tx)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[153])
        plt.subplot(3,1,2)
        plt.plot(t_sim_rx, deltae_trim_tx)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[154])
        plt.subplot(3,1,3)
        plt.plot(t_sim_rx, deltar_trim_tx)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[155])
        plt.show()

    if 53 in PLOTS:

        fig = plt.figure('Position state error')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the position state error')
        plt.subplot(3,1,1)
        plt.plot(t_sim_rx, pn_er)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[5])
        plt.subplot(3,1,2)
        plt.plot(t_sim_rx, pe_er)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[6])
        plt.subplot(3,1,3)
        plt.plot(t_sim_rx, pd_er)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[7])
        plt.show()

    if 54 in PLOTS:

        fig = plt.figure('Attitude state error')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the attitude state error')
        plt.subplot(3,1,1)
        plt.plot(t_sim_rx, phi_er)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[14])
        plt.subplot(3,1,2)
        plt.plot(t_sim_rx, theta_er)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[15])
        plt.subplot(3,1,3)
        plt.plot(t_sim_rx, psi_er)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[16])
        plt.show()

    if 55 in PLOTS:

        fig = plt.figure('Linear velocity state error')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the linear velocity state error')
        plt.subplot(3,1,1)
        plt.plot(t_sim_rx, u_er)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[23])
        plt.subplot(3,1,2)
        plt.plot(t_sim_rx, v_er)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[24])
        plt.subplot(3,1,3)
        plt.plot(t_sim_rx, w_er)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[25])
        plt.show()

    if 56 in PLOTS:

        fig = plt.figure('Angular velocity state error')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the angular velocity state error')
        plt.subplot(3,1,1)
        plt.plot(t_sim_rx, p_er)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[35])
        plt.subplot(3,1,2)
        plt.plot(t_sim_rx, q_er)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[36])
        plt.subplot(3,1,3)
        plt.plot(t_sim_rx, r_er)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[37])
        plt.show()

    if 57 in PLOTS:

        fig = plt.figure('Aerodynamic actuation error')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aerodynamic actuation error')
        plt.subplot(2,2,1)
        plt.plot(t_sim_rx, deltaa_er)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[147])
        plt.subplot(2,2,2)
        plt.plot(t_sim_rx, deltae_er)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[148])
        plt.subplot(2,2,3)
        plt.plot(t_sim_rx, deltaf_er)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[149])
        plt.subplot(2,2,4)
        plt.plot(t_sim_rx, deltar_er)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[150])
        plt.show()

    if 58 in PLOTS:

        fig = plt.figure('Propulsive actuation error')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the propulsive actuation error')
        plt.subplot(2,1,1)
        plt.plot(t_sim_rx, deltat_er)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[151])
        plt.subplot(2,1,2)
        plt.plot(t_sim_rx, deltam_er)
        plt.xlim(t_sim_rx[0], t_sim_rx[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[152])
        plt.show()

    if 59 in PLOTS:

        fig = plt.figure('Position equilibrium')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the position equilibrium')
        plt.subplot(3,1,1)
        plt.plot(t_sim_eq, pn_eq)
        plt.xlim(t_sim_eq[0], t_sim_eq[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[5])
        plt.subplot(3,1,2)
        plt.plot(t_sim_eq, pe_eq)
        plt.xlim(t_sim_eq[0], t_sim_eq[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[6])
        plt.subplot(3,1,3)
        plt.plot(t_sim_eq, pd_eq)
        plt.xlim(t_sim_eq[0], t_sim_eq[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[7])
        plt.show()

    if 60 in PLOTS:

        fig = plt.figure('Attitude equilibrium')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the attitude equilibrium')
        plt.subplot(3,1,1)
        plt.plot(t_sim_eq, phi_eq)
        plt.xlim(t_sim_eq[0], t_sim_eq[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[14])
        plt.subplot(3,1,2)
        plt.plot(t_sim_eq, theta_eq)
        plt.xlim(t_sim_eq[0], t_sim_eq[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[15])
        plt.subplot(3,1,3)
        plt.plot(t_sim_eq, psi_eq)
        plt.xlim(t_sim_eq[0], t_sim_eq[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[16])
        plt.show()

    if 61 in PLOTS:

        fig = plt.figure('Linear velocity equilibrium')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the linear velocity equilibrium')
        plt.subplot(3,1,1)
        plt.plot(t_sim_eq, u_eq)
        plt.xlim(t_sim_eq[0], t_sim_eq[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[23])
        plt.subplot(3,1,2)
        plt.plot(t_sim_eq, v_eq)
        plt.xlim(t_sim_eq[0], t_sim_eq[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[24])
        plt.subplot(3,1,3)
        plt.plot(t_sim_eq, w_eq)
        plt.xlim(t_sim_eq[0], t_sim_eq[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[25])
        plt.show()

    if 62 in PLOTS:

        fig = plt.figure('Angular velocity equilibrium')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the angular velocity equilibrium')
        plt.subplot(3,1,1)
        plt.plot(t_sim_eq, p_eq)
        plt.xlim(t_sim_eq[0], t_sim_eq[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[35])
        plt.subplot(3,1,2)
        plt.plot(t_sim_eq, q_eq)
        plt.xlim(t_sim_eq[0], t_sim_eq[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[37])
        plt.subplot(3,1,3)
        plt.plot(t_sim_eq, r_eq)
        plt.xlim(t_sim_eq[0], t_sim_eq[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[38])
        plt.show()

    if 63 in PLOTS:

        fig = plt.figure('Aerodynamic actuation equilibrium')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aerodynamic actuation equilibrium')
        plt.subplot(2,2,1)
        plt.plot(t_sim_eq, deltaa_eq)
        plt.xlim(t_sim_eq[0], t_sim_eq[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[147])
        plt.subplot(2,2,2)
        plt.plot(t_sim_eq, deltae_eq)
        plt.xlim(t_sim_eq[0], t_sim_eq[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[148])
        plt.subplot(2,2,3)
        plt.plot(t_sim_eq, deltaf_eq)
        plt.xlim(t_sim_eq[0], t_sim_eq[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[149])
        plt.subplot(2,2,4)
        plt.plot(t_sim_eq, deltar_eq)
        plt.xlim(t_sim_eq[0], t_sim_eq[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[150])
        plt.show()

    if 64 in PLOTS:

        fig = plt.figure('Propulsive actuation equilibrium')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry('%dx%d+0+0' % (fullwidth, fullheight))
        plt.suptitle('Evolution of the propulsive actuation equilibrium')
        plt.subplot(2,1,1)
        plt.plot(t_sim_eq, deltat_eq)
        plt.xlim(t_sim_eq[0], t_sim_eq[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[151])
        plt.subplot(2,1,2)
        plt.plot(t_sim_eq, deltam_eq)
        plt.xlim(t_sim_eq[0], t_sim_eq[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[152])
        plt.show()

