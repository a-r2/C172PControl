import matplotlib as mpl
import matplotlib.pyplot as plt
import settings

from utils import *

''' PLOTTING PARAMETERS '''
mpl.rcParams['axes.labelsize'] = 14
mpl.rcParams['axes.titlesize'] = 16
mpl.rcParams['figure.autolayout'] = True
mpl.rcParams['figure.titlesize'] = 16
mpl.rcParams['lines.linestyle'] = '-'
mpl.rcParams['lines.linewidth'] = 1
mpl.rcParams['lines.marker'] = '.'
mpl.rcParams['lines.markersize'] = 6
mpl.rcParams['xtick.labelsize'] = 12
mpl.rcParams['ytick.labelsize'] = 12

''' PLOTTING FUNCTION '''
def plot(rxdata, txdata, dyndata):

    #i,j variables are used to index duplicated RX telemetry, thus avoiding possible conversion errors
    if settings.IMPERIAL_CONVERSION:
        rxdata = imperial_conversion(rxdata)
        i = 0 
    else:
        rxdata = SI_conversion(rxdata)
        i = 1

    if settings.ANGLES_CONVERSION:
        rxdata = angles_deg_conversion(rxdata)
        j = 0
    else:
        j = 1

    TimeRX         = rxdata[:,0]
    Timestep       = rxdata[:,1]
    DeltaLong      = rxdata[:,2]
    DeltaLat       = rxdata[:,3]
    DeltaDist      = rxdata[:,4]
    LongGNSS       = rxdata[:,5+j]
    LatGNSS        = rxdata[:,7+j]
    AltGNSS        = rxdata[:,9+i]
    BaroAltQFE     = rxdata[:,11+i]
    BaroAltQNH     = rxdata[:,13+i]
    RollAng        = rxdata[:,15+j]
    PitchAng       = rxdata[:,17+j]
    YawAng         = rxdata[:,19+j]
    AttackAng      = rxdata[:,21+j]
    SideSlipAng    = rxdata[:,23+j]
    PathAng        = rxdata[:,25+j]
    NVel           = rxdata[:,27]
    EVel           = rxdata[:,28]
    DVel           = rxdata[:,29]
    XVel           = rxdata[:,30]
    YVel           = rxdata[:,31]
    ZVel           = rxdata[:,32]
    XAeroVel       = rxdata[:,33]
    YAeroVel       = rxdata[:,34]
    ZAeroVel       = rxdata[:,35]
    NWindVel       = rxdata[:,36]
    EWindVel       = rxdata[:,37]
    DWindVel       = rxdata[:,38]
    RollAngDot     = rxdata[:,39]
    PitchAngDot    = rxdata[:,40]
    YawAngDot      = rxdata[:,41]
    RollVel        = rxdata[:,42]
    PitchVel       = rxdata[:,43]
    YawVel         = rxdata[:,44]
    AttackAngDot   = rxdata[:,45+j]
    SideSlipAngDot = rxdata[:,47+j]
    XAccel         = rxdata[:,49]
    YAccel         = rxdata[:,50]
    ZAccel         = rxdata[:,51]
    RollAccel      = rxdata[:,52]
    PitchAccel     = rxdata[:,53]
    YawAccel       = rxdata[:,54]
    XAeroForce     = rxdata[:,55]
    XExtForce      = rxdata[:,56]
    XGearForce     = rxdata[:,57]
    XPropForce     = rxdata[:,58]
    XTotalForce    = rxdata[:,59]
    YAeroForce     = rxdata[:,60]
    YExtForce      = rxdata[:,61]
    YGearForce     = rxdata[:,62]
    YPropForce     = rxdata[:,63]
    YTotalForce    = rxdata[:,64]
    ZAeroForce     = rxdata[:,65]
    ZExtForce      = rxdata[:,66]
    ZGearForce     = rxdata[:,67]
    ZPropForce     = rxdata[:,68]
    ZTotalForce    = rxdata[:,69]
    PAeroMoment    = rxdata[:,70]
    PExtMoment     = rxdata[:,71]
    PGearMoment    = rxdata[:,72]
    PPropMoment    = rxdata[:,73]
    PTotalMoment   = rxdata[:,74]
    QAeroMoment    = rxdata[:,75]
    QExtMoment     = rxdata[:,76]
    QGearMoment    = rxdata[:,77]
    QPropMoment    = rxdata[:,78]
    QTotalMoment   = rxdata[:,79]
    RAeroMoment    = rxdata[:,80]
    RExtMoment     = rxdata[:,81]
    RGearMoment    = rxdata[:,82]
    RPropMoment    = rxdata[:,83]
    RTotalMoment   = rxdata[:,84]
    RAilPos        = rxdata[:,85+j]
    NormRAilPos    = rxdata[:,87]
    LAilPos        = rxdata[:,88+j]
    NormLAilPos    = rxdata[:,90]
    ElevsPos       = rxdata[:,91+j]
    NormElevsPos   = rxdata[:,93]
    FlapsPos       = rxdata[:,94+j]
    NormFlapsPos   = rxdata[:,96]
    RudderPos      = rxdata[:,97+j]
    NormRudderPos  = rxdata[:,99]
    EngThrottPos   = rxdata[:,100]
    EngMixPos      = rxdata[:,101]
    UpDown         = rxdata[:,102]
    WOW1           = rxdata[:,103]
    WOW2           = rxdata[:,104]
    WOW3           = rxdata[:,105]
    SpanVel        = rxdata[:,106]
    ChordVel       = rxdata[:,107]
    AltMACSpan     = rxdata[:,108]
    DynPress       = rxdata[:,109]
    DynPressXZ     = rxdata[:,110]
    DynPressProp   = rxdata[:,111]
    DynPressInd    = rxdata[:,112]
    NormStallHyst  = rxdata[:,113]
    AdvRatio       = rxdata[:,114]
    PropRPM        = rxdata[:,115]
    
    NormAilCmd       = txdata[:,0]
    NormElevsCmd     = txdata[:,1]
    NormFlapsCmd     = txdata[:,2]
    NormRudderCmd    = txdata[:,3]
    NormEngThrottCmd = txdata[:,4]
    NormEngMixCmd    = txdata[:,5]
    NormRollTrimCmd  = txdata[:,6]
    NormPitchTrimCmd = txdata[:,7]
    NormYawTrimCmd   = txdata[:,8]

    TimeDyn = dyndata[:,0]
    D       = dyndata[:,1]
    S       = dyndata[:,2]
    L       = dyndata[:,3]
    l       = dyndata[:,4]
    m       = dyndata[:,5]
    n       = dyndata[:,6]

    Faero = np.zeros((dyndata.shape[0], 3))
    Maero = np.zeros((dyndata.shape[0], 3))
    for i in range(min(rxdata.shape[0], dyndata.shape[0])):
        Faero[i,:] = wind_to_body(AttackAng[i], SideSlipAng[i]).rotate([-D[i], S[i], -L[i]])
        Maero[i,:] = [l[i], m[i], n[i]]

    labels = list()
    for key, value in settings.TELEM_RX_PLOT.items():
        labels.append(key + ' ' + value[1] + ' [' + value[0][0] + ']')
    for key, value in settings.TELEM_TX_PLOT.items():
        labels.append(key + ' ' + value[1] + ' [' + value[0][0] + ']')

    if 1 in settings.PLOTS:

        fig = plt.figure('Aircraft relative position in ECEF frame')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry("%dx%d+0+0" % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aircraft relative position in ECEF frame')
        plt.subplot(3,1,1)
        plt.plot(TimeRX, DeltaLong)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[2])
        plt.subplot(3,1,2)
        plt.plot(TimeRX, DeltaLat)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[3])
        plt.subplot(3,1,3)
        plt.plot(TimeRX, DeltaDist)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[4])
        plt.show()
        
    if 2 in settings.PLOTS:

        fig = plt.figure('Aircraft GNSS position in ECEF frame')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry("%dx%d+0+0" % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aircraft GNSS position in ECEF frame')
        plt.subplot(3,1,1)
        plt.plot(TimeRX, LongGNSS)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[5])
        plt.subplot(3,1,2)
        plt.plot(TimeRX, LatGNSS)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[6])
        plt.subplot(3,1,3)
        plt.plot(TimeRX, AltGNSS)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[7])
        plt.show()

    if 3 in settings.PLOTS:

        fig = plt.figure('Aircraft barometric altitude')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry("%dx%d+0+0" % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aircraft barometric altitude')
        plt.subplot(2,1,1)
        plt.plot(TimeRX, BaroAltQFE)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[8])
        plt.subplot(2,1,2)
        plt.plot(TimeRX, BaroAltQNH)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[9])
        plt.show()
        
    if 4 in settings.PLOTS:

        fig = plt.figure('Aircraft attitude in body frame')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry("%dx%d+0+0" % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aircraft attitude in body frame')
        plt.subplot(3,1,1)
        plt.plot(TimeRX, RollAng)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[10])
        plt.subplot(3,1,2)
        plt.plot(TimeRX, PitchAng)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[11])
        plt.subplot(3,1,3)
        plt.plot(TimeRX, YawAng)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[12])
        plt.show()

    if 5 in settings.PLOTS:

        fig = plt.figure('Wind angles')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry("%dx%d+0+0" % (fullwidth, fullheight))
        plt.suptitle('Evolution of the wind angles')
        plt.subplot(2,1,1)
        plt.plot(TimeRX, AttackAng)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[13])
        plt.subplot(2,1,2)
        plt.plot(TimeRX, SideSlipAng)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[14])
        plt.show()

    if 6 in settings.PLOTS:

        fig = plt.figure('Aircraft path angle')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry("%dx%d+0+0" % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aircraft path angle')
        plt.subplot(1,1,1)
        plt.plot(TimeRX, PathAng)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[15])
        plt.show()

    if 7 in settings.PLOTS:

        fig = plt.figure('Aircraft linear velocity in NED frame')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry("%dx%d+0+0" % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aircraft linear velocity in NED frame')
        plt.subplot(3,1,1)
        plt.plot(TimeRX, NVel)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[16])
        plt.subplot(3,1,2)
        plt.plot(TimeRX, EVel)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[17])
        plt.subplot(3,1,3)
        plt.plot(TimeRX, DVel)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[18])
        plt.show()

    if 8 in settings.PLOTS:

        fig = plt.figure('Aircraft linear velocity in body frame')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry("%dx%d+0+0" % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aircraft linear velocity in body frame')
        plt.subplot(3,1,1)
        plt.plot(TimeRX, XVel)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[19])
        plt.subplot(3,1,2)
        plt.plot(TimeRX, YVel)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[20])
        plt.subplot(3,1,3)
        plt.plot(TimeRX, ZVel)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[21])
        plt.show()

    if 9 in settings.PLOTS:

        fig = plt.figure('Aircraft linear velocity in wind frame')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry("%dx%d+0+0" % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aircraft linear velocity in wind frame')
        plt.subplot(3,1,1)
        plt.plot(TimeRX, XAeroVel)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[22])
        plt.subplot(3,1,2)
        plt.plot(TimeRX, YAeroVel)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[23])
        plt.subplot(3,1,3)
        plt.plot(TimeRX, ZAeroVel)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[24])
        plt.show()

    if 10 in settings.PLOTS:

        fig = plt.figure('Wind linear velocity in NED frame')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry("%dx%d+0+0" % (fullwidth, fullheight))
        plt.suptitle('Evolution of the wind linear velocity in NED frame')
        plt.subplot(3,1,1)
        plt.plot(TimeRX, NWindVel)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[25])
        plt.subplot(3,1,2)
        plt.plot(TimeRX, EWindVel)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[26])
        plt.subplot(3,1,3)
        plt.plot(TimeRX, DWindVel)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[27])
        plt.show()

    if 11 in settings.PLOTS:

        fig = plt.figure('Aircraft attitude rate of change in body frame')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry("%dx%d+0+0" % (fullwidth, fullheight))
        plt.suptitle('Evolution of the rate of change of the aircraft attitude in body frame')
        plt.subplot(3,1,1)
        plt.plot(TimeRX, RollAngDot)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[28])
        plt.subplot(3,1,2)
        plt.plot(TimeRX, PitchAngDot)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[29])
        plt.subplot(3,1,3)
        plt.plot(TimeRX, YawAngDot)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[30])
        plt.show()

    if 12 in settings.PLOTS:

        fig = plt.figure('Aircraft rotational velocity in body frame')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry("%dx%d+0+0" % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aircraft rotational velocity in body frame')
        plt.subplot(3,1,1)
        plt.plot(TimeRX, RollVel)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[31])
        plt.subplot(3,1,2)
        plt.plot(TimeRX, PitchVel)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[32])
        plt.subplot(3,1,3)
        plt.plot(TimeRX, YawVel)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[33])
        plt.show()

    if 13 in settings.PLOTS:

        fig = plt.figure('Wind angles rate of change')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry("%dx%d+0+0" % (fullwidth, fullheight))
        plt.suptitle('Evolution of the rate of change of the wind angles')
        plt.subplot(2,1,1)
        plt.plot(TimeRX, AttackAngDot)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[34])
        plt.subplot(2,1,2)
        plt.plot(TimeRX, SideSlipAngDot)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[35])
        plt.show()

    if 14 in settings.PLOTS:

        fig = plt.figure('Aircraft linear acceleration in body frame')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry("%dx%d+0+0" % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aircraft linear acceleration in body frame')
        plt.subplot(3,1,1)
        plt.plot(TimeRX, XAccel)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[36])
        plt.subplot(3,1,2)
        plt.plot(TimeRX, YAccel)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[37])
        plt.subplot(3,1,3)
        plt.plot(TimeRX, ZAccel)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[38])
        plt.show()

    if 15 in settings.PLOTS:

        fig = plt.figure('Aircraft rotational acceleration in body frame')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry("%dx%d+0+0" % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aircraft rotational acceleration in body frame')
        plt.subplot(3,1,1)
        plt.plot(TimeRX, RollAccel)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[39])
        plt.subplot(3,1,2)
        plt.plot(TimeRX, PitchAccel)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[40])
        plt.subplot(3,1,3)
        plt.plot(TimeRX, YawAccel)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[41])
        plt.show()
    
    if 16 in settings.PLOTS:

        fig = plt.figure('Aircraft aerodynamic force in body frame')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry("%dx%d+0+0" % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aircraft aerodynamic force in body frame')
        plt.subplot(3,1,1)
        plt.plot(TimeRX, XAeroForce, TimeDyn, Faero[:,0],'.--')
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[42])
        plt.subplot(3,1,2)
        plt.plot(TimeRX, YAeroForce, TimeDyn, Faero[:,1],'.--')
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[47])
        plt.subplot(3,1,3)
        plt.plot(TimeRX, ZAeroForce, TimeDyn, Faero[:,2],'.--')
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[52])
        plt.show()
    
    if 17 in settings.PLOTS:

        fig = plt.figure('Aircraft external force in body frame')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry("%dx%d+0+0" % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aircraft external force in body frame')
        plt.subplot(3,1,1)
        plt.plot(TimeRX, XExtForce)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[43])
        plt.subplot(3,1,2)
        plt.plot(TimeRX, YExtForce)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[48])
        plt.subplot(3,1,3)
        plt.plot(TimeRX, ZExtForce)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[53])
        plt.show()
    
    if 18 in settings.PLOTS:

        fig = plt.figure('Aircraft gear force in body frame')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry("%dx%d+0+0" % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aircraft gear force in body frame')
        plt.subplot(3,1,1)
        plt.plot(TimeRX, XGearForce)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[44])
        plt.subplot(3,1,2)
        plt.plot(TimeRX, YGearForce)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[49])
        plt.subplot(3,1,3)
        plt.plot(TimeRX, ZGearForce)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[54])
        plt.show()
    
    if 19 in settings.PLOTS:

        fig = plt.figure('Aircraft propeller force in body frame')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry("%dx%d+0+0" % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aircraft propeller force in body frame')
        plt.subplot(3,1,1)
        plt.plot(TimeRX, XPropForce)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[45])
        plt.subplot(3,1,2)
        plt.plot(TimeRX, YPropForce)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[50])
        plt.subplot(3,1,3)
        plt.plot(TimeRX, ZPropForce)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[55])
        plt.show()
    
    if 20 in settings.PLOTS:

        fig = plt.figure('Aircraft total force in body frame')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry("%dx%d+0+0" % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aircraft total force in body frame')
        plt.subplot(3,1,1)
        plt.plot(TimeRX, XTotalForce)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[46])
        plt.subplot(3,1,2)
        plt.plot(TimeRX, YTotalForce)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[51])
        plt.subplot(3,1,3)
        plt.plot(TimeRX, ZTotalForce)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[56])
        plt.show()
    
    if 21 in settings.PLOTS:

        fig = plt.figure('Aircraft aerodynamic moment in body frame')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry("%dx%d+0+0" % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aircraft aerodynamic moment in body frame')
        plt.subplot(3,1,1)
        plt.plot(TimeRX, PAeroMoment, TimeDyn, Maero[:,0],'.--')
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[57])
        plt.subplot(3,1,2)
        plt.plot(TimeRX, QAeroMoment, TimeDyn, Maero[:,1],'.--')
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[62])
        plt.subplot(3,1,3)
        plt.plot(TimeRX, RAeroMoment, TimeDyn, Maero[:,2],'.--')
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[67])
        plt.show()
    
    if 22 in settings.PLOTS:

        fig = plt.figure('Aircraft external moment in body frame')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry("%dx%d+0+0" % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aircraft external moment in body frame')
        plt.subplot(3,1,1)
        plt.plot(TimeRX, PExtMoment)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[58])
        plt.subplot(3,1,2)
        plt.plot(TimeRX, QExtMoment)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[63])
        plt.subplot(3,1,3)
        plt.plot(TimeRX, RExtMoment)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[68])
        plt.show()
    
    if 23 in settings.PLOTS:

        fig = plt.figure('Aircraft gear moment in body frame')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry("%dx%d+0+0" % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aircraft gear moment in body frame')
        plt.subplot(3,1,1)
        plt.plot(TimeRX, PGearMoment)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[59])
        plt.subplot(3,1,2)
        plt.plot(TimeRX, QGearMoment)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[64])
        plt.subplot(3,1,3)
        plt.plot(TimeRX, RGearMoment)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[69])
        plt.show()
    
    if 24 in settings.PLOTS:

        fig = plt.figure('Aircraft propeller moment in body frame')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry("%dx%d+0+0" % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aircraft propeller moment in body frame')
        plt.subplot(3,1,1)
        plt.plot(TimeRX, PPropMoment)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[60])
        plt.subplot(3,1,2)
        plt.plot(TimeRX, QPropMoment)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[65])
        plt.subplot(3,1,3)
        plt.plot(TimeRX, RPropMoment)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[70])
        plt.show()
    
    if 25 in settings.PLOTS:

        fig = plt.figure('Aircraft total moment in body frame')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry("%dx%d+0+0" % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aircraft total moment in body frame')
        plt.subplot(3,1,1)
        plt.plot(TimeRX, PTotalMoment)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[61])
        plt.subplot(3,1,2)
        plt.plot(TimeRX, QTotalMoment)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[66])
        plt.subplot(3,1,3)
        plt.plot(TimeRX, RTotalMoment)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[71])
        plt.show()
    
    if 26 in settings.PLOTS:

        fig = plt.figure('Aerodynamic actuators position')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry("%dx%d+0+0" % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aerodynamic actuators position')
        plt.subplot(5,1,1)
        plt.plot(TimeRX, RAilPos)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[72])
        plt.subplot(5,1,2)
        plt.plot(TimeRX, LAilPos)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[74])
        plt.subplot(5,1,3)
        plt.plot(TimeRX, ElevsPos)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[76])
        plt.subplot(5,1,4)
        plt.plot(TimeRX, FlapsPos)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[78])
        plt.subplot(5,1,5)
        plt.plot(TimeRX, RudderPos)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[80])
        plt.show()

    if 27 in settings.PLOTS:

        fig = plt.figure('Aerodynamic actuators normalized position')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry("%dx%d+0+0" % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aerodynamic actuators normalized position')
        plt.subplot(5,1,1)
        plt.plot(TimeRX, NormRAilPos)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[73])
        plt.subplot(5,1,2)
        plt.plot(TimeRX, NormLAilPos)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[75])
        plt.subplot(5,1,3)
        plt.plot(TimeRX, NormElevsPos)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[77])
        plt.subplot(5,1,4)
        plt.plot(TimeRX, NormFlapsPos)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[79])
        plt.subplot(5,1,5)
        plt.plot(TimeRX, NormRudderPos)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[81])
        plt.show()

    if 28 in settings.PLOTS:

        fig = plt.figure('Propulsive actuators normalized position')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry("%dx%d+0+0" % (fullwidth, fullheight))
        plt.suptitle('Evolution of the propulsive actuators normalized position')
        plt.subplot(2,1,1)
        plt.plot(TimeRX, EngThrottPos)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[82])
        plt.subplot(2,1,2)
        plt.plot(TimeRX, EngMixPos)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[83])
        plt.show()

    if 29 in settings.PLOTS:

        fig = plt.figure('Upside-down state')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry("%dx%d+0+0" % (fullwidth, fullheight))
        plt.suptitle('Evolution of the upside-down state')
        plt.subplot(1,1,1)
        plt.plot(TimeRX, UpDown)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[84])
        plt.show()

    if 30 in settings.PLOTS:

        fig = plt.figure('Weight-on-wheel states')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry("%dx%d+0+0" % (fullwidth, fullheight))
        plt.suptitle('Evolution of the weight-on-wheel states')
        plt.subplot(3,1,1)
        plt.plot(TimeRX, WOW1)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[85])
        plt.subplot(3,1,2)
        plt.plot(TimeRX, WOW2)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[86])
        plt.subplot(3,1,3)
        plt.plot(TimeRX, WOW3)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[87])
        plt.show()

    if 31 in settings.PLOTS:

        fig = plt.figure('Aerodynamic ratios')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry("%dx%d+0+0" % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aerodynamic ratios')
        plt.subplot(3,1,1)
        plt.plot(TimeRX, SpanVel)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[88])
        plt.subplot(3,1,2)
        plt.plot(TimeRX, ChordVel)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[89])
        plt.subplot(3,1,3)
        plt.plot(TimeRX, AltMACSpan)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[90])
        plt.show()

    if 32 in settings.PLOTS:

        fig = plt.figure('Dynamic pressure')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry("%dx%d+0+0" % (fullwidth, fullheight))
        plt.suptitle('Evolution of the dynamic pressure')
        plt.subplot(2,2,1)
        plt.plot(TimeRX, DynPress)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[91])
        plt.subplot(2,2,2)
        plt.plot(TimeRX, DynPressXZ)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[92])
        plt.subplot(2,2,3)
        plt.plot(TimeRX, DynPressProp)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[93])
        plt.subplot(2,2,4)
        plt.plot(TimeRX, DynPressInd)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[94])
        plt.show()

    if 33 in settings.PLOTS:

        fig = plt.figure('Normalized stall hysteresis')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry("%dx%d+0+0" % (fullwidth, fullheight))
        plt.suptitle('Evolution of the normalized stall hysteresis')
        plt.subplot(1,1,1)
        plt.plot(TimeRX, NormStallHyst)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[95])
        plt.show()

    if 34 in settings.PLOTS:

        fig = plt.figure('Advance ratio')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry("%dx%d+0+0" % (fullwidth, fullheight))
        plt.suptitle('Evolution of the advance ratio')
        plt.subplot(1,1,1)
        plt.plot(TimeRX, AdvRatio)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[96])
        plt.show()

    if 35 in settings.PLOTS:

        fig = plt.figure('Propeller revolutions')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry("%dx%d+0+0" % (fullwidth, fullheight))
        plt.suptitle('Evolution of the propeller revolutions per minute')
        plt.subplot(1,1,1)
        plt.plot(TimeRX, PropRPM)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[97])
        plt.show()

    if 36 in settings.PLOTS:

        if (len(TimeRX) != len(NormAilCmd)) or (len(TimeRX) != len(NormElevsCmd)) or (len(TimeRX) != len(NormFlapsCmd)) or (len(TimeRX) != len(NormRudderCmd)): 
            TimeRX = TimeRX[:-1]

        fig = plt.figure('Aerodynamic actuators commands')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry("%dx%d+0+0" % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aerodynamic actuators commands')
        plt.subplot(4,1,1)
        plt.plot(TimeRX, NormAilCmd)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[98])
        plt.subplot(4,1,2)
        plt.plot(TimeRX, NormElevsCmd)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[99])
        plt.subplot(4,1,3)
        plt.plot(TimeRX, NormFlapsCmd)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[100])
        plt.subplot(4,1,4)
        plt.plot(TimeRX, NormRudderCmd)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[101])
        plt.show()

    if 37 in settings.PLOTS:

        if (len(TimeRX) != len(NormEngThrottCmd)) or (len(TimeRX) != len(NormEngMixCmd)): 
            TimeRX = TimeRX[:-1]

        fig = plt.figure('Propulsive actuators commands')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry("%dx%d+0+0" % (fullwidth, fullheight))
        plt.suptitle('Evolution of the propulsive actuators commands')
        plt.subplot(2,1,1)
        plt.plot(TimeRX, NormEngThrottCmd)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[102])
        plt.subplot(2,1,2)
        plt.plot(TimeRX, NormEngMixCmd)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[103])
        plt.show()

    if 38 in settings.PLOTS:

        if (len(TimeRX) != len(NormRollTrimCmd)) or (len(TimeRX) != len(NormPitchTrimCmd)) or (len(TimeRX) != len(NormYawTrimCmd)): 
            TimeRX = TimeRX[:-1]

        fig = plt.figure('Aerodynamic actuators trim commands')
        fullwidth, fullheight = fig.canvas.manager.window.winfo_screenwidth(), fig.canvas.manager.window.winfo_screenheight()
        fig.canvas.manager.window.wm_geometry("%dx%d+0+0" % (fullwidth, fullheight))
        plt.suptitle('Evolution of the aerodynamic actuators trim commands')
        plt.subplot(3,1,1)
        plt.plot(TimeRX, NormRollTrimCmd)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[104])
        plt.subplot(3,1,2)
        plt.plot(TimeRX, NormPitchTrimCmd)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[105])
        plt.subplot(3,1,3)
        plt.plot(TimeRX, NormYawTrimCmd)
        plt.xlim(TimeRX[0], TimeRX[-1])
        plt.grid()
        plt.xlabel(labels[0])
        plt.ylabel(labels[106])
        plt.show()

