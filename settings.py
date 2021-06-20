''' SIMULATION '''
SIM_TYPE = 'recursive' #simulation type: 0 = 'single', 1 = 'recursive'
SIM_ITER_NUM = 10 #number of simulation iterations(only applicable when SIM_TYPE = 0/'recursive')
SIM_RATE = 1 #simulation rate with respect to real time (recommended: 0.125, 0.25, 0.5, 1)
FPS = 100 #frames per real time second
MAX_TIME_PER_FRAME = 1/100 #timestep per video frame
MODEL_HZ = 100 #model iterations per simulation second
ACT_HZ = 10 #actuation frequency
CM_HZ = 10 #control model frequency
CFG_IP_ADDRESS = 'localhost'
CFG_PORT = 60000 #config port

''' ACTUATION '''
ACT_TYPE = 'random' #'random'

''' CONTROL MODEL '''
CM_TYPE = 'ANL' #'ANL', 'AL'
CM_LOG_FILENAME = 'control_model_log'

''' DYNAMICS '''
DYN_LOG_FILENAME = 'dynamics_log'

''' EQUILIBRIUM POINT '''
EQ_TYPE = 'constant'
EQ_POINT_INIT = (0,0,100,0,0,0,100,0,0,0,0,0) #(pn_eq, pe_eq, pd_eq, phi_eq, theta_eq, psi_eq, u_eq, v_eq, w_eq, p_eq, q_eq, r_eq)
EQ_LOG_FILENAME = 'eq_point_log'

''' KINEMATICS '''
KIN_LOG_FILENAME = 'kinematics_log'

''' INITIALIZATION '''

''' TELEMETRY '''
TELEM_LOG_FILENAME   = 'telemetry_log'
TELEM_RX_IP_ADDRESS  = 'localhost'
TELEM_RX_PORT        = 60001 #receiving link port
TELEM_RX_BUFFER_SIZE = 10240 #enough to allocate a complete frame
TELEM_TX_IP_ADDRESS  = 'localhost'
TELEM_TX_PORT        = 60002 #transmitting link port
TELEM_WAIT           = 10 #wait in seconds after FlightGear start before establishing telemetry communication

''' NAVIGATION MAP '''
AC_ICON_SIZE = 0.1
NAV_FIG_SIZE = (9.56, 4.61)
LAT_DELTA_INIT = 1
LAT_DELTA_UPDT= 0.05 * LAT_DELTA_INIT
LONG_DELTA_INIT = 2.63
LONG_DELTA_UPDT = 0.05 * LONG_DELTA_INIT
NAVMAP_PERIOD = 1
NAVMAP_TITLE = 'Navigation map'
PLOT_NAVMAP = True
WP_ICON_SIZE = 16

''' MISSION '''
WP_TUPLE =  (
            (-4.853942, 37.834722),
            (-4.853942, 37.837855),
            (-4.838241, 37.840979),
            )

''' PLOTTING '''
PLOTS = [16, 19, 21] #list(range(1,39))
IMPERIAL_CONVERSION = True #from SI to imperial
ANGLES_CONVERSION = False #from radians to degrees

''' SCENARIO '''
#https://wiki.flightgear.org/Command_line_options
SCENARIO_TYPE         = 'cruise'

AIRPORT               = 'LEBA' #ICAO code
RUNWAY                = '03' #double digit
CFG_PROTOCOL_FILENAME = 'py2fg_cfg'
RX_PROTOCOL_FILENAME  = 'fg2py'
SEASON                = 'summer'
TIME_OF_DAY           = 'morning'
TURBULENCE            = 0 #[0, 1] -> 0 = calm, 1 = severe
TX_PROTOCOL_FILENAME  = 'py2fg_act'
VISIBILITY            = 10000 #[m]
WIND                  = '0@0' #direction [deg] @ speed [knots]
LONGITUDE_START       = 0 #[deg]
LATITUDE_START        = 0 #[deg]
ALTITUDE_START        = 1000 #[m] if --units-meters else [ft]
PHI_START             = 0 #[deg]
THETA_START           = 0 #[deg]
PSI_START             = 0 #[deg]
U_START               = 100 #[m/s] if --units-meters else [ft/s]
V_START               = 0.0000 #[m/s] if --units-meters else [ft/s]
W_START               = 0.0000 #[m/s] if --units-meters else [ft/s]

FG_AIRCRAFT_OPTIONS =  ["--disable-auto-coordination", "--disable-fuel-freeze", "--notrim"]

FG_ENVIRONMENT_OPTIONS = ["--disable-ai-models", "--disable-ai-traffic", "--disable-clouds", "--disable-clouds3d", "--disable-distance-attenuation", "--disable-fullscreen", "--disable-horizon-effect", "--disable-random-buildings", "--disable-random-objects", "--disable-random-vegetation", "--disable-real-weather-fetch", "--disable-sound", "--disable-specular-highlight", "--disable-splash-screen", "--fog-disable", "--units-meters"]
