''' ACTUATION '''
# 0 / 'random' | 'AL', 'ANL'
# 1 / 'acker'  | 'AL' (https://en.wikipedia.org/wiki/Ackermann%27s_formula)
ACT_TYPE = 'acker'

''' CONFIG '''
CFG_IP_ADDRESS = 'localhost'
CFG_PORT       = 60000 #config port

''' CONTROL MODEL '''
# 0 / 'AL'  | Analytic linear control model
# 1 / 'ANL' | Analytic non-linear control model
CM_TYPE = 'AL'

''' CSV LOGGING '''
CM_LOG_FILENAME       = 'control_model_log'
DYN_LOG_FILENAME      = 'dynamics_log'
SP_LOG_FILENAME       = 'setpoint_log'
TELEM_RX_LOG_FILENAME = 'telemetry_rx_log'
TELEM_TX_LOG_FILENAME = 'telemetry_tx_log'
CSV_LOG_DIR           = 'sim_logs'

''' PLOTTING '''
ANGLES_CONVERSION   = False #from radians to degrees
IMPERIAL_CONVERSION = True #from SI to imperial
PLOTS               = (16, 19, 21) #list(range(1,39))

''' SETPOINT '''
SP_TYPE = 'constant' #0 = 'constant'
SP_INIT = (0, 0, 100, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0) #(pn_sp, pe_sp, pd_sp, q0_sp, q1_sp, q2_sp, q3_sp, u_sp, v_sp, w_sp, p_sp, q_sp, r_sp)

''' SCENARIO '''
#https://wiki.flightgear.org/Command_line_options
AIRPORT                = 'LEBA' #ICAO code
ALTITUDE_START         = 100 #[ft]
FG_AIRCRAFT_OPTIONS    = ('--disable-auto-coordination', '--disable-fuel-freeze')
FG_ENVIRONMENT_OPTIONS = ('--disable-ai-models', '--disable-ai-traffic', '--disable-clouds', '--disable-clouds3d', '--disable-distance-attenuation', '--disable-fullscreen', '--disable-horizon-effect', '--disable-random-buildings', '--disable-random-objects', '--disable-random-vegetation', '--disable-real-weather-fetch', '--disable-sound', '--disable-specular-highlight', '--disable-splash-screen', '--fog-disable')
LATITUDE_START         = 0 #[deg]
LONGITUDE_START        = 0 #[deg]
PHI_START              = 0 #[deg]
PSI_START              = 0 #[deg]
RUNWAY                 = '03' #double digit
SCEN_TYPE              = 'cruise'
SEASON                 = 'summer'
THETA_START            = 0 #[deg]
TIME_OF_DAY            = 'morning'
TURBULENCE             = 0 #[0, 1] -> 0 = calm, 1 = severe
U_START                = 100 #[ft/s]
V_START                = 0.0000 #[ft/s]
VISIBILITY             = 10000 #[m]
W_START                = 0.0000 #[ft/s]
WIND                   = '0@0' #direction [deg] @ speed [knots]

''' SIMULATION '''
ACT_HZ             = 10 #actuation frequency
CM_HZ              = 10 #control model frequency
FPS                = 100 #frames per real time second
MAX_TIME_PER_FRAME = 1 / 100 #timestep per video frame
MODEL_HZ           = 100 #model iterations per simulation second
SIM_ITER_NUM       = 2 #number of simulation iterations(only applicable when SIM_TYPE = 0/'recursive')
SIM_RATE           = 1 #simulation rate with respect to real time (recommended: 0.125, 0.25, 0.5, 1)
SIM_TYPE           = 'multiple' #simulation type: 0 = 'single', 1 = 'multiple'

''' TELEMETRY '''
TELEM_RX_BUFFER_SIZE = 10240 #long enough to allocate a complete frame
TELEM_RX_IP_ADDRESS  = 'localhost'
TELEM_RX_PORT        = 60001 #receiving link port
TELEM_TX_IP_ADDRESS  = 'localhost'
TELEM_TX_PORT        = 60002 #transmitting link port
TELEM_WAIT           = 10 #wait in seconds after FlightGear start before trying to establish telemetry communication

