''' ACTUATION '''
# 0 / 'random' | 'AL', 'ANL'
# 1 / 'fsfb'   | 'AL', 'LANL' (https://en.wikipedia.org/wiki/Full_state_feedback)
# 2 / 'lqr'    | 'AL', 'LANL' (https://en.wikipedia.org/wiki/Linear%E2%80%93quadratic_regulator)
# 3 / 'mpc'    | 'AL', 'ANL', 'LANL' (https://en.wikipedia.org/wiki/Model_predictive_control)
ACT_TYPE     = 'fsfb'
MPC_HORSTEPS = 2 #>=2

''' CONFIG '''
CFG_IP_ADDRESS = 'localhost'
CFG_PORT       = 60000 #config port

''' CONTROL MODEL '''
# 0 / 'AL'   | Analytic linear control model
# 1 / 'ANL'  | Analytic non-linear control model
# 2 / 'LANL' | Linearized analytic non-linear control model
CM_TYPE       = 'AL'
LAT_STATE_IND = (3, 5, 7, 9, 11) #Indexes of reduced control model state variables (phi, psi, v, p, r)
LAT_INPUT_IND = (0, 3) #Indexes of reduced control model input variables (deltaa, deltar)
LON_STATE_IND = (2, 4, 6, 8, 10) #Indexes of reduced control model state variables (pd, theta, u, w, q)
LON_INPUT_IND = (1, 4) #Indexes of reduced control model input variables (deltae, deltat)

''' CSV LOGGING '''
CM_LOG_FILENAME       = 'control_model_log'
DYN_LOG_FILENAME      = 'dynamics_log'
EQ_LOG_FILENAME       = 'eqpoint_log'
SP_LOG_FILENAME       = 'setpoint_log'
TELEM_RX_LOG_FILENAME = 'telemetry_rx_log'
TELEM_TX_LOG_FILENAME = 'telemetry_tx_log'
CSV_LOG_DIR           = 'sim_logs'

''' EQUILIBRIUM '''
EQ_STATE_IND = (3, 4, 6, 7, 8, 9, 10, 11) #Indexes of control model state variables used in order to find the equilibrium point (phi, theta, u, v, w, p, q, r)
EQ_INPUT_IND = (0, 1, 3, 4) #Indexes of control model input variables used in order to find the equilibrium point (deltaa, deltae, deltar, deltat)

''' PLOTTING '''
ANGLES_CONVERSION   = False #from radians to degrees
IMPERIAL_CONVERSION = True #from SI to imperial
#PLOTS               = (53, 54, 55, 56, 57, 58) #list(range(1,64))
PLOTS               = (36,) #list(range(1,64))

''' SCENARIO '''
#https://wiki.flightgear.org/Command_line_options
AIRPORT                = 'LEBA' #ICAO code
ALTITUDE_START         = 3310 #[ft]
FG_AIRCRAFT_OPTIONS    = ('--disable-auto-coordination', '--enable-fuel-freeze')
FG_ENVIRONMENT_OPTIONS = ('--disable-ai-models', '--disable-ai-traffic', '--disable-clouds', '--disable-clouds3d', '--disable-distance-attenuation', '--disable-fullscreen', '--disable-horizon-effect', '--disable-random-buildings', '--disable-random-objects', '--disable-random-vegetation', '--disable-real-weather-fetch', '--disable-sound', '--disable-specular-highlight', '--disable-splash-screen', '--fog-disable')
LATITUDE_START         = 0 #[deg]
LONGITUDE_START        = 0 #[deg]
PHI_START              = 0 #[deg]
PSI_START              = 0 #[deg]
RUNWAY                 = '03' #double digit
SEASON                 = 'summer'
THETA_START            = 0 #[deg]
TIME_OF_DAY            = 'morning'
TURBULENCE             = 0 #[0, 1] -> 0 = calm, 1 = severe
U_START                = 100 #[ft/s]
V_START                = 0.0000 #[ft/s]
VISIBILITY             = 10000 #[m]
W_START                = 0.0000 #[ft/s]
WIND                   = '0@0' #direction [deg] @ speed [knots]

''' SETPOINT '''
# 0 / 'constant'      | Constant setpoint
# 1 / 'straight_line' | Straight line setpoint
SP_TYPE = 'constant'
SP_INIT = (0, 0, -1000, 0, 0, 0, 50, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0) #(pn_sp, pe_sp, pd_sp, phi_sp, theta_sp, psi_sp, u_sp, v_sp, w_sp, p_sp, q_sp, r_sp, deltaa_sp, deltae_sp, deltaf_sp, deltar_sp, deltat_sp, deltam_sp)

''' SIMULATION '''
ACT_HZ             = 10 #actuation frequency
CM_HZ              = 10 #control model frequency
FPS                = 100 #frames per real time second
MAX_TIME_PER_FRAME = 1 / 100 #timestep per video frame
MODEL_HZ           = 100 #model iterations per simulation second
SIM_ITER_NUM       = 2 #number of simulation iterations(only applicable when SIM_TYPE = 1 / 'multiple')
SIM_RATE           = 1 #simulation rate with respect to real time (recommended: 0.125, 0.25, 0.5, 1)
SIM_TYPE           = 'single' #simulation type: 0 = 'single', 1 = 'multiple'

''' TELEMETRY '''
TELEM_RX_BUFFER_SIZE = 1024 * 8 #long enough to allocate a complete frame
TELEM_RX_IP_ADDRESS  = 'localhost'
TELEM_RX_PORT        = 60001 #receiving link port
TELEM_TX_IP_ADDRESS  = 'localhost'
TELEM_TX_PORT        = 60002 #transmitting link port
TELEM_WAIT           = 10 #wait in seconds after FlightGear start before trying to establish telemetry communication

