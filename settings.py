''' SIMULATION '''
SIM_RATE = 1 #simulation rate with respect to real time (recommended: 0.125, 0.25, 0.5, 1)
FPS = 100 #frames per real time second
MAX_TIME_PER_FRAME = 1/100 #1/(10 * FPS) #timestep per video frame
MODEL_HZ = 100 #model iterations per simulation second
ACT_HZ = 10 #actuation frequency
CFG_IP_ADDRESS = 'localhost'
CFG_PORT = 60003 #config port

''' ACTUATION '''
ACT_TYPE = 1

''' DYNAMICS '''
DYN_LOG_FILENAME = 'dynamics_log'

''' KINEMATICS '''
KIN_LOG_FILENAME = 'kinematics_log'

''' INITIALIZATION '''
HEAD_INIT = 30
LONG_INIT= -4.853942
LAT_INIT = 37.834722

''' TELEMETRY '''
TELEM_IP_ADDRESS = 'localhost'
TELEM_LOG_FILENAME = 'telemetry_log'
TELEM_RX_PORT = 60001 #receiving link port
TELEM_RX_BUFFER_SIZE = 10240 #enough to allocate a complete frame
TELEM_TX_PORT = 60002 #transmitting link port
TELEM_WAIT = 10 #wait in seconds after FlightGear start before establishing telemetry communication

''' RECEIVING TELEMETRY '''
#0 Simulation time from start in seconds
#1 Simulation timestep in seconds
#2 Delta longitude from start in meters
#3 Delta latitude from start in meters
#4 Distance from start in meters
#5 Longitude (GNSS) in degrees
#6 Longitude (GNSS) in radians
#7 Latitude (GNSS) in degrees
#8 Latitude (GNSS) in radians
#9 Altitude (GNSS) in feet
#10 Altitude (GNSS) in kilometers
#11 Barometric altitude (QFE) in feet
#12 Barometric altitude (QFE) in kilometers
#13 Barometric altitude (QNH) in feet
#14 Barometric altitude (QNH) in meters
#15 Roll angle in degrees
#16 Roll angle in radians
#17 Pitch angle in degrees
#18 Pitch angle in radians
#19 Yaw angle in degrees
#20 Yaw angle in radians
#21 Angle of attack in degrees
#22 Angle of attack in radians
#23 Side-slip angle in degrees
#24 Side-slip angle in radians
#25 Path angle in degrees
#26 Path angle in radians
#27 North velocity (NED) in feet per second
#28 East velocity (NED) in feet per second
#29 Down velocity (NED) in feet per second
#30 Longitudinal velocity (body) in feet per second
#31 Transversal velocity (body) in feet per second
#32 Vertical velocity (body) in feet per second
#33 Longitudinal aerodynamic velocity (wind) in feet per second
#34 Transversal aerodynamic velocity (wind) in feet per second
#35 Vertical aerodynamic velocity (wind) in feet per second
#36 North wind velocity (NED) in feet per second
#37 East wind velocity (NED) in feet per second
#38 Down wind velocity (NED) in feet per second
#39 Rate of change of roll angle in radians per second
#40 Rate of change of pitch angle in radians per second
#41 Rate of change of yaw angle in radians per second
#42 Roll velocity (body) in radians per second
#43 Pitch velocity (body) in radians per second
#44 Yaw velocity (body) in radians per second
#45 Rate of change of angle of attack in degrees per second
#46 Rate of change of angle of attack in radians per second
#47 Rate of change of side-slip angle in degrees per second
#48 Rate of change of side-slip angle in radians per second
#49 Longitudinal acceleration (body) in feet per squared second
#50 Transversal acceleration (body) in feet per squared second 
#51 Vertical acceleration (body) in feet per squared second 
#52 Roll acceleration (body) in radians per squared second 
#53 Pitch acceleration (body) in radians per squared second 
#54 Yaw acceleration (body) in radians per squared second 
#55 Longitudinal aerodynamic force (body) in pounds
#56 Longitudinal external force (body) in pounds
#57 Longitudinal gear force (body) in pounds
#58 Longitudinal propeller force (body) in pounds
#59 Longitudinal total force (body) in pounds
#60 Transversal aerodynamic force (body) in pounds
#61 Transversal external force (body) in pounds
#62 Transversal gear force (body) in pounds
#63 Transversal propeller force (body) in pounds
#64 Transversal total force (body) in pounds
#65 Vertical aerodynamic force (body) in pounds
#66 Vertical external force (body) in pounds
#67 Vertical gear force (body) in pounds
#68 Vertical propeller force (body) in pounds
#69 Vertical total force (body) in pounds
#70 Aerodynamic roll moment (body) in pounds-feet
#71 External roll moment (body) in pounds-feet
#72 Gear roll moment (body) in pounds-feet
#73 Propeller roll moment (body) in pounds-feet
#74 Total roll moment (body) in pounds-feet
#75 Aerodynamic pitch moment (body) in pounds-feet
#76 External pitch moment (body) in pounds-feet
#77 Gear pitch moment (body) in pounds-feet
#78 Propeller pitch moment (body) in pounds-feet
#79 Total pitch moment (body) in pounds-feet
#80 Aerodynamic yaw moment (body) in pounds-feet
#81 External yaw moment (body) in pounds-feet
#82 Gear yaw moment (body) in pounds-feet
#83 Propeller yaw moment (body) in pounds-feet
#84 Total yaw moment (body) in pounds-feet
#85 Right aileron position in degrees
#86 Right aileron position in radians
#87 Normalized right aileron position
#88 Left aileron position in degrees
#89 Left aileron position in radians
#90 Normalized left aileron position
#91 Elevators position in degrees
#92 Elevators position in radians
#93 Normalized elevators position
#94 Flaps position in degrees
#95 Flaps position in radians
#96 Normalized flaps position
#97 Rudder position in degrees
#98 Rudder position in radians
#99 Normalized rudder position
#100 Normalized engine throttle position
#101 Normalized engine mixture position
#102 Upside-down state
#103 Wheel 1 weight-on-wheel state
#104 Wheel 2 weight-on-wheel state
#105 Wheel 3 weight-on-wheel state
#106 Wing span divided by twice the velocity
#107 Wing chord divided by twice the velocity
#108 Altitude of mean aerodynamic chord (MAC) divided by wing span
#109 Dynamic pressure in pounds per squared foot
#110 Dynamic pressure (XZ plane of wind plane) in pounds per squared foot
#111 Dynamic pressure due to propeller induced velocity in pounds per squared foot
#112 Dynamic pressure including propulsion induced velocity in pounds per squared foot
#113 Normalized stall (aerodynamic) hysteresis
#114 Density in slugs per cubic foot
#115 Advance ratio
#116 Propeller revolutions per minute
TELEM_RX_LEN = 117 #do not modify unless fg2py.xml is also modified accordingly

''' TRANSMITTING TELEMETRY '''
#0 Normalized aileron command
#1 Normalized elevators command
#2 Normalized flaps command
#3 Normalized rudder command
#4 Normalized engine throttle command
#5 Normalized engine mixture command
#6 Normalized roll trim command
#7 Normalized pitch trim command
#8 Normalized yaw trim command
TELEM_TX_LEN = 9 #do not modify unless py2fg.xml is also modified accordingly

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
#0 Time
#1 Timestep
#2 Delta longitude
#3 Delta latitude
#4 Distance
#5 Longitude
#6 Latitude
#7 Altitude
#8 Baro. altitude (QFE)
#9 Baro. altitude (QNH)
#10 Roll ang.
#11 Pitch ang.
#12 Yaw ang.
#13 Attack ang.
#14 Side-slip ang.
#15 Path ang.
#16 North vel.
#17 East vel.
#18 Down vel.
#19 Long. vel.
#20 Transv. vel.
#21 Vert. vel.
#22 Long. aero. vel.
#23 Transv. aero. vel.
#24 Vert. aero. vel.
#25 North wind vel.
#26 East wind vel.
#27 Down wind vel.
#28 Roll ang. change
#29 Pitch ang. change
#30 Yaw ang. change
#31 Roll vel.
#32 Pitch vel.
#33 Yaw vel.
#34 Angle of attack change
#35 Side-slip ang. change
#36 Long. accel.
#37 Transv. accel.
#38 Vert. accel.
#39 Roll accel.
#40 Pitch accel.
#41 Yaw accel.
#42 Long. aero. force
#43 Long. ext. force
#44 Long. gear force
#45 Long. prop. force
#46 Long. total force
#47 Transv. aero. force
#48 Transv. ext. force
#49 Transv. gear force
#50 Transv. prop. force
#51 Transv. total force
#52 Vert. aero. force
#53 Vert. ext. force
#54 Vert. gear force
#55 Vert. prop. force
#56 Vert. total force
#57 Aero. roll mom.
#58 Ext. roll mom.
#59 Gear roll mom.
#60 Prop. roll mom.
#61 Total roll mom.
#62 Aero. pitch mom.
#63 Ext. pitch mom.
#64 Gear pitch mom.
#65 Prop. pitch mom.
#66 Total pitch mom.
#67 Aero. yaw mom.
#68 Ext. yaw mom.
#69 Gear yaw mom.
#70 Prop. yaw mom.
#71 Total yaw mom.
#72 R ail. pos.
#73 N. R ail. pos.
#74 L ail. pos.
#75 N. L ail. pos.
#76 Elevs. pos.
#77 N. elevs. pos.
#78 Flaps pos.
#79 N. flaps pos.
#80 Rudder pos.
#81 N. rudder pos.
#82 N. eng. thrott. pos.
#83 N. eng. mix. pos.
#84 Upside-down
#85 WOW1
#86 WOW2
#87 WOW3
#88 Span-vel. ratio
#89 Chord-vel. ratio 
#90 MAC-span ratio
#91 Dyn. press.
#92 Dyn. press. XZ
#93 Dyn. press. prop.
#94 Dyn. press. ind.
#95 N. stall hyst.
#96 Advance ratio
#97 Prop. revs.
TELEM_RX_PLOT = {
                 'Time' : [['s',('s','s')], r'$t$'],
                 'Timestep' : [['s',('s','s')], r'$t_{step}$'],
                 'Delta longitude' : [['m',('m','ft')], r'$\Delta_{\Lambda}$'],
                 'Delta latitude' : [['m',('m','ft')], r'$\Delta_{\Phi}$'],
                 'Distance' : [['m',('m','ft')], r'$r$'],
                 'Longitude' : [['rad',('rad','rad')], r'$\Lambda$'],
                 'Latitude' : [['rad',('rad','rad')], r'$\Phi$'],
                 'Altitude' : [['km',('m','ft')], r'$H$'],
                 'Baro. altitude (QFE)' : [['km',('m','ft')], r'$h_{QFE}$'],
                 'Baro. altitude (QNH)' : [['km',('m','ft')], r'$h_{QNH}$'],
                 'Roll ang.' : [['rad',('rad','rad')], r'$\phi$'],
                 'Pitch ang.' : [['rad',('rad','rad')], r'$\theta$'],
                 'Yaw ang.' : [['rad',('rad','rad')], r'$\psi$'],
                 'Attack ang.' : [['rad',('rad','rad')], r'$\alpha$'],
                 'Side-slip ang.' : [['rad',('rad','rad')], r'$\beta$'],
                 'Path ang.' : [['rad',('rad','rad')], r'$\gamma$'],
                 'North vel.' : [['ft/s',('m/s', 'ft/s')], r'$v_N$'],
                 'East vel.' : [['ft/s',('m/s','ft/s')], r'$v_E$'],
                 'Down vel.' : [['ft/s',('m/s','ft/s')], r'$v_D$'],
                 'Long. vel.' : [['ft/s',('m/s','ft/s')], r'$v_X$'],
                 'Transv. vel.' : [['ft/s',('m/s','ft/s')], r'$v_Y$'],
                 'Vert. vel.' : [['ft/s',('m/s','ft/s')], r'$v_Z$'],
                 'Long. aero. vel.' : [['ft/s',('m/s','ft/s')], r'$v_U$'],
                 'Transv. aero. vel.' : [['ft/s',('m/s','ft/s')], r'$v_V$'],
                 'Vert. aero. vel.' : [['ft/s',('m/s','ft/s')], r'$v_W$'],
                 'North wind vel.' : [['ft/s',('m/s','ft/s')], r'$w_N$'],
                 'East wind vel.' : [['ft/s',('m/s','ft/s')], r'$w_E$'],
                 'Down wind vel.' : [['ft/s',('m/s','ft/s')], r'$w_D$'],
                 'Roll ang. change' : [['rad/s',('rad/s','rad/s')], r'$\dot \phi$'],
                 'Pitch ang. change' : [['rad/s',('rad/s','rad/s')], r'$\dot \theta$'],
                 'Yaw ang. change' : [['rad/s',('rad/s','rad/s')], r'$\dot \psi$'],
                 'Roll vel.' : [['rad/s',('rad/s','rad/s')], r'$p$'],
                 'Pitch vel.' : [['rad/s',('rad/s','rad/s')], r'$q$'],
                 'Yaw vel.' : [['rad/s',('rad/s','rad/s')], r'$r$'],
                 'Attack ang. change' : [['rad/s',('rad/s','rad/s')], r'$\dot \alpha$'],
                 'Side-slip ang. change' : [['rad/s',('rad/s','rad/s')], r'$\dot \beta$'],
                 'Long. accel.' : [['ft/s^2',('m/s²','ft/s²')], r'$a_X$'],
                 'Transv. accel.' : [['ft/s^2',('m/s²','ft/s²')], r'$a_Y$'],
                 'Vert. accel.' : [['ft/s^2',('m/s²','ft/s²')], r'$a_Z$'],
                 'Roll accel.' : [['rad/s^2',('rad/s²','rad/s²')], r'$\dot p$'],
                 'Pitch accel.' : [['rad/s^2',('rad/s²','rad/s²')], r'$\dot q$'],
                 'Yaw accel.' : [['rad/s^2',('rad/s²','rad/s²')], r'$\dot r$'],
                 'Long. aero. force' : [['lbs',('N','lbs')], r'$Faero_X$'],
                 'Long. ext. force' : [['lbs',('N','lbs')], r'$Fext_X$'],
                 'Long. gear force' : [['lbs',('N','lbs')], r'$Fgear_X$'],
                 'Long. prop. force' : [['lbs',('N','lbs')], r'$Fprop_X$'],
                 'Long. total force' : [['lbs',('N','lbs')], r'$Ftotal_X$'],
                 'Transv. aero. force' : [['lbs',('N','lbs')], r'$Faero_Y$'],
                 'Transv. ext. force' : [['lbs',('N','lbs')], r'$Fext_Y$'],
                 'Transv. gear force' : [['lbs',('N','lbs')], r'$Fgear_Y$'],
                 'Transv. prop. force' : [['lbs',('N','lbs')], r'$Fprop_Y$'],
                 'Transv. total force' : [['lbs',('N','lbs')], r'$Ftotal_Y$'],
                 'Vert. aero. force' : [['lbs',('N','lbs')], r'$Faero_Z$'],
                 'Vert. ext. force' : [['lbs',('N','lbs')], r'$Fext_Z$'],
                 'Vert. gear force' : [['lbs',('N','lbs')], r'$Fgear_Z$'],
                 'Vert. prop. force' : [['lbs',('N','lbs')], r'$Fprop_Z$'],
                 'Vert. total force' : [['lbs',('N','lbs')], r'$Ftotal_Z$'],
                 'Aero. roll mom.' : [['lbs·ft',('N·m','lbs·ft')], r'$Maero_P$'],
                 'Ext. roll mom.' : [['lbs·ft',('N·m','lbs·ft')], r'$Mext_P$'],
                 'Gear roll mom.' : [['lbs·ft',('N·m','lbs·ft')], r'$Mgear_P$'],
                 'Prop. roll mom.' : [['lbs·ft',('N·m','lbs·ft')], r'$Mprop_P$'],
                 'Total roll mom.' : [['lbs·ft',('N·m','lbs·ft')], r'$Mtotal_P$'],
                 'Aero. pitch mom.' : [['lbs·ft',('N·m','lbs·ft')], r'$Maero_Q$'],
                 'Ext. pitch mom.' : [['lbs·ft',('N·m','lbs·ft')], r'$Mext_Q$'],
                 'Gear pitch mom.' : [['lbs·ft',('N·m','lbs·ft')], r'$Mgear_Q$'],
                 'Prop. pitch mom.' : [['lbs·ft',('N·m','lbs·ft')], r'$Mprop_Q$'],
                 'Total pitch mom.' : [['lbs·ft',('N·m','lbs·ft')], r'$Mtotal_Q$'],
                 'Aero. yaw mom.' : [['lbs·ft',('N·m','lbs·ft')], r'$Maero_R$'],
                 'Ext. yaw mom.' : [['lbs·ft',('N·m','lbs·ft')], r'$Mext_R$'],
                 'Gear yaw mom.' : [['lbs·ft',('N·m','lbs·ft')], r'$Mgear_R$'],
                 'Prop. yaw mom.' : [['lbs·ft',('N·m','lbs·ft')], r'$Mprop_R$'],
                 'Total yaw mom.' : [['lbs·ft',('N·m','lbs·ft')], r'$Mtotal_R$'],
                 'R ail. pos.' : [['rad',('rad','rad')], r'$\sigma_{ra}$'],
                 'N. R ail. pos.' : [['-',('-','-')], r'$\sigma_{ra}$'],
                 'L ail. pos.' : [['rad',('rad','rad')], r'$\sigma_{la}$'],
                 'N. L ail. pos.' : [['-',('-','-')], r'$\sigma_{la}$'],
                 'Elevs. pos.' : [['rad',('rad','rad')], r'$\sigma_e$'],
                 'N. elevs. pos.' : [['-',('-','-')], r'$\sigma_e$'],
                 'Flaps pos.' : [['rad',('rad','rad')], r'$\sigma_f$'],
                 'N. flaps pos.' : [['-',('-','-')], r'$\sigma_f$'],
                 'Rudder pos.' : [['rad',('rad','rad')], r'$\sigma_r$'],
                 'N. rudder pos.' : [['-',('-','-')], r'$\sigma_r$'],
                 'N. eng. thrott. pos.' : [['-',('-','-')], r'$\sigma_t$'],
                 'N. eng. mix. pos.' : [['-',('-','-')], r'$\sigma_{mix}$'],
                 'Upside-down' : [['-',('-','-')], ''],
                 'WOW1' : [['-',('-','-')], ''],
                 'WOW2' : [['-',('-','-')], ''],
                 'WOW3' : [['-',('-','-')], ''],
                 'Span-vel. ratio' : [['-',('-','-')], ''],
                 'Chord-vel. ratio' : [['-',('-','-')], ''],
                 'MAC-span ratio' : [['-',('-','-')], ''],
                 'Dyn. press.' : [['-',('-','-')], r'$q$'],
                 'Dyn. press. XZ' : [['-',('-','-')], r'$q_{XZ}$'],
                 'Dyn. press. prop.' : [['-',('-','-')], r'$q_{prop}$'],
                 'Dyn. press. ind.' : [['-',('-','-')], r'$q_{ind}$'],
                 'N. stall hyst.' : [['-',('-','-')], ''],
                 'Advance ratio' : [['-',('-','-')], r'$J$'],
                 'Prop. revs.' : [['RPM',('RPM','RPM')], '']
                }
TELEM_TX_PLOT = {
                 'N. ails. cmd' : [['-',('-','-')], r'$\delta_a$'],
                 'N. elevs. cmd' : [['-',('-','-')], r'$\delta_e$'],
                 'N. rudder cmd' : [['-',('-','-')], r'$\delta_r$'],
                 'N. flaps cmd' : [['-',('-','-')], r'$\delta_f$'],
                 'N. eng. thrott. cmd' : [['-',('-','-')], r'$\delta_t$'],
                 'N. eng. mix. cmd' : [['-',('-','-')], r'$\delta_{mix}$'],
                 'N. roll trim cmd' : [['-',('-','-')], r'$\phi_{trim}$'],
                 'N. pitch trim cmd' : [['-',('-','-')], r'$\theta_{trim}$'],
                 'N. yaw trim cmd' : [['-',('-','-')], r'$\psi_{trim}$']
                }
