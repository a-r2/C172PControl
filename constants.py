''' PROTOCOLS FILES '''
CFG_PROTOCOL_FILENAME = 'py2fg_cfg' #DO NOT MODIFY UNLESS PY2FG_CFG.XML IS ALSO RENAMED ACCORDINGLY
RX_PROTOCOL_FILENAME  = 'fg2py' #DO NOT MODIFY UNLESS FG2PY.XML IS ALSO RENAMED ACCORDINGLY
TX_PROTOCOL_FILENAME  = 'py2fg_act' #DO NOT MODIFY UNLESS PY2FG_ACT.XML IS ALSO RENAMED ACCORDINGLY

# VARIABLE DEFINITION FORMAT
#
# 0 | 1 | 2 | 3 | 4
#
# 0 : index
# 1 : variable name
# 2 : description
# 3 : frame of reference (ECEF, NED, XYZ, UVW)
# 4 : unit

''' ACTUATION '''
# 0 | deltaa      | Normalized ailerons position        | - | -
# 1 | deltae      | Normalized elevators position       | - | -
# 2 | deltaf      | Normalized flaps position           | - | -
# 3 | deltar      | Normalized rudder position          | - | -
# 4 | deltat      | Normalized engine throttle position | - | -
# 5 | deltam      | Normalized engine mixture position  | - | -
# 6 | deltaa_trim | Normalized ailerons trim position   | - | -
# 7 | deltae_trim | Normalized elevators trim position  | - | -
# 8 | deltar_trim | Normalized rudder trim position     | - | -
ACT_STR = ('deltaa', 'deltae', 'deltaf', 'deltar', 'deltat', 'deltam', 'deltaa_trim', 'deltae_trim', 'deltar_trim')
ACT_LEN = len(ACT_STR)

''' CONTROL MODEL '''
# 0 | deltaa | Normalized ailerons position        | - | - 
# 1 | deltae | Normalized elevators position       | - | - 
# 2 | deltaf | Normalized flaps position           | - | - 
# 3 | deltar | Normalized rudder position          | - | - 
# 4 | deltat | Normalized engine throttle position | - | - 
# 5 | deltam | Normalized engine mixture position  | - | - 
CM_INPUT_STR = ('deltaa', 'deltae', 'deltaf', 'deltar', 'deltat', 'deltam')
CM_INPUT_LEN = len(CM_INPUT_STR)

# 0  | pn | North position                               | NED | m
# 1  | pe | East position                                | NED | m
# 2  | pd | Down position                                | NED | m
# 3  | q0 | Scalar component of attitude quaternion      | -   | -
# 4  | q1 | Vectorial component 1 of attitude quaternion | -   | -
# 5  | q2 | Vectorial component 2 of attitude quaternion | -   | -
# 6  | q3 | Vectorial component 3 of attitude quaternion | -   | -
# 7  | u  | Longitudinal velocity                        | XYZ | m/s
# 8  | v  | Transversal velocity                         | XYZ | m/s
# 9  | w  | Vertical velocity                            | XYZ | m/s
# 10 | p  | Roll velocity                                | XYZ | rad/s
# 11 | q  | Pitch velocity                               | XYZ | rad/s
# 12 | r  | Yaw velocity                                 | XYZ | rad/s
CM_STATE_STR = ('pn', 'pe', 'pd', 'q0', 'q1', 'q2', 'q3', 'u', 'v', 'w', 'p', 'q', 'r')
CM_STATE_LEN = len(CM_STATE_STR)

''' DYNAMICS '''
# 0 | D  | Longitudinal aerodynamic force   | XYZ | N
# 1 | C  | Transversal aerodynamic force    | XYZ | N
# 2 | L  | Vertical aerodynamic force       | XYZ | N
# 3 | T  | Engine thrust force              | XYZ | N
# 4 | gx | Longitudinal gravitational force | XYZ | N
# 5 | gy | Transversal gravitational force  | XYZ | N
# 6 | gz | Vertical gravitational force     | XYZ | N
# 7 | l  | Longitudinal moment              | XYZ | N·m
# 8 | m  | Transversal moment               | XYZ | N·m
# 9 | n  | Vertical moment                  | XYZ | N·m
DYN_STR = ('D', 'C', 'L', 'T', 'gx', 'gy', 'gz', 'l', 'm', 'n')
DYN_LEN = len(DYN_STR)

''' RX TELEMETRY '''
# 0   | t_sim         | Simulation time from start                                    | -    | s
# 1   | dt_sim        | Simulation timestep                                           | -    | s
# 2   | longd         | Delta longitude from start                                    | -    | m
# 3   | latd          | Delta latitude from start                                     | -    | m
# 4   | distd         | Distance from start                                           | -    | m
# 5   | pn_ecef       | North position                                                | ECEF | ft
# 6   | pe_ecef       | East position                                                 | ECEF | ft
# 7   | pd_ecef       | Down position                                                 | ECEF | ft
# 8   | long_deg      | Longitude (GNSS)                                              | ECEF | deg
# 9   | long_rad      | Longitude (GNSS)                                              | ECEF | rad
# 10  | lat_deg       | Latitude (GNSS)                                               | ECEF | deg
# 11  | lat_rad       | Latitude (GNSS)                                               | ECEF | rad
# 12  | hgnss_ft      | Altitude (GNSS)                                               | ECEF | ft
# 13  | hgnss_km      | Altitude (GNSS)                                               | ECEF | km
# 14  | hqfe_ft       | Barometric altitude AGL (QFE)                                 | ECEF | ft
# 15  | hqfe_km       | Barometric altitude AGL (QFE)                                 | ECEF | km
# 16  | hqnh_ft       | Barometric altitude ASL (QNH)                                 | ECEF | ft
# 17  | hqnh_m        | Barometric altitude ASL (QNH)                                 | ECEF | m
# 18  | hterr         | Terrain elevation ASL                                         | ECEF | ft
# 19  | phi_deg       | Roll angle                                                    | -    | deg
# 20  | phi_rad       | Roll angle                                                    | -    | rad
# 21  | theta_deg     | Pitch angle                                                   | -    | deg
# 22  | theta_rad     | Pitch angle                                                   | -    | rad
# 23  | psi_deg       | Yaw angle                                                     | -    | deg
# 24  | psi_rad       | Yaw angle                                                     | -    | rad
# 25  | alpha_deg     | Angle of attack                                               | -    | deg
# 26  | alpha_rad     | Angle of attack                                               | -    | rad
# 27  | beta_deg      | Side-slip angle                                               | -    | deg
# 28  | beta_rad      | Side-slip angle                                               | -    | rad
# 29  | gamma_deg     | Path angle                                                    | -    | deg
# 30  | gamma_rad     | Path angle                                                    | -    | rad
# 31  | vn            | North velocity                                                | NED  | ft/s
# 32  | ve            | East velocity                                                 | NED  | ft/s
# 33  | vd            | Down velocity                                                 | NED  | ft/s
# 34  | u             | Longitudinal velocity                                         | XYZ  | ft/s
# 35  | v             | Transversal velocity                                          | XYZ  | ft/s
# 36  | w             | Vertical velocity                                             | XYZ  | ft/s 
# 37  | uaero         | Aerodynamic longitudinal velocity                             | XYZ  | ft/s
# 38  | vaero         | Aerodynamic transversal velocity                              | XYZ  | ft/s
# 39  | waero         | Aerodynamic vertical velocity                                 | XYZ  | ft/s
# 40  | wn            | North wind velocity                                           | NED  | ft/s
# 41  | we            | East wind velocity                                            | NED  | ft/s
# 42  | wd            | Down wind velocity                                            | NED  | ft/s
# 43  | phidot        | Rate of change of roll angle                                  | -    | rad/s
# 44  | thetadot      | Rate of change of pitch angle                                 | -    | rad/s
# 45  | psidot        | Rate of change of yaw angle                                   | -    | rad/s
# 46  | p             | Roll velocity                                                 | XYZ  | rad/s
# 47  | q             | Pitch velocity                                                | XYZ  | rad/s
# 48  | r             | Yaw velocity                                                  | XYZ  | rad/s
# 49  | paero         | Aerodynamic roll velocity                                     | XYZ  | rad/s
# 50  | qaero         | Aerodynamic pitch velocity                                    | XYZ  | rad/s
# 51  | raero         | Aerodynamic yaw velocity                                      | XYZ  | rad/s
# 52  | alphadot_degs | Rate of change of angle of attack                             | -    | deg/s
# 53  | alphadot_rads | Rate of change of angle of attack                             | -    | rad/s
# 54  | betadot_degs  | Rate of change of side-slip angle                             | -    | deg/s
# 55  | betadot_rads  | Rate of change of side-slip angle                             | -    | rad/s
# 56  | udot          | Longitudinal acceleration                                     | XYZ  | ft/s^2
# 57  | vdot          | Transversal acceleration                                      | XYZ  | ft/s^2
# 58  | wdot          | Vertical acceleration                                         | XYZ  | ft/s^2
# 59  | pdot          | Roll acceleration                                             | XYZ  | rad/s^2
# 60  | qdot          | Pitch acceleration                                            | XYZ  | rad/s^2
# 61  | rdot          | Yaw acceleration                                              | XYZ  | rad/s^2
# 62  | fxaero        | Longitudinal aerodynamic force                                | XYZ  | lbs
# 63  | fxext         | Longitudinal external force                                   | XYZ  | lbs
# 64  | fxgear        | Longitudinal gear force                                       | XYZ  | lbs
# 65  | fxprop        | Longitudinal propeller force                                  | XYZ  | lbs
# 66  | fx            | Longitudinal total force                                      | XYZ  | lbs
# 67  | fyaero        | Transversal aerodynamic force                                 | XYZ  | lbs
# 68  | fyext         | Transversal external force                                    | XYZ  | lbs
# 69  | fygear        | Transversal gear force                                        | XYZ  | lbs
# 70  | fyprop        | Transversal propeller force                                   | XYZ  | lbs
# 71  | fy            | Transversal total force                                       | XYZ  | lbs
# 72  | fzaero        | Vertical aerodynamic force                                    | XYZ  | lbs
# 73  | fzext         | Vertical external force                                       | XYZ  | lbs
# 74  | fzgear        | Vertical gear force                                           | XYZ  | lbs
# 75  | fzprop        | Vertical propeller force                                      | XYZ  | lbs
# 76  | fz            | Vertical total force                                          | XYZ  | lbs
# 77  | laero         | Aerodynamic roll moment                                       | XYZ  | lbs·ft
# 78  | lext          | External roll moment                                          | XYZ  | lbs·ft
# 79  | lgear         | Gear roll moment                                              | XYZ  | lbs·ft
# 80  | lprop         | Propeller roll moment                                         | XYZ  | lbs·ft
# 81  | l             | Total roll moment                                             | XYZ  | lbs·ft
# 82  | maero         | Aerodynamic pitch moment                                      | XYZ  | lbs·ft
# 83  | mext          | External pitch moment                                         | XYZ  | lbs·ft
# 84  | mgear         | Gear pitch moment                                             | XYZ  | lbs·ft
# 85  | mprop         | Propeller pitch moment                                        | XYZ  | lbs·ft
# 86  | m             | Total pitch moment                                            | XYZ  | lbs·ft
# 87  | naero         | Aerodynamic yaw moment                                        | XYZ  | lbs·ft
# 88  | next          | External yaw moment                                           | XYZ  | lbs·ft
# 89  | ngear         | Gear yaw moment                                               | XYZ  | lbs·ft
# 90  | nprop         | Propeller yaw moment                                          | XYZ  | lbs·ft
# 91  | n             | Total yaw moment                                              | XYZ  | lbs·ft
# 92  | sigmara_deg   | Right aileron position                                        | -    | deg
# 93  | sigmara_rad   | Right aileron position                                        | -    | rad
# 94  | deltara       | Normalized right aileron position                             | -    | -
# 95  | sigmala_deg   | Left aileron position                                         | -    | deg
# 96  | sigmala_rad   | Left aileron position                                         | -    | rad
# 97  | deltala       | Normalized left aileron position                              | -    | -
# 98  | sigmae_deg    | Elevators position                                            | -    | deg
# 99  | sigmae_rad    | Elevators position                                            | -    | rad
# 100 | deltae        | Normalized elevators position                                 | -    | -
# 101 | sigmaf_deg    | Flaps position                                                | -    | deg
# 102 | sigmaf_rad    | Flaps position                                                | -    | rad
# 103 | deltaf        | Normalized flaps position                                     | -    | -
# 104 | sigmar_deg    | Rudder position                                               | -    | deg
# 105 | sigmar_rad    | Rudder position                                               | -    | rad
# 106 | deltar        | Normalized rudder position                                    | -    | -
# 107 | deltat        | Normalized engine throttle position                           | -    | -
# 108 | deltam        | Normalized engine mixture position                            | -    | -
# 109 | Bw2Va         | Wing span divided by twice the aerodynamic velocity           | -    | s
# 110 | Cw2Va         | Wing chord divided by twice the aerodynamic velocity          | -    | s
# 111 | hmacb         | Altitude of mean aerodynamic chord (MAC) divided by wing span | -    | -
# 112 | qbar          | Dynamic pressure                                              | -    | psf
# 113 | qbaruw        | Dynamic pressure (UW plane of wind frame)                     | -    | psf
# 114 | qbarprop      | Dynamic pressure due to propeller induced velocity            | -    | psf
# 115 | qbarind       | Dynamic pressure including propulsion induced velocity        | -    | psf
# 116 | stall         | Normalized stall (aerodynamic) hysteresis                     | -    | -
# 117 | rho           | Air density in slugs per cubic foot                           | -    | slug/ft^3
# 118 | J             | Advance ratio                                                 | -    | -
# 119 | rpmprop       | Propeller revolutions per minute                              | -    | rev/min
# 120 | Ixx           | Moment of inercia Ixx                                         | -    | slug/ft^2
# 121 | Ixy           | Moment of inercia Ixy                                         | -    | slug/ft^2
# 122 | Ixz           | Moment of inercia Ixz                                         | -    | slug/ft^2
# 123 | Iyy           | Moment of inercia Iyy                                         | -    | slug/ft^2
# 124 | Iyz           | Moment of inercia Iyz                                         | -    | slug/ft^2
# 125 | Izz           | Moment of inercia Izz                                         | -    | slug/ft^2
# 126 | mass          | Mass                                                          | -    | slug
# 127 | grav          | Gravitational acceleration                                    | -    | ft/s^2
# 128 | up_down       | Upside-down state                                             | -    | -
# 129 | wow1          | Wheel 1 weight-on-wheel state                                 | -    | -
# 130 | wow2          | Wheel 2 weight-on-wheel state                                 | -    | -
# 131 | wow3          | Wheel 3 weight-on-wheel state                                 | -    | -
# 132 | contact1      | Wheel 1 contact state                                         | -    | -
# 133 | contact2      | Wheel 2 contact state                                         | -    | -
# 134 | contact3      | Wheel 3 contact state                                         | -    | -
TELEM_RX_STR = ('t_sim', 'dt_sim', 'longd', 'latd', 'distd', 'pn_ecef', 'pe_ecef', 'pd_ecef', 'long_deg', 'long_rad', 'lat_deg', 'lat_rad', 'hgnss_ft', 'hgnss_km', 'hqfe_ft', 'hqfe_km', 'hqnh_ft', 'hqnh_m', 'hterr', 'phi_deg', 'phi_rad', 'theta_deg', 'theta_rad', 'psi_deg', 'psi_rad', 'alpha_deg', 'alpha_rad', 'beta_deg', 'beta_rad', 'gamma_deg', 'gamma_rad', 'vn', 've', 'vd', 'u', 'v', 'w', 'uaero', 'vaero', 'waero', 'wn', 'we', 'wd', 'phidot', 'thetadot', 'psidot', 'p', 'q', 'r', 'paero', 'qaero', 'raero', 'alphadot_degs', 'alphadot_rads', 'betadot_degs', 'betadot_rads', 'udot', 'vdot', 'wdot', 'pdot', 'qdot', 'rdot', 'fxaero', 'fxext', 'fxgear', 'fxprop', 'fx', 'fyaero', 'fyext', 'fygear', 'fyprop', 'fy', 'fzaero', 'fzext', 'fzgear', 'fzprop', 'fz', 'laero', 'lext', 'lgear', 'lprop', 'l', 'maero', 'mext', 'mgear', 'mprop', 'm', 'naero', 'next', 'ngear', 'nprop', 'n', 'sigmara_deg', 'sigmara_rad', 'deltara', 'sigmala_deg', 'sigmala_rad', 'deltala', 'sigmae_deg', 'sigmae_rad', 'deltae', 'sigmaf_deg', 'sigmaf_rad', 'deltaf', 'sigmar_deg', 'sigmar_rad', 'deltar', 'deltat', 'deltam', 'Bw2Va', 'Cw2Va', 'hmacb', 'qbar', 'qbaruw', 'qbarprop', 'qbarind', 'stall', 'rho', 'J', 'rpmprop', 'Ixx', 'Ixy', 'Ixz', 'Iyy', 'Iyz', 'Izz', 'mass', 'grav', 'up_down', 'wow1', 'wow2', 'wow3', 'contact1', 'contact2', 'contact3') #RX telemetry str tuple (DO NOT MODIFY UNLESS FG2PY.XML IS ALSO MODIFIED ACCORDINGLY)
TELEM_RX_LEN = len(TELEM_RX_STR)

''' TX TELEMETRY '''
# 0 | deltaa      | Normalized aileron command         | - | -
# 1 | deltae      | Normalized elevators command       | - | -
# 2 | deltaf      | Normalized flaps command           | - | -
# 3 | deltar      | Normalized rudder command          | - | -
# 4 | deltat      | Normalized engine throttle command | - | -
# 5 | deltam      | Normalized engine mixture command  | - | -
# 6 | deltaa_trim | Normalized aileron trim command    | - | -
# 7 | deltae_trim | Normalized elevator trim command   | - | -
# 8 | deltar_trim | Normalized rudder trim command     | - | -
TELEM_TX_STR = ('deltaa', 'deltae', 'deltaf', 'deltar', 'deltat', 'deltam', 'deltaa_trim', 'deltae_trim', 'deltar_trim') #TX telemetry str tuple (DO NOT MODIFY UNLESS PY2FG_ACT.XML IS ALSO MODIFIED ACCORDINGLY)
TELEM_TX_LEN = len(TELEM_TX_STR)

''' PLOT '''
# 0   | t_sim    | Sim. time              | -    | s
# 1   | dt_sim   | Sim. timestep          | -    | s
# 2   | long_d   | Delta long. from start | -    | m
# 3   | lat_d    | Delta lat. from start  | -    | m
# 4   | dist_d   | Distance from start    | -    | m
# 5   | pn_ecef  | North position         | ECEF | ft
# 6   | pe_ecef  | East position          | ECEF | ft
# 7   | pd_ecef  | Down position          | ECEF | ft
# 8   | long     | Longitude (GNSS)       | ECEF | rad
# 9   | lat      | Latitude (GNSS)        | ECEF | rad
# 10  | hgnss    | Altitude (GNSS)        | ECEF | km
# 11  | hqfe     | Baro. altitude (QFE)   | ECEF | km
# 12  | hqnh     | Baro. altitude (QNH)   | ECEF | km
# 13  | hterr    | Terrain elevation ASL  | ECEF | ft
# 14  | phi      | Roll ang.              | -    | rad
# 15  | theta    | Pitch ang.             | -    | rad
# 16  | psi      | Yaw ang.               | -    | rad
# 17  | alpha    | Attack ang.            | -    | rad/s
# 18  | beta     | Side-slip ang.         | -    | rad/s
# 19  | gamma    | Path ang.              | -    | rad
# 20  | vn       | North vel.             | NED  | ft/s
# 21  | ve       | East vel.              | NED  | ft/s
# 22  | vd       | Down vel.              | NED  | ft/s
# 23  | u        | Long. vel.             | XYZ  | ft/s
# 24  | v        | Transv. vel.           | XYZ  | ft/s
# 25  | w        | Vert. vel.             | XYZ  | ft/s
# 26  | uaero    | Long. aero. vel.       | XYZ  | ft/s
# 27  | vaero    | Transv. aero. vel.     | XYZ  | ft/s
# 28  | waero    | Vert. aero. vel.       | XYZ  | ft/s
# 29  | wn       | North wind vel.        | NED  | ft/s
# 30  | we       | East wind vel.         | NED  | ft/s
# 31  | wd       | Down wind vel.         | NED  | ft/s
# 32  | phidot   | Roll ang. change       | -    | rad/s
# 33  | thetadot | Pitch ang. change      | -    | rad/s
# 34  | psidot   | Yaw ang. change        | -    | rad/s
# 35  | p        | Roll vel.              | -    | rad/s
# 36  | q        | Pitch vel.             | -    | rad/s
# 37  | r        | Yaw vel.               | -    | rad/s
# 38  | paero    | Aero. roll vel.        | -    | rad/s
# 39  | qaero    | Aero. pitch vel.       | -    | rad/s
# 40  | raero    | Aero. yaw vel.         | -    | rad/s
# 41  | alphadot | Angle of attack change | -    | rad/s
# 42  | betadot  | Side-slip ang. change  | -    | rad/s
# 43  | udot     | Long. accel.           | XYZ  | ft/s^2
# 44  | vdot     | Transv. accel.         | XYZ  | ft/s^2
# 45  | wdot     | Vert. accel.           | XYZ  | ft/s^2
# 46  | pdot     | Roll accel.            | XYZ  | rad/s^2
# 47  | qdot     | Pitch accel.           | XYZ  | rad/s^2
# 48  | rdot     | Yaw accel.             | XYZ  | rad/s^2
# 49  | fxaero   | Long. aero. force      | XYZ  | lbs
# 50  | fxext    | Long. ext. force       | XYZ  | lbs
# 51  | fxgear   | Long. gear force       | XYZ  | lbs
# 52  | fxprop   | Long. prop. force      | XYZ  | lbs
# 53  | fx       | Long. total force      | XYZ  | lbs
# 54  | fyaero   | Transv. aero. force    | XYZ  | lbs
# 55  | fyext    | Transv. ext. force     | XYZ  | lbs
# 56  | fygear   | Transv. gear force     | XYZ  | lbs
# 57  | fyprop   | Transv. prop. force    | XYZ  | lbs
# 58  | fy       | Transv. total force    | XYZ  | lbs
# 59  | fzaero   | Vert. aero. force      | XYZ  | lbs
# 60  | fzext    | Vert. ext. force       | XYZ  | lbs
# 61  | fzgear   | Vert. gear force       | XYZ  | lbs
# 62  | fzprop   | Vert. prop. force      | XYZ  | lbs
# 63  | fz       | Vert. total force      | XYZ  | lbs
# 64  | laero    | Aero. roll mom.        | XYZ  | lbs·ft
# 65  | lext     | Ext. roll mom.         | XYZ  | lbs·ft
# 66  | lgear    | Gear roll mom.         | XYZ  | lbs·ft
# 67  | lprop    | Prop. roll mom.        | XYZ  | lbs·ft
# 68  | l        | Total roll mom.        | XYZ  | lbs·ft
# 69  | maero    | Aero. pitch mom.       | XYZ  | lbs·ft
# 70  | mext     | Ext. pitch mom.        | XYZ  | lbs·ft
# 71  | mgear    | Gear pitch mom.        | XYZ  | lbs·ft
# 72  | mprop    | Prop. pitch mom.       | XYZ  | lbs·ft
# 73  | m        | Total pitch mom.       | XYZ  | lbs·ft
# 74  | naero    | Aero. yaw mom.         | XYZ  | lbs·ft
# 75  | next     | Ext. yaw mom.          | XYZ  | lbs·ft
# 76  | ngear    | Gear yaw mom.          | XYZ  | lbs·ft
# 77  | nprop    | Prop. yaw mom.         | XYZ  | lbs·ft
# 78  | n        | Total yaw mom.         | XYZ  | lbs·ft
# 79  | sigmaa_r | R ail. pos.            | -    | rad
# 80  | deltaa_r | N. R ail. pos.         | -    | -
# 81  | sigmaa_l | L ail. pos.            | -    | rad
# 82  | deltaa_l | N. L ail. pos.         | -    | -
# 83  | sigmae   | Elevs. pos.            | -    | rad
# 84  | deltae   | N. elevs. pos.         | -    | -
# 85  | sigmaf   | Flaps pos.             | -    | rad
# 86  | deltaf   | N. flaps pos.          | -    | -
# 87  | sigmar   | Rudder pos.            | -    | rad
# 88  | deltar   | N. rudder pos.         | -    | -
# 89  | deltat   | N. eng. thrott. pos.   | -    | -
# 90  | deltam   | N. eng. mix. pos.      | -    | -
# 91  | up_down  | Upside-down            | -    | -
# 92  | wow1     | WOW1                   | -    | -
# 93  | wow2     | WOW2                   | -    | -
# 94  | wow3     | WOW3                   | -    | -
# 95  | Bw2Va    | Span-vel. ratio        | -    | s
# 96  | Cw2Va    | Chord-vel. ratio       | -    | s
# 97  | hmacb    | MAC-span ratio         | -    | -
# 98  | qbar     | Dyn. press.            | -    | psf
# 99  | qbaruw   | Dyn. press. UW         | -    | psf
# 100 | qbarprop | Dyn. press. prop.      | -    | psf
# 101 | qbarind  | Dyn. press. ind.       | -    | psf
# 102 | stall    | Stall                  | -    | -
# 103 | rho      | Air density            | -    | slug/ft^3
# 104 | J        | Advance ratio          | -    | -
# 105 | rpmprop  | Prop. RPM              | -    | -
# 106 | Ixx      | Mom. inert. XX         | -    | slug/ft^2
# 107 | Ixy      | Mom. inert. XY         | -    | slug/ft^2
# 108 | Ixz      | Mom. inert. XZ         | -    | slug/ft^2
# 109 | Iyy      | Mom. inert. YY         | -    | slug/ft^2
# 110 | Iyz      | Mom. inert. YZ         | -    | slug/ft^2
# 111 | Izz      | Mom. inert. ZZ         | -    | slug/ft^2
# 112 | mass     | Mass                   | -    | slug
# 113 | g        | Grav. accel.           | -    | ft/s^2
TELEM_RX_PLOT = {
                 'Sim. time' :              [['s',('s','s')], r'$t_{sim}$'],
                 'Sim. timestep' :          [['s',('s','s')], r'$dt_{sim}$'],
                 'Delta long. from start' : [['m',('m','ft')], r'$\Delta_{\Lambda}$'],
                 'Delta lat. from start' :  [['m',('m','ft')], r'$\Delta_{\Phi}$'],
                 'Distance from start' :    [['m',('m','ft')], r'$\Delta_{dist}$'],
                 'Longitude' :              [['rad',('rad','rad')], r'$\Lambda$'],
                 'Latitude' :               [['rad',('rad','rad')], r'$\Phi$'],
                 'Altitude (GNSS)' :        [['km',('m','ft')], r'$h_{GNSS}$'],
                 'Baro. altitude (QFE)' :   [['km',('m','ft')], r'$h_{QFE}$'],
                 'Baro. altitude (QNH)' :   [['km',('m','ft')], r'$h_{QNH}$'],
                 'Terrain elevation ASL' :  [['ft',('m','ft')], r'$h_{terr}$'],
                 'Roll ang.' :              [['rad',('rad','rad')], r'$\phi$'],
                 'Pitch ang.' :             [['rad',('rad','rad')], r'$\theta$'],
                 'Yaw ang.' :               [['rad',('rad','rad')], r'$\psi$'],
                 'Attack ang.' :            [['rad',('rad','rad')], r'$\alpha$'],
                 'Side-slip ang.' :         [['rad',('rad','rad')], r'$\beta$'],
                 'Path ang.' :              [['rad',('rad','rad')], r'$\gamma$'],
                 'North vel.' :             [['ft/s',('m/s', 'ft/s')], r'$v_n$'],
                 'East vel.' :              [['ft/s',('m/s','ft/s')], r'$v_e$'],
                 'Down vel.' :              [['ft/s',('m/s','ft/s')], r'$v_d$'],
                 'Long. vel.' :             [['ft/s',('m/s','ft/s')], r'$u$'],
                 'Transv. vel.' :           [['ft/s',('m/s','ft/s')], r'$v$'],
                 'Vert. vel.' :             [['ft/s',('m/s','ft/s')], r'$w$'],
                 'Long. aero. vel.' :       [['ft/s',('m/s','ft/s')], r'$u_{aero}$'],
                 'Transv. aero. vel.' :     [['ft/s',('m/s','ft/s')], r'$v_{aero}$'],
                 'Vert. aero. vel.' :       [['ft/s',('m/s','ft/s')], r'$w_{aero}$'],
                 'North wind vel.' :        [['ft/s',('m/s','ft/s')], r'$w_n$'],
                 'East wind vel.' :         [['ft/s',('m/s','ft/s')], r'$w_e$'],
                 'Down wind vel.' :         [['ft/s',('m/s','ft/s')], r'$w_d$'],
                 'Roll ang. change' :       [['rad/s',('rad/s','rad/s')], r'$\dot \phi$'],
                 'Pitch ang. change' :      [['rad/s',('rad/s','rad/s')], r'$\dot \theta$'],
                 'Yaw ang. change' :        [['rad/s',('rad/s','rad/s')], r'$\dot \psi$'],
                 'Roll vel.' :              [['rad/s',('rad/s','rad/s')], r'$p$'],
                 'Pitch vel.' :             [['rad/s',('rad/s','rad/s')], r'$q$'],
                 'Yaw vel.' :               [['rad/s',('rad/s','rad/s')], r'$r$'],
                 'Aero. roll vel.' :        [['rad/s',('rad/s','rad/s')], r'$p_{aero}$'],
                 'Aero. pitch vel.' :       [['rad/s',('rad/s','rad/s')], r'$q_{aero}$'],
                 'Aero. yaw vel.' :         [['rad/s',('rad/s','rad/s')], r'$r_{aero}$'],
                 'Attack ang. change' :     [['rad/s',('rad/s','rad/s')], r'$\dot \alpha$'],
                 'Side-slip ang. change' :  [['rad/s',('rad/s','rad/s')], r'$\dot \beta$'],
                 'Long. accel.' :           [['ft/s^2',('m/s²','ft/s²')], r'$\dot u$'],
                 'Transv. accel.' :         [['ft/s^2',('m/s²','ft/s²')], r'$\dot v$'],
                 'Vert. accel.' :           [['ft/s^2',('m/s²','ft/s²')], r'$\dot w$'],
                 'Roll accel.' :            [['rad/s^2',('rad/s²','rad/s²')], r'$\dot p$'],
                 'Pitch accel.' :           [['rad/s^2',('rad/s²','rad/s²')], r'$\dot q$'],
                 'Yaw accel.' :             [['rad/s^2',('rad/s²','rad/s²')], r'$\dot r$'],
                 'Long. aero. force' :      [['lbs',('N','lbs')], r'$fx_{aero}$'],
                 'Long. ext. force' :       [['lbs',('N','lbs')], r'$fx_{ext}$'],
                 'Long. gear force' :       [['lbs',('N','lbs')], r'$fx_{gear}$'],
                 'Long. prop. force' :      [['lbs',('N','lbs')], r'$fx_{prop}$'],
                 'Long. total force' :      [['lbs',('N','lbs')], r'$fx$'],
                 'Transv. aero. force' :    [['lbs',('N','lbs')], r'$fy_{aero}$'],
                 'Transv. ext. force' :     [['lbs',('N','lbs')], r'$fy_{ext}$'],
                 'Transv. gear force' :     [['lbs',('N','lbs')], r'$fy_{gear}$'],
                 'Transv. prop. force' :    [['lbs',('N','lbs')], r'$fy_{prop}$'],
                 'Transv. total force' :    [['lbs',('N','lbs')], r'$fy$'],
                 'Vert. aero. force' :      [['lbs',('N','lbs')], r'$fz_{aero}$'],
                 'Vert. ext. force' :       [['lbs',('N','lbs')], r'$fz_{ext}$'],
                 'Vert. gear force' :       [['lbs',('N','lbs')], r'$fz_{gear}$'],
                 'Vert. prop. force' :      [['lbs',('N','lbs')], r'$fz_{prop}$'],
                 'Vert. total force' :      [['lbs',('N','lbs')], r'$fz$'],
                 'Aero. roll mom.' :        [['lbs·ft',('N·m','lbs·ft')], r'$l_{aero}$'],
                 'Ext. roll mom.' :         [['lbs·ft',('N·m','lbs·ft')], r'$l_{ext}$'],
                 'Gear roll mom.' :         [['lbs·ft',('N·m','lbs·ft')], r'$l_{gear}$'],
                 'Prop. roll mom.' :        [['lbs·ft',('N·m','lbs·ft')], r'$l_{prop}$'],
                 'Total roll mom.' :        [['lbs·ft',('N·m','lbs·ft')], r'$l$'],
                 'Aero. pitch mom.' :       [['lbs·ft',('N·m','lbs·ft')], r'$m_{aero}$'],
                 'Ext. pitch mom.' :        [['lbs·ft',('N·m','lbs·ft')], r'$m_{ext}$'],
                 'Gear pitch mom.' :        [['lbs·ft',('N·m','lbs·ft')], r'$m_{gear}$'],
                 'Prop. pitch mom.' :       [['lbs·ft',('N·m','lbs·ft')], r'$m_{prop}$'],
                 'Total pitch mom.' :       [['lbs·ft',('N·m','lbs·ft')], r'$m$'],
                 'Aero. yaw mom.' :         [['lbs·ft',('N·m','lbs·ft')], r'$n_{aero}$'],
                 'Ext. yaw mom.' :          [['lbs·ft',('N·m','lbs·ft')], r'$n_{ext}$'],
                 'Gear yaw mom.' :          [['lbs·ft',('N·m','lbs·ft')], r'$n_{gear}$'],
                 'Prop. yaw mom.' :         [['lbs·ft',('N·m','lbs·ft')], r'$n_{prop}$'],
                 'Total yaw mom.' :         [['lbs·ft',('N·m','lbs·ft')], r'$n$'],
                 'R ail. pos.' :            [['rad',('rad','rad')], r'$\sigma_{ra}$'],
                 'N. R ail. pos.' :         [['-',('-','-')], r'$\delta_{ra}$'],
                 'L ail. pos.' :            [['rad',('rad','rad')], r'$\sigma_{la}$'],
                 'N. L ail. pos.' :         [['-',('-','-')], r'$\delta_{la}$'],
                 'Elevs. pos.' :            [['rad',('rad','rad')], r'$\sigma_e$'],
                 'N. elevs. pos.' :         [['-',('-','-')], r'$\delta_e$'],
                 'Flaps pos.' :             [['rad',('rad','rad')], r'$\sigma_f$'],
                 'N. flaps pos.' :          [['-',('-','-')], r'$\delta_f$'],
                 'Rudder pos.' :            [['rad',('rad','rad')], r'$\sigma_r$'],
                 'N. rudder pos.' :         [['-',('-','-')], r'$\delta_r$'],
                 'N. eng. thrott. pos.' :   [['-',('-','-')], r'$\delta_t$'],
                 'N. eng. mix. pos.' :      [['-',('-','-')], r'$\delta_m$'],
                 'Upside-down' :            [['-',('-','-')], ''],
                 'WOW1' :                   [['-',('-','-')], ''],
                 'WOW2' :                   [['-',('-','-')], ''],
                 'WOW3' :                   [['-',('-','-')], ''],
                 'Span-vel. ratio' :        [['s',('s','s')], ''],
                 'Chord-vel. ratio' :       [['s',('s','s')], ''],
                 'MAC-span ratio' :         [['-',('-','-')], ''],
                 'Dyn. press.' :            [['N/m^2',('N/m²','psf')], r'$\Bar{q}$'],
                 'Dyn. press. UW' :         [['N/m^2',('N/m²','psf')], r'$\Bar{q_{UW}}$'],
                 'Dyn. press. prop.' :      [['N/m^2',('N/m²','psf')], r'$\Bar{q_{prop}}$'],
                 'Dyn. press. ind.' :       [['N/m^2',('N/m²','psf')], r'$\Bar{q_{ind}}$'],
                 'Stall' :                  [['-',('-','-')], ''],
                 'Advance ratio' :          [['-',('-','-')], r'$J$'],
                 'Prop. RPM' :              [['rev/min',('rev/min','rev/min')], ''],
                 'Mom. inert. XX' :         [['slug/ft^2',('kg/m²','slug/ft²')], r'$I_{XX}$'],
                 'Mom. inert. XY' :         [['slug/ft^2',('kg/m²','slug/ft²')], r'$I_{XY}$'],
                 'Mom. inert. XZ' :         [['slug/ft^2',('kg/m²','slug/ft²')], r'$I_{XZ}$'],
                 'Mom. inert. YY' :         [['slug/ft^2',('kg/m²','slug/ft²')], r'$I_{YY}$'],
                 'Mom. inert. YZ' :         [['slug/ft^2',('kg/m²','slug/ft²')], r'$I_{YZ}$'],
                 'Mom. inert. ZZ' :         [['slug/ft^2',('kg/m²','slug/ft²')], r'$I_{ZZ}$'],
                 'Mass' :                   [['slug',('kg','slug')], r'm'],
                 'Grav. accel.' :           [['ft/s^2',('m/s²','ft/s²')], r'$g$']
                }
TELEM_TX_PLOT = {
                 'N. ails. cmd' :        [['-',('-','-')], r'$\delta_a$'],
                 'N. elevs. cmd' :       [['-',('-','-')], r'$\delta_e$'],
                 'N. rudder cmd' :       [['-',('-','-')], r'$\delta_r$'],
                 'N. flaps cmd' :        [['-',('-','-')], r'$\delta_f$'],
                 'N. eng. thrott. cmd' : [['-',('-','-')], r'$\delta_t$'],
                 'N. eng. mix. cmd' :    [['-',('-','-')], r'$\delta_{mix}$'],
                 'N. ails. trim cmd' :   [['-',('-','-')], r'$\delta_{a_{trim}}$'],
                 'N. elevs. trim cmd' :  [['-',('-','-')], r'$\delta_{e_{trim}}$'],
                 'N. rudder trim cmd' :  [['-',('-','-')], r'$\delta_{r_{trim}}$']
                }
