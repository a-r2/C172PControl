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

# 0  | pn    | North position                               | NED | m
# 1  | pe    | East position                                | NED | m
# 2  | pd    | Down position                                | NED | m
# 3  | phi   | Roll angle                                   | -   | rad
# 4  | theta | Pitch angle                                  | -   | rad
# 5  | psi   | Yaw angle                                    | -   | rad
# 6  | u     | Longitudinal velocity                        | XYZ | m/s
# 7  | v     | Transversal velocity                         | XYZ | m/s
# 8  | w     | Vertical velocity                            | XYZ | m/s
# 9  | p     | Roll velocity                                | XYZ | rad/s
# 10 | q     | Pitch velocity                               | XYZ | rad/s
# 11 | r     | Yaw velocity                                 | XYZ | rad/s
CM_STATE_STR = ('pn', 'pe', 'pd', 'phi', 'theta', 'psi', 'u', 'v', 'w', 'p', 'q', 'r')
CM_STATE_LEN = len(CM_STATE_STR)

''' DYNAMICS '''
# 0  | D  | Longitudinal aerodynamic force       | UVW | N
# 1  | C  | Transversal aerodynamic force        | UVW | N
# 2  | L  | Vertical aerodynamic force           | UVW | N
# 3  | T  | Engine thrust force                  | UVW | N
# 4  | gx | Longitudinal gravitational force     | UVW | N
# 5  | gy | Transversal gravitational force      | UVW | N
# 6  | gz | Vertical gravitational force         | UVW | N
# 7  | l  | Longitudinal moment                  | UVW | N·m
# 8  | m  | Transversal moment                   | UVW | N·m
# 9  | n  | Vertical moment                      | UVW | N·m
# 10 | D1 | Aerodynamic drag contribution 1      | -   | -
# 11 | D2 | Aerodynamic drag contribution 2      | -   | -
# 12 | D3 | Aerodynamic drag contribution 3      | -   | -
# 13 | D4 | Aerodynamic drag contribution 4      | -   | -
# 14 | C1 | Aerodynamic crosswind contribution 1 | -   | -
# 15 | C2 | Aerodynamic crosswind contribution 2 | -   | -
# 16 | L1 | Aerodynamic lift contribution 1      | -   | -
# 17 | L2 | Aerodynamic lift contribution 2      | -   | -
# 18 | L3 | Aerodynamic lift contribution 3      | -   | -
# 19 | L4 | Aerodynamic lift contribution 4      | -   | -
# 20 | L5 | Aerodynamic lift contribution 5      | -   | -
# 21 | l1 | Aerodynamic roll contribution 1      | -   | -
# 22 | l2 | Aerodynamic roll contribution 2      | -   | -
# 23 | l3 | Aerodynamic roll contribution 3      | -   | -
# 24 | l4 | Aerodynamic roll contribution 4      | -   | -
# 25 | l5 | Aerodynamic roll contribution 5      | -   | -
# 26 | m1 | Aerodynamic pitch contribution 1     | -   | -
# 27 | m2 | Aerodynamic pitch contribution 2     | -   | -
# 28 | m3 | Aerodynamic pitch contribution 3     | -   | -
# 29 | m4 | Aerodynamic pitch contribution 4     | -   | -
# 30 | m5 | Aerodynamic pitch contribution 5     | -   | -
# 31 | m6 | Aerodynamic pitch contribution 6     | -   | -
# 32 | n1 | Aerodynamic yaw contribution 1       | -   | -
# 33 | n2 | Aerodynamic yaw contribution 2       | -   | -
# 34 | n3 | Aerodynamic yaw contribution 3       | -   | -
# 35 | n4 | Aerodynamic yaw contribution 4       | -   | -
# 36 | n5 | Aerodynamic yaw contribution 5       | -   | -
# 37 | n6 | Aerodynamic yaw contribution 6       | -   | -
DYN_STR = ('D', 'C', 'L', 'T', 'gx', 'gy', 'gz', 'l', 'm', 'n', 'CD1', 'CD2', 'CD3', 'CD4', 'CC1', 'CC2', 'CL1', 'CL2', 'CL3', 'CL4', 'CL5', 'Cl1', 'Cl2', 'Cl3', 'Cl4', 'Cl5', 'Cm1', 'Cm2', 'Cm3', 'Cm4', 'Cm5', 'Cm6', 'Cn1', 'Cn2', 'Cn3', 'Cn4', 'Cn5', 'Cn6')
DYN_LEN = len(DYN_STR)

''' RX TELEMETRY '''
# 0   | t_sim      | Simulation time from start                                    | -    | s
# 1   | dt_sim     | Simulation timestep                                           | -    | s
# 2   | long_delta | Delta longitude from start                                    | -    | m
# 3   | lat_delta  | Delta latitude from start                                     | -    | m
# 4   | dist_delta | Distance from start                                           | -    | m
# 5   | pn_ecef    | North position                                                | ECEF | ft
# 6   | pe_ecef    | East position                                                 | ECEF | ft
# 7   | pd_ecef    | Down position                                                 | ECEF | ft
# 8   | long_gnss  | Longitude (GNSS)                                              | ECEF | rad
# 9   | lat_gnss   | Latitude (GNSS)                                               | ECEF | rad
# 10  | h_gnss     | Altitude (GNSS)                                               | ECEF | km
# 11  | h_qfe      | Barometric altitude AGL (QFE)                                 | ECEF | km
# 12  | h_qnh      | Barometric altitude ASL (QNH)                                 | ECEF | m
# 13  | h_terr     | Terrain elevation ASL                                         | ECEF | ft
# 14  | phi        | Roll angle                                                    | -    | rad
# 15  | theta      | Pitch angle                                                   | -    | rad
# 16  | psi        | Yaw angle                                                     | -    | rad
# 17  | alpha      | Angle of attack                                               | -    | rad
# 18  | beta       | Side-slip angle                                               | -    | rad
# 19  | gamma      | Path angle                                                    | -    | rad
# 20  | vn         | North velocity                                                | NED  | ft/s
# 21  | ve         | East velocity                                                 | NED  | ft/s
# 22  | vd         | Down velocity                                                 | NED  | ft/s
# 23  | u          | Longitudinal velocity                                         | XYZ  | ft/s
# 24  | v          | Transversal velocity                                          | XYZ  | ft/s
# 25  | w          | Vertical velocity                                             | XYZ  | ft/s 
# 26  | u_aero     | Aerodynamic longitudinal velocity                             | XYZ  | ft/s
# 27  | v_aero     | Aerodynamic transversal velocity                              | XYZ  | ft/s
# 28  | w_aero     | Aerodynamic vertical velocity                                 | XYZ  | ft/s
# 29  | wn         | North wind velocity                                           | NED  | ft/s
# 30  | we         | East wind velocity                                            | NED  | ft/s
# 31  | wd         | Down wind velocity                                            | NED  | ft/s
# 32  | phidot     | Rate of change of roll angle                                  | -    | rad/s
# 33  | thetadot   | Rate of change of pitch angle                                 | -    | rad/s
# 34  | psidot     | Rate of change of yaw angle                                   | -    | rad/s
# 35  | p          | Roll velocity                                                 | XYZ  | rad/s
# 36  | q          | Pitch velocity                                                | XYZ  | rad/s
# 37  | r          | Yaw velocity                                                  | XYZ  | rad/s
# 38  | p_aero     | Aerodynamic roll velocity                                     | XYZ  | rad/s
# 39  | q_aero     | Aerodynamic pitch velocity                                    | XYZ  | rad/s
# 40  | r_aero     | Aerodynamic yaw velocity                                      | XYZ  | rad/s
# 41  | alphadot   | Rate of change of angle of attack                             | -    | rad/s
# 42  | betadot    | Rate of change of side-slip angle                             | -    | rad/s
# 43  | udot       | Longitudinal acceleration                                     | XYZ  | ft/s^2
# 44  | vdot       | Transversal acceleration                                      | XYZ  | ft/s^2
# 45  | wdot       | Vertical acceleration                                         | XYZ  | ft/s^2
# 46  | pdot       | Roll acceleration                                             | XYZ  | rad/s^2
# 47  | qdot       | Pitch acceleration                                            | XYZ  | rad/s^2
# 48  | rdot       | Yaw acceleration                                              | XYZ  | rad/s^2
# 49  | fx_aero    | Longitudinal aerodynamic force                                | XYZ  | lbs
# 50  | fx_ext     | Longitudinal external force                                   | XYZ  | lbs
# 51  | fx_gear    | Longitudinal gear force                                       | XYZ  | lbs
# 52  | fx_prop    | Longitudinal propeller force                                  | XYZ  | lbs
# 53  | fx         | Longitudinal total force                                      | XYZ  | lbs
# 54  | fy_aero    | Transversal aerodynamic force                                 | XYZ  | lbs
# 55  | fy_ext     | Transversal external force                                    | XYZ  | lbs
# 56  | fy_gear    | Transversal gear force                                        | XYZ  | lbs
# 57  | fy_prop    | Transversal propeller force                                   | XYZ  | lbs
# 58  | fy         | Transversal total force                                       | XYZ  | lbs
# 59  | fz_aero    | Vertical aerodynamic force                                    | XYZ  | lbs
# 60  | fz_ext     | Vertical external force                                       | XYZ  | lbs
# 61  | fz_gear    | Vertical gear force                                           | XYZ  | lbs
# 62  | fz_prop    | Vertical propeller force                                      | XYZ  | lbs
# 63  | fz         | Vertical total force                                          | XYZ  | lbs
# 64  | l_aero     | Aerodynamic roll moment                                       | XYZ  | lbs·ft
# 65  | l_ext      | External roll moment                                          | XYZ  | lbs·ft
# 66  | l_gear     | Gear roll moment                                              | XYZ  | lbs·ft
# 67  | l_prop     | Propeller roll moment                                         | XYZ  | lbs·ft
# 68  | l          | Total roll moment                                             | XYZ  | lbs·ft
# 69  | m_aero     | Aerodynamic pitch moment                                      | XYZ  | lbs·ft
# 70  | m_ext      | External pitch moment                                         | XYZ  | lbs·ft
# 71  | m_gear     | Gear pitch moment                                             | XYZ  | lbs·ft
# 72  | m_prop     | Propeller pitch moment                                        | XYZ  | lbs·ft
# 73  | m          | Total pitch moment                                            | XYZ  | lbs·ft
# 74  | n_aero     | Aerodynamic yaw moment                                        | XYZ  | lbs·ft
# 75  | n_ext      | External yaw moment                                           | XYZ  | lbs·ft
# 76  | n_gear     | Gear yaw moment                                               | XYZ  | lbs·ft
# 77  | n_prop     | Propeller yaw moment                                          | XYZ  | lbs·ft
# 78  | n          | Total yaw moment                                              | XYZ  | lbs·ft
# 79  | sigmara    | Right aileron position                                        | -    | rad
# 80  | deltara    | Normalized right aileron position                             | -    | -
# 81  | sigmala    | Left aileron position                                         | -    | rad
# 82  | deltala    | Normalized left aileron position                              | -    | -
# 83  | sigmae     | Elevators position                                            | -    | rad
# 84  | deltae     | Normalized elevators position                                 | -    | -
# 85  | sigmaf     | Flaps position                                                | -    | rad
# 86  | deltaf     | Normalized flaps position                                     | -    | -
# 87  | sigmar     | Rudder position                                               | -    | rad
# 88  | deltar     | Normalized rudder position                                    | -    | -
# 89  | deltat     | Normalized engine throttle position                           | -    | -
# 90  | deltam     | Normalized engine mixture position                            | -    | -
# 91  | Bw2Va      | Wing span divided by twice the aerodynamic velocity           | -    | s
# 92  | Cw2Va      | Wing chord divided by twice the aerodynamic velocity          | -    | s
# 93  | hmacb      | Altitude of mean aerodynamic chord (MAC) divided by wing span | -    | -
# 94  | qbar       | Dynamic pressure                                              | -    | psf
# 95  | qbaruw     | Dynamic pressure (UW plane of wind frame)                     | -    | psf
# 96  | qbarprop   | Dynamic pressure due to propeller induced velocity            | -    | psf
# 97  | qbarind    | Dynamic pressure including propulsion induced velocity        | -    | psf
# 98  | stall      | Normalized stall hysteresis                                   | -    | -
# 99  | rho        | Air density                                                   | -    | slug/ft^3
# 100 | J          | Advance ratio                                                 | -    | -
# 101 | revprop    | Propeller revolutions                                         | -    | rev/min
# 102 | Ixx        | Moment of inercia Ixx                                         | -    | slug/ft^2
# 103 | Ixy        | Moment of inercia Ixy                                         | -    | slug/ft^2
# 104 | Ixz        | Moment of inercia Ixz                                         | -    | slug/ft^2
# 105 | Iyy        | Moment of inercia Iyy                                         | -    | slug/ft^2
# 106 | Iyz        | Moment of inercia Iyz                                         | -    | slug/ft^2
# 107 | Izz        | Moment of inercia Izz                                         | -    | slug/ft^2
# 108 | mass       | Mass                                                          | -    | slug
# 109 | grav       | Gravitational acceleration                                    | -    | ft/s^2
# 110 | up_down    | Upside-down state                                             | -    | -
# 111 | wow1       | Wheel 1 weight-on-wheel state                                 | -    | -
# 112 | wow2       | Wheel 2 weight-on-wheel state                                 | -    | -
# 113 | wow3       | Wheel 3 weight-on-wheel state                                 | -    | -
# 114 | wc1        | Wheel 1 contact state                                         | -    | -
# 115 | wc2        | Wheel 2 contact state                                         | -    | -
# 116 | wc3        | Wheel 3 contact state                                         | -    | -
# 117 | D1         | Aerodynamic drag contribution 1                               | UVW  | N
# 118 | D2         | Aerodynamic drag contribution 2                               | UVW  | N
# 119 | D3         | Aerodynamic drag contribution 3                               | UVW  | N
# 120 | D4         | Aerodynamic drag contribution 4                               | UVW  | N
# 121 | C1         | Aerodynamic crosswind contribution 1                          | UVW  | N
# 122 | C2         | Aerodynamic crosswind contribution 2                          | UVW  | N
# 123 | L1         | Aerodynamic lift contribution 1                               | UVW  | N
# 124 | L2         | Aerodynamic lift contribution 2                               | UVW  | N
# 125 | L3         | Aerodynamic lift contribution 3                               | UVW  | N
# 126 | L4         | Aerodynamic lift contribution 4                               | UVW  | N
# 127 | L5         | Aerodynamic lift contribution 5                               | UVW  | N
# 128 | l1         | Aerodynamic roll contribution 1                               | XYZ  | N
# 129 | l2         | Aerodynamic roll contribution 2                               | XYZ  | N
# 130 | l3         | Aerodynamic roll contribution 3                               | XYZ  | N
# 131 | l4         | Aerodynamic roll contribution 4                               | XYZ  | N
# 132 | l5         | Aerodynamic roll contribution 5                               | XYZ  | N
# 133 | m1         | Aerodynamic pitch contribution 1                              | XYZ  | N
# 134 | m2         | Aerodynamic pitch contribution 2                              | XYZ  | N
# 135 | m3         | Aerodynamic pitch contribution 3                              | XYZ  | N
# 136 | m4         | Aerodynamic pitch contribution 4                              | XYZ  | N
# 137 | m5         | Aerodynamic pitch contribution 5                              | XYZ  | N
# 138 | m6         | Aerodynamic pitch contribution 6                              | XYZ  | N
# 139 | n1         | Aerodynamic yaw contribution 1                                | XYZ  | N
# 140 | n2         | Aerodynamic yaw contribution 2                                | XYZ  | N
# 141 | n3         | Aerodynamic yaw contribution 3                                | XYZ  | N
# 142 | n4         | Aerodynamic yaw contribution 4                                | XYZ  | N
# 143 | n5         | Aerodynamic yaw contribution 5                                | XYZ  | N
# 144 | n6         | Aerodynamic yaw contribution 6                                | XYZ  | N
# 145 | Vprop      | Propeller induced velocity                                    | XYZ  | ft/s
# 146 | Vind       | Velocity including the propulsion induced velocity            | XYZ  | ft/s
TELEM_RX_STR = ('t_sim', 'dt_sim', 'long_delta', 'lat_delta', 'dist_delta', 'pn_ecef', 'pe_ecef', 'pd_ecef', 'long_gnss', 'lat_gnss', 'h_gnss', 'h_qfe', 'h_qnh', 'h_terr', 'phi', 'theta', 'psi', 'alpha', 'beta', 'gamma', 'vn', 've', 'vd', 'u', 'v', 'w', 'u_aero', 'v_aero', 'w_aero', 'wn', 'we', 'wd', 'phidot', 'thetadot', 'psidot', 'p', 'q', 'r', 'p_aero', 'q_aero', 'r_aero', 'alphadot', 'betadot', 'udot', 'vdot', 'wdot', 'pdot', 'qdot', 'rdot', 'fx_aero', 'fx_ext', 'fx_gear', 'fx_prop', 'fx', 'fy_aero', 'fy_ext', 'fy_gear', 'fy_prop', 'fy', 'fz_aero', 'fz_ext', 'fz_gear', 'fz_prop', 'fz', 'l_aero', 'l_ext', 'l_gear', 'l_prop', 'l', 'm_aero', 'm_ext', 'm_gear', 'm_prop', 'm', 'n_aero', 'n_ext', 'n_gear', 'n_prop', 'n', 'sigmara', 'deltara', 'sigmala', 'deltala', 'sigmae', 'deltae', 'sigmaf', 'deltaf', 'sigmar', 'deltar', 'deltat', 'deltam', 'Bw2Va', 'Cw2Va', 'hmacb', 'qbar', 'qbaruw', 'qbarprop', 'qbarind', 'stall', 'rho', 'J', 'revprop', 'Ixx', 'Ixy', 'Ixz', 'Iyy', 'Iyz', 'Izz', 'mass', 'grav', 'up_down', 'wow1', 'wow2', 'wow3', 'wc1', 'wc2', 'wc3', 'D1', 'D2', 'D3', 'D4', 'C1', 'C2', 'L1', 'L2', 'L3', 'L4', 'L5', 'l1', 'l2', 'l3', 'l4', 'l5', 'm1', 'm2', 'm3', 'm4', 'm5', 'm6', 'n1', 'n2', 'n3', 'n4', 'n5', 'n6', 'Vprop', 'Vind') #RX telemetry str tuple (DO NOT MODIFY UNLESS FG2PY.XML IS ALSO MODIFIED ACCORDINGLY)
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
# 0   | t_sim      | Sim. time                    | -    | s
# 1   | dt_sim     | Sim. timestep                | -    | s
# 2   | long_delta | Delta long. from start       | -    | m
# 3   | lat_delta  | Delta lat. from start        | -    | m
# 4   | dist_delta | Distance from start          | -    | m
# 5   | pn_ecef    | North position               | ECEF | m
# 6   | pe_ecef    | East position                | ECEF | m
# 7   | pd_ecef    | Down position                | ECEF | m
# 8   | long_gnss  | Longitude (GNSS)             | ECEF | rad
# 9   | lat_gnss   | Latitude (GNSS)              | ECEF | rad
# 10  | h_gnss     | Altitude (GNSS)              | ECEF | m
# 11  | h_qfe      | Baro. altitude (QFE)         | ECEF | m
# 12  | h_qnh      | Baro. altitude (QNH)         | ECEF | m
# 13  | h_terr     | Terrain elevation ASL        | ECEF | m
# 14  | phi        | Roll ang.                    | -    | rad
# 15  | theta      | Pitch ang.                   | -    | rad
# 16  | psi        | Yaw ang.                     | -    | rad
# 17  | alpha      | Attack ang.                  | -    | rad/s
# 18  | beta       | Side-slip ang.               | -    | rad/s
# 19  | gamma      | Path ang.                    | -    | rad
# 20  | vn         | North vel.                   | NED  | m/s
# 21  | ve         | East vel.                    | NED  | m/s
# 22  | vd         | Down vel.                    | NED  | m/s
# 23  | u          | Long. vel.                   | XYZ  | m/s
# 24  | v          | Transv. vel.                 | XYZ  | m/s
# 25  | w          | Vert. vel.                   | XYZ  | m/s
# 26  | u_aero     | Long. aero. vel.             | XYZ  | m/s
# 27  | v_aero     | Transv. aero. vel.           | XYZ  | m/s
# 28  | w_aero     | Vert. aero. vel.             | XYZ  | m/s
# 29  | wn         | North wind vel.              | NED  | m/s
# 30  | we         | East wind vel.               | NED  | m/s
# 31  | wd         | Down wind vel.               | NED  | m/s
# 32  | phidot     | Roll ang. change             | -    | rad/s
# 33  | thetadot   | Pitch ang. change            | -    | rad/s
# 34  | psidot     | Yaw ang. change              | -    | rad/s
# 35  | p          | Roll vel.                    | -    | rad/s
# 36  | q          | Pitch vel.                   | -    | rad/s
# 37  | r          | Yaw vel.                     | -    | rad/s
# 38  | p_aero     | Aero. roll vel.              | -    | rad/s
# 39  | q_aero     | Aero. pitch vel.             | -    | rad/s
# 40  | r_aero     | Aero. yaw vel.               | -    | rad/s
# 41  | alphadot   | Angle of attack change       | -    | rad/s
# 42  | betadot    | Side-slip ang. change        | -    | rad/s
# 43  | udot       | Long. accel.                 | XYZ  | m/s^2
# 44  | vdot       | Transv. accel.               | XYZ  | m/s^2
# 45  | wdot       | Vert. accel.                 | XYZ  | m/s^2
# 46  | pdot       | Roll accel.                  | XYZ  | rad/s^2
# 47  | qdot       | Pitch accel.                 | XYZ  | rad/s^2
# 48  | rdot       | Yaw accel.                   | XYZ  | rad/s^2
# 49  | fx_aero    | Long. aero. force            | XYZ  | N
# 50  | fx_ext     | Long. ext. force             | XYZ  | N
# 51  | fx_gear    | Long. gear force             | XYZ  | N
# 52  | fx_prop    | Long. prop. force            | XYZ  | N
# 53  | fx         | Long. total force            | XYZ  | N
# 54  | fy_aero    | Transv. aero. force          | XYZ  | N
# 55  | fy_ext     | Transv. ext. force           | XYZ  | N
# 56  | fy_gear    | Transv. gear force           | XYZ  | N
# 57  | fy_prop    | Transv. prop. force          | XYZ  | N
# 58  | fy         | Transv. total force          | XYZ  | N
# 59  | fz_aero    | Vert. aero. force            | XYZ  | N
# 60  | fz_ext     | Vert. ext. force             | XYZ  | N
# 61  | fz_gear    | Vert. gear force             | XYZ  | N
# 62  | fz_prop    | Vert. prop. force            | XYZ  | N
# 63  | fz         | Vert. total force            | XYZ  | N
# 64  | l_aero     | Aero. roll mom.              | XYZ  | N·m
# 65  | l_ext      | Ext. roll mom.               | XYZ  | N·m
# 66  | l_gear     | Gear roll mom.               | XYZ  | N·m
# 67  | l_prop     | Prop. roll mom.              | XYZ  | N·m
# 68  | l          | Total roll mom.              | XYZ  | N·m
# 69  | m_aero     | Aero. pitch mom.             | XYZ  | N·m
# 70  | m_ext      | Ext. pitch mom.              | XYZ  | N·m
# 71  | m_gear     | Gear pitch mom.              | XYZ  | N·m
# 72  | m_prop     | Prop. pitch mom.             | XYZ  | N·m
# 73  | m          | Total pitch mom.             | XYZ  | N·m
# 74  | n_aero     | Aero. yaw mom.               | XYZ  | N·m
# 75  | n_ext      | Ext. yaw mom.                | XYZ  | N·m
# 76  | n_gear     | Gear yaw mom.                | XYZ  | N·m
# 77  | n_prop     | Prop. yaw mom.               | XYZ  | N·m
# 78  | n          | Total yaw mom.               | XYZ  | N·m
# 79  | sigmara    | R ail. pos.                  | -    | rad
# 80  | deltara    | N. R ail. pos.               | -    | -
# 81  | sigmala    | L ail. pos.                  | -    | rad
# 82  | deltala    | N. L ail. pos.               | -    | -
# 83  | sigmae     | Elevs. pos.                  | -    | rad
# 84  | deltae     | N. elevs. pos.               | -    | -
# 85  | sigmaf     | Flaps pos.                   | -    | rad
# 86  | deltaf     | N. flaps pos.                | -    | -
# 87  | sigmar     | Rudder pos.                  | -    | rad
# 88  | deltar     | N. rudder pos.               | -    | -
# 89  | deltat     | N. eng. thrott. pos.         | -    | -
# 90  | deltam     | N. eng. mix. pos.            | -    | -
# 91  | Bw2Va      | Span-vel. ratio              | -    | s
# 92  | Cw2Va      | Chord-vel. ratio             | -    | s
# 93  | hmacb      | MAC-span ratio               | -    | -
# 94  | qbar       | Dyn. press.                  | -    | Pa
# 95  | qbaruw     | Dyn. press. UW               | -    | Pa
# 96  | qbarprop   | Dyn. press. prop.            | -    | Pa
# 97  | qbarind    | Dyn. press. ind.             | -    | Pa
# 98  | stall      | Stall                        | -    | -
# 99  | rho        | Air density                  | -    | kg/m^3
# 100 | J          | Advance ratio                | -    | -
# 101 | revprop    | Prop. rev.                   | -    | rev/s
# 102 | Ixx        | Mom. inert. XX               | -    | kg/m^2
# 103 | Ixy        | Mom. inert. XY               | -    | kg/m^2
# 104 | Ixz        | Mom. inert. XZ               | -    | kg/m^2
# 105 | Iyy        | Mom. inert. YY               | -    | kg/m^2
# 106 | Iyz        | Mom. inert. YZ               | -    | kg/m^2
# 107 | Izz        | Mom. inert. ZZ               | -    | kg/m^2
# 108 | mass       | Mass                         | -    | kg
# 109 | grav       | Grav. accel.                 | -    | m/s^2
# 110 | up_down    | Upside-down                  | -    | -
# 111 | wow1       | WOW1                         | -    | -
# 112 | wow2       | WOW2                         | -    | -
# 113 | wow3       | WOW3                         | -    | -
# 114 | wc1        | WC1                          | -    | -
# 115 | wc2        | WC2                          | -    | -
# 116 | wc3        | WC3                          | -    | -
# 117 | D1         | Aero. drag contrib. 1        | UVW  | N
# 118 | D2         | Aero. drag contrib. 2        | UVW  | N
# 119 | D3         | Aero. drag contrib. 3        | UVW  | N
# 120 | D4         | Aero. drag contrib. 4        | UVW  | N
# 121 | C1         | Aero. crossw. contrib. 1     | UVW  | N
# 122 | C2         | Aero. crossw. contrib. 2     | UVW  | N
# 123 | L1         | Aero. lift contrib. 1        | UVW  | N
# 124 | L2         | Aero. lift contrib. 2        | UVW  | N
# 125 | L3         | Aero. lift contrib. 3        | UVW  | N
# 126 | L4         | Aero. lift contrib. 4        | UVW  | N
# 127 | L5         | Aero. lift contrib. 5        | UVW  | N
# 128 | l1         | Aero. roll contrib. 1        | XYZ  | N
# 129 | l2         | Aero. roll contrib. 2        | XYZ  | N
# 130 | l3         | Aero. roll contrib. 3        | XYZ  | N
# 131 | l4         | Aero. roll contrib. 4        | XYZ  | N
# 132 | l5         | Aero. roll contrib. 5        | XYZ  | N
# 133 | m1         | Aero. pitch contrib. 1       | XYZ  | N
# 134 | m2         | Aero. pitch contrib. 2       | XYZ  | N
# 135 | m3         | Aero. pitch contrib. 3       | XYZ  | N
# 136 | m4         | Aero. pitch contrib. 4       | XYZ  | N
# 137 | m5         | Aero. pitch contrib. 5       | XYZ  | N
# 138 | m6         | Aero. pitch contrib. 6       | XYZ  | N
# 139 | n1         | Aero. yaw contrib. 1         | XYZ  | N
# 140 | n2         | Aero. yaw contrib. 2         | XYZ  | N
# 141 | n3         | Aero. yaw contrib. 3         | XYZ  | N
# 142 | n4         | Aero. yaw contrib. 4         | XYZ  | N
# 143 | n5         | Aero. yaw contrib. 5         | XYZ  | N
# 144 | n6         | Aero. yaw contrib. 6         | XYZ  | N
# 145 | Vprop      | Prop. induc. vel.            | XYZ  | m/s
# 146 | Vind       | Vel. incl. prop. induc. vel. | XYZ  | m/s
TELEM_RX_PLOT = {
                 'Sim. time' :                    ['s', r'$t_{sim}$'],
                 'Sim. timestep' :                ['s', r'$dt_{sim}$'],
                 'Delta long. from start' :       ['m', r'$\Delta_{\Lambda}$'],
                 'Delta lat. from start' :        ['m', r'$\Delta_{\Phi}$'],
                 'Distance from start' :          ['m', r'$\Delta_{dist}$'],
                 'North position' :               ['m', r'$p_n$'],
                 'East position' :                ['m', r'$p_e$'],
                 'Down position' :                ['m', r'$p_d$'],
                 'Longitude (GNSS)' :             ['rad', r'$\Lambda_{GNSS}$'],
                 'Latitude (GNSS)' :              ['rad', r'$\Phi_{GNSS}$'],
                 'Altitude (GNSS)' :              ['m', r'$h_{GNSS}$'],
                 'Baro. altitude (QFE)' :         ['m', r'$h_{QFE}$'],
                 'Baro. altitude (QNH)' :         ['m', r'$h_{QNH}$'],
                 'Terrain elevation ASL' :        ['m', r'$h_{terr}$'],
                 'Roll ang.' :                    ['rad', r'$\phi$'],
                 'Pitch ang.' :                   ['rad', r'$\theta$'],
                 'Yaw ang.' :                     ['rad', r'$\psi$'],
                 'Attack ang.' :                  ['rad/s', r'$\alpha$'],
                 'Side-slip ang.' :               ['rad/s', r'$\beta$'],
                 'Path ang.' :                    ['rad', r'$\gamma$'],
                 'North vel.' :                   ['m/s', r'$v_n$'],
                 'East vel.' :                    ['m/s', r'$v_e$'],
                 'Down vel.' :                    ['m/s', r'$v_d$'],
                 'Long. vel.' :                   ['m/s', r'$u$'],
                 'Transv. vel.' :                 ['m/s', r'$v$'],
                 'Vert. vel.' :                   ['m/s', r'$w$'],
                 'Long. aero. vel.' :             ['m/s', r'$u_{aero}$'],
                 'Transv. aero. vel.' :           ['m/s', r'$v_{aero}$'],
                 'Vert. aero. vel.' :             ['m/s', r'$w_{aero}$'],
                 'North wind vel.' :              ['m/s', r'$w_n$'],
                 'East wind vel.' :               ['m/s', r'$w_e$'],
                 'Down wind vel.' :               ['m/s', r'$w_d$'],
                 'Roll ang. change' :             ['rad/s', r'$\dot \phi$'],
                 'Pitch ang. change' :            ['rad/s', r'$\dot \theta$'],
                 'Yaw ang. change' :              ['rad/s', r'$\dot \psi$'],
                 'Roll vel.' :                    ['rad/s', r'$p$'],
                 'Pitch vel.' :                   ['rad/s', r'$q$'],
                 'Yaw vel.' :                     ['rad/s', r'$r$'],
                 'Aero. roll vel.' :              ['rad/s', r'$p_{aero}$'],
                 'Aero. pitch vel.' :             ['rad/s', r'$q_{aero}$'],
                 'Aero. yaw vel.' :               ['rad/s', r'$r_{aero}$'],
                 'Attack ang. change' :           ['rad/s', r'$\dot \alpha$'],
                 'Side-slip ang. change' :        ['rad/s', r'$\dot \beta$'],
                 'Long. accel.' :                 ['m/s^2', r'$\dot u$'],
                 'Transv. accel.' :               ['m/s^2', r'$\dot v$'],
                 'Vert. accel.' :                 ['m/s^2', r'$\dot w$'],
                 'Roll accel.' :                  ['rad/s^2', r'$\dot p$'],
                 'Pitch accel.' :                 ['rad/s^2', r'$\dot q$'],
                 'Yaw accel.' :                   ['rad/s^2', r'$\dot r$'],
                 'Long. aero. force' :            ['N', r'$fx_{aero}$'],
                 'Long. ext. force' :             ['N', r'$fx_{ext}$'],
                 'Long. gear force' :             ['N', r'$fx_{gear}$'],
                 'Long. prop. force' :            ['N', r'$fx_{prop}$'],
                 'Long. total force' :            ['N', r'$fx$'],
                 'Transv. aero. force' :          ['N', r'$fy_{aero}$'],
                 'Transv. ext. force' :           ['N', r'$fy_{ext}$'],
                 'Transv. gear force' :           ['N', r'$fy_{gear}$'],
                 'Transv. prop. force' :          ['N', r'$fy_{prop}$'],
                 'Transv. total force' :          ['N', r'$fy$'],
                 'Vert. aero. force' :            ['N', r'$fz_{aero}$'],
                 'Vert. ext. force' :             ['N', r'$fz_{ext}$'],
                 'Vert. gear force' :             ['N', r'$fz_{gear}$'],
                 'Vert. prop. force' :            ['N', r'$fz_{prop}$'],
                 'Vert. total force' :            ['N', r'$fz$'],
                 'Aero. roll mom.' :              ['N·m', r'$l_{aero}$'],
                 'Ext. roll mom.' :               ['N·m', r'$l_{ext}$'],
                 'Gear roll mom.' :               ['N·m', r'$l_{gear}$'],
                 'Prop. roll mom.' :              ['N·m', r'$l_{prop}$'],
                 'Total roll mom.' :              ['N·m', r'$l$'],
                 'Aero. pitch mom.' :             ['N·m', r'$m_{aero}$'],
                 'Ext. pitch mom.' :              ['N·m', r'$m_{ext}$'],
                 'Gear pitch mom.' :              ['N·m', r'$m_{gear}$'],
                 'Prop. pitch mom.' :             ['N·m', r'$m_{prop}$'],
                 'Total pitch mom.' :             ['N·m', r'$m$'],
                 'Aero. yaw mom.' :               ['N·m', r'$n_{aero}$'],
                 'Ext. yaw mom.' :                ['N·m', r'$n_{ext}$'],
                 'Gear yaw mom.' :                ['N·m', r'$n_{gear}$'],
                 'Prop. yaw mom.' :               ['N·m', r'$n_{prop}$'],
                 'Total yaw mom.' :               ['N·m', r'$n$'],
                 'R ail. pos.' :                  ['rad', r'$\sigma_{ra}$'],
                 'N. R ail. pos.' :               ['-', r'$\delta_{ra}$'],
                 'L ail. pos.' :                  ['rad', r'$\sigma_{la}$'],
                 'N. L ail. pos.' :               ['-', r'$\delta_{la}$'],
                 'Elevs. pos.' :                  ['rad', r'$\sigma_e$'],
                 'N. elevs. pos.' :               ['-', r'$\delta_e$'],
                 'Flaps pos.' :                   ['rad', r'$\sigma_f$'],
                 'N. flaps pos.' :                ['-', r'$\delta_f$'],
                 'Rudder pos.' :                  ['rad', r'$\sigma_r$'],
                 'N. rudder pos.' :               ['-', r'$\delta_r$'],
                 'N. eng. thrott. pos.' :         ['-', r'$\delta_t$'],
                 'N. eng. mix. pos.' :            ['-', r'$\delta_m$'],
                 'Span-vel. ratio' :              ['s', ''],
                 'Chord-vel. ratio' :             ['s', ''],
                 'MAC-span ratio' :               ['-', ''],
                 'Dyn. press.' :                  ['Pa', r'$\bar{q}$'],
                 'Dyn. press. UW' :               ['Pa', r'$\bar{q}_{UW}$'],
                 'Dyn. press. prop.' :            ['Pa', r'$\bar{q}_{prop}$'],
                 'Dyn. press. ind.' :             ['Pa', r'$\bar{q}_{ind}$'],
                 'Stall' :                        ['-', ''],
                 'Air density' :                  ['-', r'$\rho$'],
                 'Advance ratio' :                ['-', r'$J$'],
                 'Prop. rev.' :                   ['rev/s', r'n'],
                 'Mom. inert. XX' :               ['kg/m²', r'$I_{XX}$'],
                 'Mom. inert. XY' :               ['kg/m²', r'$I_{XY}$'],
                 'Mom. inert. XZ' :               ['kg/m²', r'$I_{XZ}$'],
                 'Mom. inert. YY' :               ['kg/m²', r'$I_{YY}$'],
                 'Mom. inert. YZ' :               ['kg/m²', r'$I_{YZ}$'],
                 'Mom. inert. ZZ' :               ['kg/m²', r'$I_{ZZ}$'],
                 'Mass' :                         ['kg', r'm'],
                 'Grav. accel.' :                 ['m/s²', r'$g$'],
                 'Upside-down' :                  ['-', ''],
                 'WOW1' :                         ['-', ''],
                 'WOW2' :                         ['-', ''],
                 'WOW3' :                         ['-', ''],
                 'WC1' :                          ['-', ''],
                 'WC2' :                          ['-', ''],
                 'WC3' :                          ['-', ''],
                 'Aero. drag contrib. 1' :        ['N', r'$D_1$'],
                 'Aero. drag contrib. 2' :        ['N', r'$D_2$'],
                 'Aero. drag contrib. 3' :        ['N', r'$D_3$'],
                 'Aero. drag contrib. 4' :        ['N', r'$D_4$'],
                 'Aero. crossw. contrib. 1' :     ['N', r'$C_1$'],
                 'Aero. crossw. contrib. 2' :     ['N', r'$C_2$'],
                 'Aero. lift contrib. 1' :        ['N', r'$L_1$'],
                 'Aero. lift contrib. 2' :        ['N', r'$L_2$'],
                 'Aero. lift contrib. 3' :        ['N', r'$L_3$'],
                 'Aero. lift contrib. 4' :        ['N', r'$L_4$'],
                 'Aero. lift contrib. 5' :        ['N', r'$L_5$'],
                 'Aero. roll contrib. 1' :        ['N', r'$l_1$'],
                 'Aero. roll contrib. 2' :        ['N', r'$l_2$'],
                 'Aero. roll contrib. 3' :        ['N', r'$l_3$'],
                 'Aero. roll contrib. 4' :        ['N', r'$l_4$'],
                 'Aero. roll contrib. 5' :        ['N', r'$l_5$'],
                 'Aero. pitch contrib. 1' :       ['N', r'$m_1$'],
                 'Aero. pitch contrib. 2' :       ['N', r'$m_2$'],
                 'Aero. pitch contrib. 3' :       ['N', r'$m_3$'],
                 'Aero. pitch contrib. 4' :       ['N', r'$m_4$'],
                 'Aero. pitch contrib. 5' :       ['N', r'$m_5$'],
                 'Aero. pitch contrib. 6' :       ['N', r'$m_6$'],
                 'Aero. yaw contrib. 1' :         ['N', r'$n_1$'],
                 'Aero. yaw contrib. 2' :         ['N', r'$n_2$'],
                 'Aero. yaw contrib. 3' :         ['N', r'$n_3$'],
                 'Aero. yaw contrib. 4' :         ['N', r'$n_4$'],
                 'Aero. yaw contrib. 5' :         ['N', r'$n_5$'],
                 'Aero. yaw contrib. 6' :         ['N', r'$n_6$'],
                 'Prop. induc. vel.' :            ['m/s²', r'$V_{prop}$'],
                 'Vel. incl. prop. induc. vel.' : ['m/s²', r'$V_{ind}$']
                }
TELEM_TX_PLOT = {
                 'N. ails. cmd' :        ['-', r'$\delta_a$'],
                 'N. elevs. cmd' :       ['-', r'$\delta_e$'],
                 'N. flaps cmd' :        ['-', r'$\delta_f$'],
                 'N. rudder cmd' :       ['-', r'$\delta_r$'],
                 'N. eng. thrott. cmd' : ['-', r'$\delta_t$'],
                 'N. eng. mix. cmd' :    ['-', r'$\delta_m$'],
                 'N. ails. trim cmd' :   ['-', r'$\delta_{a_{trim}}$'],
                 'N. elevs. trim cmd' :  ['-', r'$\delta_{e_{trim}}$'],
                 'N. rudder trim cmd' :  ['-', r'$\delta_{r_{trim}}$']
                }
