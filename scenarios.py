from settings import *

class Scenario():
    def __init__(self):
        self.model_hz              = MODEL_HZ
        self.cfg_ip_address        = CFG_IP_ADDRESS
        self.cfg_port              = CFG_PORT
        self.telem_rx_ip_address   = TELEM_RX_IP_ADDRESS
        self.telem_rx_port         = TELEM_RX_PORT
        self.telem_rx_hz           = MODEL_HZ
        self.telem_tx_ip_address   = TELEM_TX_IP_ADDRESS
        self.telem_tx_port         = TELEM_TX_PORT
        self.telem_tx_hz           = ACT_HZ
        self.cfg_protocol_filename = CFG_PROTOCOL_FILENAME
        self.rx_protocol_filename  = RX_PROTOCOL_FILENAME
        self.tx_protocol_filename  = TX_PROTOCOL_FILENAME
        self.time_of_day           = TIME_OF_DAY
        self.season                = SEASON
        self.visibility            = VISIBILITY
        self.wind                  = WIND
        self.turbulence            = TURBULENCE
        self.longitude_start       = LONGITUDE_START
        self.latitude_start        = LATITUDE_START
        self.altitude_start        = ALTITUDE_START
        self.phi_start             = PHI_START
        self.theta_start           = THETA_START
        self.psi_start             = PSI_START
        self.u_start               = U_START
        self.v_start               = V_START
        self.w_start               = W_START
        self._fixed_options        = ["fgfs", "--aircraft=c172p", "--fdm=jsb"]
    def cruise(self):
        model_hz_str              = "--model-hz=" + str(self.model_hz)
        cfg_str                   = "--generic=socket,in,1," + self.cfg_ip_address + "," + str(self.cfg_port) + ",tcp," + self.cfg_protocol_filename
        telem_rx_str              = "--generic=socket,out," + str(self.telem_rx_hz) + "," + self.telem_rx_ip_address + "," + str(self.telem_rx_port) + ",tcp," + self.rx_protocol_filename
        telem_tx_str              = "--generic=socket,in," + str(self.telem_tx_hz) + "," + self.telem_tx_ip_address + "," + str(self.telem_tx_port) + ",tcp," + self.tx_protocol_filename
        timeofday_str             = "--timeofday=" + self.time_of_day
        season_str                = "--season=" + self.season
        visibility_str            = "--visibility=" + str(self.visibility)
        wind_str                  = "--wind=" + self.wind
        turbulence_str            = "--turbulence=" + str(self.turbulence)
        longitude_str             = "--lon=" + str(self.longitude_start)
        latitude_str              = "--lat=" + str(self.latitude_start)
        altitude_str              = "--altitude=" + str(self.altitude_start)
        phi_str                   = "--roll=" + str(self.phi_start)
        theta_str                 = "--pitch=" + str(self.theta_start)
        psi_str                   = "--heading=" + str(self.psi_start)
        u_str                     = "--uBody=" + str(self.u_start)
        v_str                     = "--vBody=" + str(self.v_start)
        w_str                     = "--wBody=" + str(self.w_start)
        self._var_options         = ["--in-air", model_hz_str, timeofday_str, season_str, visibility_str, wind_str, turbulence_str, longitude_str, latitude_str, altitude_str, phi_str, theta_str, psi_str, u_str, v_str, w_str, cfg_str, telem_rx_str, telem_tx_str]
        command_args_str          = self._fixed_options + self._var_options + FG_AIRCRAFT_OPTIONS + FG_ENVIRONMENT_OPTIONS
        return command_args_str
