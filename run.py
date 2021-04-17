import multiprocessing as mp

from actuation import *
from csv_logging import *
from config import *
from dynamics import *
from settings import *
from telemetry import *

def initialize():
    #Events
    event_rxtcp = mp.Event() #RX TCP connection event
    event_txtcp = mp.Event() #TX TCP connection event
    event_start = mp.Event() #simulation start event

    #Pipes
    rx2act_out, rx2act_in = mp.Pipe() #RX telemetry data pipe to calculate actuation
    rx2csv_out, rx2csv_in = mp.Pipe() #RX telemetry data pipe to store in CSV
    rx2dyn_out, rx2dyn_in = mp.Pipe() #RX telemetry data pipe to calculate dynamics 
    act2csv_out, act2csv_in = mp.Pipe() #actuation data pipe to store in CSV
    act2tx_out, act2tx_in = mp.Pipe() #actuation data pipe to transmit TX telemetry
    dyn2csv_out, dyn2csv_in = mp.Pipe() #dynamics data pipe to store in CSV

    #Config
    Config(CFG_IP_ADDRESS, CFG_PORT, event_rxtcp) #simulator configuration

    #Telemetry
    Telemetry(TELEM_IP_ADDRESS, TELEM_RX_PORT, TELEM_IP_ADDRESS, TELEM_TX_PORT, rx2act_in, rx2csv_in, rx2dyn_in, act2tx_out, event_rxtcp, event_txtcp, event_start) #telemetry links

    #CSV logs
    csvtelemargs = {'rx2csv_out':rx2csv_out, 'act2csv_out':act2csv_out, 'event_start':event_start}
    CSVTelemetryLog(TELEM_LOG_FILENAME, **csvtelemargs) #RX telemetry log
    csvdynargs = {'dyn2csv_out':dyn2csv_out, 'event_start':event_start}
    CSVDynamicsLog(DYN_LOG_FILENAME, **csvdynargs) #calculated dynamics log
    #csvkinargs = {'kin2csv_out':kin2csv_out, 'event_start':event_start}
    #CSVKinematicsLog(KIN_LOG_FILENAME, **csvkinargs) #calculated kinematics log

    #Dynamics
    Aerodynamics(rx2dyn_out, dyn2csv_in, event_start) #calculate aerodynamics

    #Actuation
    Actuation(ACT_TYPE, rx2act_out, act2csv_in, act2tx_in, event_start) #calculate actuation

if __name__ == "__main__":
    initialize()
    while True:
        pass
