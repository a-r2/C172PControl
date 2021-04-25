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
    #rx2dyn_out, rx2dyn_in = mp.Pipe() #RX telemetry data pipe to calculate dynamics 
    #rx2kin_out, rx2kin_in = mp.Pipe() #RX telemetry data pipe to calculate kinematics 
    rx2nlm_out, rx2nlm_in = mp.Pipe() #RX telemetry data pipe to calculate non-linear model
    rx2lm_out, rx2lm_in = mp.Pipe() #RX telemetry data pipe to calculate linear model
    nlm2act_out, nlm2act_in = mp.Pipe() #non-linear model data pipe to calculate actuation
    lm2act_out, lm2act_in = mp.Pipe() #linear model data pipe to calculate actuation
    act2csv_out, act2csv_in = mp.Pipe() #actuation data pipe to store in CSV
    act2tx_out, act2tx_in = mp.Pipe() #actuation data pipe to transmit TX telemetry
    #dyn2csv_out, dyn2csv_in = mp.Pipe() #dynamics data pipe to store in CSV
    #kin2csv_out, kin2csv_in = mp.Pipe() #kinematics data pipe to store in CSV
    nlm2csv_out, nlm2csv_in = mp.Pipe() #non-linear model data pipe to store in CSV
    lm2csv_out, lm2csv_in = mp.Pipe() #linear model data pipe to store in CSV

    #Config
    Config(CFG_IP_ADDRESS, CFG_PORT, event_rxtcp) #simulator configuration

    #Telemetry
    Telemetry(TELEM_IP_ADDRESS, TELEM_RX_PORT, TELEM_IP_ADDRESS, TELEM_TX_PORT, rx2act_in, rx2csv_in, rx2dyn_in, act2tx_out, event_rxtcp, event_txtcp, event_start) #telemetry links

    #CSV logs
    csvtelemargs = {'rx2csv_out':rx2csv_out, 'act2csv_out':act2csv_out, 'event_start':event_start}
    CSVTelemetryLog(TELEM_LOG_FILENAME, **csvtelemargs) #RX telemetry log
    #csvdynargs = {'dyn2csv_out':dyn2csv_out, 'event_start':event_start}
    #CSVDynamicsLog(DYN_LOG_FILENAME, **csvdynargs) #calculated dynamics log
    #csvkinargs = {'kin2csv_out':kin2csv_out, 'event_start':event_start}
    #CSVKinematicsLog(KIN_LOG_FILENAME, **csvkinargs) #calculated kinematics log

    #Dynamics
    #Dynamics(rx2dyn_out, dyn2csv_in, event_start) #calculate dynamics 

    #Kinematics
    #Kinematics(rx2dyn_out, kin2csv_in, event_start) #calculate kinematics 

    #Actuation
    Actuation(ACT_TYPE, rx2act_out, act2csv_in, act2tx_in, event_start) #calculate actuation

if __name__ == "__main__":
    initialize()
    while True:
        pass
