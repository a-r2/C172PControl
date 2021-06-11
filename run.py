import multiprocessing as mp

from actuation import *
from csv_logging import *
from config import *
from dynamics import *
from equilibrium import *
from settings import *
from telemetry import *

def initialize():
    #Events
    event_rxtcp = mp.Event() #RX TCP connection event
    event_txtcp = mp.Event() #TX TCP connection event
    event_start = mp.Event() #simulation start event

    #Pipes
    act2csv_out, act2csv_in = mp.Pipe() #actuation data pipe to CSV
    act2tx_out, act2tx_in = mp.Pipe() #actuation data pipe to TX telemetry
    dyn2csv_out, dyn2csv_in = mp.Pipe() #dynamics data pipe to CSV
    eq2act_out, eq2act_in = mp.Pipe() #equilibrium point data pipe to actuation
    eq2mod_out, eq2mod_in = mp.Pipe() #equilibrium point data pipe to control model
    eq2csv_out, eq2csv_in = mp.Pipe() #equilibrium point data pipe to CSV
    #kin2csv_out, kin2csv_in = mp.Pipe() #kinematics data pipe to CSV
    mod2act_out, mod2act_in = mp.Pipe() #control model data pipe to actuation
    mod2csv_out, mod2csv_in = mp.Pipe() #control model data pipe to CSV
    rx2act_out, rx2act_in = mp.Pipe() #RX telemetry data pipe to actuation
    rx2csv_out, rx2csv_in = mp.Pipe() #RX telemetry data pipe to CSV
    rx2dyn_out, rx2dyn_in = mp.Pipe() #RX telemetry data pipe to dynamics 
    rx2eq_out, rx2eq_in = mp.Pipe() #RX telemetry data pipe to equilibrium point
    #rx2kin_out, rx2kin_in = mp.Pipe() #RX telemetry data pipe to kinematics 
    rx2mod_out, rx2mod_in = mp.Pipe() #RX telemetry data pipe to control model

    #Config
    Config(CFG_IP_ADDRESS, CFG_PORT, event_rxtcp) #simulator configuration

    #Telemetry
    Telemetry(TELEM_IP_ADDRESS, TELEM_RX_PORT, TELEM_IP_ADDRESS, TELEM_TX_PORT, act2tx_out, rx2act_in, rx2csv_in, rx2dyn_in, rx2eq_in, rx2mod_in, event_rxtcp, event_txtcp, event_start) #telemetry links

    #CSV logs
    csvtelemargs = {'act2csv_out':act2csv_out, 'rx2csv_out':rx2csv_out, 'event_start':event_start}
    CSVTelemetryLog(TELEM_LOG_FILENAME, **csvtelemargs) #RX telemetry log
    csvdynargs = {'dyn2csv_out':dyn2csv_out, 'event_start':event_start}
    CSVDynamicsLog(DYN_LOG_FILENAME, **csvdynargs) #calculated dynamics log
    #csvkinargs = {'kin2csv_out':kin2csv_out, 'event_start':event_start}
    #CSVKinematicsLog(KIN_LOG_FILENAME, **csvkinargs) #calculated kinematics log
    csvmodargs = {'mod2csv_out':mod2csv_out, 'event_start':event_start}
    CSVControlModelLog(CM_LOG_FILENAME, **csvmodargs) #calculated control model log
    csveqargs = {'eq2csv_out':eq2csv_out, 'event_start':event_start}
    CSVEquilibriumLog(EQ_LOG_FILENAME, **csveqargs) #calculated equilibrium point log

    #Dynamics
    Dynamics(dyn2csv_in, rx2dyn_out, event_start) #calculate dynamics 

    #Kinematics
    #Kinematics(kin2csv_in, rx2dyn_out, event_start) #calculate kinematics 

    #Actuation
    Actuation(ACT_TYPE, act2csv_in, act2tx_in, eq2act_out, mod2act_out, rx2act_out, event_start) #calculate actuation

    #Control model
    ControlModel(CM_TYPE, eq2mod_out, mod2act_in, mod2csv_in, rx2mod_out, event_start) #calculate control model

    #Equilibrium point
    Equilibrium(EQ_TYPE, EQ_POINT_INIT, eq2csv_in, eq2mod_in, rx2eq_out, event_start) #calculate equilibrium point

if __name__ == "__main__":
    initialize()
    while True:
        pass
