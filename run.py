import multiprocessing as mp

from actuation import *
from csv_logging import *
from config import *
from constants import *
from dynamics import *
from equilibrium import *
from scenarios import *
from settings import *
from supervisor import *
from telemetry import *

if __name__ == "__main__":

    #Events
    event_rxtcp = mp.Event() #RX TCP connection event
    event_txtcp = mp.Event() #TX TCP connection event
    event_start = mp.Event() #simulation start event

    events_dict = {'event_rxtcp':event_rxtcp, 'event_txtcp':event_txtcp, 'event_start':event_start}

    #Pipes
    act2csv_out, act2csv_in = mp.Pipe() #actuation data pipe to CSV
    act2tx_out, act2tx_in = mp.Pipe() #actuation data pipe to TX telemetry
    dyn2csv_out, dyn2csv_in = mp.Pipe() #dynamics data pipe to CSV
    eq2act_out, eq2act_in = mp.Pipe() #equilibrium point data pipe to actuation
    eq2cm_out, eq2cm_in = mp.Pipe() #equilibrium point data pipe to control model
    eq2csv_out, eq2csv_in = mp.Pipe() #equilibrium point data pipe to CSV
    #kin2csv_out, kin2csv_in = mp.Pipe() #kinematics data pipe to CSV
    cm2act_out, cm2act_in = mp.Pipe() #control model data pipe to actuation
    cm2csv_out, cm2csv_in = mp.Pipe() #control model data pipe to CSV
    rx2act_out, rx2act_in = mp.Pipe() #RX telemetry data pipe to actuation
    rx2csv_out, rx2csv_in = mp.Pipe() #RX telemetry data pipe to CSV
    rx2dyn_out, rx2dyn_in = mp.Pipe() #RX telemetry data pipe to dynamics 
    rx2eq_out, rx2eq_in = mp.Pipe() #RX telemetry data pipe to equilibrium point
    #rx2kin_out, rx2kin_in = mp.Pipe() #RX telemetry data pipe to kinematics 
    rx2cm_out, rx2cm_in = mp.Pipe() #RX telemetry data pipe to control model
    rx2sup_out, rx2sup_in = mp.Pipe() #RX telemetry data pipe to supervisor

    pipes_dict = {'act2csv_out':act2csv_out, 'act2csv_in':act2csv_in, 'act2tx_out':act2tx_out, 'act2tx_in':act2tx_in, 'dyn2csv_out':dyn2csv_out, 'dyn2csv_in':dyn2csv_in, 'eq2act_out':eq2act_out, 'eq2act_in':eq2act_in, 'eq2cm_out':eq2cm_out, 'eq2cm_in':eq2cm_in, 'eq2csv_out':eq2csv_out, 'eq2csv_in':eq2csv_in, 'cm2act_out':cm2act_out, 'cm2act_in':cm2act_in, 'cm2csv_out':cm2csv_out, 'cm2csv_in':cm2csv_in, 'rx2act_out':rx2act_out, 'rx2act_in':rx2act_in, 'rx2csv_out':rx2csv_out, 'rx2csv_in':rx2csv_in, 'rx2dyn_out':rx2dyn_out, 'rx2dyn_in':rx2dyn_in, 'rx2eq_out':rx2eq_out, 'rx2eq_in':rx2eq_in, 'rx2cm_out':rx2cm_out, 'rx2cm_in':rx2cm_in, 'rx2sup_out':rx2sup_out, 'rx2sup_in':rx2sup_in}

    #Config
    ConfigurationModule = Config() #simulator configuration

    #Telemetry
    TelemetryModule = Telemetry() #telemetry links

    #CSV logs
    csvtelemargs = {'act2csv_out':act2csv_out, 'rx2csv_out':rx2csv_out, 'event_start':event_start}
    CSVTelemetryModule = CSVTelemetryLog(TELEM_LOG_FILENAME, **csvtelemargs) #RX telemetry log
    csvdynargs = {'dyn2csv_out':dyn2csv_out, 'event_start':event_start}
    CSVDynamicsModule = CSVDynamicsLog(DYN_LOG_FILENAME, **csvdynargs) #calculated dynamics log
    #csvkinargs = {'kin2csv_out':kin2csv_out, 'event_start':event_start}
    #CSVKinematicsLog(KIN_LOG_FILENAME, **csvkinargs) #calculated kinematics log
    csvcmargs = {'cm2csv_out':cm2csv_out, 'event_start':event_start}
    CSVControlModelModule = CSVControlModelLog(CM_LOG_FILENAME, **csvcmargs) #calculated control model log
    csveqargs = {'eq2csv_out':eq2csv_out, 'event_start':event_start}
    CSVEquilibriumModule = CSVEquilibriumLog(EQ_LOG_FILENAME, **csveqargs) #calculated equilibrium point log
    #Dynamics
    DynamicsModule = Dynamics() #calculate dynamics 

    #Kinematics
    #Kinematics(kin2csv_in, rx2dyn_out, event_start) #calculate kinematics 

    #Actuation
    ActuationModule = Actuation()

    #Control model
    ControlModelModule = ControlModel(eq2cm_out, cm2act_in, cm2csv_in, rx2cm_out, event_start) #calculate control model

    #Equilibrium point
    EquilibriumModule = Equilibrium() #calculate equilibrium point

    #Scenarios
    ScenarioModule = Scenario() #initialize flightgear scenario

    mods_dict = {'act_mod':ActuationModule, 'cfg_mod':ConfigurationModule, 'cm_mod':ControlModelModule, 'dyn_mod':DynamicsModule, 'eq_mod':EquilibriumModule, 'scen_mod':ScenarioModule, 'telem_mod':TelemetryModule, 'csv_cm_mod':CSVControlModelModule , 'csv_dyn_mod':CSVDynamicsModule, 'csv_eq_mod':CSVEquilibriumModule, 'csv_telem_mod':CSVTelemetryModule}

    global_dict = {**events_dict, **pipes_dict, **mods_dict}

    #Supervisor
    SupervisorModule = Supervisor(**global_dict) 
    
    while True:
        pass
