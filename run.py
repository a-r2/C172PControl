import multiprocessing as mp

from actuation import *
from csv_logging import *
from config import *
from constants import *
from dynamics import *
from setpoint import *
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
    act2tx_out, act2tx_in   = mp.Pipe() #actuation data pipe to TX telemetry
    dyn2csv_out, dyn2csv_in = mp.Pipe() #dynamics data pipe to CSV
    sp2act_out, sp2act_in   = mp.Pipe() #setpoint data pipe to actuation
    sp2cm_out, sp2cm_in     = mp.Pipe() #setpoint data pipe to control model
    sp2csv_out, sp2csv_in   = mp.Pipe() #setpoint data pipe to CSV
    cm2act_out, cm2act_in   = mp.Pipe() #control model data pipe to actuation
    cm2csv_out, cm2csv_in   = mp.Pipe() #control model data pipe to CSV
    rx2act_out, rx2act_in   = mp.Pipe() #RX telemetry data pipe to actuation
    rx2csv_out, rx2csv_in   = mp.Pipe() #RX telemetry data pipe to CSV
    rx2dyn_out, rx2dyn_in   = mp.Pipe() #RX telemetry data pipe to dynamics 
    rx2sp_out, rx2sp_in     = mp.Pipe() #RX telemetry data pipe to setpoint
    rx2cm_out, rx2cm_in     = mp.Pipe() #RX telemetry data pipe to control model
    rx2sup_out, rx2sup_in   = mp.Pipe() #RX telemetry data pipe to supervisor

    pipes_dict = {'act2csv_out':act2csv_out, 'act2csv_in':act2csv_in, 'act2tx_out':act2tx_out, 'act2tx_in':act2tx_in, 'dyn2csv_out':dyn2csv_out, 'dyn2csv_in':dyn2csv_in, 'sp2act_out':sp2act_out, 'sp2act_in':sp2act_in, 'sp2cm_out':sp2cm_out, 'sp2cm_in':sp2cm_in, 'sp2csv_out':sp2csv_out, 'sp2csv_in':sp2csv_in, 'cm2act_out':cm2act_out, 'cm2act_in':cm2act_in, 'cm2csv_out':cm2csv_out, 'cm2csv_in':cm2csv_in, 'rx2act_out':rx2act_out, 'rx2act_in':rx2act_in, 'rx2csv_out':rx2csv_out, 'rx2csv_in':rx2csv_in, 'rx2dyn_out':rx2dyn_out, 'rx2dyn_in':rx2dyn_in, 'rx2sp_out':rx2sp_out, 'rx2sp_in':rx2sp_in, 'rx2cm_out':rx2cm_out, 'rx2cm_in':rx2cm_in, 'rx2sup_out':rx2sup_out, 'rx2sup_in':rx2sup_in}

    #Modules
    ConfigModule       = Config() #FlightGear configuration
    TelemetryModule    = Telemetry() #RX and TX telemetry links between FlightGear and this project
    CSVLoggingModule   = CSVLogging() #CSV logging (control model, dynamics, setpoint, RX telemetry, TX telemetry)
    DynamicsModule     = Dynamics() #dynamics calculation
    ActuationModule    = Actuation() #actuation calculation based on control model and setpoint
    ControlModelModule = ControlModel() #control model calculation
    SetpointModule     = Setpoint() #setpoint calculation
    ScenarioModule     = Scenario() #FlightGear scenario

    mods_dict = {'act_mod':ActuationModule, 'cfg_mod':ConfigModule, 'cm_mod':ControlModelModule, 'dyn_mod':DynamicsModule, 'sp_mod':SetpointModule, 'scen_mod':ScenarioModule, 'telem_mod':TelemetryModule, 'csvlog_mod':CSVLoggingModule}

    #Construct global dictionary
    global_dict = {**events_dict, **pipes_dict, **mods_dict}

    #Supervisor
    SupervisorModule = Supervisor(**global_dict) #simulations management 
    
    while True:
        pass
