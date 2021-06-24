import multiprocessing as mp

from actuation import *
from csv_logging import *
from config import *
from constants import *
from dynamics import *
from scenarios import *
from setpoint import *
from settings import *
from supervisor import *
from telemetry import *

if __name__ == "__main__":

    #Modules
    ActuationModule    = Actuation() #actuation calculation based on control model and setpoint
    ConfigModule       = Config() #FlightGear configuration
    ControlModelModule = ControlModel() #control model calculation
    CSVLoggingModule   = CSVLogging() #CSV logging (control model, dynamics, setpoint, RX telemetry, TX telemetry)
    DynamicsModule     = Dynamics() #dynamics calculation
    ScenarioModule     = Scenario() #FlightGear scenario
    SetpointModule     = Setpoint() #setpoint calculation
    TelemetryModule    = Telemetry() #RX and TX telemetry links between FlightGear and this project

    mods_dict = {'act_mod':ActuationModule, 'cfg_mod':ConfigModule, 'cm_mod':ControlModelModule, 'csvlog_mod':CSVLoggingModule, 'dyn_mod':DynamicsModule, 'scen_mod':ScenarioModule, 'sp_mod':SetpointModule, 'telem_mod':TelemetryModule}

    #Supervisor
    SupervisorModule = Supervisor(**mods_dict) #simulations management 
    
    while True:
        pass
