from constants import *
from settings import *
from sim_modules.actuation import *
from sim_modules.csv_logging import *
from sim_modules.config import *
from sim_modules.dynamics import *
from sim_modules.scenarios import *
from sim_modules.setpoint import *
from sim_modules.telemetry import *
from supervisor import *

if __name__ == "__main__":
    #Modules instantiation
    ActuationModule    = Actuation() #actuation calculation based on control model and setpoint
    ConfigModule       = Config() #FlightGear configuration
    ControlModelModule = ControlModel() #control model calculation
    CSVLoggingModule   = CSVLogging() #CSV logging (control model, dynamics, setpoint, RX telemetry, TX telemetry)
    DynamicsModule     = Dynamics() #dynamics calculation
    ScenarioModule     = Scenario() #FlightGear scenario
    SetpointModule     = Setpoint() #setpoint calculation
    TelemetryModule    = Telemetry() #RX and TX telemetry links between FlightGear and this project
    #Modules dictionary definition
    mods_dict = {'act_mod':ActuationModule, 'cfg_mod':ConfigModule, 'cm_mod':ControlModelModule, 'csvlog_mod':CSVLoggingModule, 'dyn_mod':DynamicsModule, 'scen_mod':ScenarioModule, 'sp_mod':SetpointModule, 'telem_mod':TelemetryModule}
    #Supervisor instantiation
    SupervisorModule = Supervisor(**mods_dict) #simulations management 
    while True:
        pass
