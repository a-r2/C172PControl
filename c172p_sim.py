from constants import *
from settings import *
from modules.sim.actuation import *
from modules.sim.csv_logging import *
from modules.sim.config import *
from modules.sim.control_models import *
from modules.sim.dynamics import *
from modules.sim.eqpoint import *
from modules.sim.scenarios import *
from modules.sim.setpoint import *
from modules.sim.telemetry import *
from modules.supervisor import *

if __name__ == '__main__':
    #Modules instantiation
    ActuationModule    = Actuation() #actuation module
    ConfigModule       = FGConfig() #FlightGear configuration module
    ControlModelModule = ControlModel() #control model module
    CSVLoggingModule   = CSVLogging() #CSV logging module (control model, dynamics, setpoint, RX telemetry, TX telemetry)
    DynamicsModule     = Dynamics() #dynamics module
    EquilibriumModule  = Equilibrium() #equilibrium point module
    ScenarioModule     = Scenario() #FlightGear scenario module
    SetpointModule     = Setpoint() #setpoint module
    TelemetryModule    = Telemetry() #RX and TX telemetry links from/to FlightGear
    #Modules dictionary definition
    mods_dict = {'act_mod':ActuationModule, 'cfg_mod':ConfigModule, 'cm_mod':ControlModelModule, 'csvlog_mod':CSVLoggingModule, 'dyn_mod':DynamicsModule, 'eq_mod':EquilibriumModule, 'scen_mod':ScenarioModule, 'sp_mod':SetpointModule, 'telem_mod':TelemetryModule}
    #Supervisor instantiation
    Supervisor = Supervisor(**mods_dict) #simulations manager
    #Simulation loop
    while True:
        pass
