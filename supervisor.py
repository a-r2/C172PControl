import multiprocessing as mp
import time

from settings import *

class Supervisor():
    def __init__(self, ActuationModule, ConfigurationModule, ControlModelModule, DynamicsModule, EquilibriumModule, ScenarioModule, TelemetryModule, CSVControlModelModule , CSVDynamicsModule,  CSVEquilibriumModule, CSVTelemetryModule, rx2sup_out, event_start):
        if (SIM_TYPE == 1) or (SIM_TYPE == 'recursive'):
            self.recursive_sim(ActuationModule, ConfigurationModule, ControlModelModule, DynamicsModule, EquilibriumModule, ScenarioModule, TelemetryModule, CSVControlModelModule , CSVDynamicsModule,  CSVEquilibriumModule, CSVTelemetryModule, rx2sup_out, event_start) #process for receiving RX telemetry
    def recursive_sim(self, ActuationModule, ConfigurationModule, ControlModelModule, DynamicsModule, EquilibriumModule, ScenarioModule, TelemetryModule, CSVControlModelModule , CSVDynamicsModule,  CSVEquilibriumModule, CSVTelemetryModule, rx2sup_out, event_start):
        event_start.wait() #wait for simulation start event
        while True:
            rxdata    = rx2sup_out.recv() #receive RX telemetry
            lastframe = rxdata[-1]
            terminate_flag = sim_watchdog(self, lastframe)
            if terminate_flag:
                ActuationModule.proc.terminate()
                ConfigurationModule.proc.terminate()
                ControlModelModule.proc.terminate()
                DynamicsModule.proc.terminate()
                EquilibriumModule.proc.terminate()
                ScenarioModule.proc.terminate()
                TelemetryModule.rxproc.terminate()
                TelemetryModule.txproc.terminate()
                CSVControlModelModule.proc.terminate()
                CSVDynamicsModule.proc.terminate()
                CSVEquilibriumModule.proc.terminate()
                CSVTelemetryModule.rxproc.terminate()
                CSVTelemetryModule.txproc.terminate()
                time.sleep(1)
                print('ActuationModule: ' + str(ActuationModule.proc.is_alive()))
                print('ConfigurationModule: ' + str(ConfigurationModule.proc.is_alive()))
                print('ControlModelModule: ' + str(ControlModelModule.proc.is_alive()))
                print('DynamicsModule: ' + str(DynamicsModule.proc.is_alive()))
                print('EquilibriumModule: ' + str(EquilibriumModule.proc.is_alive()))
                print('RX TelemetryModule: ' + str(TelemetryModule.rxproc.is_alive()))
                print('TX TelemetryModule: ' + str(TelemetryModule.txproc.is_alive()))
                print('CSVControlModelModule: ' + str(CSVControlModelModule.proc.is_alive()))
                print('CSVEquilibriumModule: ' + str(CSVEquilibriumModule.proc.is_alive()))
                print('CSVRXTelemetryModule: ' + str(CSVTelemetryModule.rxproc.is_alive()))
                print('CSVTXTelemetryModule: ' + str(CSVTelemetryModule.txproc.is_alive()))

def sim_watchdog(self, lastframe):
    #Activate watchdog when aircraft is upside down or contacts ground/water
    up_down  = lastframe[128]
    wow1     = lastframe[129]
    wow2     = lastframe[130]
    wow3     = lastframe[131]
    contact1 = lastframe[132]
    contact2 = lastframe[133]
    contact3 = lastframe[134]
    return True if (up_down == 1) or (wow1 == 1) or (wow2 == 1) or (wow3 == 1) or (contact1 == 1) or (contact2 == 1) or (contact3 == 1) else False
