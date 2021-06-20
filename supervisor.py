import multiprocessing as mp
import socket
import subprocess
import time

from settings import *
from utils import *

class Supervisor():

    def __init__(self, **global_dict):
        #Select supervisor based on type of simulation
        if (SIM_TYPE == 1) or (SIM_TYPE == 'recursive'):
            self.recursive_sim(**global_dict)

    def start_processes(self, **global_dict):
        #Create processes
        act_proc                     = self.actuation_process(**global_dict)
        cfg_proc                     = self.configuration_process(**global_dict)
        dyn_proc                     = self.dynamics_process(**global_dict)
        eq_proc                      = self.equilibrium_process(**global_dict)
        telem_rx_proc, telem_tx_proc = self.telemetry_processes(**global_dict)
        '''
        cm_proc           = control_model_process(**global_dict)
        csv_cm_proc       = csv_control_model_process(**global_dict)
        csv_dyn_proc      = csv_dynamics_process(**global_dict)
        csv_eq_proc       = csv_equilibrium_process(**global_dict)
        csv_telem_rx_proc = csv_telem_rx_process(**global_dict)
        csv_telem_tx_proc = csv_telem_tx_process(**global_dict)
        '''
        #Start processes
        act_proc.start()
        cfg_proc.start()
        dyn_proc.start()
        eq_proc.start()
        telem_rx_proc.start()
        telem_tx_proc.start()
        time.sleep(1)
        scen_proc = self.scenario_process(**global_dict) #scenario subprocess automatically starts when called
        print('act_proc: ' + str(act_proc.is_alive()))
        print('cfg_mod: ' + str(cfg_proc.is_alive()))
        print('eq_mod: ' + str(eq_proc.is_alive()))
        print('dyn_mod: ' + str(dyn_proc.is_alive()))
        print('RX telem_mod: ' + str(telem_rx_proc.is_alive()))
        print('TX telem_mod: ' + str(telem_tx_proc.is_alive()))
        '''
        print('cm_mod: ' + str(cm_proc.is_alive()))
        print('csv_cm_mod: ' + str(csv_cm_proc.is_alive()))
        print('csv_eq_mod: ' + str(csv_eq_proc.is_alive()))
        print('CSVRXTelemetryModule: ' + str(csv_telem_rx_proc.is_alive()))
        print('CSVTXTelemetryModule: ' + str(csv_telem_tx_proc.is_alive()))
        '''
        #Create processes dictionary
        procs_dict = {'act_proc':act_proc, 'cfg_proc':cfg_proc, 'dyn_proc':dyn_proc, 'eq_proc':eq_proc, 'telem_rx_proc':telem_rx_proc, 'telem_tx_proc':telem_tx_proc, 'scen_proc':scen_proc}
        #procs_dict = {'act_proc':act_proc, 'cfg_proc':cfg_proc, 'cm_proc':cm_proc, 'dyn_proc':dyn_proc, 'eq_proc':eq_proc, 'scen_proc':scen_proc, 'telem_rx_proc':telem_rx_proc,, 'telem_tx_proc':telem_tx_proc, 'csv_cm_proc':csv_cm_proc , 'csv_dyn_proc':csv_dyn_proc, 'csv_eq_proc':csv_eq_proc, 'csv_telem_rx_proc':csv_telem_rx_proc, 'csv_telem_tx_proc':csv_telem_tx_proc}
        return procs_dict

    def terminate_processes(self, **procs_dict):
        #Get processes
        act_proc      = procs_dict['act_proc']
        cfg_proc      = procs_dict['cfg_proc']
        dyn_proc      = procs_dict['dyn_proc']
        eq_proc       = procs_dict['eq_proc']
        telem_rx_proc = procs_dict['telem_rx_proc']
        telem_tx_proc = procs_dict['telem_tx_proc']
        scen_proc     = procs_dict['scen_proc']
        #Terminate processes
        scen_proc.terminate()
        time.sleep(1)
        act_proc.terminate()
        cfg_proc.terminate()
        dyn_proc.terminate()
        eq_proc.terminate()
        telem_rx_proc.terminate()
        telem_tx_proc.terminate()
        '''
        cm_proc.terminate()
        csv_cm_proc.terminate()
        csv_dyn_proc.terminate()
        csv_eq_proc.terminate()
        csv_telem_rx_proc.terminate()
        csv_telem_tx_proc.terminate()
        '''
        time.sleep(1)
        print('act_proc: ' + str(act_proc.is_alive()))
        print('cfg_mod: ' + str(cfg_proc.is_alive()))
        print('dyn_mod: ' + str(dyn_proc.is_alive()))
        print('eq_mod: ' + str(eq_proc.is_alive()))
        print('RX telem_mod: ' + str(telem_rx_proc.is_alive()))
        print('TX telem_mod: ' + str(telem_tx_proc.is_alive()))
        '''
        print('cm_mod: ' + str(cm_proc.is_alive()))
        print('csv_cm_mod: ' + str(csv_cm_proc.is_alive()))
        print('csv_eq_mod: ' + str(csv_eq_proc.is_alive()))
        print('CSVRXTelemetryModule: ' + str(csv_telem_rx_proc.is_alive()))
        print('CSVTXTelemetryModule: ' + str(csv_telem_tx_proc.is_alive()))
        '''

    def terminate_sockets(self, **global_dict):
        #Get modules
        cfg_mod   = global_dict['cfg_mod']
        telem_mod = global_dict['telem_mod']
        #Shutdown sockets
        cfg_mod.sock.shutdown(socket.SHUT_RDWR)
        telem_mod.rx_sock.shutdown(socket.SHUT_RDWR)
        #telem_mod.tx_sock.shutdown(socket.SHUT_RDWR)
        #Close sockets
        cfg_mod.sock.close()
        telem_mod.rx_sock.close()
        telem_mod.tx_sock.close()

    def actuation_process(self, **global_dict):
        act_mod     = global_dict['act_mod']
        act2csv_in  = global_dict['act2csv_in']
        act2tx_in   = global_dict['act2tx_in']
        rx2act_out  = global_dict['rx2act_out']
        event_start = global_dict['event_start']
        if (ACT_TYPE == 0) or (ACT_TYPE == 'random'):
            act_proc = mp.Process(target=act_mod.random_control, args=(act2csv_in, act2tx_in, rx2act_out, event_start), daemon=True) #process for random actuation
        return act_proc

    def configuration_process(self, **global_dict):
        cfg_mod     = global_dict['cfg_mod']
        event_rxtcp = global_dict['event_rxtcp']
        cfg_proc    = mp.Process(target=cfg_mod.transmit, args=(event_rxtcp,), daemon=True) #process for configuring FlightGear
        return cfg_proc

    def equilibrium_process(self, **global_dict):
        eq_mod      = global_dict['eq_mod']
        eq2csv_in   = global_dict['eq2csv_in']
        eq2cm_in    = global_dict['eq2cm_in']
        rx2eq_out   = global_dict['rx2eq_out']
        event_start = global_dict['event_start']
        if (EQ_TYPE == 0) or (EQ_TYPE == 'constant'):
            eq_proc = mp.Process(target=eq_mod.constant, args=(eq2csv_in, eq2cm_in, rx2eq_out, event_start), daemon=True) #process for equilibrium point
        return eq_proc

    def dynamics_process(self, **global_dict):
        dyn_mod     = global_dict['dyn_mod']
        dyn2csv_in  = global_dict['dyn2csv_in']
        rx2dyn_out  = global_dict['rx2dyn_out']
        event_start = global_dict['event_start']
        dyn_proc    = mp.Process(target=dyn_mod.forces_moments_body, args=(dyn2csv_in, rx2dyn_out, event_start), daemon=True) #process for dynamics
        return dyn_proc

    def scenario_process(self, **global_dict):
        scen_mod = global_dict['scen_mod']
        if (SCENARIO_TYPE == 0) or (SCENARIO_TYPE == 'cruise'):
            command_args_str = scen_mod.cruise() #scenario command
            scen_proc = subprocess.Popen(args=command_args_str) #process for scenario in FlightGear
        return scen_proc

    def telemetry_processes(self, **global_dict):
        telem_mod   = global_dict['telem_mod']
        act2tx_out  = global_dict['act2tx_out']
        rx2act_in   = global_dict['rx2act_in']
        rx2cm_in    = global_dict['rx2cm_in']
        rx2csv_in   = global_dict['rx2csv_in']
        rx2dyn_in   = global_dict['rx2dyn_in']
        rx2eq_in    = global_dict['rx2eq_in']
        rx2sup_in   = global_dict['rx2sup_in']
        event_rxtcp = global_dict['event_rxtcp']
        event_txtcp = global_dict['event_txtcp']
        event_start = global_dict['event_start']

        telem_rx_proc = mp.Process(target=telem_mod.receive, args=(rx2act_in, rx2csv_in, rx2dyn_in, rx2eq_in, rx2cm_in, rx2sup_in, event_rxtcp, event_start), daemon=True) #process for RX telemetry
        telem_tx_proc = mp.Process(target=telem_mod.transmit, args=(act2tx_out, event_rxtcp, event_txtcp, event_start), daemon=True) #process for TX telemetry
        return telem_rx_proc, telem_tx_proc

    def recursive_sim(self, **global_dict):
        #Get pipes and events
        rx2sup_out  = global_dict['rx2sup_out']
        event_start = global_dict['event_start']
        #Multiple simulations loop
        for _ in range(SIM_ITER_NUM):
            procs_dict  = self.start_processes(**global_dict)
            event_start.wait() #wait for simulation start event
            while True: #single simulation loop
                rxdata    = rx2sup_out.recv() #receive RX telemetry
                lastframe = rxdata[-1]
                terminate_flag = self.simulation_watchdog(lastframe)
                if terminate_flag:
                    self.terminate_processes(**procs_dict)
                    self.terminate_sockets(**global_dict)
                    terminate_flag = False
                    break
        quit()

    def mod_from_global_dict(self, **global_dict):
        #Construct arguments dictionary for Supervisor class
        mod_dict = {}
        for key, value in global_dict.items():
            if key == 'act_mod':
                mod_dict.update(act_mod = value)
            elif key == 'cfg_mod':
                mod_dict.update(cfg_mod = value)
            elif key == 'cm_mod':
                mod_dict.update(cm_mod = value)
            elif key == 'dyn_mod':
                mod_dict.update(dyn_mod = value)
            elif key == 'eq_mod':
                mod_dict.update(eq_mod = value)
            elif key == 'scen_mod':
                mod_dict.update(scen_mod = value)
            elif key == 'telem_mod':
                mod_dict.update(telem_mod = value)
            elif key == 'csv_cm_mod':
                mod_dict.update(csv_cm_mod = value)
            elif key == 'csv_dyn_mod':
                mod_dict.update(csv_dyn_mod = value)
            elif key == 'csv_eq_mod':
                mod_dict.update(csv_eq_mod = value)
            elif key == 'csv_telem_mod':
                mod_dict.update(csv_telem_mod = value)
            else:
                pass
        return mod_dict

    def simulation_watchdog(self, lastframe):
        #Activate watchdog when aircraft is upside down or contacts ground/watr
        up_down  = lastframe[128]
        wow1     = lastframe[129]
        wow2     = lastframe[130]
        wow3     = lastframe[131]
        contact1 = lastframe[132]
        contact2 = lastframe[133]
        contact3 = lastframe[134]
        return True if (up_down == 1) or (wow1 == 1) or (wow2 == 1) or (wow3 == 1) or (contact1 == 1) or (contact2 == 1) or (contact3 == 1) else False
