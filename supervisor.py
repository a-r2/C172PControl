import multiprocessing as mp
import socket
import subprocess
import time

from settings import *
from utils import *

class Supervisor():

    def __init__(self, **mods_dict):
        #Events
        event_rxtcp = mp.Event() #RX TCP connection event
        event_txtcp = mp.Event() #TX TCP connection event
        event_start = mp.Event() #simulation run start event
        event_end   = mp.Event() #simulation run end event

        events_dict = {'event_rxtcp':event_rxtcp, 'event_txtcp':event_txtcp, 'event_start':event_start, 'event_end':event_end}

        #Pipes
        act2csv_out, act2csv_in = mp.Pipe() #actuation data pipe to CSV
        act2tx_out, act2tx_in   = mp.Pipe() #actuation data pipe to TX telemetry
        cm2act_out, cm2act_in   = mp.Pipe() #control model data pipe to actuation
        cm2csv_out, cm2csv_in   = mp.Pipe() #control model data pipe to CSV
        dyn2csv_out, dyn2csv_in = mp.Pipe() #dynamics data pipe to CSV
        rx2act_out, rx2act_in   = mp.Pipe() #RX telemetry data pipe to actuation
        rx2cm_out, rx2cm_in     = mp.Pipe() #RX telemetry data pipe to control model
        rx2csv_out, rx2csv_in   = mp.Pipe() #RX telemetry data pipe to CSV
        rx2dyn_out, rx2dyn_in   = mp.Pipe() #RX telemetry data pipe to dynamics 
        rx2sp_out, rx2sp_in     = mp.Pipe() #RX telemetry data pipe to setpoint
        rx2sup_out, rx2sup_in   = mp.Pipe() #RX telemetry data pipe to supervisor
        sp2act_out, sp2act_in   = mp.Pipe() #setpoint data pipe to actuation
        sp2cm_out, sp2cm_in     = mp.Pipe() #setpoint data pipe to control model
        sp2csv_out, sp2csv_in   = mp.Pipe() #setpoint data pipe to CSV

        pipes_dict = {'act2csv_out':act2csv_out, 'act2csv_in':act2csv_in, 'act2tx_out':act2tx_out, 'act2tx_in':act2tx_in, 'cm2act_out':cm2act_out, 'cm2act_in':cm2act_in, 'cm2csv_out':cm2csv_out, 'cm2csv_in':cm2csv_in, 'dyn2csv_out':dyn2csv_out, 'dyn2csv_in':dyn2csv_in, 'rx2act_out':rx2act_out, 'rx2act_in':rx2act_in, 'rx2csv_out':rx2csv_out, 'rx2csv_in':rx2csv_in, 'rx2dyn_out':rx2dyn_out, 'rx2dyn_in':rx2dyn_in, 'rx2cm_out':rx2cm_out, 'rx2cm_in':rx2cm_in, 'rx2sp_out':rx2sp_out, 'rx2sp_in':rx2sp_in, 'rx2sup_out':rx2sup_out, 'rx2sup_in':rx2sup_in, 'sp2act_out':sp2act_out, 'sp2act_in':sp2act_in, 'sp2cm_out':sp2cm_out, 'sp2cm_in':sp2cm_in, 'sp2csv_out':sp2csv_out, 'sp2csv_in':sp2csv_in}

        #Construct global dictionary
        self.global_dict = {**events_dict, **pipes_dict, **mods_dict}

        #Select supervisor based on type of simulation
        if (SIM_TYPE == 1) or (SIM_TYPE == 'recursive'):
            self.recursive_sim()

    def start_processes(self):
        #Create processes
        act_proc = self.actuation_process()
        cm_proc  = self.control_model_process()
        cfg_proc = self.configuration_process()
        dyn_proc = self.dynamics_process()
        sp_proc  = self.setpoint_process()
        telemrx_proc, telemtx_proc = self.telemetry_processes()
        cmlog_proc, dynlog_proc, splog_proc, telemrxlog_proc, telemtxlog_proc = self.csvlogging_processes()

        #Start processes (order is important)
        telemrx_proc.start()
        telemtx_proc.start()
        cfg_proc.start()
        act_proc.start()
        cm_proc.start()
        dyn_proc.start()
        sp_proc.start()
        cmlog_proc.start()
        dynlog_proc.start()
        splog_proc.start()
        telemrxlog_proc.start()
        telemtxlog_proc.start()
        time.sleep(1)
        scen_proc = self.scenario_process() #scenario subprocess automatically starts when called

        #Update global dictionary with new processes
        self.global_dict.update(act_proc = act_proc)
        self.global_dict.update(cfg_proc = cfg_proc)
        self.global_dict.update(cm_proc = cm_proc)
        self.global_dict.update(cmlog_proc = cmlog_proc)
        self.global_dict.update(dyn_proc = dyn_proc)
        self.global_dict.update(dynlog_proc = dynlog_proc)
        self.global_dict.update(scen_proc = scen_proc)
        self.global_dict.update(sp_proc = sp_proc)
        self.global_dict.update(splog_proc = splog_proc)
        self.global_dict.update(telemrx_proc = telemrx_proc)
        self.global_dict.update(telemrxlog_proc = telemrxlog_proc)
        self.global_dict.update(telemtx_proc = telemtx_proc)
        self.global_dict.update(telemtxlog_proc = telemtxlog_proc)

    def terminate_processes(self):
        #Get processes
        act_proc        = self.global_dict['act_proc']
        cfg_proc        = self.global_dict['cfg_proc']
        cm_proc         = self.global_dict['cm_proc']
        cmlog_proc      = self.global_dict['cmlog_proc']
        dyn_proc        = self.global_dict['dyn_proc']
        dynlog_proc     = self.global_dict['dynlog_proc']
        scen_proc       = self.global_dict['scen_proc']
        sp_proc         = self.global_dict['sp_proc']
        splog_proc      = self.global_dict['splog_proc']
        telemrx_proc    = self.global_dict['telemrx_proc']
        telemrxlog_proc = self.global_dict['telemrxlog_proc']
        telemtx_proc    = self.global_dict['telemtx_proc']
        telemtxlog_proc = self.global_dict['telemtxlog_proc']

        #Terminate processes (order is important)
        scen_proc.terminate()
        time.sleep(1)
        act_proc.terminate()
        cm_proc.terminate()
        dyn_proc.terminate()
        sp_proc.terminate()
        cfg_proc.terminate()
        telemtx_proc.terminate()
        telemrx_proc.terminate()
        cmlog_proc.terminate()
        dynlog_proc.terminate()
        splog_proc.terminate()
        telemrxlog_proc.terminate()
        telemtxlog_proc.terminate()

        #Update global dictionary with new processes
        self.global_dict.update(act_proc = act_proc)
        self.global_dict.update(cfg_proc = cfg_proc)
        self.global_dict.update(cm_proc = cm_proc)
        self.global_dict.update(cmlog_proc = cmlog_proc)
        self.global_dict.update(dyn_proc = dyn_proc)
        self.global_dict.update(dynlog_proc = dynlog_proc)
        self.global_dict.update(scen_proc = scen_proc)
        self.global_dict.update(sp_proc = sp_proc)
        self.global_dict.update(splog_proc = splog_proc)
        self.global_dict.update(telemrx_proc = telemrx_proc)
        self.global_dict.update(telemrxlog_proc = telemrxlog_proc)
        self.global_dict.update(telemtx_proc = telemtx_proc)
        self.global_dict.update(telemtxlog_proc = telemtxlog_proc)

    def clear_events(self):
        event_rxtcp = self.global_dict['event_rxtcp']
        event_txtcp = self.global_dict['event_txtcp']
        event_start = self.global_dict['event_start']
        event_end   = self.global_dict['event_end']

        #Clear events
        event_rxtcp.clear()
        event_txtcp.clear()
        event_start.clear()
        event_end.clear()

        #Update global dictionary with cleared events
        self.global_dict.update(event_rxtcp = event_rxtcp)
        self.global_dict.update(event_txtcp = event_txtcp)
        self.global_dict.update(event_start = event_start)
        self.global_dict.update(event_end = event_end)

    def restore_pipes(self):
        act2csv_out = self.global_dict['act2csv_out']
        act2csv_in  = self.global_dict['act2csv_in']
        act2tx_out  = self.global_dict['act2tx_out']
        act2tx_in   = self.global_dict['act2tx_in']
        cm2act_out  = self.global_dict['cm2act_out']
        cm2act_in   = self.global_dict['cm2act_in']
        cm2csv_out  = self.global_dict['cm2csv_out']
        cm2csv_in   = self.global_dict['cm2csv_in']
        dyn2csv_out = self.global_dict['dyn2csv_out']
        dyn2csv_in  = self.global_dict['dyn2csv_in']
        rx2act_out  = self.global_dict['rx2act_out']
        rx2act_in   = self.global_dict['rx2act_in']
        rx2cm_out   = self.global_dict['rx2cm_out']
        rx2cm_in    = self.global_dict['rx2cm_in']
        rx2csv_out  = self.global_dict['rx2csv_out']
        rx2csv_in   = self.global_dict['rx2csv_in']
        rx2dyn_out  = self.global_dict['rx2dyn_out']
        rx2dyn_in   = self.global_dict['rx2dyn_in']
        rx2sp_out   = self.global_dict['rx2sp_out']
        rx2sp_in    = self.global_dict['rx2sp_in']
        rx2sup_out  = self.global_dict['rx2sup_out']
        rx2sup_in   = self.global_dict['rx2sup_in']
        sp2act_out  = self.global_dict['sp2act_out']
        sp2act_in   = self.global_dict['sp2act_in']
        sp2cm_out   = self.global_dict['sp2cm_out']
        sp2cm_in    = self.global_dict['sp2cm_in']
        sp2csv_out  = self.global_dict['sp2csv_out']
        sp2csv_in   = self.global_dict['sp2csv_in']

        #Create new pipes
        act2csv_out, act2csv_in = mp.Pipe() #actuation data pipe to CSV
        act2tx_out, act2tx_in   = mp.Pipe() #actuation data pipe to TX telemetry
        cm2act_out, cm2act_in   = mp.Pipe() #control model data pipe to actuation
        cm2csv_out, cm2csv_in   = mp.Pipe() #control model data pipe to CSV
        dyn2csv_out, dyn2csv_in = mp.Pipe() #dynamics data pipe to CSV
        rx2act_out, rx2act_in   = mp.Pipe() #RX telemetry data pipe to actuation
        rx2cm_out, rx2cm_in     = mp.Pipe() #RX telemetry data pipe to control model
        rx2csv_out, rx2csv_in   = mp.Pipe() #RX telemetry data pipe to CSV
        rx2dyn_out, rx2dyn_in   = mp.Pipe() #RX telemetry data pipe to dynamics 
        rx2sp_out, rx2sp_in     = mp.Pipe() #RX telemetry data pipe to setpoint
        rx2sup_out, rx2sup_in   = mp.Pipe() #RX telemetry data pipe to supervisor
        sp2act_out, sp2act_in   = mp.Pipe() #setpoint data pipe to actuation
        sp2cm_out, sp2cm_in     = mp.Pipe() #setpoint data pipe to control model
        sp2csv_out, sp2csv_in   = mp.Pipe() #setpoint data pipe to CSV

        #Update global dictionary with new pipes
        self.global_dict.update(act2csv_out = act2csv_out)
        self.global_dict.update(act2csv_in = act2csv_in)
        self.global_dict.update(act2tx_out = act2tx_out)
        self.global_dict.update(act2tx_in = act2tx_in)
        self.global_dict.update(cm2act_out = cm2act_out)
        self.global_dict.update(cm2act_in = cm2act_in)
        self.global_dict.update(cm2csv_out = cm2csv_out)
        self.global_dict.update(cm2csv_in = cm2csv_in)
        self.global_dict.update(dyn2csv_out = dyn2csv_out)
        self.global_dict.update(dyn2csv_in = dyn2csv_in)
        self.global_dict.update(rx2act_out = rx2act_out)
        self.global_dict.update(rx2act_in = rx2act_in)
        self.global_dict.update(rx2cm_out = rx2cm_out)
        self.global_dict.update(rx2cm_in = rx2cm_in)
        self.global_dict.update(rx2csv_out = rx2csv_out)
        self.global_dict.update(rx2csv_in = rx2csv_in)
        self.global_dict.update(rx2dyn_out = rx2dyn_out)
        self.global_dict.update(rx2dyn_in = rx2dyn_in)
        self.global_dict.update(rx2sp_out = rx2sp_out)
        self.global_dict.update(rx2sp_in = rx2sp_in)
        self.global_dict.update(rx2sup_out = rx2sup_out)
        self.global_dict.update(rx2sup_in = rx2sup_in)
        self.global_dict.update(sp2act_out = sp2act_out)
        self.global_dict.update(sp2act_in = sp2act_in)
        self.global_dict.update(sp2cm_out = sp2cm_out)
        self.global_dict.update(sp2cm_in = sp2cm_in)
        self.global_dict.update(sp2csv_out = sp2csv_out)
        self.global_dict.update(sp2csv_in = sp2csv_in)

    def actuation_process(self):
        act_mod     = self.global_dict['act_mod']
        act2csv_in  = self.global_dict['act2csv_in']
        act2tx_in   = self.global_dict['act2tx_in']
        rx2act_out  = self.global_dict['rx2act_out']
        event_start = self.global_dict['event_start']
        event_end   = self.global_dict['event_end']
        if (ACT_TYPE == 0) or (ACT_TYPE == 'random'):
            act_proc = mp.Process(target=act_mod.random_control, args=(act2csv_in, act2tx_in, rx2act_out, event_start, event_end), daemon=True) #process for random actuation
        return act_proc

    def control_model_process(self):
        cm_mod      = self.global_dict['cm_mod']
        cm2act_in   = self.global_dict['cm2act_in']
        cm2csv_in   = self.global_dict['cm2csv_in']
        rx2cm_out   = self.global_dict['rx2cm_out']
        sp2cm_out   = self.global_dict['sp2cm_out']
        event_start = self.global_dict['event_start']
        event_end   = self.global_dict['event_end']
        if CM_TYPE == 'ANL': #analytic non-linear control model
            cm_proc = mp.Process(target=cm_mod.anlcm, args=(cm2act_in, cm2csv_in, rx2cm_out, sp2cm_out, event_start, event_end), daemon=True) #process for analytic non-linear control model
        elif CM_TYPE == 'AL': #analytic linear control model
            cm_proc = mp.Process(target=cm_mod.alcm, args=(cm2act_in, cm2csv_in, rx2cm_out, sp2cm_out, event_start, event_end), daemon=True) #process for analytic linear control model
        return cm_proc

    def configuration_process(self):
        cfg_mod     = self.global_dict['cfg_mod']
        event_rxtcp = self.global_dict['event_rxtcp']
        cfg_proc    = mp.Process(target=cfg_mod.transmit, args=(event_rxtcp,), daemon=True) #process for configuring FlightGear
        return cfg_proc

    def setpoint_process(self):
        sp_mod      = self.global_dict['sp_mod']
        rx2sp_out   = self.global_dict['rx2sp_out']
        sp2cm_in    = self.global_dict['sp2cm_in']
        sp2csv_in   = self.global_dict['sp2csv_in']
        event_start = self.global_dict['event_start']
        event_end   = self.global_dict['event_end']
        if (SP_TYPE == 0) or (SP_TYPE == 'constant'):
            sp_proc = mp.Process(target=sp_mod.constant, args=(rx2sp_out, sp2cm_in, sp2csv_in, event_start, event_end), daemon=True) #process for setpoint
        return sp_proc

    def dynamics_process(self):
        dyn_mod     = self.global_dict['dyn_mod']
        dyn2csv_in  = self.global_dict['dyn2csv_in']
        rx2dyn_out  = self.global_dict['rx2dyn_out']
        event_start = self.global_dict['event_start']
        event_end   = self.global_dict['event_end']
        dyn_proc    = mp.Process(target=dyn_mod.forces_moments_body, args=(dyn2csv_in, rx2dyn_out, event_start, event_end), daemon=True) #process for dynamics
        return dyn_proc

    def csvlogging_processes(self):
        csvlog_mod      = self.global_dict['csvlog_mod']
        act2csv_out     = self.global_dict['act2csv_out']
        cm2csv_out      = self.global_dict['cm2csv_out']
        dyn2csv_out     = self.global_dict['dyn2csv_out']
        rx2csv_out      = self.global_dict['rx2csv_out']
        sp2csv_out      = self.global_dict['sp2csv_out']
        event_start     = self.global_dict['event_start']
        event_end       = self.global_dict['event_end']
        cmlog_proc      = mp.Process(target=csvlog_mod.write_cmlog, args=(cm2csv_out, event_start, event_end), daemon=True) #process for logging control model
        dynlog_proc     = mp.Process(target=csvlog_mod.write_dynlog, args=(dyn2csv_out, event_start, event_end), daemon=True) #process for logging dynamics
        splog_proc      = mp.Process(target=csvlog_mod.write_splog, args=(sp2csv_out, event_start, event_end), daemon=True) #process for logging setpoint
        telemrxlog_proc = mp.Process(target=csvlog_mod.write_telemrxlog, args=(rx2csv_out, event_start, event_end), daemon=True) #process for logging RX telemetry
        telemtxlog_proc = mp.Process(target=csvlog_mod.write_telemtxlog, args=(act2csv_out, event_start, event_end), daemon=True) #process for logging TX telemetry
        return cmlog_proc, dynlog_proc, splog_proc, telemrxlog_proc, telemtxlog_proc

    def scenario_process(self):
        scen_mod = self.global_dict['scen_mod']
        if (SCENARIO_TYPE == 0) or (SCENARIO_TYPE == 'cruise'):
            command_args_str = scen_mod.cruise() #scenario command
            scen_proc = subprocess.Popen(args=command_args_str) #process for FlightGear scenario
        return scen_proc

    def telemetry_processes(self):
        telem_mod   = self.global_dict['telem_mod']
        act2tx_out  = self.global_dict['act2tx_out']
        rx2act_in   = self.global_dict['rx2act_in']
        rx2cm_in    = self.global_dict['rx2cm_in']
        rx2csv_in   = self.global_dict['rx2csv_in']
        rx2dyn_in   = self.global_dict['rx2dyn_in']
        rx2sp_in    = self.global_dict['rx2sp_in']
        rx2sup_in   = self.global_dict['rx2sup_in']
        event_rxtcp = self.global_dict['event_rxtcp']
        event_txtcp = self.global_dict['event_txtcp']
        event_start = self.global_dict['event_start']
        event_end   = self.global_dict['event_end']

        telemrx_proc = mp.Process(target=telem_mod.receive, args=(rx2act_in, rx2csv_in, rx2dyn_in, rx2sp_in, rx2cm_in, rx2sup_in, event_rxtcp, event_start, event_end), daemon=True) #process for RX telemetry
        telemtx_proc = mp.Process(target=telem_mod.transmit, args=(act2tx_out, event_rxtcp, event_txtcp, event_start, event_end), daemon=True) #process for TX telemetry
        return telemrx_proc, telemtx_proc

    def recursive_sim(self):
        #Multiple simulations loop
        for _ in range(SIM_ITER_NUM):
            self.start_processes()
            print(0)
            self.print_procs()
            self.print_events()
            #Get pipes and events
            rx2sup_out  = self.global_dict['rx2sup_out']
            event_start = self.global_dict['event_start']
            event_end   = self.global_dict['event_end']
            #Wait for simulation run start event
            event_start.wait()
            #Single simulation loop
            while True:
                rxdata    = rx2sup_out.recv() #receive RX telemetry
                lastframe = rxdata[-1]
                terminate_run = self.simulation_watchdog(lastframe)
                if terminate_run:
                    print(1)
                    self.print_procs()
                    self.print_events()
                    event_end.set()
                    self.global_dict.update(event_end = event_end)
                    print(2)
                    self.print_procs()
                    self.print_events()
                    time.sleep(1)
                    self.terminate_processes()
                    self.clear_events()
                    self.restore_pipes()
                    terminate_run = False
                    print(3)
                    self.print_procs()
                    self.print_events()
                    break
        quit()

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

    def print_procs(self):
        #Get processes
        act_proc        = self.global_dict['act_proc']
        cfg_proc        = self.global_dict['cfg_proc']
        cm_proc         = self.global_dict['cm_proc']
        cmlog_proc      = self.global_dict['cmlog_proc']
        dyn_proc        = self.global_dict['dyn_proc']
        dynlog_proc     = self.global_dict['dynlog_proc']
        scen_proc       = self.global_dict['scen_proc']
        sp_proc         = self.global_dict['sp_proc']
        splog_proc      = self.global_dict['splog_proc']
        telemrx_proc    = self.global_dict['telemrx_proc']
        telemrxlog_proc = self.global_dict['telemrxlog_proc']
        telemtx_proc    = self.global_dict['telemtx_proc']
        telemtxlog_proc = self.global_dict['telemtxlog_proc']
        print('RX telem_mod: ' + str(telemrx_proc.is_alive()))
        print('TX telem_mod: ' + str(telemtx_proc.is_alive()))
        print('cfg_mod: ' + str(cfg_proc.is_alive()))
        print('act_proc: ' + str(act_proc.is_alive()))
        print('cm_mod: ' + str(cm_proc.is_alive()))
        print('dyn_mod: ' + str(dyn_proc.is_alive()))
        print('sp_mod: ' + str(sp_proc.is_alive()))
        print('cmlog_mod: ' + str(cmlog_proc.is_alive()))
        print('dynlog_mod: ' + str(dynlog_proc.is_alive()))
        print('splog_mod: ' + str(splog_proc.is_alive()))
        print('telemrxlog_mod: ' + str(telemrxlog_proc.is_alive()))
        print('telemtxlog_mod: ' + str(telemtxlog_proc.is_alive()))

    def print_events(self):
        #Get processes
        event_rxtcp = self.global_dict['event_rxtcp']
        event_txtcp = self.global_dict['event_txtcp']
        event_start = self.global_dict['event_start']
        event_end   = self.global_dict['event_end']
        print('event_rxtcp: ' + str(event_rxtcp.is_set()))
        print('event_txtcp: ' + str(event_txtcp.is_set()))
        print('event_start: ' + str(event_start.is_set()))
        print('event_end: ' + str(event_end.is_set()))
