import multiprocessing as mp
import socket
import subprocess
import time

from settings import *
from utils import *

class Supervisor():

    def __init__(self, **global_dict):
        self.global_dict = global_dict
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

        time.sleep(1)
        scen_proc = self.scenario_process() #scenario subprocess automatically starts when called

        #Update global dict with new processes
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
        time.sleep(1)
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

        '''
        #Delete processes
        del act_proc
        del cm_proc
        del cmlog_proc
        del cfg_proc
        del dyn_proc
        del dynlog_proc
        del scen_proc
        del sp_proc
        del splog_proc
        del telemrx_proc
        del telemrxlog_proc
        del telemtx_proc
        del telemtxlog_proc
        '''

        #Update global dict with new processes
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

        #Clear events
        event_rxtcp.clear()
        event_txtcp.clear()
        event_start.clear()

        #Update global dictionary with cleared events
        self.global_dict.update(event_rxtcp = event_rxtcp)
        self.global_dict.update(event_txtcp = event_txtcp)
        self.global_dict.update(event_start = event_start)

    def actuation_process(self):
        act_mod     = self.global_dict['act_mod']
        act2csv_in  = self.global_dict['act2csv_in']
        act2tx_in   = self.global_dict['act2tx_in']
        rx2act_out  = self.global_dict['rx2act_out']
        event_start = self.global_dict['event_start']
        if (ACT_TYPE == 0) or (ACT_TYPE == 'random'):
            act_proc = mp.Process(target=act_mod.random_control, args=(act2csv_in, act2tx_in, rx2act_out, event_start), daemon=True) #process for random actuation
        return act_proc

    def control_model_process(self):
        cm_mod      = self.global_dict['cm_mod']
        cm2act_in   = self.global_dict['cm2act_in']
        cm2csv_in   = self.global_dict['cm2csv_in']
        rx2cm_out   = self.global_dict['rx2cm_out']
        sp2cm_out   = self.global_dict['sp2cm_out']
        event_start = self.global_dict['event_start']
        if CM_TYPE == 'ANL': #analytic non-linear control model
            cm_proc = mp.Process(target=cm_mod.anlcm, args=(cm2act_in, cm2csv_in, rx2cm_out, sp2cm_out, event_start), daemon=True) #process for analytic non-linear control model
        elif CM_TYPE == 'AL': #analytic linear control model
            cm_proc = mp.Process(target=cm_mod.alcm, args=(cm2act_in, cm2csv_in, rx2cm_out, sp2cm_out, event_start), daemon=True) #process for analytic linear control model
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
        if (SP_TYPE == 0) or (SP_TYPE == 'constant'):
            sp_proc = mp.Process(target=sp_mod.constant, args=(rx2sp_out, sp2cm_in, sp2csv_in, event_start), daemon=True) #process for setpoint
        return sp_proc

    def dynamics_process(self):
        dyn_mod     = self.global_dict['dyn_mod']
        dyn2csv_in  = self.global_dict['dyn2csv_in']
        rx2dyn_out  = self.global_dict['rx2dyn_out']
        event_start = self.global_dict['event_start']
        dyn_proc    = mp.Process(target=dyn_mod.forces_moments_body, args=(dyn2csv_in, rx2dyn_out, event_start), daemon=True) #process for dynamics
        return dyn_proc

    def csvlogging_processes(self):
        csvlog_mod      = self.global_dict['csvlog_mod']
        act2csv_out     = self.global_dict['act2csv_out']
        cm2csv_out      = self.global_dict['cm2csv_out']
        dyn2csv_out     = self.global_dict['dyn2csv_out']
        rx2csv_out      = self.global_dict['rx2csv_out']
        sp2csv_out      = self.global_dict['sp2csv_out']
        event_start     = self.global_dict['event_start']
        cmlog_proc      = mp.Process(target=csvlog_mod.write_cmlog, args=(cm2csv_out, event_start), daemon=True) #process for logging control model
        dynlog_proc     = mp.Process(target=csvlog_mod.write_dynlog, args=(dyn2csv_out, event_start), daemon=True) #process for logging dynamics
        splog_proc      = mp.Process(target=csvlog_mod.write_splog, args=(sp2csv_out, event_start), daemon=True) #process for logging setpoint
        telemrxlog_proc = mp.Process(target=csvlog_mod.write_telemrxlog, args=(rx2csv_out, event_start), daemon=True) #process for logging RX telemetry
        telemtxlog_proc = mp.Process(target=csvlog_mod.write_telemtxlog, args=(act2csv_out, event_start), daemon=True) #process for logging TX telemetry
        return cmlog_proc, dynlog_proc, splog_proc, telemrxlog_proc, telemtxlog_proc

    def scenario_process(self):
        scen_mod = self.global_dict['scen_mod']
        if (SCENARIO_TYPE == 0) or (SCENARIO_TYPE == 'cruise'):
            command_args_str = scen_mod.cruise() #scenario command
            scen_proc = subprocess.Popen(args=command_args_str) #process for scenario in FlightGear
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

        telemrx_proc = mp.Process(target=telem_mod.receive, args=(rx2act_in, rx2csv_in, rx2dyn_in, rx2sp_in, rx2cm_in, rx2sup_in, event_rxtcp, event_start), daemon=True) #process for RX telemetry
        telemtx_proc = mp.Process(target=telem_mod.transmit, args=(act2tx_out, event_rxtcp, event_txtcp, event_start), daemon=True) #process for TX telemetry
        return telemrx_proc, telemtx_proc

    def recursive_sim(self):
        #Get pipes and events
        rx2sup_out  = self.global_dict['rx2sup_out']
        event_start = self.global_dict['event_start']
        #Multiple simulations loop
        for _ in range(SIM_ITER_NUM):
            self.start_processes()
            event_start.wait() #wait for simulation start event
            while True: #single simulation loop
                rxdata    = rx2sup_out.recv() #receive RX telemetry
                lastframe = rxdata[-1]
                terminate_flag = self.simulation_watchdog(lastframe)
                if terminate_flag:
                    self.terminate_processes()
                    self.clear_events()
                    terminate_flag = False
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
