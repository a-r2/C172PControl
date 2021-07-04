import csv
import numpy as np
import os
import time
import sys

from constants import *
from settings import *

class CSVLogging():

    def __init__(self):
        #CSV filenames
        self.cmfn      = CM_LOG_FILENAME + '.csv'
        self.dynfn     = DYN_LOG_FILENAME + '.csv'
        self.spfn      = SP_LOG_FILENAME + '.csv'
        self.telemrxfn = TELEM_RX_LOG_FILENAME + '.csv'
        self.telemtxfn = TELEM_TX_LOG_FILENAME + '.csv'
        
    def header_cmlog(self, log_dir):
        cmpath = os.path.join(log_dir, self.cmfn)
        with open(cmpath, 'w', newline='') as csvfile:
            cmlog = csv.writer(csvfile, delimiter=' ') #CSV writer object
            timestamp = time.strftime('%Y|%b|%d|%H:%M:%S(UTC)', time.gmtime())
            cmlog.writerow(timestamp) #write timestamp into CSV 
            cmlog.writerow('actuation={}|control_model={}|scenario={}|setpoint={}'.format(ACT_TYPE, CM_TYPE, SCEN_TYPE, SCEN_TYPE)) #write simulation information into CSV 
            del cmlog
            fieldnames = ('t_sim', *CM_STATE_STR)
            cmlog = csv.DictWriter(csvfile, fieldnames=fieldnames) #CSV dictionary writer object
            cmlog.writeheader()

    def header_dynlog(self, log_dir):
        dynpath = os.path.join(log_dir, self.dynfn)
        with open(dynpath, 'w', newline='') as csvfile:
            dynlog = csv.writer(csvfile, delimiter=' ') #CSV writer object
            timestamp = time.strftime('%Y|%b|%d|%H:%M:%S(UTC)', time.gmtime())
            dynlog.writerow(timestamp) #write timestamp into CSV 
            dynlog.writerow('actuation={}|control_model={}|scenario={}|setpoint={}'.format(ACT_TYPE, CM_TYPE, SCEN_TYPE, SCEN_TYPE)) #write simulation information into CSV 
            del dynlog
            fieldnames = ('t_sim', *DYN_STR)
            dynlog = csv.DictWriter(csvfile, fieldnames=fieldnames) #CSV dictionary writer object
            dynlog.writeheader()

    def header_splog(self, log_dir):
        sppath = os.path.join(log_dir, self.spfn)
        with open(sppath, 'w', newline='') as csvfile:
            splog = csv.writer(csvfile, delimiter=' ') #CSV writer object
            timestamp = time.strftime('%Y|%b|%d|%H:%M:%S(UTC)', time.gmtime())
            splog.writerow(timestamp) #write timestamp into CSV 
            splog.writerow('actuation={}|control_model={}|scenario={}|setpoint={}'.format(ACT_TYPE, CM_TYPE, SCEN_TYPE, SCEN_TYPE)) #write simulation information into CSV 
            del splog
            fieldnames = ('t_sim', *CM_STATE_STR, *CM_INPUT_STR)
            splog = csv.DictWriter(csvfile, fieldnames=fieldnames) #CSV dictionary writer object
            splog.writeheader()

    def header_telemrxlog(self, log_dir):
        telemrxpath = os.path.join(log_dir, self.telemrxfn)
        with open(telemrxpath, 'w', newline='') as csvfile:
            telemrxlog = csv.writer(csvfile, delimiter=' ') #CSV writer object
            timestamp = time.strftime('%Y|%b|%d|%H:%M:%S(UTC)', time.gmtime())
            telemrxlog.writerow(timestamp) #write timestamp into CSV 
            telemrxlog.writerow('actuation={}|control_model={}|scenario={}|setpoint={}'.format(ACT_TYPE, CM_TYPE, SCEN_TYPE, SCEN_TYPE)) #write simulation information into CSV 
            del telemrxlog
            fieldnames = TELEM_RX_STR
            telemrxlog = csv.DictWriter(csvfile, fieldnames=fieldnames) #CSV dictionary writer object
            telemrxlog.writeheader()

    def header_telemtxlog(self, log_dir):
        telemtxpath = os.path.join(log_dir, self.telemtxfn)
        with open(telemtxpath, 'w', newline='') as csvfile:
            telemtxlog = csv.writer(csvfile, delimiter=' ') #CSV writer object
            timestamp = time.strftime('%Y|%b|%d|%H:%M:%S(UTC)', time.gmtime())
            telemtxlog.writerow(timestamp) #write timestamp into CSV 
            telemtxlog.writerow('actuation={}|control_model={}|scenario={}|setpoint={}'.format(ACT_TYPE, CM_TYPE, SCEN_TYPE, SCEN_TYPE)) #write simulation information into CSV 
            del telemtxlog
            fieldnames = ('t_sim', *TELEM_TX_STR)
            telemtxlog = csv.DictWriter(csvfile, fieldnames=fieldnames) #CSV dictionary writer object
            telemtxlog.writeheader()

    def read_cmlog(self):
        with open(self.cmfn, 'r', newline='') as csvfile:
            cmlog     = csv.reader(csvfile, delimiter=' ') #CSV reader object
            rowscount = len(list(cmlog))
            csvfile.seek(0) #go to first row
            cmdata = np.zeros((rowscount, CM_STATE_LEN + 1))
            i = 2 #avoid reading CSV log header
            for row in cmlog:
                cmdata[i,:] = row
                i += 1
            return cmdata

    def read_dynlog(self):
        with open(self.dynfn, 'r', newline='') as csvfile:
            dynlog = csv.reader(csvfile, delimiter=' ') #CSV reader object
            rowscount = len(list(dynlog))
            csvfile.seek(0) #go to first row
            dyndata = np.zeros((rowscount, DYN_LEN + 1))
            i = 2 #avoid reading CSV log header
            for row in dynlog:
                dyndata[i,:] = row
                i += 1
            return dyndata

    def read_splog(self):
        with open(self.spfn, 'r', newline='') as csvfile:
            splog = csv.reader(csvfile, delimiter=' ') #CSV reader object
            rowscount = len(list(splog))
            csvfile.seek(0) #go to first row
            spdata = np.zeros((rowscount, CM_STATE_LEN + CM_INPUT_LEN + 1))
            i = 2 #avoid reading CSV log header
            for row in splog:
                spdata[i,:] = row
                i += 1
            return spdata

    def read_telemrxlog(self):
        with open(self.telemrxfn, 'r', newline='') as csvfile:
            telemrxlog = csv.reader(csvfile, delimiter=' ') #CSV reader object
            rowscount = len(list(telemrxlog))
            csvfile.seek(0) #go to first row
            rxdata = np.zeros((rowscount, TELEM_RX_LEN))
            i = 2 #avoid reading CSV log header
            for row in telemrxlog:
                rxdata[i,:] = row
                i += 1
            return rxdata

    def read_telemtxlog(self):
        with open(self.telemtxfn, 'r', newline='') as csvfile:
            telemtxlog = csv.reader(csvfile, delimiter=' ') #CSV reader object
            rowscount = len(list(telemtxlog))
            csvfile.seek(0) #go to first row
            txdata = np.zeros((rowscount, TELEM_TX_LEN + 1))
            i = 2
            for row in telemtxlog:
                txdata[i,:] = row
                i += 1
            return txdata

    def write_cmlog(self, log_dir, cm2csv_out, event_start, event_end):
        cmpath = os.path.join(log_dir, self.cmfn)
        fieldnames = ('time', *CM_STATE_STR)
        csvdata = np.zeros((CM_HZ, CM_STATE_LEN + 1)) #array for storing data frames
        i = 0
        event_start.wait() #wait for simulation start
        while True:
            try:
                if event_end.is_set():
                    #Close pipe
                    cm2csv_out.close()
                    break
                else:
                    cmdata       = cm2csv_out.recv()
                    csvdata[i,:] = cmdata
                    i += 1
                    if i > (CM_HZ - 1):
                        with open(cmpath, 'a+', newline='') as csvfile:
                            cmlog = csv.DictWriter(csvfile, fieldnames=fieldnames) #CSV writer object
                            cmlog.writerows((dict(zip(fieldnames, csvdata[k,:])) for k in range(CM_HZ))) #write array into CSV 
                        i = 0
            except:
                raise RuntimeError('.'.join((__name__, sys._getframe().f_code.co_name)))

    def write_dynlog(self, log_dir, dyn2csv_out, event_start, event_end):
        dynpath = os.path.join(log_dir, self.dynfn)
        fieldnames = ('time', *DYN_STR)
        csvdata1 = np.zeros((MODEL_HZ, DYN_LEN + 1)) #array for storing data frames
        csvdata2 = np.zeros((MODEL_HZ, DYN_LEN + 1)) #backup array for storing overflowed data frames
        i = 0
        event_start.wait() #wait for simulation start
        while True:
            try:
                if event_end.is_set():
                    #Close pipe
                    dyn2csv_out.close()
                    break
                else:
                    dyndata = dyn2csv_out.recv() #receive RX telemetry
                    framescount = dyndata.shape[0]
                    for j in range(framescount):                
                        if i > (MODEL_HZ - 1):
                            csvdata2[i-MODEL_HZ,:] = dyndata[j,:] 
                        else:
                            csvdata1[i,:] = dyndata[j,:]
                        i += 1
                    if i > (MODEL_HZ - 1):
                        with open(dynpath, 'a+', newline='') as csvfile:
                            dynlog = csv.DictWriter(csvfile, fieldnames=fieldnames) #CSV writer object
                            dynlog.writerows((dict(zip(fieldnames, csvdata1[k,:])) for k in range(MODEL_HZ))) #write array into CSV 
                        i = (i - MODEL_HZ) if (i - MODEL_HZ) > 0 else 0
                        csvdata1 = csvdata2
                        csvdata2 = np.empty((MODEL_HZ, (DYN_LEN + 1))) #empty backup array 
            except:
                raise RuntimeError('.'.join((__name__, sys._getframe().f_code.co_name)))

    def write_splog(self, log_dir, sp2csv_out, event_start, event_end):
        sppath = os.path.join(log_dir, self.spfn)
        fieldnames = ('time', *CM_STATE_STR, *CM_INPUT_STR)
        csvdata = np.zeros((CM_HZ, CM_STATE_LEN + CM_INPUT_LEN + 1)) #array for storing data frames
        i = 0
        event_start.wait() #wait for simulation start
        while True:
            try:
                if event_end.is_set():
                    #Close pipe
                    sp2csv_out.close()
                    break
                else:
                    spdata = sp2csv_out.recv()
                    csvdata[i,:] = spdata
                    i += 1
                    if i > (CM_HZ - 1):
                        with open(sppath, 'a+', newline='') as csvfile:
                            splog = csv.DictWriter(csvfile, fieldnames=fieldnames) #CSV writer object
                            splog.writerows((dict(zip(fieldnames, csvdata[k,:])) for k in range(CM_HZ))) #write array into CSV 
                        i = 0
            except:
                raise RuntimeError('.'.join((__name__, sys._getframe().f_code.co_name)))

    def write_telemrxlog(self, log_dir, rx2csv_out, event_start, event_end):
        telemrxpath = os.path.join(log_dir, self.telemrxfn)
        fieldnames = TELEM_RX_STR
        csvdata1 = np.zeros((MODEL_HZ, TELEM_RX_LEN)) #array for storing data frames
        csvdata2 = np.zeros((MODEL_HZ, TELEM_RX_LEN)) #backup array for storing overflowed data frames
        i = 0
        event_start.wait() #wait for simulation start
        while True:
            try:
                if event_end.is_set():
                    #Close pipe
                    rx2csv_out.close()
                    break
                else:
                    rxdata = rx2csv_out.recv() #receive RX telemetry
                    framescount = rxdata.shape[0]
                    for j in range(framescount):                
                        if i > (MODEL_HZ - 1):
                            csvdata2[i-MODEL_HZ,:] = rxdata[j,:] 
                        else:
                            csvdata1[i,:] = rxdata[j,:]
                        i += 1
                    if i > (MODEL_HZ - 1):
                        with open(telemrxpath, 'a+', newline='') as csvfile:
                            telemrxlog = csv.DictWriter(csvfile, fieldnames=fieldnames) #CSV writer object
                            telemrxlog.writerows((dict(zip(fieldnames, csvdata1[k,:])) for k in range(MODEL_HZ))) #write array into CSV 
                        i = (i - MODEL_HZ) if (i - MODEL_HZ) > 0 else 0
                        csvdata1 = csvdata2
                        csvdata2 = np.empty((MODEL_HZ, TELEM_RX_LEN)) #empty backup array 
            except:
                raise RuntimeError('.'.join((__name__, sys._getframe().f_code.co_name)))

    def write_telemtxlog(self, log_dir, act2csv_out, event_start, event_end):
        telemtxpath = os.path.join(log_dir, self.telemtxfn)
        fieldnames = ('time', *TELEM_TX_STR)
        csvdata = np.zeros((ACT_HZ, TELEM_TX_LEN + 1)) #array for storing data frames
        i = 0
        event_start.wait() #wait for simulation start
        while True:
            try:
                if event_end.is_set():
                    #Close pipe
                    act2csv_out.close()
                    break
                else:
                    actdata = act2csv_out.recv()
                    csvdata[i,:] = actdata
                    i += 1
                    if i > (ACT_HZ - 1):
                        with open(telemtxpath, 'a+', newline='') as csvfile:
                            telemtxlog = csv.DictWriter(csvfile, fieldnames=fieldnames) #CSV writer object
                            telemtxlog.writerows((dict(zip(fieldnames, csvdata[k,:])) for k in range(ACT_HZ))) #write array into CSV 
                        i = 0
            except:
                raise RuntimeError('.'.join((__name__, sys._getframe().f_code.co_name)))
