import csv
import numpy as np
import os

from constants import *
from settings import *

class CSVLogging():

    def __init__(self):
        #CSV filenames
        self.cmfn      = CM_LOG_FILENAME + '.csv'
        self.dynfn     = DYN_LOG_FILENAME + '.csv'
        self.telemrxfn = 'rx_' + TELEM_LOG_FILENAME + '.csv'
        self.telemtxfn = 'tx_' + TELEM_LOG_FILENAME + '.csv'
        self.spfn      = SP_LOG_FILENAME + '.csv'
        
    def read_cmlog(self):
        with open(self.cmfn, 'r', newline='') as csvfile:
            cmlog     = csv.reader(csvfile, delimiter=' ') #CSV reader object
            rowscount = len(list(cmlog))
            csvfile.seek(0) #go to first row
            cmdata = np.zeros((rowscount, CM_STATE_LEN))
            i = 0
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
            i = 0
            for row in dynlog:
                dyndata[i,:] = row
                i += 1
            return dyndata

    def read_splog(self):
        with open(self.spfn, 'r', newline='') as csvfile:
            splog = csv.reader(csvfile, delimiter=' ') #CSV reader object
            rowscount = len(list(splog))
            csvfile.seek(0) #go to first row
            spdata = np.zeros((rowscount, CM_STATE_LEN))
            i = 0
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
            i = 0
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
            i = 0
            for row in telemtxlog:
                txdata[i,:] = row
                i += 1
            return txdata

    def write_cmlog(self, log_dir, cm2csv_out, event_start, event_end):
        cmpath = os.path.join(log_dir, self.cmfn)
        with open(cmpath, 'a+', newline='') as csvfile:
            cmlog = csv.writer(csvfile, delimiter=' ') #CSV writer object
            csvdata = np.zeros((CM_HZ, CM_STATE_LEN + 1)) #array for storing data frames
            i = 0
            event_start.wait() #wait for simulation start
            while True:
                if event_end.is_set():
                    #Close pipe
                    cm2csv_out.close()
                    break
                else:
                    cmdata       = cm2csv_out.recv()
                    csvdata[i,:] = cmdata
                    i += 1
                    if i > (CM_HZ - 1):
                        cmlog.writerows((csvdata[k,:] for k in range(CM_HZ))) #write array into CSV 
                        i = 0

    def write_dynlog(self, log_dir, dyn2csv_out, event_start, event_end):
        dynpath = os.path.join(log_dir, self.dynfn)
        with open(dynpath, 'a+', newline='') as csvfile:
            dynlog = csv.writer(csvfile, delimiter=' ') #CSV writer object
            csvdata1 = np.zeros((MODEL_HZ, DYN_LEN + 1)) #array for storing data frames
            csvdata2 = np.zeros((MODEL_HZ, DYN_LEN + 1)) #backup array for storing overflowed data frames
            i = 0
            event_start.wait() #wait for simulation start
            while True:
                if event_end.is_set():
                    #Close pipe
                    dyn2csv_out.close()
                    break
                else:
                    try:
                        dyndata = dyn2csv_out.recv() #receive RX telemetry
                        framescount = dyndata.shape[0]
                        for j in range(framescount):                
                            if i > (MODEL_HZ - 1):
                                csvdata2[i-MODEL_HZ,:] = dyndata[j,:] 
                            else:
                                csvdata1[i,:] = dyndata[j,:]
                            i += 1
                        if i > (MODEL_HZ - 1):
                            dynlog.writerows((csvdata1[k,:] for k in range(MODEL_HZ))) #write array into CSV 
                            i = (i - MODEL_HZ) if (i - MODEL_HZ) > 0 else 0
                            csvdata1 = csvdata2
                            csvdata2 = np.empty((MODEL_HZ, (DYN_LEN + 1))) #empty backup array 
                    except:
                        pass

    def write_splog(self, log_dir, sp2csv_out, event_start, event_end):
        sppath = os.path.join(log_dir, self.spfn)
        with open(sppath, 'a+', newline='') as csvfile:
            splog = csv.writer(csvfile, delimiter=' ') #CSV writer object
            csvdata = np.zeros((CM_HZ, CM_STATE_LEN + 1)) #array for storing data frames
            i = 0
            event_start.wait() #wait for simulation start
            while True:
                if event_end.is_set():
                    #Close pipe
                    sp2csv_out.close()
                    break
                else:
                    spdata = sp2csv_out.recv()
                    csvdata[i,:] = spdata
                    i += 1
                    if i > (CM_HZ - 1):
                        splog.writerows((csvdata[k,:] for k in range(CM_HZ))) #write array into CSV 
                        i = 0

    def write_telemrxlog(self, log_dir, rx2csv_out, event_start, event_end):
        telemrxpath = os.path.join(log_dir, self.telemrxfn)
        with open(telemrxpath, 'a+', newline='') as csvfile:
            telemrxlog = csv.writer(csvfile, delimiter=' ') #CSV writer object
            csvdata1 = np.zeros((MODEL_HZ, TELEM_RX_LEN)) #array for storing data frames
            csvdata2 = np.zeros((MODEL_HZ, TELEM_RX_LEN)) #backup array for storing overflowed data frames
            i = 0
            event_start.wait() #wait for simulation start
            while True:
                if event_end.is_set():
                    #Close pipe
                    rx2csv_out.close()
                    break
                else:
                    try:
                        rxdata = rx2csv_out.recv() #receive RX telemetry
                        framescount = rxdata.shape[0]
                        for j in range(framescount):                
                            if i > (MODEL_HZ - 1):
                                csvdata2[i-MODEL_HZ,:] = rxdata[j,:] 
                            else:
                                csvdata1[i,:] = rxdata[j,:]
                            i += 1
                        if i > (MODEL_HZ - 1):
                            telemrxlog.writerows((csvdata1[k,:] for k in range(MODEL_HZ))) #write array into CSV 
                            i = (i - MODEL_HZ) if (i - MODEL_HZ) > 0 else 0
                            csvdata1 = csvdata2
                            csvdata2 = np.empty((MODEL_HZ, TELEM_RX_LEN)) #empty backup array 
                    except:
                        pass

    def write_telemtxlog(self, log_dir, act2csv_out, event_start, event_end):
        telemtxpath = os.path.join(log_dir, self.telemtxfn)
        with open(telemtxpath, 'a+', newline='') as csvfile:
            telemtxlog = csv.writer(csvfile, delimiter=' ') #CSV writer object
            csvdata = np.zeros((ACT_HZ, TELEM_TX_LEN + 1)) #array for storing data frames
            i = 0
            event_start.wait() #wait for simulation start
            while True:
                if event_end.is_set():
                    #Close pipe
                    act2csv_out.close()
                    break
                else:
                    actdata = act2csv_out.recv()
                    csvdata[i,:] = actdata
                    i += 1
                    if i > (ACT_HZ - 1):
                        telemtxlog.writerows((csvdata[k,:] for k in range(ACT_HZ))) #write array into CSV 
                        i = 0
