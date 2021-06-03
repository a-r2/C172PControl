import csv
import multiprocessing as mp
import numpy as np

from actuation import *
from dynamics import *
from settings import *

class CSVTelemetryLog():
    def __init__(self, name, **kwargs):
        for key, value in kwargs.items():
            if key == 'rx2csv_out':
                rx2csv_out = value
            elif key == 'act2csv_out':
                act2csv_out = value
            elif key == 'event_start':
                event_start = value
        self.rxname = 'rx_' + name + '.csv'
        self.txname = 'tx_' + name + '.csv'
        if len(kwargs) > 0:
            self.rxproc = mp.Process(target=self.write_rx_log, args=(rx2csv_out, event_start), daemon=True) #process for logging RX telemetry
            self.rxproc.start()
            self.txproc = mp.Process(target=self.write_tx_log, args=(act2csv_out, event_start), daemon=True) #process for logging TX telemetry
            self.txproc.start()
    def read_rx_log(self):
        with open(self.rxname, 'r', newline='') as csvfile:
            rxtelemlog = csv.reader(csvfile, delimiter=' ') #CSV reader object
            rowscount = len(list(rxtelemlog))
            csvfile.seek(0) #go to first row
            rxdata = np.zeros((rowscount, TELEM_RX_LEN))
            i = 0
            for row in rxtelemlog:
                rxdata[i,:] = row
                i += 1
            return rxdata
    def read_tx_log(self):
        with open(self.txname, 'r', newline='') as csvfile:
            txtelemlog = csv.reader(csvfile, delimiter=' ') #CSV reader object
            rowscount = len(list(txtelemlog))
            csvfile.seek(0) #go to first row
            txdata = np.zeros((rowscount, TELEM_TX_LEN + 1))
            i = 0
            for row in txtelemlog:
                txdata[i,:] = row
                i += 1
            return txdata
    def write_rx_log(self, rx2csv_out, event_start):
        with open(self.rxname, 'w', newline='') as csvfile:
            rxtelemlog = csv.writer(csvfile, delimiter=' ') #CSV writer object
            csvdata1 = np.zeros((MODEL_HZ, TELEM_RX_LEN)) #array for storing data frames
            csvdata2 = np.zeros((MODEL_HZ, TELEM_RX_LEN)) #backup array for storing overflowed data frames
            i = 0
            event_start.wait() #wait for simulation start
            while True:
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
                        rxtelemlog.writerows((csvdata1[k,:] for k in range(MODEL_HZ))) #write array into CSV 
                        i = (i - MODEL_HZ) if (i - MODEL_HZ) > 0 else 0
                        csvdata1 = csvdata2
                        csvdata2 = np.empty((MODEL_HZ, TELEM_RX_LEN)) #empty backup array 
                except:
                    pass
    def write_tx_log(self, act2csv_out, event_start):
        with open(self.txname, 'w', newline='') as csvfile:
            txtelemlog = csv.writer(csvfile, delimiter=' ') #CSV writer object
            csvdata = np.zeros((ACT_HZ, TELEM_TX_LEN + 1)) #array for storing data frames
            i = 0
            event_start.wait() #wait for simulation start
            while True:
                actdata = act2csv_out.recv()
                csvdata[i,:] = actdata
                i += 1
                if i > (ACT_HZ - 1):
                    txtelemlog.writerows((csvdata[k,:] for k in range(ACT_HZ))) #write array into CSV 
                    i = 0

class CSVDynamicsLog():
    def __init__(self, name, **kwargs):
        for key, value in kwargs.items():
            if key == 'dyn2csv_out':
                dyn2csv_out = value
            elif key == 'event_start':
                event_start = value
        self.name = name + '.csv'
        if len(kwargs) > 0:
            self.proc = mp.Process(target=self.write_log, args=(dyn2csv_out, event_start), daemon=True) #process for logging dynamics
            self.proc.start()
    def read_log(self):
        with open(self.name, 'r', newline='') as csvfile:
            dynlog = csv.reader(csvfile, delimiter=' ') #CSV reader object
            rowscount = len(list(dynlog))
            csvfile.seek(0) #go to first row
            dyndata = np.zeros((rowscount, DYN_LEN + 1))
            i = 0
            for row in dynlog:
                dyndata[i,:] = row
                i += 1
            return dyndata
    def write_log(self, dyn2csv_out, event_start):
        with open(self.name, 'w', newline='') as csvfile:
            dynlog = csv.writer(csvfile, delimiter=' ') #CSV writer object
            csvdata1 = np.zeros((MODEL_HZ, DYN_LEN + 1)) #array for storing data frames
            csvdata2 = np.zeros((MODEL_HZ, DYN_LEN + 1)) #backup array for storing overflowed data frames
            i = 0
            event_start.wait() #wait for simulation start
            while True:
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

class CSVKinematicsLog():
    def __init__(self, name, **kwargs):
        for key, value in kwargs.items():
            if key == 'kin2csv_out':
                kin2csv_out = value
            elif key == 'event_start':
                event_start = value
        self.name = name + '.csv'
        if len(kwargs) > 0:
            self.proc = mp.Process(target=self.write_log, args=(kin2csv_out, event_start), daemon=True) #process for logging kinematics
            self.proc.start()
    def read_log(self):
        with open(self.name, 'r', newline='') as csvfile:
            kinlog = csv.reader(csvfile, delimiter=' ') #CSV reader object
            rowscount = len(list(kinlog))
            csvfile.seek(0) #go to first row
            kindata = np.zeros((rowscount, KIN_LEN + 1))
            i = 0
            for row in kinlog:
                kindata[i,:] = row
                i += 1
            return kindata
    def write_log(self, kin2csv_out, event_start):
        event_start.wait() #wait for simulation start
        with open(self.name, 'w', newline='') as csvfile:
            kinlog = csv.writer(csvfile, delimiter=' ') #CSV writer object
            csvdata = np.zeros((MODEL_HZ, KIN_LEN)) #array for storing data frames
            i = 0
            event_start.wait() #wait for simulation start
            while True:
                kindata = kin2csv_out.recv()
                csvdata[i,:] = kindata 
                i += 1
                if i > (MODEL_HZ - 1):
                    kinlog.writerows((csvdata[k,:] for k in range(MODEL_HZ))) #write array into CSV 
                    i = 0
