import multiprocessing as mp
import numpy as np
import socket

from constants import *
from settings import *

class Telemetry():
    def __init__(self, rx_ip_address, rx_port, tx_ip_address, tx_port, act2tx_out, rx2act_in, rx2csv_in, rx2dyn_in, rx2eq_in, rx2mod_in, rx2sup_in, event_rxtcp, event_txtcp, event_start):
        self.RX_IP_ADDRESS = rx_ip_address
        self.RX_PORT = rx_port
        self.TX_IP_ADDRESS = tx_ip_address
        self.TX_PORT = tx_port
        self.rxsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM, socket.IPPROTO_TCP) #TCP RX socket
        self.rxsock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) #set RX socket reusability
        self.txsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM, socket.IPPROTO_TCP) #TCP TX socket
        self.txsock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) #set TX socket reusability
        self.rxproc = mp.Process(target=self.receive, args=(rx2act_in, rx2csv_in, rx2dyn_in, rx2eq_in, rx2mod_in, rx2sup_in, event_rxtcp, event_start), daemon=True) #process for receiving RX telemetry
        self.rxproc.start()
        self.txproc = mp.Process(target=self.transmit, args=(act2tx_out, event_rxtcp, event_txtcp, event_start), daemon=True) #process for transmitting TX telemetry
        self.txproc.start()
    def receive(self, rx2act_in, rx2csv_in, rx2dyn_in, rx2eq_in, rx2mod_in, rx2sup_in, event_rxtcp, event_start):
        self.rxsock.bind((self.RX_IP_ADDRESS, self.RX_PORT)) #bind TCP RX socket to ip and port
        self.rxsock.listen(1) #listen to flighgear TCP request
        print("Waiting for RX link with FlightGear...")
        conn, _ = self.rxsock.accept() #incoming TCP connection
        print("RX link established!")
        event_rxtcp.set() #set RX TCP connection event
        dataframe = np.zeros(TELEM_RX_LEN) #array for storing one data frame
        framesarray = np.zeros((MODEL_HZ, TELEM_RX_LEN)) #array for storing data frames
        #Connecting loop
        while True:
            try:
                tcpdata = conn.recv(TELEM_RX_BUFFER_SIZE) #TCP data buffer
                tcpdata = tcpdata.decode() #decode TCP data
                tcpframes = tcpdata.split('\n') #split TCP data into frames 
                tcpframes = tcpframes[:-1] #remove empty last string
                framescount = len(tcpframes) #count received frames
                for i in range(framescount):
                    dataframe[:] = tcpframes[i].split('\t') #split frames data 
                    framesarray[i,:] = tuple(map(float,dataframe)) #store data tuple in array
                framesarray = framesarray[:framescount,:]
                if (framescount > 0) and (np.any(framesarray[:,0] >= TELEM_WAIT)): #if at least one frame was received and simulation time greater than TELEM_WAIT
                    i = np.where(framesarray[:,0] >= TELEM_WAIT)[0][0] #find first frame index
                    framesarray = framesarray[i:,:] #remove earlier frames
                    rx2act_in.send(framesarray) #send RX telemetry to calculate actuation 
                    rx2csv_in.send(framesarray) #send RX telemetry to store in CSV
                    rx2dyn_in.send(framesarray) #send RX telemetry to calculate dynamics
                    rx2eq_in.send(framesarray) #send RX telemetry to calculate equilibrium point 
                    rx2mod_in.send(framesarray) #send RX telemetry to calculate control model 
                    rx2sup_in.send(framesarray) #send RX telemetry to calculate supervisor 
                    event_start.set() #set simulation start event
                    framesarray = np.empty((MODEL_HZ, TELEM_RX_LEN)) #empty data frames array
                    break
                else:
                    framesarray = np.empty((MODEL_HZ, TELEM_RX_LEN)) #empty data frames array
            except:
                pass
        #Connected loop
        while True:
            try:
                tcpdata = conn.recv(TELEM_RX_BUFFER_SIZE) #TCP data buffer
                tcpdata = tcpdata.decode() #decode TCP data
                tcpframes = tcpdata.split('\n') #split TCP data into frames 
                tcpframes = tcpframes[:-1] #remove empty last string
                framescount = len(tcpframes) #count received frames
                for i in range(framescount):
                    dataframe = tcpframes[i].split('\t') #split frames data 
                    framesarray[i,:] = tuple(map(float,dataframe)) #store data tuple in array
                framesarray = framesarray[:framescount,:]
                if (framescount > 0): #if at least one frame was received
                    rx2act_in.send(framesarray) #send RX telemetry to calculate actuation 
                    rx2csv_in.send(framesarray) #send RX telemetry to store in CSV
                    rx2dyn_in.send(framesarray) #send RX telemetry to calculate dynamics
                    rx2eq_in.send(framesarray) #send RX telemetry to calculate equilibrium point 
                    rx2mod_in.send(framesarray) #send RX telemetry to calculate control model 
                    rx2sup_in.send(framesarray) #send RX telemetry to calculate supervisor 
                    framesarray = np.empty((MODEL_HZ, TELEM_RX_LEN)) #empty data frames array
                else:
                    framesarray = np.empty((MODEL_HZ, TELEM_RX_LEN)) #empty data frames array
            except:
                pass
    def transmit(self, act2tx_out, event_rxtcp, event_txtcp, event_start):
        event_rxtcp.wait() #wait for RX TCP connection event
        print("Waiting for TX link with FlightGear...")
        self.txsock.connect((self.TX_IP_ADDRESS, self.TX_PORT)) #outgoing TCP connection
        print("TX link established!")
        event_txtcp.set() #wait for TX TCP connection event
        event_start.wait() #wait for simulation start event
        while True:
            try:
                txdata = act2tx_out.recv()
                self.txsock.sendall(txdata.encode()) #sending TX telemetry data
            except:
                pass
