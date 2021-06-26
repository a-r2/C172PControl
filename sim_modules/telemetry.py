import numpy as np
import socket

from constants import *
from settings import *

class Telemetry():
    def __init__(self):
        self.RX_IP_ADDRESS = TELEM_RX_IP_ADDRESS
        self.RX_PORT       = TELEM_RX_PORT
        self.TX_IP_ADDRESS = TELEM_TX_IP_ADDRESS
        self.TX_PORT       = TELEM_TX_PORT
    def receive(self, rx2act_in, rx2csv_in, rx2dyn_in, rx2eq_in, rx2cm_in, rx2sup_in, event_rxtcp, event_start, event_end):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM, socket.IPPROTO_TCP) #TCP RX socket
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) #set RX socket reusability
        sock.bind((self.RX_IP_ADDRESS, self.RX_PORT)) #bind TCP RX socket to ip and port
        sock.listen(1) #listen to flighgear TCP request
        print('Waiting for RX link with FlightGear...')
        conn, _ = sock.accept() #incoming TCP connection
        print('RX link established!')
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
                    rx2cm_in.send(framesarray) #send RX telemetry to calculate control model 
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
            if event_end.is_set():
                #Close pipes
                rx2act_in.close()
                rx2csv_in.close()
                rx2dyn_in.close()
                rx2eq_in.close()
                rx2cm_in.close()
                rx2sup_in.close()
                #Close socket
                sock.shutdown(socket.SHUT_RDWR)
                sock.close()
                break
            else:
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
                        rx2cm_in.send(framesarray) #send RX telemetry to calculate control model 
                        rx2sup_in.send(framesarray) #send RX telemetry to calculate supervisor 
                        framesarray = np.empty((MODEL_HZ, TELEM_RX_LEN)) #empty data frames array
                    else:
                        framesarray = np.empty((MODEL_HZ, TELEM_RX_LEN)) #empty data frames array
                except:
                    pass

    def transmit(self, act2tx_out, event_rxtcp, event_txtcp, event_start, event_end):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM, socket.IPPROTO_TCP) #TCP TX socket
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) #set TX socket reusability
        event_rxtcp.wait() #wait for RX TCP connection event
        print('Waiting for TX link with FlightGear...')
        sock.connect((self.TX_IP_ADDRESS, self.TX_PORT)) #outgoing TCP connection
        print('TX link established!')
        event_txtcp.set() #wait for TX TCP connection event
        event_start.wait() #wait for simulation start event
        while True:
            if event_end.is_set():
                #Close pipe
                act2tx_out.close()
                #Close socket
                sock.shutdown(socket.SHUT_RDWR)
                sock.close()
                break
            else:
                try:
                    txdata = act2tx_out.recv()
                    sock.sendall(txdata.encode()) #sending TX telemetry data
                except:
                    pass
