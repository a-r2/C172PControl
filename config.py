import multiprocessing as mp
import socket

from settings import *

class Config():

    def __init__(self):
        self.IP_ADDRESS = CFG_IP_ADDRESS
        self.PORT       = CFG_PORT
        self.sock       = socket.socket(socket.AF_INET, socket.SOCK_STREAM, socket.IPPROTO_TCP) #TCP socket
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) #set socket reusability

    def transmit(self, event_rxtcp):
        try:
            event_rxtcp.wait() #wait for RX TCP connection event
            print("Configuring FlightGear...")
            self.sock.connect((self.IP_ADDRESS, self.PORT)) #outgoing TCP connection socket
            cfgdata = str(SIM_RATE) + '\t' + str(FPS) + '\t' + str(MAX_TIME_PER_FRAME) + '\n'
            self.sock.sendall(cfgdata.encode()) #sending configuration data
            print("FlightGear configured!")
        except:
            pass
