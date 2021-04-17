import multiprocessing as mp
import socket

from settings import *

class Config():
    def __init__(self, ip_address, port, event_rxtcp):
        self.IP_ADDRESS = ip_address
        self.PORT = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM, socket.IPPROTO_TCP) #TCP socket
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) #set socket reusability
        self.proc = mp.Process(target=self.transmit, args=(event_rxtcp,), daemon=True) #process for configuring FlightGear
        self.proc.start()
    def transmit(self, event_rxtcp):
        event_rxtcp.wait() #wait for RX TCP connection event
        print("Configuring FlightGear...")
        self.sock.connect((self.IP_ADDRESS, self.PORT)) #outgoing TCP connection socket
        print("FlightGear configured!")
        try:
            cfgdata = str(SIM_RATE) + '\t' + str(FPS) + '\t' + str(MAX_TIME_PER_FRAME) + '\n'
            self.sock.sendall(cfgdata.encode()) #sending configuration data
            self.proc.terminate() #terminate process
            self.__del__ #delete object
        except:
            pass
