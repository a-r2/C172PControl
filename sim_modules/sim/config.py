import socket

from settings import *

''' FLIGHTGEAR CONFIGURATION MODULE '''
class FGConfig():

    def __init__(self):
        self.IP_ADDRESS = CFG_IP_ADDRESS
        self.PORT       = CFG_PORT

    def transmit(self, event_rxtcp):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM, socket.IPPROTO_TCP) #TCP socket
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) #set socket reusability
        cfgdata = str(SIM_RATE) + '\t' + str(FPS) + '\t' + str(MAX_TIME_PER_FRAME) + '\n'
        event_rxtcp.wait() #wait for RX TCP connection event
        print('Configuring FlightGear...')
        sock.connect((self.IP_ADDRESS, self.PORT)) #outgoing TCP connection socket
        sock.sendall(cfgdata.encode()) #send configuration data
        print('FlightGear configured!')
        sock.shutdown(socket.SHUT_RDWR) #shutdown socket
        sock.close() #close socket
