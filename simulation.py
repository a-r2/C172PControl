import multiprocessing as mp
import numpy as np
import random

from actuation import *
from dynamics import *
#from kinematics import *
from settings import *

def calculate_dynamics(rxqueue, dynqueue, event_rx):
    print('11')
    event_rx.wait() #wait for received RX telemetry event
    print('12')
    event_rx.clear() #clear received RX telemetry event
    print('13')
    rxdata = rxqueue.get()
    print('14')
    #Faero = forces_wind(rxdata) #calculate aerodynamic forces
    #dynqueue.put(Faero)
    Faero_R = wind_to_body(rxdata[22], rxdata[24]).rotate(forces_wind(rxdata)) #calculate aerodynamic forces
    dynqueue.put(Faero_R)
    print('15')

def calculate_kinematics(dynqueue, kinqueue):
    print('21')
    dyndata = dynqueue.get()
    print('22')
    kindata = (0, 0, 0) 
    kinqueue.put(kindata)
    print('23')

def simulation_loop(rxqueue, dynqueue, kinqueue, txqueue, event_start, event_rx, event_log):
    event_start.wait() #wait for simulation start event
    while True:
        print(1)
        calculate_dynamics(rxqueue, dynqueue, event_rx)
        print(2)
        calculate_kinematics(dynqueue, kinqueue)
        print(3)
        calculate_actuation(kinqueue, txqueue)
        print(4)
        event_log.set() #set logging event
