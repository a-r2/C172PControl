import control
import math
import numpy as np

def updfcn(t,x,u,params={}):
    return [math.sin(x[1]), (x[0] ** 4) * math.cos(x[1]) + u[0]]

def outfcn(t,x,u,params={}):
    return x[0]

sys = control.NonlinearIOSystem(updfcn, outfcn)
