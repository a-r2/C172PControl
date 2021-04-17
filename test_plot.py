import multiprocessing as mp
import numpy as np

from csv_logging import *
from plot_results import *

event = mp.Event()
pipe = mp.Pipe()

A = CSVTelemetryLog('telemetry_log')
rxdata = A.read_rx_log()
txdata = A.read_tx_log()
B = CSVDynamicsLog('dynamics_log')
dyndata = B.read_log()
plot(rxdata, txdata, dyndata)
