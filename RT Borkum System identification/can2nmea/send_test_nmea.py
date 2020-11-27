import time

from cap_connections.nmea.control import NMEAControls

ctrl = NMEAControls("localhost", 7778, verbose=True)

while True:
    ctrl.TRC(0)
    time.sleep(0.1)
