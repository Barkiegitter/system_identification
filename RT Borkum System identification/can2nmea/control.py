import struct
import time
from canard import can
from canard.hw import cantact

dev = cantact.CantactDev("/dev/cu.usbmodem141101")  # Connect to CANable that enumerated as ttyACM0

dev.set_bitrate(500000)  # Set the bitrate to a 1Mbaud
dev.start() # Go on the bus

def create_az(az):
    struct.pack("HH")

    # 00 00 f1 ff 20 ff 64 8c

while True:
    bs = bytes(bytearray.fromhex('0000f1ff20ff648c'))
    ints = [int(b) for b in bs]
    frame = can.Frame(251598624, dlc=8, data=ints, is_extended_id=True)
    dev.send(frame)  # Echo the CAN frame back out on the bus

    print("Sending frame ", frame)

    time.sleep(0.1)

