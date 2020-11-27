import struct
import time
from canard import can
from canard.hw import cantact

from threading import Thread

dev = cantact.CantactDev("/dev/cu.usbmodem141101")  # Connect to CANable that enumerated as ttyACM0
dev.set_bitrate(500000)  # Set the bitrate to a 1Mbaud
dev.start() # Go on the bus

# from_id = 218044192 # Starboard az
from_id = 218051120

# to_id = 234821408 # Center az

# to_id = 251598624 
to_id = 251605552 

def frame_sender(frame):

    def __sender():
        print("send frame = ")
        print(frame)
        while True:
            print(".", end='')

            dev.send(frame)  # Echo the CAN frame back out on the bus
            time.sleep(0.1)
            # break

    t = Thread(target=(__sender))
    t.start()

while True:
    frame = dev.recv()  # Receive a CAN frame

    if frame.id == from_id:
        print("FOUND!", frame.id)
        print(str(frame))  # Print out the received frame

        frame.id = to_id
        frame_sender(frame)
        break