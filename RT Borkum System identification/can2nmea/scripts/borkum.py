import time
import struct
from canard import can
from canard.hw import cantact

from threading import Thread
from util import pretty_hex


class CANMessage:
    """ Wraps a can frame and assigns attributes based on """
    
    def __init__(self, name, can_frame, value_names, values):
        if type(values) != list:
            values = [values]
        if type(value_names) != list:
            value_names = [value_names]

        self.name = name
        self.can_frame = can_frame
        self.value_names = value_names
        self.values = values
        self.time = time.time()

    def __str__(self):
        # print("Hello")
        values_str = [f"{x}={y}" for x,y in zip(self.value_names, self.values)]
        return f"CAN {self.name}, frame = {self.can_frame}; {values_str}"
        # return f"CAN {self.name}, content = {pretty_hex(bytes(self.can_frame.data))}; {values_str}"

# class CANValue:

#     def __init__(self, can_id)

port_az_id = 218044192
center_az_id = 234821408
starboard_az_id = 251598624

port_rpm_id = 218051120
center_rpm_id = 234828336
starboard_rpm_id = 251605552 

all_ids = [port_az_id, center_az_id, starboard_az_id, port_rpm_id, center_rpm_id, starboard_az_id]

can_messages = dict()

dev = cantact.CantactDev("/dev/cu.usbmodem141101")  # Connect to CANable that enumerated as ttyACM0
dev.set_bitrate(500000)  # Set the bitrate to a 1Mbaud
dev.start()  # Go on the bus
count = 0

# port_az, stbd_az, center_az, port_speed, stbd_speed, center_speed = None

azimuth = 0

def process_can_data(data):
    hx = pretty_hex(msg)
    (az,) = struct.unpack("H", msg[-2:])

    alarm_info = msg[2]
    status_info = msg[3]

    return az, alarm_info, status_info

def decode_az_data(msg):
    """
    Azimuth: 0...3599, 0 in ahead position, CW direction increased the output value.
    """
    msg = bytes(msg)
    (az,) = struct.unpack("H", msg[-2:])

    alarm_info = msg[2]
    status_info = msg[3]

    return az, alarm_info, status_info

def decode_thruster_speed(msg):

    msg = bytes(msg)
    hx = pretty_hex(msg)

    (speed, ) = struct.unpack("H", msg[0:2])
    alarm_info = msg[3]
    status_info = msg[4]

    return speed, alarm_info, status_info

# def can_msg_report(can_name, can_frame, value_name, value):
#     print(can_name, "\t", pretty_hex(bytes(can_frame.data)), "Raw Az = ", value)

# def can_msg_report(can_msg):
    # print
    # print(can_msg.name, "\t", pretty_hex(bytes(can_msg.frame.data)), can_msg.value_name, value)

def process_can(can_frame):
    global port_az, stbd_az, center_az, port_speed, stbd_speed, center_speed

    if frame.id == port_az_id:
        # print("Port Az Message Found")
        az, ai, si = decode_az_data(can_frame.data)
        can_messages[frame.id] = CANMessage("Port Az", frame, "az", az)

        # can_msg_report("Port Azimuth", frame, "az", az)
    elif frame.id == center_az_id:
        az, ai, si = decode_az_data(can_frame.data)
        # can_msg_report("Center Azimuth", frame, "az", az)
        can_messages[frame.id] = CANMessage("Center Az", frame, "az", az)
    elif frame.id == starboard_az_id:
        az, ai, si = decode_az_data(can_frame.data)
        # can_msg_report("Stbd Azimuth", frame, "az", az)
        can_messages[frame.id] = CANMessage("Stbd Az", frame, "az", az)
    elif frame.id == port_rpm_id:
        speed, ai, si = decode_thruster_speed(can_frame.data)
        # print(f"Port Speed Message {pretty_hex(can_frame.data)}; Raw Speed = {speed}")
        # can_msg_report("Port Speed", can_frame, "speed", speed)
        can_messages[frame.id] = CANMessage("Port Spd", frame, "speed", speed)
    elif frame.id == center_rpm_id:
        speed, ai, si = decode_thruster_speed(can_frame.data)
        # can_msg_report("Center Speed", can_frame, "speed", speed)
        can_messages[frame.id] = CANMessage("Center Spd", frame, "speed", speed)
    elif frame.id == starboard_rpm_id:
        speed, ai, si = decode_thruster_speed(can_frame.data)
        can_messages[frame.id] = CANMessage("Stbd Spd", frame, "speed", speed)
        
    # else:
        # print(type(can_frame.data))
        # print(bytes(can_frame.data))
        # print("Untracked CAN message id = ", can_frame.id, " content : ", pretty_hex(bytes(can_frame.data)))
        # return
    # print("")

def report_values():

    def __report():
        global can_messages

        while True:
            for can_msg in can_messages.values():
                print(can_msg)
            time.sleep(1)


    t = Thread(target=__report)
    t.start()

# Init can_messages
# for can_id in all_ids:
#     can_messages[can_id] = None
report_values()

while True:
    count += 1
    frame = dev.recv()  # Receive a CAN frame

    

    process_can(frame)





