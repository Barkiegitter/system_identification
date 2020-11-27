import struct
from util import pretty_hex, chunks

def decode_thruster_az_spec(data, msg_size):

    print(" ----------- Thruster Azimuth Messages Spec ----------- ")
    for i, msg in enumerate(chunks(data, msg_size)):
        hx = pretty_hex(msg)

        (az,) = struct.unpack("H", msg[0:2])
        alarm_info = msg[2]
        status_info = msg[3]

        print(f"Thruster Azimuth Message #{i} :\t {hx} --> {az},{alarm_info},{status_info}")

def decode_thruster_speed_spec(data, msg_size):

    print(" ----------- Thruster Speed Messages Spec ----------- ")
    for i, msg in enumerate(chunks(data, msg_size)):
        hx = pretty_hex(msg)

        (speed, ) = struct.unpack("H", msg[0:2])
        alarm_info = msg[2]
        status_info = msg[3]

        print(f"Thruster Speed Message #{i} :\t {hx} --> {speed},{alarm_info},{status_info}")
    # print(f"Min, max az = ({min_az}, {max_az} ")

if __name__ == '__main__':
    
    # Read bin file content
    port_azimuth_file = 'data/218044192.bin'
    with open(port_azimuth_file, 'rb') as fp:
        # decode_thruster_az(fp.read(), 8)
        decode_thruster_az_spec(fp.read(), 8)
    
    port_speed_file = 'data/218051120.bin'
    with open(port_speed_file, 'rb') as fp:
        # decode_thruster_speed(fp.read(), 8)
        decode_thruster_speed_spec(fp.read(), 8)


