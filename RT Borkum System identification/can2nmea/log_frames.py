import os
import time

from canard.hw import cantact
import struct
import pickle

# These are the IDs for the Borkum that indicate the current setting of the joystick
port_az_setting_id = 218044192
port_az_current_id = 218050608
port_rpm_setting_id = 218051120
port_rpm_current_id = 218050864  

center_az_setting_id = 234821408
center_az_current_id = 234827824
center_rpm_setting_id = 234828336
center_rpm_current_id = 234828080    #234831664


starboard_az_setting_id = 251598624
starboard_az_current_id = 251605040
starboard_rpm_setting_id = 251605552
starboard_rpm_current_id = 251605296

t_reg = 2.
# Change these to get the IDs of interest, or set to
#all_ids = [port_az_id, center_az_id, starboard_az_id, port_rpm_id, center_rpm_id, starboard_rpm_id]

all_ids = [port_az_setting_id, port_az_current_id, port_rpm_setting_id, port_rpm_current_id,
           center_az_setting_id, center_az_current_id, center_rpm_setting_id, center_rpm_current_id,
           starboard_az_setting_id, starboard_az_current_id, starboard_rpm_setting_id, starboard_rpm_current_id]

SERIAL_PORT="/dev/tty.usbmodem1411"
BITRATE = 500000

class CANTimedFrame:
    """ Wraps a can frame and assigns attributes based on """
    
    def __init__(self, can_frame):
        self.can_frame = can_frame
        self.time = time.time()

    def __str__(self):
        return str(self.time) + str(self.can_frame)
        # return f"CAN {self.name}, content = {pretty_hex(bytes(self.can_frame.data))}; {values_str}"


dev = cantact.CantactDev(SERIAL_PORT)  # Connect to CANable that enumerated as ttyACM0
dev.set_bitrate(BITRATE)  # Set the bitrate to a 1Mbaud
dev.start() # Go on the bus

frame_dir = f'logged_frames_borkum_09_12_model_az/'
#os.makedirs(frame_dir, exist_ok=True)

for id_ in all_ids:
    os.makedirs(os.path.join(frame_dir, str(id_)), exist_ok=True)

while True:
    frame = dev.recv()
    if frame.id in all_ids:
        cf = CANTimedFrame(frame)
        # print('frame binnen')
        if time.time()-t_reg>2.0:
            print('frame binnen')
            # if frame.id==starboard_az_current_id or frame.id==port_az_current_id:
                # print(struct.unpack("H", bytes(frame.data)[2:4])/600.)
            t_reg = time.time()
#        file_name = os.path.join(frame_dir, str(frame.id), f"{time.time()}.pkl")
        
        with open(os.path.join(frame_dir, f"{frame.id}.pkl"), 'ab+') as fp:
            pickle.dump(cf, fp)

#centre
#            12:44 even chillen
#starboard
#            12:48
#ps
# def main():
#     pass

# if __name__ == "__main__":
#    # stuff only to run when not called via 'import' here
#    main()