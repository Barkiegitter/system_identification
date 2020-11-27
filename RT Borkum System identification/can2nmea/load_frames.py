import pickle
import fire
import os
import glob
import time

# from log_frames import CANTimedFrame
file_name = './logged_frames/218050608_1.pkl'

class CANTimedFrame:
    """ Wraps a can frame and assigns attributes based on """
    
    def __init__(self, can_frame):
        self.can_frame = can_frame
        self.time = time.time()

    def __str__(self):
        return str(self.time) + str(self.can_frame)
        # return f"CAN {self.name}, content = {pretty_hex(bytes(self.can_frame.data))}; {values_str}"

def load_frames(pkl_glob, frame_id):

    frame_pkls = glob.glob(pkl_glob)

    # file_name = os.path.join(str(frame_dir), f"{frame_id}.pkl")
    
    print(file_name)
    pkl_frames = open(file_name, 'rb')

    for frame_pkl in frame_pkls:
        count = 0
        while True:
            try:
                frame = pickle.load(pkl_frames)
                print(frame)
            except EOFError as e:
                print("DONE!")
                break


if __name__ == '__main__':
    fire.Fire(load_frames)