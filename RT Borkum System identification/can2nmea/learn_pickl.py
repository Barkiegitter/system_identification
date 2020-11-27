#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Aug 31 12:04:43 2020

@author: erwinlodder
"""
import pickle
import time
import struct
import matplotlib.pyplot as plt
import datetime
class CANTimedFrame:
    """ Wraps a can frame and assigns attributes based on """
    
    def __init__(self, can_frame):
        self.can_frame = can_frame
        self.time = time.time()

    def __str__(self):
        return str(self.time) + str(self.can_frame)
        # return f"CAN {self.name}, content = {pretty_hex(bytes(self.can_frame.data))}; {values_str}"

# target = './logged_frames_borkum/234827824.pkl'

# import os
# # scores = {} # scores is an empty dict already
# start = time.time()
# if os.path.getsize(target) > 0:      
#     with open(target, "rb") as f:
#         unpickled = []
#         while True:
#             try:
#                 (az_current, ) = struct.unpack("H", bytes(pickle.load(f).can_frame.data)[2:4])
#                 (az_setting, ) = struct.unpack("H", bytes(pickle.load(f).can_frame.data)[:2])
#                 # (az, ) = struct.unpack("H", bytes(pickle.load(f).can_frame.data)[:2])

#                 # unpickled.append([datetime.datetime.fromtimestamp(pickle.load(f).time), pickle.load(f).can_frame.data])
#                 unpickled.append([datetime.datetime.fromtimestamp(pickle.load(f).time), az_current, az_setting])
                
#             except EOFError:
#                 break
#         end_time = time.time()

# # plt.plot([i[0] for i in unpickled] ,[i[6] for i in [k[1] for k in unpickled]])
# figsize = (15, 9 * 9 / 16)
# fig, ax1 = plt.subplots(figsize=figsize)
# # ax1.plot([i[0] for i in unpickled], [i[1] for i in unpickled])
# ax1.plot([i[0] for i in unpickled], [i[2] for i in unpickled])
# plt.savefig('azblabla.png')

#rpm
target = './logged_frames_borkum/251605296.pkl'

import os
# scores = {} # scores is an empty dict already
start = time.time()
if os.path.getsize(target) > 0:      
    with open(target, "rb") as f:
        unpickled = []
        while True:
            try:
                (rpm_current, ) = struct.unpack("H", bytes(pickle.load(f).can_frame.data)[2:4])
                (rpm_setting, ) = struct.unpack("H", bytes(pickle.load(f).can_frame.data)[:2])
                # (az, ) = struct.unpack("H", bytes(pickle.load(f).can_frame.data)[:2])

                # unpickled.append([datetime.datetime.fromtimestamp(pickle.load(f).time), pickle.load(f).can_frame.data])
                unpickled.append([datetime.datetime.fromtimestamp(pickle.load(f).time), rpm_current, rpm_setting])
                
            except EOFError:
                break
        end_time = time.time()

# unpickled = unpickled[:400]
figsize = (15, 9 * 9 / 16)
fig, ax1 = plt.subplots(figsize=figsize)
ax1.plot([i[0] for i in unpickled], [i[1] for i in unpickled])
ax1.plot([i[0] for i in unpickled], [i[2] for i in unpickled])
# plt.plot([i[0] for i in unpickled] ,[i[5] for i in [k[1] for k in unpickled]])

# plt.show()
plt.savefig('rpmblabla.png')






