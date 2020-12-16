import numpy as np
import pandas as pd
import os
import time
import struct
import pickle
import matplotlib.pyplot as plt
import datetime


class CANTimedFrame:
    """ Wraps a can frame and assigns attributes based on """
    
    def __init__(self, can_frame):
        self.can_frame = can_frame
        self.time = time.time()

    def __str__(self):
        return str(self.time) + str(self.can_frame)
    
centre_az_current_id = 234827824  

centre_rpm_current_id = 234828080       #( or 234831664 —> 234827824 )

ps_az_current_id = 218050608     

ps_rpm_current_id = 218050864       

sb_az_current_id = 251605040
 
sb_rpm_current_id = 251605296   

components_lst = [centre_az_current_id, centre_rpm_current_id, ps_az_current_id, ps_rpm_current_id, sb_az_current_id, sb_rpm_current_id ]
components_lst_id = ['rsa_0', 'rpm_0', 'rsa_2', 'rpm_2', 'rsa_1', 'rpm_1']

# target = './nmea_encoding/logged_frames_borkum/251605296.pkl'
df = pd.DataFrame([])

import os
# scores = {} # scores is an empty dict already
start = time.time()
for component in components_lst:
    target = './can2nmea/logged_frames_borkum_09_12/' + str(component) + '.pkl'
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
    # ax1.plot([i[0] for i in unpickled], [i[2] for i in unpickled])
    plt.show()
    plt.close()
    df_temp = pd.DataFrame(list(zip([i[0] for i in unpickled], [i[1] for i in unpickled])), 
                columns =[components_lst_id[components_lst.index(component)]+'_time',components_lst_id[components_lst.index(component)] ])
    df = pd.concat([df, df_temp], axis = 1)

# df = df[::6]
#%%
    
df_gps = pd.read_csv('nmea_1607507642.csv')

df_gps.timestamp = df_gps.timestamp.str.replace('T', ' ', regex=True)

df_gps['time'] = pd.to_datetime(df_gps.timestamp, format='%Y-%m-%d %H:%M:%S.%f')

df_gps = df_gps[df_gps.payload.str[:6]=='$GPRMC']
#%%
# df_gps = df_gps[:1000]

df_gps['lat'] = df_gps.payload.str[16:25].astype(float)/100.
df_gps['lon'] = df_gps.payload.str[28:37].astype(float)/100.

df_gps = df_gps.reset_index(drop=True)

df_hdg = pd.read_csv('nmea_1607507642.csv')

df_hdg.timestamp = df_hdg.timestamp.str.replace('T', ' ', regex=True)

df_hdg['time'] = pd.to_datetime(df_hdg.timestamp, format='%Y-%m-%d %H:%M:%S.%f')

df_hdg = df_hdg[df_hdg.payload.str[:6]=='$GPHDT']

# df_hdg = df_hdg[:1000]

df_hdg['hdg'] = df_hdg.payload.str[7:12].astype(float)
df_hdg = df_hdg.reset_index(drop=True)

#%%
integer_loop = 1
for i in df_gps[1:].index:
    for k in df_hdg[integer_loop:].index:
        
        if df_hdg.loc[k,'time']>df_gps.loc[i,'time']:
            # print(k)
            if abs(df_hdg.loc[k, 'hdg'] - df_hdg.loc[k - 1, 'hdg']) > 300.0:
                if df_hdg.loc[k, 'hdg'] > df_hdg.loc[k - 1, 'hdg']:
                    delta_hdg = df_hdg.loc[k, 'hdg'] - 360 - df_hdg.loc[k - 1, 'hdg']
                else:
                    delta_hdg = df_hdg.loc[k, 'hdg'] + 360 - df_hdg.loc[k - 1, 'hdg']
            else:
                delta_hdg = df_hdg.loc[k, 'hdg'] - df_hdg.loc[k - 1, 'hdg']
            # print(k)
            hdg_inp = df_hdg.loc[k, 'hdg'] - \
                      ((delta_hdg)/((df_hdg.loc[k, 'time'] - df_hdg.loc[k-1, 'time']).total_seconds())) * \
                      ((df_hdg.loc[k, 'time']-df_gps.loc[i, 'time']).total_seconds())
            df_gps.loc[i, 'hdg'] = hdg_inp
            if hdg_inp > 360.0:
                df_gps.loc[i, 'hdg'] = hdg_inp - 360  #
            elif hdg_inp < 0.0:
                df_gps.loc[i, 'hdg'] = hdg_inp + 360
            integer_loop = k - 1
            break
        else:
            continue


df_gps = df_gps[df_gps.time>df.loc[df.index[0],'rsa_0_time']]

for component in components_lst_id:
    df_gps[component] = 0.0
    integer_loop=1
    for i in df_gps[integer_loop:].index:
        for k in df[[component+'_time', component]][integer_loop:].index:
            if df[[component+'_time', component]].loc[k,component+'_time']>df_gps.loc[i,'time']:
                input_ = df[[component+'_time', component]].loc[k, component] - \
                          ((df[[component+'_time', component]].loc[k, component] - df[[component+'_time', component]].loc[k-1, component])/((df[[component+'_time', component]].loc[k, component+'_time'] - df[[component+'_time', component]].loc[k-1, component+'_time']).total_seconds())) * \
                          ((df[[component+'_time', component]].loc[k, component+'_time']-df_gps.loc[i, 'time']).total_seconds())
                df_gps.loc[i, component] = input_
                
                integer_loop = k - 1
                break
            else:
                None



df_gps.rsa_0 = df_gps.rsa_0/100 + 180

df_gps.rsa_1 = df_gps.rsa_1/100 + 180
df_gps.rsa_2 = df_gps.rsa_2/100 + 180


df_gps.to_csv('RTBorkum.csv', index=False)




























