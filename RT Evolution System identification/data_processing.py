import numpy as np
import pandas as pd
from numpy.linalg import inv
import csv
import time
import matplotlib.pyplot as plt
def haversine(lon1, lat1, lon2, lat2):
    """
    Calculate the great circle distance between two points
    on the earth (specified in decimal degrees)
    """
    # convert decimal degrees to radians
    lon1, lat1, lon2, lat2 = map(np.deg2rad, [lon1, lat1, lon2, lat2])

    # haversine formula
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = np.sin(dlat/2)**2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon/2)**2
    c = 2 * np.arcsin(np.sqrt(a))
    r = 6371 # Radius of earth in kilometers. Use 3956 for miles
    return c * r *1000

# from math import radians, cos, sin, asin, sqrt
###%

manoeuvres = ['circle_right', 'circle_left'] #'zigzag_20', 'zigzag_10', 'astern',
df_all = pd.DataFrame([])
for manoeuvre in manoeuvres:
    file_path = './Autopilot_light/RT_Evolution_manoeuvre_' + manoeuvre + '_2020-08-18.csv'
    df_main = pd.read_csv(file_path, sep=',')
    df_main.columns = ['timestamp', 'lat', 'lon', 'hdg', 'rsa_0', 'rsa_1', 'rsa_2', 'rpm_0', 'rpm_1', 'rpm_2']
    df_main = df_main.drop_duplicates(subset=['timestamp', 'lat', 'lon', 'hdg', 'rsa_0', 'rsa_1', 'rsa_2', 'rpm_0', 'rpm_1', 'rpm_2'])[1:]

    # df_main = pd.read_csv('test.csv', sep=',')
    # df_main = df_main[30400:31600].reset_index(inplace=False)

    df_main = df_main.apply(pd.to_numeric, errors='coerce')
    df_main.timestamp = df_main.timestamp*1000000000
    df_main.timestamp = pd.to_datetime(df_main.timestamp, format='%Y-%m-%d %H:%M:%S.%f')

    time_begin = df_main.timestamp[1]
    df_main['timestamp_norm'] = df_main.timestamp.apply(lambda x: (x-time_begin).total_seconds())
    # df_main.hdg = df_main.hdg + 4.2
    ##
    #calculate speeds, ROT, acc.

    df_main['delta_time'] = df_main.timestamp_norm.diff()
    df_main['dummy'] = df_main.lat
    # df_main.lat = df_main.lon; df_main.lon = df_main.dummy
    df_main['x'] = 0.0
    df_main['y'] = 0.0
    df_main['delta_psi'] = 0.0
    df_main = df_main.reset_index(drop=True)
    for i in df_main[1:].index:
        df_main.loc[i, 'x'] = np.sign(df_main.loc[i, 'lon'] - df_main.loc[i - 1, 'lon']) * haversine(df_main.loc[i - 1, 'lon'], df_main.loc[i, 'lat'], df_main.loc[i, 'lon'], df_main.loc[i, 'lat'])
        df_main.loc[i, 'y'] = np.sign(df_main.loc[i, 'lat'] - df_main.loc[i - 1, 'lat']) * haversine(df_main.loc[i, 'lon'], df_main.loc[i - 1, 'lat'], df_main.loc[i, 'lon'], df_main.loc[i, 'lat'])
        if abs(df_main.loc[i,'hdg'] - df_main.loc[i-1,'hdg'])>300.0:
            if df_main.loc[i,'hdg'] > df_main.loc[i-1,'hdg'] :
                df_main.loc[i, 'delta_psi'] = df_main.loc[i,'hdg'] - 360 - df_main.loc[i-1,'hdg']
            elif df_main.loc[i,'hdg'] < df_main.loc[i-1,'hdg'] :
                df_main.loc[i, 'delta_psi'] = df_main.loc[i,'hdg'] - (df_main.loc[i-1,'hdg']-360)
        else:
            df_main.loc[i, 'delta_psi'] = df_main.loc[i,'hdg'] - df_main.loc[i-1,'hdg']

    # df_main.x = df_main.x[df_main.x.between(df_main.x.quantile(.01), df_main.x.quantile(.99))]
    # df_main.y = df_main.y[df_main.y.between(df_main.y.quantile(.01), df_main.y.quantile(.99))]
    # df_main.delta_psi = df_main.delta_psi[df_main.delta_psi.between(df_main.delta_psi.quantile(.01), df_main.delta_psi.quantile(.99))]
    # # df_main = df_main[abs(df_main.x)<20.0]
    # states = df_main[['x', 'y', 'delta_psi']].to_numpy()
    # states = states.reshape([states.shape[0], 3, 1])
    # # add z score filtering
    #
    #
    # A = np.identity(3)
    # P = np.identity(3)*0
    # #measurement noise
    # value = 0.2
    # Q = np.diag([0.02,value,0.06])
    # H = np.identity(3)
    # R = np.diag([3, 10, 0.8])
    # B = 0
    # u = 0
    # x = inv(H).dot(states[0])
    # P = inv(H).dot(R).dot(inv(H.T))
    # new_states = np.zeros(shape=[1, 3, 1])
    # for state in states:
    #     x = A.dot(x)
    #     P = A.dot(P).dot(A.T) + Q
    #     S = H.dot(P).dot(H.T) + R
    #     K = P.dot(H.T).dot(inv(S))
    #     x = x + K.dot(state - H.dot(x))
    #     P = P - K.dot(H).dot(P)
    #     new_states = np.concatenate([new_states, np.expand_dims(x, axis=0)], axis=0)
    # new_states = new_states[1:]
    # df_main.x = new_states[:,0,:] ;df_main.y = new_states[:,1,:] ;df_main.delta_psi = new_states[:,2,:]

    df_main['x_dot'] = df_main.x / df_main.delta_time
    df_main['y_dot'] = df_main.y / df_main.delta_time
    df_main['delta_psi_dot'] = df_main.delta_psi / df_main.delta_time

    df_main['u'] = df_main.apply(lambda row: row.x_dot * np.sin(np.deg2rad(row.hdg)) + row.y_dot * np.cos(np.deg2rad(row.hdg)), axis=1)
    df_main['v'] = df_main.apply(lambda row: row.y_dot * np.sin(np.deg2rad(row.hdg)) - row.x_dot * np.cos(np.deg2rad(row.hdg)), axis=1)
    df_main['r'] = df_main.delta_psi_dot.apply(lambda x : np.deg2rad(x))

    df_main.u = (df_main.delta_time.shift(3)*df_main.u.shift(3)).rolling(window=7).sum()/df_main.delta_time.shift(3).rolling(window=7).sum()
    # plt.plot(df_main.u.tolist()); plt.plot(df_main.u_1.tolist())
    df_main.v = (df_main.delta_time.shift(3)*df_main.v.shift(3)).rolling(window=7).sum()/df_main.delta_time.shift(3).rolling(window=7).sum()
    df_main.r = (df_main.delta_time.shift(3)*df_main.r.shift(3)).rolling(window=7).sum()/df_main.delta_time.shift(3).rolling(window=7).sum()

    df_main['u_dot'] = (df_main.u - df_main.u.shift(1))/df_main.delta_time
    df_main['v_dot'] = (df_main.v - df_main.v.shift(1))/df_main.delta_time
    df_main['r_dot'] = (df_main.r - df_main.r.shift(1))/df_main.delta_time

    df_main.u_dot = (df_main.delta_time.shift(3)*df_main.u_dot.shift(3)).rolling(window=7).sum()/df_main.delta_time.shift(3).rolling(window=7).sum()
    df_main.v_dot = (df_main.delta_time.shift(3)*df_main.v_dot.shift(3)).rolling(window=7).sum()/df_main.delta_time.shift(3).rolling(window=7).sum()
    df_main.r_dot = (df_main.delta_time.shift(3)*df_main.r_dot.shift(3)).rolling(window=7).sum()/df_main.delta_time.shift(3).rolling(window=7).sum()


    df_main['x_real'] = df_main.x.cumsum()
    df_main['y_real'] = df_main.y.cumsum()
    # plt.plot(df_main.index.tolist(), df_main.r.tolist())
    plt.plot(df_main.index.tolist(), df_main.v.tolist())
    # plt.plot(df_main.index.tolist(), df_main.rsa_0.tolist())
    # plt.plot(df_main.y.tolist())
    # plt.plot(df_main.x_real.tolist()[:],df_main.y_real.tolist()[:])

    plt.show()
    df_all = pd.concat([df_all, df_main], axis=0)
df_all.to_csv('test_1.csv', index =False)






