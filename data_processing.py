import numpy as np
import pandas as pd
from numpy.linalg import inv
import csv
import time
import matplotlib.pyplot as plt

from math import radians, cos, sin, asin, sqrt

def haversine(lon1, lat1, lon2, lat2):
    """
    Calculate the great circle distance between two points
    on the earth (specified in decimal degrees)
    """
    # convert decimal degrees to radians
    lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])

    # haversine formula
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a))
    r = 6371 # Radius of earth in kilometers. Use 3956 for miles
    return c * r *1000

df_main = pd.read_csv('test.csv', sep=',')
# df_main = df_main[30400:31600].reset_index(inplace=False)
df_main.timestamp = pd.to_datetime(df_main.timestamp, format='%Y-%m-%d %H:%M:%S.%f')


time_begin = df_main.timestamp[0]
df_main['timestamp_norm'] = df_main.timestamp.apply(lambda x: (x-time_begin).total_seconds())
df_main.hdg = df_main.hdg + 4.2

#calculate speeds, ROT, acc.

df_main['delta_time'] = df_main.timestamp_norm.diff()
df_main.lat = df_main.lat; df_main.lon = df_main.lon
df_main['x'] = 0.0
df_main['y'] = 0.0
df_main['delta_psi'] = 0.0
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
df_main = df_main[abs(df_main.x)<20.0]
states = df_main[['x', 'y', 'delta_psi']].to_numpy()
states = states.reshape([states.shape[0], 3, 1])
# add z score filtering


A = np.identity(3)
P = np.identity(3)*0
#measurement noise
Q = np.diag([0.02,0.02,0.06])
H = np.identity(3)
R = np.diag([3, 3, 0.8])
B = 0
u = 0
x = inv(H).dot(states[0])
P = inv(H).dot(R).dot(inv(H.T))
new_states = np.zeros(shape=[1, 3, 1])
for state in states:
    x = A.dot(x)
    P = A.dot(P).dot(A.T) + Q
    S = H.dot(P).dot(H.T) + R
    K = P.dot(H.T).dot(inv(S))
    x = x + K.dot(state - H.dot(x))
    P = P - K.dot(H).dot(P)
    new_states = np.concatenate([new_states, np.expand_dims(x, axis=0)], axis=0)
new_states = new_states[1:]

# df_main.x = new_states[:,0,:] ;df_main.y = new_states[:,1,:] ;df_main.delta_psi = new_states[:,2,:]



df_main['x_dot'] = df_main.x / df_main.delta_time
df_main['y_dot'] = df_main.y / df_main.delta_time
df_main['delta_psi_dot'] = df_main.delta_psi / df_main.delta_time

df_main['u'] = df_main.apply(lambda row: row.x_dot * cos(radians(row.delta_psi)) + row.y_dot * sin(radians(row.delta_psi)), axis=1)
df_main['v'] = df_main.apply(lambda row:-row.x_dot * sin(radians(row.delta_psi)) + row.y_dot * cos(radians(row.delta_psi)), axis=1)
df_main['r'] = df_main.delta_psi_dot.apply(lambda x : radians(x))

df_main.u = (df_main.delta_time.shift(3)*df_main.u.shift(3)).rolling(window=7).sum()/df_main.delta_time.shift(3).rolling(window=7).sum()
# plt.plot(df_main.u.tolist()); plt.plot(df_main.u_1.tolist())
df_main.v = (df_main.delta_time.shift(3)*df_main.v.shift(3)).rolling(window=7).sum()/df_main.delta_time.shift(3).rolling(window=7).sum()
df_main.r = (df_main.delta_time.shift(3)*df_main.r.shift(3)).rolling(window=7).sum()/df_main.delta_time.shift(3).rolling(window=7).sum()

MA_ = 17
MA_bound = int((MA_-1)/2)

MA__r = 17
MA_bound_r = int((MA__r-1)/2)

MA_acc = 35
MA_bound_acc = int((MA_acc-1)/2)

MA_acc_r = 35
MA_bound_acc_r = int((MA_acc_r-1)/2)

df_main.u = (df_main.delta_time.shift(MA_bound)*df_main.u.shift(MA_bound)).rolling(window=MA_).sum()/df_main.delta_time.shift(MA_bound).rolling(window=MA_).sum()
df_main.v = (df_main.delta_time.shift(MA_bound)*df_main.v.shift(MA_bound)).rolling(window=MA_).sum()/df_main.delta_time.shift(MA_bound).rolling(window=MA_).sum()
df_main.r = (df_main.delta_time.shift(MA_bound_r)*df_main.r.shift(MA_bound_r)).rolling(window=MA__r).sum()/df_main.delta_time.shift(MA_bound_r).rolling(window=MA__r).sum()

df_main.u = df_main.u.shift(-MA_acc)
df_main.v = df_main.v.shift(-MA_acc)
df_main.r = df_main.r.shift(-MA_acc_r)

df_main['u_dot'] = (df_main.u - df_main.u.shift(1))/df_main.delta_time
df_main['v_dot'] = (df_main.v - df_main.v.shift(1))/df_main.delta_time
df_main['r_dot'] = (df_main.r - df_main.r.shift(1))/df_main.delta_time

df_main.u_dot = (df_main.delta_time.shift(3)*df_main.u_dot.shift(3)).rolling(window=7).sum()/df_main.delta_time.shift(3).rolling(window=7).sum()
df_main.v_dot = (df_main.delta_time.shift(3)*df_main.v_dot.shift(3)).rolling(window=7).sum()/df_main.delta_time.shift(3).rolling(window=7).sum()
df_main.r_dot = (df_main.delta_time.shift(3)*df_main.r_dot.shift(3)).rolling(window=7).sum()/df_main.delta_time.shift(3).rolling(window=7).sum()


df_main['x_real'] = df_main.x.cumsum()
df_main['y_real'] = df_main.y.cumsum()
# plt.plot(df_main.x_real.tolist(), df_main.y_real.tolist())
# plt.plot(df_main.index.tolist(), df_main.u.tolist())
# plt.plot(df_main.y.tolist())
plt.plot(df_main.x_real.tolist()[:],df_main.y_real.tolist()[:])
plt.show()
# df_main.to_csv('test_1.csv', index =False)






