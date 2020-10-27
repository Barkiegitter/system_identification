import numpy as np
import pandas as pd
from ship_class import ship
from manoeuvre_model_rpa3 import ship_model
import matplotlib.pyplot as plt
ship = ship()
# ship_model = ship_model()

df_main = pd.read_csv('test_sunday_evening.csv', sep=',')

df_main = df_main[30:800]
u, v, r, hdg = df_main.loc[df_main.index[0], 'u'],df_main.loc[df_main.index[0], 'v'], df_main.loc[df_main.index[0], 'r'], df_main.loc[df_main.index[0], 'hdg']
ship_model = ship_model(df_main.loc[df_main.index[0], 'u_dot'],df_main.loc[df_main.index[0], 'v_dot'], df_main.loc[df_main.index[0], 'r_dot'])

df_input = df_main[['rpm', 'rsa']]
print(u, v, r, hdg)
df_sim = pd.DataFrame([])
for i in df_main[:-1].index:
    u, v, r, hdg, delta_x_0, delta_y_0, delta_r_0, u_dot, v_dot, r_dot = ship_model.manoeuvre_model_rpa_3(u, v, r, hdg,
                                                                                                   df_main.loc[i, 'rpm']/60.,
                                                                                                   df_main.loc[i, 'rsa'], 
                                                                                                   df_main.loc[i, 'delta_time']
                                                                                                   )
    df_temp = pd.DataFrame({
                        'index_sim' : [i+1],
                        'x_delta_sim': [delta_x_0],
                        'y_delta_sim': [delta_y_0],
                        'hdg_delta_sim': [delta_r_0]
                                        })
    df_sim = pd.concat([df_sim, df_temp], axis=0)
    
df_main = pd.merge(df_main, df_sim, how='left', left_on=df_main.index, right_on='index_sim')

df_main['x_real_sim'] = df_main.x_delta_sim.cumsum()
df_main['y_real_sim'] = df_main.y_delta_sim.cumsum()
df_main['psi_sim'] = df_main.hdg_delta_sim.cumsum()

df_main['x_real'] = df_main.x.cumsum()
df_main['y_real'] = df_main.y.cumsum()
# df_main['psi_sim'] = df_main.hdg_delta_sim.cumsum()
    

df_main['traj_error'] = (np.sqrt((df_main['x_real_sim'] - df_main['x_real'])**2 + (df_main['y_real_sim'] - df_main['y_real'])**2)).cumsum()

# plt.plot(df_main.x_real.tolist()[:],df_main.y_real.tolist()[:])
# plt.plot(df_main.x_real_sim.tolist()[:],df_main.y_real_sim.tolist()[:])
plt.plot(df_main.traj_error)
plt.show()

# load initial states (lengte brackets bepalen van simuleren om error te bepalen. )
# recognize manoeuvre segments from trajectory


# use separate manoeuvres to compare performance 