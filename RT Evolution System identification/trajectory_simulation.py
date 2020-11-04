import numpy as np
import pandas as pd
from numpy import newaxis
from sklearn.linear_model import Lasso
from sklearn.linear_model import RidgeCV
# from sklearn.linear_model import LassoCV
import matplotlib.pyplot as plt
import math

from sklearn.model_selection import GridSearchCV
##
from ship_class import ship
from manoeuvre_model_evo import ship_model
ship = ship()

df_main = pd.read_csv('test_1_morechill.csv', sep=',')
coef_ = np.genfromtxt('foo_evo.csv', delimiter=',')

df_main = df_main[20:750]
u, v, r, hdg = df_main.loc[df_main.index[0], 'u'],df_main.loc[df_main.index[0], 'v'], df_main.loc[df_main.index[0], 'r'], df_main.loc[df_main.index[0], 'hdg']
ship_model = ship_model(df_main.loc[df_main.index[0], 'u_dot'],df_main.loc[df_main.index[0], 'v_dot'], df_main.loc[df_main.index[0], 'r_dot'], ship, coef_)

df_input = df_main[['rsa_0', 'rsa_1', 'rsa_2', 'rpm_0', 'rpm_1', 'rpm_2']]
print(u, v, r, hdg)
df_sim = pd.DataFrame([])
for i in df_main[:-1].index:
    u, v, r, hdg, delta_x_0, delta_y_0, delta_r_0, u_dot, v_dot, r_dot = ship_model.manoeuvre_model_rt_evolution(u, v, r, hdg,
                                                                                                   df_main.loc[i, 'rpm_0']/60., df_main.loc[i, 'rpm_1']/60., df_main.loc[i, 'rpm_2']/60.,
                                                                                                   df_main.loc[i, 'rsa_0'], df_main.loc[i, 'rsa_1'], df_main.loc[i, 'rsa_2'],
                                                                                                   df_main.loc[i, 'delta_time']
                                                                                                   )
    # print(u, v, r, hdg)
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
    
df_main['traj_error'] = (np.sqrt((df_main['x_real_sim'] - df_main['x_real'])**2 + (df_main['y_real_sim'] - df_main['y_real'])**2)).cumsum()
df_main['x_sim_diff_avg'] = df_main['x_real_sim'] - df_main.x_delta_sim.mean()
df_main['y_sim_diff_avg'] = df_main['y_real_sim'] - df_main.y_delta_sim.mean()

df_main['x_real_diff_avg'] = df_main['x_real'] - df_main.x.mean()
df_main['y_real_diff_avg'] = df_main['y_real'] - df_main.y.mean()

rho_x = (df_main['x_sim_diff_avg']*df_main['x_real_diff_avg']).sum()/np.sqrt((df_main['x_sim_diff_avg']**2).sum()*(df_main['x_real_diff_avg']**2).sum() )
rho_y = (df_main['y_sim_diff_avg']*df_main['y_real_diff_avg']).sum()/np.sqrt((df_main['y_sim_diff_avg']**2).sum()*(df_main['y_real_diff_avg']**2).sum() )
    
plt.plot(df_main.x_real.tolist()[:],df_main.y_real.tolist()[:])
plt.plot(df_main.x_real_sim.tolist()[:],df_main.y_real_sim.tolist()[:])
# plt.plot(df_main.traj_error)
plt.show()
print(rho_x*rho_y)




# import numpy as np
# import pandas as pd
# from ship_class import ship
# from manoeuvre_model_rpa3 import ship_model
# import matplotlib.pyplot as plt
# ship = ship()
# # ship_model = ship_model()
# coef_ = np.genfromtxt('foo_rpa3.csv', delimiter=',')
# df_main = pd.read_csv('test_sunday_evening.csv', sep=',')

# df_main = df_main[30:700]
# # df_main.rsa = df_main.rsa*0.88
# u, v, r, hdg = df_main.loc[df_main.index[0], 'u'],df_main.loc[df_main.index[0], 'v'], df_main.loc[df_main.index[0], 'r'], df_main.loc[df_main.index[0], 'hdg']
# ship_model = ship_model(df_main.loc[df_main.index[0], 'u_dot'],df_main.loc[df_main.index[0], 'v_dot'], df_main.loc[df_main.index[0], 'r_dot'], ship, coef_)

# df_input = df_main[['rpm', 'rsa']]
# print(u, v, r, hdg)
# df_sim = pd.DataFrame([])
# for i in df_main[:-1].index:
#     u, v, r, hdg, delta_x_0, delta_y_0, delta_r_0, u_dot, v_dot, r_dot = ship_model.manoeuvre_model_rpa_3(u, v, r, hdg,
#                                                                                                    df_main.loc[i, 'rpm']/60.,
#                                                                                                    df_main.loc[i, 'rsa'], 
#                                                                                                    df_main.loc[i, 'delta_time'],
#                                                                                                    )
#     df_temp = pd.DataFrame({
#                         'index_sim' : [i+1],
#                         'x_delta_sim': [delta_x_0],
#                         'y_delta_sim': [delta_y_0],
#                         'hdg_delta_sim': [delta_r_0]
#                                         })
#     df_sim = pd.concat([df_sim, df_temp], axis=0)
    
# df_main = pd.merge(df_main, df_sim, how='left', left_on=df_main.index, right_on='index_sim')

# df_main['x_real_sim'] = df_main.x_delta_sim.cumsum()
# df_main['y_real_sim'] = df_main.y_delta_sim.cumsum()
# df_main['psi_sim'] = df_main.hdg_delta_sim.cumsum()

# df_main['x_real'] = df_main.x.cumsum()
# df_main['y_real'] = df_main.y.cumsum()
    
# df_main['traj_error'] = (np.sqrt((df_main['x_real_sim'] - df_main['x_real'])**2 + (df_main['y_real_sim'] - df_main['y_real'])**2)).cumsum()
# df_main['x_sim_diff_avg'] = df_main['x_real_sim'] - df_main.x_delta_sim.mean()
# df_main['y_sim_diff_avg'] = df_main['y_real_sim'] - df_main.y_delta_sim.mean()

# df_main['x_real_diff_avg'] = df_main['x_real'] - df_main.x.mean()
# df_main['y_real_diff_avg'] = df_main['y_real'] - df_main.y.mean()

# rho_x = (df_main['x_sim_diff_avg']*df_main['x_real_diff_avg']).sum()/np.sqrt((df_main['x_sim_diff_avg']**2).sum()*(df_main['x_real_diff_avg']**2).sum() )
# rho_y = (df_main['y_sim_diff_avg']*df_main['y_real_diff_avg']).sum()/np.sqrt((df_main['y_sim_diff_avg']**2).sum()*(df_main['y_real_diff_avg']**2).sum() )
    
# plt.plot(df_main.x_real.tolist()[:],df_main.y_real.tolist()[:])
# plt.plot(df_main.x_real_sim.tolist()[:],df_main.y_real_sim.tolist()[:])
# # plt.plot(df_main.traj_error)
# plt.show()
# print(rho_x*rho_y)