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
from manoeuvre_model_borkum import ship_model
ship = ship()

df_main = pd.read_csv('test_1_large.csv', sep=',')
df_main = df_main[:100]
coef_ = np.genfromtxt('foo_evo_general.csv', delimiter=',')

u, v, r, hdg = df_main.loc[df_main.index[0],'u'],df_main.loc[df_main.index[0],'v'], df_main.loc[df_main.index[0],'r'], df_main.loc[df_main.index[0], 'hdg']
ship_model = ship_model(df_main.loc[df_main.index[0], 'u_dot'],df_main.loc[df_main.index[0], 'v_dot'], df_main.loc[df_main.index[0], 'r_dot'], ship, coef_)
df_sim = pd.DataFrame([])

# coef_[2][5] = coef_[2][5] -1000000
# coef_[2][1] = coef_[2][1] +10000
for i in df_main[:-1].index:
    if i%5==0:
        
        u, v, r, hdg = df_main.loc[i,'u'],df_main.loc[i,'v'], df_main.loc[i,'r'], df_main.loc[i, 'hdg']
    u, v, r, hdg, delta_x_0, delta_y_0, delta_r_0, u_dot, v_dot, r_dot = ship_model.manoeuvre_model_rt_evolution(u, v, r, hdg,
                                                                                                   df_main.loc[i, 'rpm_0']/60., df_main.loc[i, 'rpm_1']/60., df_main.loc[i, 'rpm_2']/60.,
                                                                                                   df_main.loc[i, 'rsa_0'], df_main.loc[i, 'rsa_1'], df_main.loc[i, 'rsa_2'],
                                                                                                   df_main.loc[i, 'delta_time'],
                                                                                                   # u_dot_arr, v_dot_arr, r_rot_arr
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

df_main['x_diff'] = df_main.x_delta_sim - df_main.x

df_main['x_diff'] = abs(df_main.x_delta_sim - df_main.x)
df_main['y_diff'] = abs(df_main.y_delta_sim - df_main.y)
df_main['error'] = np.sqrt(df_main.x_diff**2 + df_main.y_diff**2)
print(df_main.error.sum())
# df_main['traj_error'] = (np.sqrt((df_main['x_real_sim'] - df_main['x_real'])**2 + (df_main['y_real_sim'] - df_main['y_real'])**2)).cumsum()
# df_main['x_sim_diff_avg'] = df_main['x_real_sim'] - df_main.x_delta_sim.mean()
# df_main['y_sim_diff_avg'] = df_main['y_real_sim'] - df_main.y_delta_sim.mean()

# df_main['x_real_diff_avg'] = df_main['x_real'] - df_main.x.mean()
# df_main['y_real_diff_avg'] = df_main['y_real'] - df_main.y.mean()

# rho_x = (df_main['x_sim_diff_avg']*df_main['x_real_diff_avg']).sum()/np.sqrt((df_main['x_sim_diff_avg']**2).sum()*(df_main['x_real_diff_avg']**2).sum() )
# rho_y = (df_main['y_sim_diff_avg']*df_main['y_real_diff_avg']).sum()/np.sqrt((df_main['y_sim_diff_avg']**2).sum()*(df_main['y_real_diff_avg']**2).sum() )
    
plt.plot(df_main.x_real.tolist()[:],df_main.y_real.tolist()[:])
plt.plot(df_main.x_real_sim.tolist()[:],df_main.y_real_sim.tolist()[:])
# plt.plot(df_main.traj_error)
plt.title('Simulated vs Sailed trajectory')
plt.savefig('example.png')
plt.show()
# print(rho_x*rho_y)




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


# [93366.04075268705, 0, -1817.6020691465803, 87.06249852139729, 0, -7581.030606425917, -445407.90131894, 116486.33810620815, -20759.64574466075, -324.28746397277, 6473.716221211791, 514951.55229089025, -4046.990425681326, -1046.4853203917237, -28.093150646732717, -41548.97174293938, 168397.17597692058, 34364.18139473641, 5051966.186761198, -392128.24442689324, -424137.88565644826, -4914.426235461651, 205034.34315327078, -6935.914438482643, 579.5457524212336, 20193.750907158843, -82327.23836097834, 549103.2567495358, -49785.0239157738]