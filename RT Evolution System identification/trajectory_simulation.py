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

df_main = pd.read_csv('test_1_large.csv', sep=',')
coef_ = np.genfromtxt('foo_evo_general.csv', delimiter=',')
acc_lim = np.genfromtxt('acc_limits.csv', delimiter=',')

df_main = df_main.dropna()
df_main = df_main[300:1500]
x = [439415.64949675516, 17382.008767413965, -8401.877111986285, 624.1173738745687, -40517.99837828789, -2985999.167457851, -906562.1722859398, 7335.14084309014, 78926.62119567573, 520594.03247537767, 360894.17072922684, -39839.67937353795, -467.5061967410806, 25151.55549362494, 668968.2162711263, -24824.619325132873, -3459.945897212267, -2488.99216061329, 58043147.92605774, 3677333.6385066556, 735.9935551955393, -15474.557257327044, -240319.1948020185, 156271.44485163546, -6563319.662628548, 388323281.6913895, -2470583.8591193086, -3487129.2821591864, -222135.75333648475, 283487.68736166746, -16735.950138382887, 3016.501753365038, 192621.24317513878, 185893793.9981207, 15976506.629899241, 525559.0929632398, -294883.3069364647, -1703965.7186298491, -817112.1358419779, -19291999.846357513]
x = [2759800.995573263, 23011.18494495929, -11701.025016885147, 849.2182390142812, 5703.835492731456, -755040.6978037895, -2333126.4007608155, -2855.008371840717, -102503.64849876355, 25339.133222710752, 2490442.265843721, -67082.9686275706, -645.6559852157586, 22714.88346582745, 2594654.8012700174, 26794.765359863315, -2277.876636660988, 23099.090254140927, -14029586.822891729, 2846014.414749532, 380725.47208945156, -44164.981903642314, -445.49673567718094, 319803.7352982848, 2816589.852597608, 787362675.7324224, 511020.966944188, -1283528.5204512358, -105077.94146040115, -609138.826002035, -13002.000636336932, -2081.78604617872, -72609.91001349717, -188125684.77176327, -33753845.69114218, -4050917.322807095, -30505.537998455246, 828891.8900567949, 2194211.007983167, 706492.2675797178]
# coef_ = np.array([np.asarray(x[:10]),np.asarray(x[10:25]),np.asarray(x[25:40])])
u, v, r, hdg = df_main.loc[df_main.index[0],'u'],df_main.loc[df_main.index[0],'v'], df_main.loc[df_main.index[0],'r'], df_main.loc[df_main.index[0], 'hdg']
ship_model = ship_model(df_main.loc[df_main.index[0], 'u_dot'],df_main.loc[df_main.index[0], 'v_dot'], df_main.loc[df_main.index[0], 'r_dot'], ship, coef_, acc_lim)
df_sim = pd.DataFrame([])

# coef_[2][5] = coef_[2][5] -1000000
# coef_[2][1] = coef_[2][1] +10000
for i in df_main[:-1].index:
    # if i%200==0:
        
    #     u, v, r, hdg = df_main.loc[i,'u'],df_main.loc[i,'v'], df_main.loc[i,'r'], df_main.loc[i, 'hdg']
    u, v, r, hdg, delta_x_0, delta_y_0, delta_r_0, u_dot, v_dot, r_dot = ship_model.manoeuvre_model_rt_evolution(u, v, r, hdg,
                                                                                                   df_main.loc[i, 'rpm_0']/60., df_main.loc[i, 'rpm_1']/60., df_main.loc[i, 'rpm_2']/60.,
                                                                                                   df_main.loc[i, 'rsa_0'], df_main.loc[i, 'rsa_1'], df_main.loc[i, 'rsa_2'],
                                                                                                   df_main.loc[i, 'delta_time'],
                                                                                                   # u_dot_arr, v_dot_arr, r_rot_arr
                                                                                                   )
    
    # u = u + (u_1 - u)*20
    # v = v + (v_1 - v)*20
    # r = r + (r_1 - r)*20
    # print(u, v, r, hdg)
    # print(i)
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