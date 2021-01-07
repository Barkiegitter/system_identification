import numpy as np
import pandas as pd
from numpy import newaxis
from sklearn.linear_model import Lasso
from sklearn.linear_model import Ridge
import matplotlib.pyplot as plt
import math


from sklearn.model_selection import RepeatedKFold
from sklearn.model_selection import GridSearchCV
##
from ship_class import ship
ship = ship()
df_main = pd.read_csv('test_1_large.csv', sep=',')
# df_main = df_main[30400:31600].reset_index(inplace=False)
df_main.timestamp = pd.to_datetime(df_main.timestamp, format='%Y-%m-%d %H:%M:%S.%f')
df_main = df_main.dropna()
# df_main = df_main[:700]
df_main['dt'] = pd.to_datetime(df_main['timestamp']).diff().dt.total_seconds()
# df_main.rsa_0 = df_main.rsa_0+10.0
def thrust_cone(x_eng, y_eng, az_eng, cone_deg, flow_distance, x_down, y_down):
    #check axis!!!!!!! coherent to choosen axis system
    top_corner = [y_eng, x_eng]
    right_corner = [y_eng + np.sin(np.deg2rad(az_eng - cone_deg/2.))*flow_distance, x_eng + np.cos(np.deg2rad(az_eng - cone_deg/2.))*flow_distance]
    left_corner = [y_eng + np.sin(np.deg2rad(az_eng + cone_deg/2.))*flow_distance, x_eng + np.cos(np.deg2rad(az_eng + cone_deg/2.))*flow_distance]
    triangle_list = [top_corner, right_corner, left_corner]
    x1, y1, x2, y2, x3, y3, xp, yp = top_corner[0], top_corner[1], right_corner[0], right_corner[1], left_corner[0], left_corner[1], y_down, x_down
    c1 = (x2 - x1) * (yp - y1) - (y2 - y1) * (xp - x1)
    c2 = (x3 - x2) * (yp - y2) - (y3 - y2) * (xp - x2)
    c3 = (x1 - x3) * (yp - y3) - (y1 - y3) * (xp - x3)
    if (c1 < 0 and c2 < 0 and c3 < 0) or (c1 > 0 and c2 > 0 and c3 > 0):
    # if max([pos[0] for pos in triangle_list])>=y_down>=min([pos[0] for pos in triangle_list]) and max([pos[1] for pos in triangle_list])>=x_down>=min([pos[1] for pos in triangle_list]):
        return (1 - np.deg2rad(abs((azimuth([y_eng, x_eng],[y_down,x_down])-az_eng))))
    else:
        return 0

def azimuth(point1, point2):
    angle = np.arctan2(point2[0] - point1[0], point2[1] - point1[1])
    return np.degrees(angle) if angle >= 0 else np.degrees(angle) + 360
    #add thrust fraction here

def thruster_interaction_coefficient(x_eng, y_eng, az_eng, cone_deg, flow_distance, x_down, y_down, az_down):  # give engine number: shipclass engine number
    thrust_cone_boolean = thrust_cone(x_eng, y_eng, az_eng, cone_deg, flow_distance, x_down, y_down)
    x_d_ratio = np.sqrt((x_down-x_eng)**2+abs(y_down-y_eng)) / ship.D_p
    t_engine = (1 - 0.75**(x_d_ratio**(2/3)))/(thrust_cone_boolean)
    # print(abs(az_eng-az_down), 1)
    t = t_engine+(1-t_engine)*(((abs(az_eng-az_down))**3)/(((130.)/(t_engine**3)) + ((abs(az_eng-az_down))**3)))
    if math.isnan(t):
        return 0
    else:
        return 1-t

df_main.rpm_0 = df_main.rpm_0/60.0
df_main.rpm_1 = df_main.rpm_1/60.0
df_main.rpm_2 = df_main.rpm_2/60.0

df_main['az_speed_0'] = (df_main.rsa_0.diff()/df_main.dt).rolling(10).mean().shift(1)
df_main['az_speed_1'] = (df_main.rsa_1.diff()/df_main.dt).rolling(10).mean().shift(1)
df_main['az_speed_2'] = (df_main.rsa_2.diff()/df_main.dt).rolling(10).mean().shift(1)
df_main['az_speed_0'] = (df_main.rsa_0.diff())
df_main['az_speed_1'] = (df_main.rsa_1.diff())
df_main['az_speed_2'] = (df_main.rsa_2.diff())

# df_main.az_speed_0 = df_main.az_speed_0.apply(lambda x: x-360. if x>360. else ())

#azimuth 2 port side
df_main['u_a_2'] = (1-ship.w)*((df_main.u+df_main.r*abs(ship.y_2))*-1*np.cos(np.deg2rad(df_main.rsa_2)) + (df_main.v+df_main.r*abs(ship.x_2))*-1*np.sin(np.deg2rad(df_main.rsa_2))) #(1-ship.w)*
df_main['u_a_1'] = (1-ship.w)*((df_main.u-df_main.r*abs(ship.y_1))*-1*np.cos(np.deg2rad(df_main.rsa_1)) + (-df_main.v-df_main.r*abs(ship.x_1))*-1*np.sin(np.deg2rad(df_main.rsa_1))) #(1-ship.w)*
df_main['u_a_0'] = (1-ship.w)*((df_main.u)*-1*np.cos(np.deg2rad(df_main.rsa_0)) + ((-df_main.v + df_main.r*abs(ship.x_0))*-1*np.sin(np.deg2rad(df_main.rsa_0))) ) #(1-ship.w)*


# df_main['u_a_2'] =  df_main.u
# df_main['u_a_1'] =  df_main.u
# df_main['u_a_0'] =  df_main.u

df_main['beta_2'] = np.rad2deg(np.arctan((df_main.u_a_2)/(0.7*np.pi*df_main.rpm_2*ship.D_p)))
#change invalse hoek acoording to engine stand
# df_main['beta_2'] = df_main.apply(lambda row: row['beta_2'] + 180 if row['u_a_2']>=0  else (row['beta_2'] + 0 if row['u_a_2']<0 and row['rsa_2']<90 or row['rsa_2']>270. else (row['beta_2'] + 180 if row['u_a_2'] <0 and 90.<row['rsa_2']<270. else row['beta_2'])) ,axis=1)
# df_main['beta_2'] = df_main.beta_2.apply(lambda x: x-360 if x>360 else x)
df_main['beta_2'] = df_main.beta_2.apply(lambda x: x+360 if x<0 else x)

df_main['beta_1'] = np.rad2deg(np.arctan((df_main.u_a_1)/(0.7*np.pi*df_main.rpm_1*ship.D_p)))
# df_main['beta_1'] = df_main.apply(lambda row: row['beta_1'] + 180 if row['u_a_1']>=0 and row['rsa_1']<90 or row['rsa_1']>270. else (row['beta_1'] + 180 if row['u_a_1']<0 and row['rsa_1']<90 or row['rsa_1']>270. else (row['beta_1'] + 360  if row['u_a_1'] <0 and 90.<row['rsa_1']<270. else row['beta_1'])) ,axis=1)
# df_main['beta_1'] = df_main.beta_1.apply(lambda x: x-360 if x>360 else x)
df_main['beta_1'] = df_main.beta_1.apply(lambda x: x+360 if x<0 else x)

df_main['beta_0'] = np.rad2deg(np.arctan((df_main.u_a_0)/(0.7*np.pi*df_main.rpm_0*ship.D_p)))
# df_main['beta_0'] = df_main.apply(lambda row: row['beta_0'] + 180 if row['u_a_0']>=0 and row['rsa_0']<90 or row['rsa_0']>270. else (row['beta_0'] + 180 if row['u_a_0']<0 and row['rsa_0']<90 or row['rsa_0']>270. else (row['beta_0'] + 360 if row['u_a_0'] <0 and 90.<row['rsa_0']<270. else row['beta_0'])) ,axis=1)
# df_main['beta_0'] = df_main.beta_0.apply(lambda x: x-360 if x>360 else x)
df_main['beta_0'] = df_main.beta_0.apply(lambda x: x+360 if x<0 else x)

# first engine listed experiences thrust decrease, t_21 means thrust reduction ratio due to downstream flow caused by engine 1
df_main['t_21_phi'] = df_main.apply(lambda row: thruster_interaction_coefficient(ship.x_1, ship.y_1, row['rsa_1'], 25.0, 100.0, ship.x_2, ship.y_2, row['rsa_2']), axis=1)
df_main['t_20_phi'] = df_main.apply(lambda row: thruster_interaction_coefficient(ship.x_0, ship.y_0, row['rsa_0'], 25.0, 100.0, ship.x_2, ship.y_2, row['rsa_2']), axis=1)
df_main['f_p_40_2'] = 1.*((1-ship.t)*ship.beta_coef(df_main.beta_2)*0.5*ship.rho*(((((1-ship.w)*df_main.u_a_2)**2)+(0.7*np.pi*df_main.rpm_2*ship.D_p)**2))*np.pi/4*ship.D_p**2)  #(1-df_main['t_21_phi'])*(1-df_main['t_20_phi'])*
##*(1-df_main['t_21_phi'])*(1-df_main['t_20_phi'])
df_main['t_12_phi'] = df_main.apply(lambda row: thruster_interaction_coefficient(ship.x_2, ship.y_2, row['rsa_2'], 25.0, 100.0, ship.x_1, ship.y_1, row['rsa_1']), axis=1)
df_main['t_10_phi'] = df_main.apply(lambda row: thruster_interaction_coefficient(ship.x_0, ship.y_0, row['rsa_0'], 25.0, 100.0, ship.x_1, ship.y_1, row['rsa_1']), axis=1)
df_main['f_p_40_1'] = 1.*((1-ship.t)*ship.beta_coef(df_main.beta_1)*0.5*ship.rho*(((((1-ship.w)*df_main.u_a_1)**2)+(0.7*np.pi*df_main.rpm_1*ship.D_p)**2))*np.pi/4*ship.D_p**2) #(1-df_main['t_12_phi'])*(1-df_main['t_10_phi'])*
#*(1-df_main['t_12_phi'])*(1-df_main['t_10_phi'])
df_main['t_02_phi'] = df_main.apply(lambda row: thruster_interaction_coefficient(ship.x_2, ship.y_2, row['rsa_2'], 25.0, 100.0, ship.x_0, ship.y_0, row['rsa_0']), axis=1)
df_main['t_01_phi'] = df_main.apply(lambda row: thruster_interaction_coefficient(ship.x_1, ship.y_1, row['rsa_1'], 25.0, 100.0, ship.x_0, ship.y_0, row['rsa_0']), axis=1)
df_main['f_p_40_0'] = 1.*((1-ship.t)*ship.beta_coef(df_main.beta_0)*0.5*ship.rho*(((((1-ship.w)*df_main.u_a_0)**2)+(0.7*np.pi*df_main.rpm_0*ship.D_p)**2))*np.pi/4*ship.D_p**2)#.rolling(20).mean()  #(1-df_main['t_02_phi'])*(1-df_main['t_01_phi'])*
# 
# (1-df_main['t_02_phi'])*(1-df_main['t_01_phi'])*
# J =(1-w)*u/(n_p*D_p);                                                               % Advance ratio of the propeller   J =(1-w)*u/(n_p*D_p)
#     F_r = - 577 * A_r * (u*1.3)^2 * sin(deg2rad(delta));                                      % Rudder force with respect to speed and rudder angle

#     F_p = (1-t)*rho * n_p^2 * D_p^4 * interp1(K_t(:,1),K_t(:,2),J,'linear','extrap');    % Propeller force with respect to speed and propeller rpm

# df_main['t_21_phi'] = df_main.apply(lambda row: thruster_interaction_coefficient(ship.x_1, ship.y_1, row['rsa_1'], 25.0, 100.0, ship.x_2, ship.y_2, row['rsa_2']), axis=1)


#slide u_dot

df_main['u_dot_spec'] = df_main.u_dot.shift(1)
df_main['v_dot_spec'] = df_main.v_dot.shift(1)
df_main['r_dot_spec'] = df_main.r_dot.shift(1)
df_main['r_dot_spec_1'] = df_main.r_dot.rolling(10).mean()

####trail


# df_main = df_main[df_main.u>0.0]

df_main = df_main[20:-7]
plt.show()

u = df_main.u.to_numpy()[:, newaxis]
v = df_main.v.to_numpy()[:, newaxis]
r = df_main.r.to_numpy()[:, newaxis]

u_a_2 = df_main.u_a_2.to_numpy()[:, newaxis]
u_a_1 = df_main.u_a_1.to_numpy()[:, newaxis]
u_a_0 = df_main.u_a_0.to_numpy()[:, newaxis]



u_dot_spec = df_main.u_dot_spec.to_numpy()[:, newaxis]
u_dot = df_main.u_dot.to_numpy()[:, newaxis]

v_dot = df_main.v_dot.to_numpy()[:, newaxis]
v_dot_spec = df_main.v_dot_spec.to_numpy()[:, newaxis]


r_dot = df_main.r_dot.to_numpy()[:, newaxis]
r_dot_spec_1 = df_main.r_dot_spec_1.to_numpy()[:, newaxis]


rsa_0 = df_main.rsa_0.to_numpy()[:, newaxis]
rsa_1 = df_main.rsa_1.to_numpy()[:, newaxis]
rsa_2 = df_main.rsa_2.to_numpy()[:, newaxis]

az_speed_0 = df_main.az_speed_0.to_numpy()[:, newaxis]
az_speed_1 = df_main.az_speed_1.to_numpy()[:, newaxis]
az_speed_2 = df_main.az_speed_2.to_numpy()[:, newaxis]

f_p_40_0 = df_main.f_p_40_0.to_numpy()[:, newaxis]
f_p_40_2 = df_main.f_p_40_2.to_numpy()[:, newaxis]
f_p_40_1 = df_main.f_p_40_1.to_numpy()[:, newaxis]



# df_main['az_speed_0'] = df_main.rsa_0.diff()/df_main.dt
###trail:

az_speed_0 = df_main.az_speed_0.to_numpy()[:, newaxis]
az_speed_1 = df_main.az_speed_1.to_numpy()[:, newaxis]
az_speed_2 = df_main.az_speed_2.to_numpy()[:, newaxis]
uv = np.arctan(v/u)

# X = u uu uuu vv rr vr uvv rvu urr
# Y = v uv ur uur uuv vvv rrr rrv vvr abs(v)v abs(r)v rabs(v) abs(r)r
# N = r uv ur uur uuv vvv rrr rrv vvr abs(v)v abs(r)v rabs(v) abs(r)r
X = np.concatenate([u_dot, u, u*u, u*u*u, v*v, r*r, v*r, u*v*v, r*v*u, u*r*r,
                    # np.deg2rad(rsa_0)*np.deg2rad(rsa_0),np.deg2rad(rsa_1)*np.deg2rad(rsa_1),np.deg2rad(rsa_2)*np.deg2rad(rsa_2),
                    # np.deg2rad(rsa_0)*r*u,np.deg2rad(rsa_1)*r*u,np.deg2rad(rsa_2)*r*u,
                    # np.deg2rad(rsa_0)*u,np.deg2rad(rsa_1)*u,np.deg2rad(rsa_2)*u, 
                    # np.deg2rad(rsa_0)*np.deg2rad(rsa_0)*u,np.deg2rad(rsa_1)*np.deg2rad(rsa_1)*u,np.deg2rad(rsa_2)*np.deg2rad(rsa_2)*u
                    ], axis=1) #, v*r, u*v*v, r*v*u, u*r*r
Y = np.concatenate([v_dot, v, v*v, u*v, u*r, u*u*r, u*u*v, v*v*v, r*r*r, r*r*v, v*v*r , abs(v)*v, abs(r)*v, r*abs(v), abs(r)*r], axis=1) #, abs(v)*v, abs(r)*v, r*abs(v), abs(r)*r
N = np.concatenate([r_dot, r, r*r, v*r, u*r, u*u*r, u*u*v, v*v*v, r*r*r, r*r*v, v*v*r, abs(v)*v, abs(r)*v, r*abs(v), abs(r)*r, 
                    # np.deg2rad(rsa_0)*np.deg2rad(rsa_0)*u,np.deg2rad(rsa_1)*np.deg2rad(rsa_1)*u,np.deg2rad(rsa_2)*np.deg2rad(rsa_2)*u,
                    # np.cos(np.deg2rad(rsa_0))*az_speed_0,np.cos(np.deg2rad(rsa_1))*az_speed_1,np.cos(np.deg2rad(rsa_2))*az_speed_2,#,np.deg2rad(rsa_1)*np.deg2rad(rsa_1),np.deg2rad(rsa_2)*np.deg2rad(rsa_2),
                    # np.deg2rad(rsa_0)*r*u,np.deg2rad(rsa_1)*r*u,np.deg2rad(rsa_2)*r*u, #np.cos(np.deg2rad(rsa_0))**2*az_speed_0*f_p_40_0,np.cos(np.deg2rad(rsa_1-30.0))**2*az_speed_1*f_p_40_1,np.cos(np.deg2rad(rsa_2+30.0))**2*az_speed_2*f_p_40_2,
                    # np.deg2rad(rsa_0)*r*r,np.deg2rad(rsa_1)*r*r,np.deg2rad(rsa_2)*r*r, 
                    # np.deg2rad(rsa_0)*v,np.deg2rad(rsa_1)*v,np.deg2rad(rsa_2)*v,
                    # np.deg2rad(rsa_0)*np.deg2rad(rsa_0)*r,np.deg2rad(rsa_1)*np.deg2rad(rsa_1)*r,np.deg2rad(rsa_2)*np.deg2rad(rsa_2)*r 
                    ], axis=1)#, u*u*r, u*u*v, v*v*v, r*r*r, r*r*v, v*v*r, abs(v)*v, abs(r)*v, r*abs(v), abs(r)*r

y_x = ship.Mass*(u_dot-r*v)-1.0*(-np.cos(np.deg2rad(rsa_0))*(f_p_40_0)-np.cos(np.deg2rad(rsa_1))*(f_p_40_1)-np.cos(np.deg2rad(rsa_2))*(f_p_40_2))
y_y = ship.Mass*(v_dot+r*u)-1.0*(-np.sin(np.deg2rad(rsa_0))*(f_p_40_0)-np.sin(np.deg2rad(rsa_1))*(f_p_40_1)-np.sin(np.deg2rad(rsa_2))*(f_p_40_2)) #np.sin(rsa_0)*abs(f_p_40_0)+np.sin(rsa_1)*abs(f_p_40_1)+
y_r = ship.I_e*r_dot-1*(ship.x_0*-1*np.sin(np.deg2rad(rsa_0))*(f_p_40_0)+ship.x_2*-1*np.sin(np.deg2rad(rsa_2))*(f_p_40_2) + ship.x_1*-1*np.sin(np.deg2rad(rsa_1))*(f_p_40_1) - ship.y_2*-1*np.cos(np.deg2rad(rsa_2))*(f_p_40_2) - ship.y_1*-1*np.cos(np.deg2rad(rsa_1))*(f_p_40_1))


model = Ridge(fit_intercept=False)
cv = RepeatedKFold(n_splits=5, n_repeats=3, random_state=1)
grid = dict()
grid['alpha'] = np.arange(0.0, 2.0, 0.5)
search = GridSearchCV(model, grid, scoring='neg_mean_absolute_error', cv=cv, n_jobs=-1)
results_x = search.fit(X, y_x)

model = Ridge(fit_intercept=False)
cv = RepeatedKFold(n_splits=5, n_repeats=3, random_state=1)
grid = dict()
grid['alpha'] = np.arange(0.0, 2.0, 0.5)
search = GridSearchCV(model, grid, scoring='neg_mean_absolute_error', cv=cv, n_jobs=-1)
results_y = search.fit(Y, y_y)

model = Ridge(fit_intercept=False)
cv = RepeatedKFold(n_splits=5, n_repeats=3, random_state=1)
grid = dict()
grid['alpha'] = np.arange(0, 120.0, 1.0)
search = GridSearchCV(model, grid, scoring='neg_mean_absolute_error')#, cv=cv)#, n_jobs=-1)
results_r = search.fit(N, y_r)

print(results_x.best_estimator_.score(X,y_x), results_x.best_estimator_.alpha)
print(results_y.best_estimator_.score(Y,y_y), results_y.best_estimator_.alpha)
print(results_r.best_estimator_.score(N,y_r), results_r.best_estimator_.alpha)


plt.plot(y_x)
plt.plot(np.sum(X*results_x.best_estimator_.coef_, axis=1))
plt.title('u_dot')
plt.savefig('u.png')
plt.show()
plt.close()
plt.plot(np.sum(Y*results_y.best_estimator_.coef_, axis=1));plt.plot(y_y)
plt.title('v_dot')
plt.savefig('v.png')
plt.show()
plt.close()
plt.plot(np.sum(N*results_r.best_estimator_.coef_, axis=1));plt.plot(y_r)
plt.title('r_dot')
plt.savefig('r.png')
plt.show()
plt.close()




# a = np.asarray([[df_main.u_dot.max(),df_main.u_dot.diff().max()],
#                 [df_main.u_dot.min(),df_main.u_dot.diff().min()],
#                 [abs(df_main.v_dot).max(),abs(df_main.v_dot.diff()).max() ],
#                 [abs(df_main.r_dot).max(),abs(df_main.r_dot.diff()).max() ]])
a = np.asarray([[df_main.u_dot.quantile(0.75),df_main.u_dot.diff().quantile(0.75)],
                    [df_main.u_dot.quantile(0.25),df_main.u_dot.diff().quantile(0.25)],
                    [abs(df_main.v_dot).quantile(0.75),abs(df_main.v_dot.diff()).quantile(0.75) ],
                    [abs(df_main.r_dot).quantile(0.75),abs(df_main.r_dot.diff()).quantile(0.75) ]])

np.savetxt("acc_limits.csv", a, delimiter=",", fmt='%s')




a_list = [list(results_x.best_estimator_.coef_[0]),list(results_y.best_estimator_.coef_[0]),list(results_r.best_estimator_.coef_[0])]
row_lengths = []

for row in a_list:
    row_lengths.append(len(row))

max_length = max(row_lengths)

for row in a_list:
    while len(row) < max_length:
        row.append(None)

balanced_array = np.array([np.asarray(a_list[0]),np.asarray(a_list[1]),np.asarray(a_list[2])])
# np.savetxt("foo_evo_general.csv", balanced_array, delimiter=",", fmt='%s')

# df_main.to_csv('test_1_1_large.csv', index =False)


