import numpy as np
import pandas as pd
from numpy import newaxis
from sklearn.linear_model import Lasso
from sklearn.linear_model import Ridge
from sklearn.linear_model import RidgeCV
import matplotlib.pyplot as plt
import math


from sklearn.model_selection import RepeatedKFold
from sklearn.model_selection import GridSearchCV
##
from ship_class import ship
ship = ship()
df_main = pd.read_csv('test_1_large_turbopolyp.csv', sep=',')[2000:]
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

df_main.rpm_0 = df_main.rpm_0/600.0
df_main.rpm_1 = df_main.rpm_1/600.0
df_main.rpm_2 = df_main.rpm_2/600.0


# df_main.rsa_0 = df_main.rsa_0/100.0 - 360
# df_main.rsa_1 = df_main.rsa_1/100.0 - 360
# df_main.rsa_2 = df_main.rsa_2/100.0 - 360

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


multi = 1.0
# first engine listed experiences thrust decrease, t_21 means thrust reduction ratio due to downstream flow caused by engine 1
df_main['t_21_phi'] = df_main.apply(lambda row: thruster_interaction_coefficient(ship.x_1, ship.y_1, row['rsa_1'], 25.0, 100.0, ship.x_2, ship.y_2, row['rsa_2']), axis=1)
df_main['t_20_phi'] = df_main.apply(lambda row: thruster_interaction_coefficient(ship.x_0, ship.y_0, row['rsa_0'], 25.0, 100.0, ship.x_2, ship.y_2, row['rsa_2']), axis=1)
df_main['f_p_40_2'] = multi*1.*((1-ship.t)*ship.beta_coef(df_main.beta_2)*0.5*ship.rho*(((((1-ship.w)*df_main.u_a_2)**2)+(0.7*np.pi*df_main.rpm_2*ship.D_p)**2))*np.pi/4*ship.D_p**2)  #(1-df_main['t_21_phi'])*(1-df_main['t_20_phi'])*
##*(1-df_main['t_21_phi'])*(1-df_main['t_20_phi'])
df_main['t_12_phi'] = df_main.apply(lambda row: thruster_interaction_coefficient(ship.x_2, ship.y_2, row['rsa_2'], 25.0, 100.0, ship.x_1, ship.y_1, row['rsa_1']), axis=1)
df_main['t_10_phi'] = df_main.apply(lambda row: thruster_interaction_coefficient(ship.x_0, ship.y_0, row['rsa_0'], 25.0, 100.0, ship.x_1, ship.y_1, row['rsa_1']), axis=1)
df_main['f_p_40_1'] = multi*1.*((1-ship.t)*ship.beta_coef(df_main.beta_1)*0.5*ship.rho*(((((1-ship.w)*df_main.u_a_1)**2)+(0.7*np.pi*df_main.rpm_1*ship.D_p)**2))*np.pi/4*ship.D_p**2) #(1-df_main['t_12_phi'])*(1-df_main['t_10_phi'])*
#*(1-df_main['t_12_phi'])*(1-df_main['t_10_phi'])
df_main['t_02_phi'] = df_main.apply(lambda row: thruster_interaction_coefficient(ship.x_2, ship.y_2, row['rsa_2'], 25.0, 100.0, ship.x_0, ship.y_0, row['rsa_0']), axis=1)
df_main['t_01_phi'] = df_main.apply(lambda row: thruster_interaction_coefficient(ship.x_1, ship.y_1, row['rsa_1'], 25.0, 100.0, ship.x_0, ship.y_0, row['rsa_0']), axis=1)
df_main['f_p_40_0'] = multi*1.*((1-ship.t)*ship.beta_coef(df_main.beta_0)*0.5*ship.rho*(((((1-ship.w)*df_main.u_a_0)**2)+(0.7*np.pi*df_main.rpm_0*ship.D_p)**2))*np.pi/4*ship.D_p**2)#.rolling(20).mean()  #(1-df_main['t_02_phi'])*(1-df_main['t_01_phi'])*


# df_main['f_p_40_0'] = df_main.apply(lambda row: 0.0 if row['rpm_0']<9.0 else row['f_p_40_0'], axis=1)
# df_main['f_p_40_1'] = df_main.apply(lambda row: 0.0 if row['rpm_1']<9.0 else row['f_p_40_1'], axis=1)
# df_main['f_p_40_2'] = df_main.apply(lambda row: 0.0 if row['rpm_2']<9.0 else row['f_p_40_2'], axis=1)

# (1-df_main['t_02_phi'])*(1-df_main['t_01_phi'])*
# J =(1-w)*u/(n_p*D_p);                                                               % Advance ratio of the propeller   J =(1-w)*u/(n_p*D_p)
#     F_r = - 577 * A_r * (u*1.3)^2 * sin(deg2rad(delta));                                      % Rudder force with respect to speed and rudder angle

#     F_p = (1-t)*rho * n_p^2 * D_p^4 * interp1(K_t(:,1),K_t(:,2),J,'linear','extrap');    % Propeller force with respect to speed and propeller rpm

# df_main['t_21_phi'] = df_main.apply(lambda row: thruster_interaction_coefficient(ship.x_1, ship.y_1, row['rsa_1'], 25.0, 100.0, ship.x_2, ship.y_2, row['rsa_2']), axis=1)

# df_main = df_main[abs(df_main.rsa_0.diff())<20.]
# df_main = df_main[abs(df_main.rsa_1.diff())<20.]

# df_main = df_main[abs(df_main.rsa_2.diff())<20.]


#slide u_dot

df_main['u_dot_spec'] = df_main.u_dot.shift(1)
df_main['v_dot_spec'] = df_main.v_dot.shift(1)
df_main['r_dot_spec'] = df_main.r_dot.shift(1)
df_main['r_dot_spec_1'] = df_main.r_dot.rolling(10).mean()

####trail


# df_main = df_main[df_main.u>0.0]

df_main = df_main[20:-7]
# plt.show()

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

# az_speed_0 = df_main.az_speed_0.to_numpy()[:, newaxis]
# az_speed_1 = df_main.az_speed_1.to_numpy()[:, newaxis]
# az_speed_2 = df_main.az_speed_2.to_numpy()[:, newaxis]

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
X = np.concatenate([u_dot,u, u*u,u*u*u, v*v, r*r, v*r, u*v*v, r*v*u, u*r*r, 
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
cv = RepeatedKFold(n_splits=5, n_repeats=10, random_state=25)
grid = dict()
grid['alpha'] = np.logspace(0.0, -4.0, num=200)
search = GridSearchCV(model, grid, scoring='neg_mean_squared_error', cv=cv, n_jobs=-1)
results_x = search.fit(X, y_x)
one_mse_value = search.cv_results_['mean_test_score'][-1] - search.cv_results_['mean_test_score'].std()*1.5
a = [i for i in range(len(search.cv_results_['mean_test_score'])) if search.cv_results_['mean_test_score'][i] > one_mse_value]
# search.param_grid['alpha'][0][int(a[0])]
clf_x = Ridge(alpha=search.param_grid['alpha'][int(a[0])])
clf_x.fit(X, y_x)
# clf.score(X, y_x)


model = Ridge(fit_intercept=False)
cv = RepeatedKFold(n_splits=5, n_repeats=10, random_state=25)
grid = dict()
grid['alpha'] = np.logspace(1.0, -4.0, num=200) #verandert
search = GridSearchCV(model, grid, scoring='neg_mean_squared_error', cv=cv, n_jobs=-1)
results_x = search.fit(Y, y_y)
one_mse_value = search.cv_results_['mean_test_score'][-1] - search.cv_results_['mean_test_score'].std()*3
a = [i for i in range(len(search.cv_results_['mean_test_score'])) if search.cv_results_['mean_test_score'][i] > one_mse_value]
# search.param_grid['alpha'][0][int(a[0])]
clf_y = Ridge(alpha=search.param_grid['alpha'][int(a[0])])
clf_y.fit(Y, y_y)

model = Ridge(fit_intercept=False)
cv = RepeatedKFold(n_splits=5, n_repeats=10, random_state=25)
grid = dict()
grid['alpha'] = np.logspace(0.0, -4.0, num=200)
search = GridSearchCV(model, grid, scoring='neg_mean_squared_error', cv=cv, n_jobs=-1)
results_x = search.fit(N, y_r)
one_mse_value = search.cv_results_['mean_test_score'][-1] - search.cv_results_['mean_test_score'].std()*1.5
a = [i for i in range(len(search.cv_results_['mean_test_score'])) if search.cv_results_['mean_test_score'][i] > one_mse_value]
# search.param_grid['alpha'][0][int(a[0])]
clf_n = Ridge(alpha=search.param_grid['alpha'][int(a[0])])
clf_n.fit(N, y_r)

# calculate standard deviation

print(clf_x.score(X,y_x), clf_x.alpha)
print(clf_y.score(Y,y_y), clf_y.alpha)
print(clf_n.score(N,y_r), clf_n.alpha)


 
plt.plot(np.sum(X*clf_x.coef_, axis=1));plt.plot(y_x)
# plt.plot(*(X/y_x))
# plt.plot(df_main.f_p_40_0)
plt.title('u_dot')
plt.savefig('u.png')
plt.show()
plt.close()
plt.plot(np.sum(Y*clf_y.coef_, axis=1));plt.plot(y_y)
plt.title('v_dot')
plt.savefig('v.png')
plt.show()
plt.close()
plt.plot(np.sum(N*clf_n.coef_, axis=1));plt.plot(y_r)
plt.title('r_dot')
plt.savefig('r.png')
plt.show()
plt.close()

a = np.asarray([[df_main.u_dot.quantile(0.75),df_main.u_dot.diff().quantile(0.75)],
                    [df_main.u_dot.quantile(0.25),df_main.u_dot.diff().quantile(0.25)],
                    [abs(df_main.v_dot).quantile(0.75),abs(df_main.v_dot.diff()).quantile(0.75) ],
                    [abs(df_main.r_dot).quantile(0.75),abs(df_main.r_dot.diff()).quantile(0.75) ]])

np.savetxt("acc_limits.csv", a, delimiter=",", fmt='%s')

a_list = [list(clf_x.coef_[0]),list(clf_y.coef_[0]),list(clf_n.coef_[0])]
row_lengths = []

for row in a_list:
    row_lengths.append(len(row))

max_length = max(row_lengths)

for row in a_list:
    while len(row) < max_length:
        row.append(None)

balanced_array = np.array([np.asarray(a_list[0]),np.asarray(a_list[1]),np.asarray(a_list[2])])
np.savetxt("borkum_general_opt_.csv", balanced_array, delimiter=",", fmt='%s')


print(balanced_array)
# import numpy as np
# from scipy.optimize import curve_fit
# from scipy.integrate import odeint
# from matplotlib.pyplot import *


# Kp = 2.0    # gain
# tau = 1.0   # time constant
# zeta = 1.0 # damping factor
# theta = 0.0 # no time delay
# du = 1.0    # change in u

# def model3(x,t):
#     y = x[0]
#     dydt = x[1]
#     dy2dt2 = (-2.0*zeta*tau*dydt - y + Kp*du)/tau**2
#     return [dydt,dy2dt2]
# t3 = np.linspace(0,25,100)
# x3 = odeint(model3,[0,0],t3)

# temps = [0.0, 0.0027777777777777779, 0.0055555555555555558, 0.0083333333333333332, 0.011111111111111112, 0.013888888888888888, 0.016666666666666666, 0.019444444444444445, 0.022222222222222223, 0.025000000000000001, 0.027777777777777776, 0.030555555555555555, 0.033333333333333333, 0.036111111111111108, 0.03888888888888889, 0.041666666666666664, 0.044444444444444446, 0.047222222222222221, 0.050000000000000003, 0.052777777777777778, 0.055555555555555552, 0.058333333333333334, 0.061111111111111109, 0.063888888888888884, 0.066666666666666666, 0.069444444444444448, 0.072222222222222215, 0.074999999999999997, 0.077777777777777779, 0.080555555555555561, 0.083333333333333329, 0.08611111111111111, 0.088888888888888892, 0.09166666666666666, 0.094444444444444442, 0.097222222222222224, 0.10000000000000001, 0.10277777777777777, 0.10555555555555556, 0.10833333333333334, 0.1111111111111111, 0.11388888888888889, 0.11666666666666667, 0.11944444444444445, 0.12222222222222222, 0.125, 0.12777777777777777, 0.13055555555555556, 0.13333333333333333, 0.1361111111111111, 0.1388888888888889, 0.14166666666666666, 0.14444444444444443, 0.14722222222222223, 0.14999999999999999, 0.15277777777777779, 0.15555555555555556, 0.15833333333333333, 0.16111111111111112, 0.16388888888888889, 0.16666666666666666, 0.16944444444444445, 0.17222222222222222, 0.17499999999999999, 0.17777777777777778, 0.18055555555555555, 0.18333333333333332, 0.18611111111111112, 0.18888888888888888, 0.19166666666666668, 0.19444444444444445, 0.19722222222222222, 0.20000000000000001, 0.20277777777777778, 0.20555555555555555, 0.20833333333333334, 0.21111111111111111, 0.21388888888888888, 0.21666666666666667, 0.21944444444444444, 0.22222222222222221, 0.22500000000000001, 0.22777777777777777, 0.23055555555555557, 0.23333333333333334, 0.2361111111111111, 0.2388888888888889, 0.24166666666666667, 0.24444444444444444, 0.24722222222222223, 0.25, 0.25277777777777777, 0.25555555555555554, 0.25833333333333336, 0.26111111111111113, 0.2638888888888889, 0.26666666666666666, 0.26944444444444443, 0.2722222222222222, 0.27500000000000002, 0.27777777777777779, 0.28055555555555556, 0.28333333333333333, 0.28611111111111109, 0.28888888888888886, 0.29166666666666669, 0.29444444444444445, 0.29722222222222222, 0.29999999999999999, 0.30277777777777776, 0.30555555555555558, 0.30833333333333335, 0.31111111111111112, 0.31388888888888888, 0.31666666666666665, 0.31944444444444442, 0.32222222222222224, 0.32500000000000001, 0.32777777777777778, 0.33055555555555555]
# conc = [0.4666023864047246, 0.28480173556707244, 0.2750391707846209, 0.26141979028564544, 0.2684343738700735, 0.2639026154031578, 0.25474267807641315, 0.2539231047366518, 0.25503193925515244, 0.25652645534530555, 0.24845124743883332, 0.24912619018922502, 0.25131975412799806, 0.25804507653368686, 0.25442931179944556, 0.2469085211522237, 0.2546944678799566, 0.2526455345305532, 0.2563818247559359, 0.24444980113293963, 0.24956008195733398, 0.2501386043148126, 0.245534530553212, 0.25025912980595394, 0.2400867783536218, 0.2414607689526335, 0.2443774858382548, 0.25592382788959867, 0.2495359768591057, 0.24218392189948174, 0.2471013619380499, 0.2592021212486441, 0.24912619018922502, 0.2466915752681692, 0.2396769916837411, 0.2452452693744727, 0.24225623719416656, 0.23856815716524044, 0.24413643485597203, 0.24818609135832229, 0.24158129444377485, 0.24432927564179824, 0.23591659636013015, 0.24237676268530794, 0.23434976497529228, 0.24172592503314452, 0.23444618536820538, 0.23960467638905628, 0.24630589369651681, 0.2388092081475232, 0.2378208991201639, 0.23186693985777992, 0.23331324575147644, 0.23972520188019766, 0.24162950464023142, 0.23786910931662048, 0.23753163794142462, 0.23478365674340124, 0.23560323008316258, 0.24269012896227551, 0.2352175485115102, 0.23548270459202122, 0.2452452693744727, 0.2366638544052067, 0.2411474026756659, 0.23883331324575147, 0.24444980113293963, 0.24519705917801615, 0.24584789683017957, 0.24459443172230927, 0.24242497288176448, 0.24575147643726647, 0.2484753525370616, 0.25025912980595394, 0.24637820899120164, 0.2376280583343377, 0.24399180426660239, 0.2400867783536218, 0.24447390623116788, 0.2370495359768591, 0.23786910931662048, 0.2399662528624804, 0.2315053633843558, 0.23172230926841028, 0.23473544654694467, 0.2393877305050018, 0.24457032662408099, 0.24150897914909003, 0.22911895865975654, 0.2367602747981198, 0.23222851633120406, 0.23649511871760878, 0.22803422923948416, 0.23324093045679162, 0.23418102928769435, 0.23348198143907437, 0.22543087863083042, 0.23222851633120406, 0.23697722068217428, 0.22699771001566832, 0.23721827166445703, 0.23155357358081236, 0.23172230926841028, 0.22928769434735446, 0.23475955164517295, 0.23625406773532603, 0.22938411474026757, 0.22810654453416898, 0.22299626370977463, 0.23030010847294202, 0.23439797517174882, 0.22697360491744004, 0.2276967578642883, 0.2276967578642883, 0.22928769434735446, 0.22449077979992768, 0.22608171628299387, 0.23097505122333373, 0.2246595154875256, 0.23232493672411714] 


# temps = np.array(temps)
# conc = np.array(conc)

# #popt, pcov = curve_fit(S, temps, conc,[1, 1, 1, 0.47], maxfev=3000)
# popt, pcov = curve_fit(S, temps, conc,[1, 2, 1, 0.47], maxfev=3000)
# #popt, pcov = curve_fit(S, temps, conc, maxfev=109000)
# print(popt)

# #plot the data
# clf() #matplotlib
# plot(temps, conc, "gp")
# plot(temps, S(temps, popt[0], popt[1], popt[2], popt[3]))

# grid(True)
# show()




