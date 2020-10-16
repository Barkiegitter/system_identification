import numpy as np
import pandas as pd
from numpy import newaxis
from sklearn.linear_model import Lasso
from sklearn.linear_model import RidgeCV
from sklearn.linear_model import LassoCV

from sklearn.model_selection import GridSearchCV

from ship_class import ship
ship = ship()
df_main = pd.read_csv('test_1.csv', sep=',')
# df_main = df_main[30400:31600].reset_index(inplace=False)
df_main.timestamp = pd.to_datetime(df_main.timestamp, format='%Y-%m-%d %H:%M:%S.%f')
df_main.rpm = df_main.rpm/60.0
df_main = df_main.dropna()
df_main['beta'] = np.degrees(np.arctan((1-ship.w)/(0.7*np.pi*df_main.rpm*ship.D_p)))


df_main['beta'] = df_main.apply(lambda row: row['beta'] + 180 if row['u']>=0 and row['u']<0 else (row['beta'] + 180 if row['u']<0 and row['rpm']<0 else (row['beta'] + 360 if row['u'] <0 and row['rpm']>=0 else row['beta'])) ,axis=1)
df_main['beta'] = df_main.beta.apply(lambda x: x-360 if x>360.0 else x)
df_main['f_p_40'] = (1-ship.t)*ship.beta_coef(df_main.beta)*0.5*ship.rho*(((((1-ship.w)*df_main.u)**2)+ (0.7*np.pi*df_main.rpm*ship.D_p)**2))*np.pi/4*ship.D_p**2
df_main['f_p_40'] = df_main.apply(lambda row: 0 if row['rpm']<5 and row['rpm']>-5 else row['f_p_40'], axis=1 )

u = df_main.u.to_numpy()[:,newaxis]
v = df_main.v.to_numpy()[:,newaxis]
r = df_main.r.to_numpy()[:,newaxis]

u_dot = df_main.u_dot.to_numpy()[:,newaxis]
v_dot = df_main.v_dot.to_numpy()[:,newaxis]
r_dot = df_main.r_dot.to_numpy()[:,newaxis]

rsa = df_main.rsa.to_numpy()[:, newaxis]
f_p_40 = df_main.f_p_40.to_numpy()[:,newaxis]

# X = u uu uuu vv rr vr uvv rvu urr
# Y = v uv ur uur uuv vvv rrr rrv vvr abs(v)v abs(r)v rabs(v) abs(r)r
# N = r uv ur uur uuv vvv rrr rrv vvr abs(v)v abs(r)v rabs(v) abs(r)r
X = np.concatenate([u_dot, u*u, u*u*u, v*v, r*r, v*r, u*v*v, r*v*u, u*r*r], axis=1)
# X = np.concatenate([u_dot, u*u], axis=1)
Y = np.concatenate([v_dot, u*v, u*r, u*u*r, u*u*v, v*v*v, r*r*r, r*r*v, v*v*r, abs(v)*v, abs(r)*v, r*abs(v), abs(r)*r], axis=1)
N = np.concatenate([r_dot, u*v, u*r, u*u*r, u*u*v, v*v*v, r*r*r, r*r*v, v*v*r, abs(v)*v, abs(r)*v, r*abs(v), abs(r)*r], axis=1)

F_r = -21.1* ship.A_r*u*u*rsa
y_x = ship.Mass*(u_dot-r*v)-2*f_p_40 - F_r*np.sin(np.radians(rsa))
y_y = ship.Mass*(v_dot+r*u)-F_r*np.cos(np.radians(rsa))
y_r = ship.I_e*r_dot + F_r*ship.x_r*np.cos(np.radians(rsa))
pair = (X,y_x)
# for i in np.linspace(0.001, 1.0, num=100, endpoint=False):
# lasso = Lasso()
# parameters = {'alpha' : [1e-15, 1e-10, 1e-8, 1e-4, 1e-3, 1e-2, 1, 5, 10, 20]}
# lasso_regressor = GridSearchCV(lasso, parameters, cv=1000) #, scoring='neg_mean_squared_error'
# lasso_regressor.fit(pair[0], pair[1])
# print(lasso_regressor.best_params_)
# print(lasso_regressor.best_score_)



lasso = RidgeCV()
lasso.fit(pair[0], pair[1] )
train_score=lasso.score(pair[0], pair[1])
print(train_score)



x = np.linspace(0, 1, len(y_x) + 1)[0:-1]

import numpy as np
import matplotlib.pyplot as plt
fig, ax1 = plt.subplots()
ax2 = ax1.twinx()
ax1.plot(df_main.lon, df_main.lat, 'g-')
# ax2.plot(df, u_dot, 'b-')
ax1.set_xlabel('X data')
ax1.set_ylabel('Y1 data', color='g')
ax2.set_ylabel('Y2 data', color='b')

plt.show()




# Lasso regression
# place EOM
#
# perform regression
#
# recreate model
#
# create self iterating improving --> significant corners and rudder deflections
