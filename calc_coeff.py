import numpy as np
import pandas as pd
from numpy import newaxis
from sklearn.linear_model import Lasso
from sklearn.linear_model import Ridge

from sklearn.linear_model import LassoCV
from sklearn.linear_model import RidgeCV
from sklearn.model_selection import RepeatedKFold

from sklearn.model_selection import GridSearchCV

from ship_class import ship
import matplotlib.pyplot as plt
ship = ship()
df_main = pd.read_csv('test_sunday_evening.csv', sep=',')
# df_main = df_main[30400:31600].reset_index(inplace=False)
# df_main.timestamp = pd.to_datetime(df_main.timestamp, format='%Y-%m-%d %H:%M:%S.%f')
# df_main.rsa = df_main.rsa*0.88

df_main.rpm = df_main.rpm/60.0
df_main = df_main.dropna()
df_main = df_main[20:700]
# df_main['beta'] = np.degrees(np.arctan((1-ship.w)/(0.7*np.pi*df_main.rpm*ship.D_p)))
df_main['beta'] = np.rad2deg(np.arctan((df_main.u)/(0.7*np.pi*df_main.rpm*ship.D_p)))

# df_main['beta'] = df_main.apply(lambda row: row['beta'] + 180 if row['u']>=0 and row['u']<0400 else (row['beta'] + 180 if row['u']<0 and row['rpm']<0 else (row['beta'] + 360 if row['u'] <0 and row['rpm']>=0 else row['beta'])) ,axis=1)
df_main['beta'] = df_main.beta.apply(lambda x: x-360 if x>360.0 else (x+360 if x<0.0 else x))
df_main['f_p_40'] = (1-ship.t)*ship.beta_coef(df_main.beta)*0.5*ship.rho*(((((1-ship.w)*df_main.u)**2)+ (0.7*np.pi*df_main.rpm*ship.D_p)**2))*np.pi/4*ship.D_p**2
# df_main['f_p_40'] = df_main.apply(lambda row: 0 if row['rpm']<5 and row['rpm']>-5 else row['f_p_40'], axis=1 )

df_main['u_dot_spec'] = df_main.u_dot.shift(1)
df_main = df_main[2:]

u = df_main.u.to_numpy()[:, newaxis]
v = df_main.v.to_numpy()[:, newaxis]
r = df_main.r.to_numpy()[:, newaxis]

u_dot = df_main.u_dot.to_numpy()[:,newaxis]
v_dot = df_main.v_dot.to_numpy()[:,newaxis]
r_dot = df_main.r_dot.to_numpy()[:,newaxis]

rsa = df_main.rsa.to_numpy()[:, newaxis]
f_p_40 = df_main.f_p_40.to_numpy()[:,newaxis]


u_dot_spec = df_main.u_dot_spec.to_numpy()[:, newaxis]

ones = np.ones((len(u_dot),1))

X = np.concatenate([u_dot, u*u, u*u*u, u*v, u*r, v*v, r*r, v*r, u*v*v, r*v*u, u*r*r], axis=1)
Y = np.concatenate([v_dot,v, u*v, u*r, u*u*r, u*u*v, v*v*v, r*r*r, r*r*v, v*v*r, abs(v)*v, abs(r)*v, r*abs(v), abs(r)*r], axis=1)
N = np.concatenate([r_dot,r, u*v, u*r, u*u*r, u*u*v, v*v*v, r*r*r, r*r*v, v*v*r, abs(v)*v, abs(r)*v, r*abs(v), abs(r)*r], axis=1)

F_r = -21.1* ship.A_r*u*u*np.deg2rad(rsa)
y_x = ship.Mass*(u_dot_spec-r*v)-2.0*f_p_40 - F_r*np.sin(np.deg2rad(rsa))
y_y = ship.Mass*(v_dot+r*u)-F_r*np.cos(np.radians(rsa))
y_r = ship.I_e*r_dot - F_r*ship.x_r*np.cos(np.radians(rsa))

# F_r = -21.1* ship.A_r*u*u*rsa
# y_x = ship.Mass*(u_dot_spec-r*v)-2.0*f_p_40 - F_r*np.sin(np.deg2rad(rsa))
# y_y = ship.Mass*(v_dot+r*u)-F_r*np.cos(np.radians(rsa))
# y_r = ship.I_e*r_dot - F_r*ship.x_r*np.cos(np.radians(rsa))
#%%
model = Ridge(fit_intercept=False)
cv = RepeatedKFold(n_splits=10, n_repeats=3, random_state=1)
grid = dict()
grid['alpha'] = np.arange(0, 0.0011, 0.0001)
search = GridSearchCV(model, grid, scoring='neg_mean_absolute_error', cv=cv, n_jobs=-1)
results_x = search.fit(X, y_x)

model = Ridge(fit_intercept=False)
cv = RepeatedKFold(n_splits=10, n_repeats=3, random_state=1)
grid = dict()
grid['alpha'] = np.arange(0, 0.0011, 0.0001)
search = GridSearchCV(model, grid, scoring='neg_mean_squared_error', cv=cv, n_jobs=-1)
results_y = search.fit(Y, y_y)

model = Ridge(fit_intercept=False)
cv = RepeatedKFold(n_splits=10, n_repeats=3, random_state=1)
grid = dict()
grid['alpha'] = np.arange(0, 0.000011, 0.000001)
search = GridSearchCV(model, grid, scoring='neg_mean_squared_error', cv=cv, n_jobs=-1)
results_r = search.fit(N, y_r)

print(results_x.best_estimator_.score(X,y_x), results_x.best_estimator_.alpha)
print(results_y.best_estimator_.score(Y,y_y), results_y.best_estimator_.alpha)
print(results_r.best_estimator_.score(N,y_r), results_r.best_estimator_.alpha)


plt.plot(y_x)
plt.plot(np.sum(X*results_x.best_estimator_.coef_, axis=1))
plt.show()
plt.close()
plt.plot(np.sum(Y*results_y.best_estimator_.coef_, axis=1));plt.plot(y_y)
plt.show()
plt.close()
plt.plot(np.sum(N*results_r.best_estimator_.coef_, axis=1));plt.plot(y_r)
plt.show()
plt.close()


a_list = [list(results_x.best_estimator_.coef_[0]),list(results_y.best_estimator_.coef_[0]),list(results_r.best_estimator_.coef_[0])]
row_lengths = []

for row in a_list:
    row_lengths.append(len(row))

max_length = max(row_lengths)

for row in a_list:
    while len(row) < max_length:
        row.append(None)

balanced_array = np.array([np.asarray(a_list[0]),np.asarray(a_list[1]),np.asarray(a_list[2])])
# a = np.asarray([results_x.best_estimator_.coef_,results_y.best_estimator_.coef_,results_r.best_estimator_.coef_])
np.savetxt("foo_rpa3.csv", balanced_array, delimiter=",", fmt='%s')

