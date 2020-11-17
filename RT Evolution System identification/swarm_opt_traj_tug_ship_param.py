import random
import math
import matplotlib.pyplot as plt
#------------------------------------------------------------------------------
# TO CUSTOMIZE THIS PSO CODE TO SOLVE UNCONSTRAINED OPTIMIZATION PROBLEMS, CHANGE THE PARAMETERS IN THIS SECTION ONLY:
# THE FOLLOWING PARAMETERS MUST BE CHANGED.



#-------------- objective function
# def objective_function(x):
#     y = 3*(1-ship_it.D_p)**2*math.exp(-ship_it.D_p**2 - (x[1]+1)**2) - 10*(ship_it.D_p/5 - ship_it.D_p**3 - x[1]**5)*math.exp(-ship_it.D_p**2 - x[1]**2) -1/3*math.exp(-(ship_it.D_p+1)**2 - x[1]**2);
#     return y

import numpy as np
import pandas as pd
from ship_class import ship
from manoeuvre_model_evo import ship_model
import matplotlib.pyplot as plt
from sklearn.linear_model import Ridge,Lasso
from numpy import newaxis
from sklearn.model_selection import RepeatedKFold
from sklearn.model_selection import GridSearchCV

df_main = pd.read_csv('test_1_large.csv', sep=',')

def traj_error(x):
    ship_it = ship()
    ship_it.D_p = ship_it.D_p*x[0]
    df_main = pd.read_csv('test_1_large.csv', sep=',')

    ship_it.I_e = x[1]*ship_it.I_e
    ship_it.Mass = x[2]*ship_it.Mass
    print(x[0],'a')
    # ship_it.Mass = ship_it.Mass*x[1]
    
    df = df_main[20:-20]
    df.rpm_0 = df.rpm_0/60.0*x[3]
    df.rpm_1 = df.rpm_1/60.0*x[3]
    df.rpm_2 = df.rpm_2/60.0*x[3]
    
    
    df = df.dropna()

    df['az_speed_0'] = df.rsa_0.diff()
    df['az_speed_1'] = df.rsa_1.diff()
    df['az_speed_2'] = df.rsa_2.diff()
    
    # df.az_speed_0 = df.az_speed_0.apply(lambda x: x-360. if x>360. else ())
    
    

    #azimuth 2 port side
    df['u_a_2'] = (1-ship_it.w)*((-df.u+df.r*abs(ship_it.y_2))*np.cos(np.deg2rad(df.rsa_2)) + (-df.v+df.r*abs(ship_it.x_2))*np.sin(np.deg2rad(df.rsa_2))) #(1-ship_it.w)*
    df['u_a_1'] = (1-ship_it.w)*((-df.u-df.r*abs(ship_it.y_1))*np.cos(np.deg2rad(df.rsa_1)) + (-df.v+df.r*abs(ship_it.x_1))*np.sin(np.deg2rad(df.rsa_1))) #(1-ship_it.w)*
    df['u_a_0'] = (1-ship_it.w)*((df.u)*+1*np.cos(np.deg2rad(df.rsa_0)) + ((-df.v - df.r*abs(ship_it.x_0))*np.sin(np.deg2rad(df.rsa_0))) ) #(1-ship_it.w)*
    
    
    # df['u_a_2'] =  df.u
    # df['u_a_1'] =  df.u
    # df['u_a_0'] =  df.u
    
    df['beta_2'] = np.rad2deg(np.arctan((df.u_a_2)/(0.7*np.pi*df.rpm_2*ship_it.D_p)))
    #change invalse hoek acoording to engine stand
    # df['beta_2'] = df.apply(lambda row: row['beta_2'] + 180 if row['u_a_2']>=0  else (row['beta_2'] + 0 if row['u_a_2']<0 and row['rsa_2']<90 or row['rsa_2']>270. else (row['beta_2'] + 180 if row['u_a_2'] <0 and 90.<row['rsa_2']<270. else row['beta_2'])) ,axis=1)
    # df['beta_2'] = df.beta_2.apply(lambda x: x-360 if x>360 else x)
    df['beta_2'] = df.beta_2.apply(lambda x: x+360 if x<0 else x)
    
    df['beta_1'] = np.rad2deg(np.arctan((df.u_a_1)/(0.7*np.pi*df.rpm_1*ship_it.D_p)))
    # df['beta_1'] = df.apply(lambda row: row['beta_1'] + 180 if row['u_a_1']>=0 and row['rsa_1']<90 or row['rsa_1']>270. else (row['beta_1'] + 180 if row['u_a_1']<0 and row['rsa_1']<90 or row['rsa_1']>270. else (row['beta_1'] + 360  if row['u_a_1'] <0 and 90.<row['rsa_1']<270. else row['beta_1'])) ,axis=1)
    # df['beta_1'] = df.beta_1.apply(lambda x: x-360 if x>360 else x)
    df['beta_1'] = df.beta_1.apply(lambda x: x+360 if x<0 else x)
    
    df['beta_0'] = np.rad2deg(np.arctan((df.u_a_0)/(0.7*np.pi*df.rpm_0*ship_it.D_p)))
    # df['beta_0'] = df.apply(lambda row: row['beta_0'] + 180 if row['u_a_0']>=0 and row['rsa_0']<90 or row['rsa_0']>270. else (row['beta_0'] + 180 if row['u_a_0']<0 and row['rsa_0']<90 or row['rsa_0']>270. else (row['beta_0'] + 360 if row['u_a_0'] <0 and 90.<row['rsa_0']<270. else row['beta_0'])) ,axis=1)
    # df['beta_0'] = df.beta_0.apply(lambda x: x-360 if x>360 else x)
    df['beta_0'] = df.beta_0.apply(lambda x: x+360 if x<0 else x)
    
    # first engine listed experiences thrust decrease, t_21 means thrust reduction ratio due to downstream flow caused by engine 1
    # df['t_21_phi'] = df.apply(lambda row: thruster_interaction_coefficient(ship_it.x_1, ship_it.y_1, row['rsa_1'], 25.0, 100.0, ship_it.x_2, ship_it.y_2, row['rsa_2']), axis=1)
    # df['t_20_phi'] = df.apply(lambda row: thruster_interaction_coefficient(ship_it.x_0, ship_it.y_0, row['rsa_0'], 25.0, 100.0, ship_it.x_2, ship_it.y_2, row['rsa_2']), axis=1)
    df['f_p_40_2'] = 1.0*((1-ship_it.t)*ship_it.beta_coef(df.beta_2)*0.5*ship_it.rho*(((((1-ship_it.w)*df.u_a_2)**2)+(0.7*np.pi*df.rpm_2*ship_it.D_p)**2))*np.pi/4*ship_it.D_p**2)  #(1-df['t_21_phi'])*(1-df['t_20_phi'])*
    ##*(1-df['t_21_phi'])*(1-df['t_20_phi'])
    # df['t_12_phi'] = df.apply(lambda row: thruster_interaction_coefficient(ship_it.x_2, ship_it.y_2, row['rsa_2'], 25.0, 100.0, ship_it.x_1, ship_it.y_1, row['rsa_1']), axis=1)
    # df['t_10_phi'] = df.apply(lambda row: thruster_interaction_coefficient(ship_it.x_0, ship_it.y_0, row['rsa_0'], 25.0, 100.0, ship_it.x_1, ship_it.y_1, row['rsa_1']), axis=1)
    df['f_p_40_1'] = 1.0*((1-ship_it.t)*ship_it.beta_coef(df.beta_1)*0.5*ship_it.rho*(((((1-ship_it.w)*df.u_a_1)**2)+(0.7*np.pi*df.rpm_1*ship_it.D_p)**2))*np.pi/4*ship_it.D_p**2) #(1-df['t_12_phi'])*(1-df['t_10_phi'])*
    #*(1-df['t_12_phi'])*(1-df['t_10_phi'])
    # df['t_02_phi'] = df.apply(lambda row: thruster_interaction_coefficient(ship_it.x_2, ship_it.y_2, row['rsa_2'], 25.0, 100.0, ship_it.x_0, ship_it.y_0, row['rsa_0']), axis=1)
    # df['t_01_phi'] = df.apply(lambda row: thruster_interaction_coefficient(ship_it.x_1, ship_it.y_1, row['rsa_1'], 25.0, 100.0, ship_it.x_0, ship_it.y_0, row['rsa_0']), axis=1)
    df['f_p_40_0'] = 1.0*((1-ship_it.t)*ship_it.beta_coef(df.beta_0)*0.5*ship_it.rho*(((((1-ship_it.w)*df.u_a_0)**2)+(0.7*np.pi*df.rpm_0*ship_it.D_p)**2))*np.pi/4*ship_it.D_p**2)  #(1-df['t_02_phi'])*(1-df['t_01_phi'])*
    # 
    # (1-df['t_02_phi'])*(1-df['t_01_phi'])*
    # J =(1-w)*u/(n_p*D_p);                                                               % Advance ratio of the propeller   J =(1-w)*u/(n_p*D_p)
    #     F_r = - 577 * A_r * (u*1.3)^2 * sin(deg2rad(delta));                                      % Rudder force with respect to speed and rudder angle
    
    #     F_p = (1-t)*rho * n_p^2 * D_p^4 * interp1(K_t(:,1),K_t(:,2),J,'linear','extrap');    % Propeller force with respect to speed and propeller rpm
    
    # df['t_21_phi'] = df.apply(lambda row: thruster_interaction_coefficient(ship_it.x_1, ship_it.y_1, row['rsa_1'], 25.0, 100.0, ship_it.x_2, ship_it.y_2, row['rsa_2']), axis=1)
    
    
    #slide u_dot
    
    df['u_dot_spec'] = df.u_dot.shift(1)
    df['v_dot_spec'] = df.v_dot.shift(1)
    df['r_dot_spec'] = df.r_dot.shift(1)
    
    # df = df[df.u>0.0]
    
    # print(df[['u_dot']].head(100))
    df = df.dropna()

    u = df.u.to_numpy()[:, newaxis]
    v = df.v.to_numpy()[:, newaxis]
    r = df.r.to_numpy()[:, newaxis]
    
    u_dot_spec = df.u_dot_spec.to_numpy()[:, newaxis]
    u_dot = df.u_dot.to_numpy()[:, newaxis]
    
    v_dot = df.v_dot.to_numpy()[:, newaxis]
    v_dot_spec = df.v_dot_spec.to_numpy()[:, newaxis]
    
    
    r_dot = df.r_dot.to_numpy()[:, newaxis]
    r_dot_spec = df.r_dot_spec.to_numpy()[:, newaxis]
    
    
    rsa_0 = df.rsa_0.to_numpy()[:, newaxis]
    rsa_1 = df.rsa_1.to_numpy()[:, newaxis]
    rsa_2 = df.rsa_2.to_numpy()[:, newaxis]
    
    az_speed_0 = df.az_speed_0.to_numpy()[:, newaxis]
    az_speed_1 = df.az_speed_1.to_numpy()[:, newaxis]
    az_speed_2 = df.az_speed_2.to_numpy()[:, newaxis]
    
    f_p_40_0 = df.f_p_40_0.to_numpy()[:, newaxis]
    f_p_40_2 = df.f_p_40_2.to_numpy()[:, newaxis]
    f_p_40_1 = df.f_p_40_1.to_numpy()[:, newaxis]
    
    # X = u uu uuu vv rr vr uvv rvu urr
    # Y = v uv ur uur uuv vvv rrr rrv vvr abs(v)v abs(r)v rabs(v) abs(r)r
    # N = r uv ur uur uuv vvv rrr rrv vvr abs(v)v abs(r)v rabs(v) abs(r)r
    X = np.concatenate([u_dot, u, u*u, u*u*u, v*v, r*r, v*r, u*v*v, r*v*u, u*r*r], axis=1)
    Y = np.concatenate([v_dot, v, v*v, u*v, u*r, u*u*r, u*u*v, v*v*v, r*r*r, r*r*v, v*v*r, abs(v)*v, abs(r)*v, r*abs(v), abs(r)*r], axis=1)
    N = np.concatenate([r_dot, r, r*r, v*r, u*r, u*u*r, u*u*v, v*v*v, r*r*r, r*r*v, v*v*r, abs(v)*v, abs(r)*v, r*abs(v), abs(r)*r ], axis=1)
    
    y_x = ship_it.Mass*(u_dot-r*v)+1*(np.cos(np.deg2rad(rsa_0))*(f_p_40_0)+np.cos(np.deg2rad(rsa_1))*(f_p_40_1)+np.cos(np.deg2rad(rsa_2))*(f_p_40_2))
    y_y = ship_it.Mass*(v_dot+r*u)+1*(np.sin(np.deg2rad(rsa_0))*(f_p_40_0)+np.sin(np.deg2rad(rsa_1))*(f_p_40_1)+np.sin(np.deg2rad(rsa_2))*(f_p_40_2)) #np.sin(rsa_0)*abs(f_p_40_0)+np.sin(rsa_1)*abs(f_p_40_1)+
    y_r = ship_it.I_e*r_dot + 1*(abs(ship_it.x_0)*np.sin(np.deg2rad(rsa_0))*(f_p_40_0) - abs(ship_it.x_2)*np.sin(np.deg2rad(rsa_0))*(f_p_40_2) - (ship_it.x_1)*np.sin(np.deg2rad(rsa_1))*(f_p_40_1) - abs(ship_it.y_2)*np.cos(np.deg2rad(rsa_2))*(f_p_40_2) + abs(ship_it.y_1)*np.cos(np.deg2rad(rsa_1))*(f_p_40_1))
    
    model = Lasso(fit_intercept=False)
    cv = RepeatedKFold(n_splits=5, n_repeats=3, random_state=1)
    grid = dict()
    grid['alpha'] = np.arange(0.001, 0.011, 0.001)
    search = GridSearchCV(model, grid, scoring='neg_mean_absolute_error', cv=cv, n_jobs=-1)
    results_x = search.fit(X, y_x)
    
    model = Lasso(fit_intercept=False)
    cv = RepeatedKFold(n_splits=5, n_repeats=3, random_state=1)
    grid = dict()
    grid['alpha'] = np.arange(0.001, 0.011, 0.001)
    search = GridSearchCV(model, grid, scoring='neg_mean_absolute_error', cv=cv, n_jobs=-1)
    results_y = search.fit(Y, y_y)
    
    model = Lasso(fit_intercept=False)
    cv = RepeatedKFold(n_splits=10, n_repeats=3, random_state=1)
    grid = dict()
    grid['alpha'] = np.arange(0.0000001, 0.0000011, 0.0000001)
    search = GridSearchCV(model, grid, scoring='neg_mean_absolute_error', cv=cv, n_jobs=-1)
    results_r = search.fit(N, y_r)
    
    print(results_x.best_estimator_.score(X,y_x), results_x.best_estimator_.alpha)
    print(results_y.best_estimator_.score(Y,y_y), results_y.best_estimator_.alpha)
    print(results_r.best_estimator_.score(N,y_r), results_r.best_estimator_.alpha)
    
    # acc_lim = np.asarray([[df.u_dot.max(),df.u_dot.diff().max()],
    #                 [df.u_dot.min(),df.u_dot.diff().min()],
    #                 [abs(df.v_dot).max(),abs(df.v_dot.diff()).max() ],
    #                 [abs(df.r_dot).max(),abs(df.r_dot.diff()).max() ]])
    
    acc_lim = np.asarray([[df_main.u_dot.quantile(0.95),df_main.u_dot.diff().quantile(0.95)],
                    [df_main.u_dot.quantile(0.05),df_main.u_dot.diff().quantile(0.05)],
                    [abs(df.v_dot).quantile(0.95),abs(df.v_dot.diff()).quantile(0.95) ],
                    [abs(df.r_dot).quantile(0.95),abs(df.r_dot.diff()).quantile(0.95) ]])
    
    
    a_list = [list(results_x.best_estimator_.coef_),list(results_y.best_estimator_.coef_),list(results_r.best_estimator_.coef_)]
    row_lengths = []
    
    for row in a_list:
        row_lengths.append(len(row))
    
    max_length = max(row_lengths)
    
    for row in a_list:
        while len(row) < max_length:
            row.append(None)
    
    coef_ = np.array([np.asarray(a_list[0]),np.asarray(a_list[1]),np.asarray(a_list[2])])
    df = df_main[20:1000]
    u, v, r, hdg = df.loc[df.index[0], 'u'],df.loc[df.index[0], 'v'], df.loc[df.index[0], 'r'], df.loc[df.index[0], 'hdg']
    ship_it_model = ship_model(df.loc[df.index[0], 'u_dot'],df.loc[df.index[0], 'v_dot'], df.loc[df.index[0], 'r_dot'], ship_it, coef_, acc_lim)
    print(df.rpm_0[:100])
    df_sim = pd.DataFrame([])
    for i in df[:-1].index:
        u, v, r, hdg, delta_x_0, delta_y_0, delta_r_0, u_dot, v_dot, r_dot = ship_it_model.manoeuvre_model_rt_evolution(u, v, r, hdg,
                                                                                                       df.loc[i, 'rpm_0'], df.loc[i, 'rpm_1'], df.loc[i, 'rpm_2'],
                                                                                                       df.loc[i, 'rsa_0'], df.loc[i, 'rsa_1'], df.loc[i, 'rsa_2'],
                                                                                                       df.loc[i, 'delta_time']
                                                                                                       )
        # print(u, v, r, hdg)
        df_temp = pd.DataFrame({
                            'index_sim' : [i+1],
                            'x_delta_sim': [delta_x_0],
                            'y_delta_sim': [delta_y_0],
                            'hdg_delta_sim': [delta_r_0]
                                            })
        df_sim = pd.concat([df_sim, df_temp], axis=0)
        
    df = pd.merge(df, df_sim, how='left', left_on=df.index, right_on='index_sim')
    
    df['x_real_sim'] = df.x_delta_sim.cumsum()
    df['y_real_sim'] = df.y_delta_sim.cumsum()
    df['psi_sim'] = df.hdg_delta_sim.cumsum()
    
    df['x_real'] = df.x.cumsum()
    df['y_real'] = df.y.cumsum()
        
    df['traj_error'] = (np.sqrt((df['x_real_sim'] - df['x_real'])**2 + (df['y_real_sim'] - df['y_real'])**2)).cumsum()
    df['x_sim_diff_avg'] = df['x_real_sim'] - df.x_delta_sim.mean()
    df['y_sim_diff_avg'] = df['y_real_sim'] - df.y_delta_sim.mean()
    
    df['x_real_diff_avg'] = df['x_real'] - df.x.mean()
    df['y_real_diff_avg'] = df['y_real'] - df.y.mean()
    
    rho_x = (df['x_sim_diff_avg']*df['x_real_diff_avg']).sum()/np.sqrt((df['x_sim_diff_avg']**2).sum()*(df['x_real_diff_avg']**2).sum() )
    rho_y = (df['y_sim_diff_avg']*df['y_real_diff_avg']).sum()/np.sqrt((df['y_sim_diff_avg']**2).sum()*(df['y_real_diff_avg']**2).sum() )
        
    plt.plot(df.x_real.tolist()[:],df.y_real.tolist()[:])
    plt.plot(df.x_real_sim.tolist()[:],df.y_real_sim.tolist()[:])
    # plt.plot(df.traj_error)
    plt.show()
    print(rho_x*rho_y)
    
    del df
    return rho_x*rho_y

    
#------------- end objective function 
    
bounds=[(0.8,1.2),(0.8,1.2),(0.8,1.2),(0.8,1.2)]   # upper and lower bounds of variables
nv = 4                # number of variables
mm = 1                   # if minimization problem, mm = -1; if maximization problem, mm = 1
 
# THE FOLLOWING PARAMETERS ARE OPTINAL.
particle_size=10    # number of particles
iterations= 10     # max number of iterations
w=0.85                    # inertia constant
c1=1                    # cognative constant   #research
c2=2                     # social constant     # research
# END OF THE CUSTOMIZATION SECTION
#------------------------------------------------------------------------------    
class Particle:
    def __init__(self,bounds):
        self.particle_position=[]                     # particle position
        self.particle_velocity=[]                     # particle velocity
        self.local_best_particle_position=[]          # best position of the particle
        self.fitness_local_best_particle_position= initial_fitness  # initial objective function value of the best particle position
        self.fitness_particle_position=initial_fitness             # objective function value of the particle position
 
        for i in range(nv):
            self.particle_position.append(random.uniform(bounds[i][0],bounds[i][1])) # generate random initial position
            self.particle_velocity.append(random.uniform(-1,1)) # generate random initial velocity
 
    def evaluate(self,objective_function):
        self.fitness_particle_position=objective_function(self.particle_position)
        if mm == -1:
            if self.fitness_particle_position < self.fitness_local_best_particle_position:
                self.local_best_particle_position=self.particle_position                  # update the local best
                self.fitness_local_best_particle_position=self.fitness_particle_position  # update the fitness of the local best
        if mm == 1:
            if self.fitness_particle_position > self.fitness_local_best_particle_position:
                self.local_best_particle_position=self.particle_position                  # update the local best
                self.fitness_local_best_particle_position=self.fitness_particle_position  # update the fitness of the local best
 
    def update_velocity(self,global_best_particle_position):
        for i in range(nv):
            r1=random.random()
            r2=random.random()
 
            cognitive_velocity = c1*r1*(self.local_best_particle_position[i] - self.particle_position[i])
            social_velocity = c2*r2*(global_best_particle_position[i] - self.particle_position[i])
            self.particle_velocity[i] = w*self.particle_velocity[i]+ cognitive_velocity + social_velocity
 
    def update_position(self,bounds):
        for i in range(nv):
            self.particle_position[i]=self.particle_position[i]+self.particle_velocity[i]
 
            # check and repair to satisfy the upper bounds
            if self.particle_position[i]>bounds[i][1]:
                self.particle_position[i]=bounds[i][1]
            # check and repair to satisfy the lower bounds
            if self.particle_position[i] < bounds[i][0]:
                self.particle_position[i]=bounds[i][0]
                 
class PSO():
    def __init__(self,objective_function,bounds,particle_size,iterations):
 
        fitness_global_best_particle_position=initial_fitness
        global_best_particle_position=[]
 
        swarm_particle=[]
        for i in range(particle_size):
            swarm_particle.append(Particle(bounds))
        A=[]
         
        for i in range(iterations):
            for j in range(particle_size):
                swarm_particle[j].evaluate(objective_function)
 
                if mm ==-1:
                    if swarm_particle[j].fitness_particle_position < fitness_global_best_particle_position:
                        global_best_particle_position = list(swarm_particle[j].particle_position)
                        fitness_global_best_particle_position = float(swarm_particle[j].fitness_particle_position)
                if mm ==1:
                    if swarm_particle[j].fitness_particle_position > fitness_global_best_particle_position:
                        global_best_particle_position = list(swarm_particle[j].particle_position)
                        fitness_global_best_particle_position = float(swarm_particle[j].fitness_particle_position)
            for j in range(particle_size):
                swarm_particle[j].update_velocity(global_best_particle_position)
                swarm_particle[j].update_position(bounds)
                 
            A.append(fitness_global_best_particle_position) # record the best fitness
        print('Optimal solution:', global_best_particle_position)
        print('Objective function value:', fitness_global_best_particle_position)
        print('Evolutionary process of the objective function value:')
        plt.plot(A)
        
        
# change objective function-->fitting score
# 

#------------------------------------------------------------------------------
if mm == -1:
    initial_fitness = float("inf") # for minimization problem
if mm == 1:
    initial_fitness = -float("inf") # for maximization problem
#------------------------------------------------------------------------------   
# Main PSO         
PSO(traj_error,bounds,particle_size,iterations)
