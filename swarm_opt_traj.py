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
from manoeuvre_model_rpa3 import ship_model
import matplotlib.pyplot as plt
from sklearn.linear_model import Ridge
from numpy import newaxis
from sklearn.model_selection import RepeatedKFold
from sklearn.model_selection import GridSearchCV

df_main = pd.read_csv('test_sunday_evening.csv', sep=',')

def traj_error(x):
    ship_it = ship()
    ship_it.D_p = x[0]
    df_main = pd.read_csv('test_sunday_evening.csv', sep=',')

    ship_it.I_e = x[1]*ship_it.I_e
    print(x[0])
    # ship_it.Mass = ship_it.Mass*x[1]
    
    df = df_main[20:800]
    df.rpm = df.rpm/60.0*x[2]
    df = df.dropna()
    # df.rsa = df.rsa*x[2]
    # df['beta'] = np.degrees(np.arctan((1-ship_it.w)/(0.7*np.pi*df.rpm*ship_it.D_p)))
    df['beta'] = np.rad2deg(np.arctan((df.u)/(0.7*np.pi*df.rpm*ship_it.D_p)))
    
    # df['beta'] = df.apply(lambda row: row['beta'] + 180 if row['u']>=0 and row['u']<0400 else (row['beta'] + 180 if row['u']<0 and row['rpm']<0 else (row['beta'] + 360 if row['u'] <0 and row['rpm']>=0 else row['beta'])) ,axis=1)
    df['beta'] = df.beta.apply(lambda x: x-360 if x>360.0 else (x+360 if x<0.0 else x))
    df['f_p_40'] = (1-ship_it.t)*ship_it.beta_coef(df.beta)*0.5*ship_it.rho*(((((1-ship_it.w)*df.u)**2)+ (0.7*np.pi*df.rpm*ship_it.D_p)**2))*np.pi/4*ship_it.D_p**2
    # df['f_p_40'] = df.apply(lambda row: 0 if row['rpm']<5 and row['rpm']>-5 else row['f_p_40'], axis=1 )
    df['u_dot_spec'] = df.u_dot.shift(1)
    df = df[2:]
    
    u = df.u.to_numpy()[:, newaxis]
    v = df.v.to_numpy()[:, newaxis]
    r = df.r.to_numpy()[:, newaxis]
    
    u_dot = df.u_dot.to_numpy()[:,newaxis]
    v_dot = df.v_dot.to_numpy()[:,newaxis]
    r_dot = df.r_dot.to_numpy()[:,newaxis]
    
    rsa = df.rsa.to_numpy()[:, newaxis]
    f_p_40 = df.f_p_40.to_numpy()[:,newaxis]
    
    
    u_dot_spec = df.u_dot_spec.to_numpy()[:, newaxis]
        
    X = np.concatenate([u_dot_spec, u*u, u*u*u, u*v, u*r, v*v, r*r, v*r, u*v*v, r*v*u, u*r*r], axis=1)
    Y = np.concatenate([v_dot,v, u*v, u*r, u*u*r, u*u*v, v*v*v, r*r*r, r*r*v, v*v*r, abs(v)*v, abs(r)*v, r*abs(v), abs(r)*r], axis=1)
    N = np.concatenate([r_dot,r, u*v, u*r, u*u*r, u*u*v, v*v*v, r*r*r, r*r*v, v*v*r, abs(v)*v, abs(r)*v, r*abs(v), abs(r)*r], axis=1)
    
    F_r = -21.1* ship_it.A_r*u*u*rsa
    y_x = ship_it.Mass*(u_dot_spec-r*v)-1.5*f_p_40 - F_r*np.sin(np.deg2rad(rsa))
    y_y = ship_it.Mass*(v_dot+r*u)-F_r*np.cos(np.radians(rsa))
    y_r = ship_it.I_e*r_dot - F_r*ship_it.x_r*np.cos(np.radians(rsa))
    
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
    grid['alpha'] = np.arange(0, 0.0011, 0.0001)
    search = GridSearchCV(model, grid, scoring='neg_mean_absolute_error', cv=cv, n_jobs=-1)
    results_r = search.fit(N, y_r)
    
    print(results_x.best_estimator_.score(X,y_x), results_x.best_estimator_.alpha)
    print(results_y.best_estimator_.score(Y,y_y), results_y.best_estimator_.alpha)
    print(results_r.best_estimator_.score(N,y_r), results_r.best_estimator_.alpha)
    
    a_list = [list(results_x.best_estimator_.coef_[0]),list(results_y.best_estimator_.coef_[0]),list(results_r.best_estimator_.coef_[0])]
    row_lengths = []
    
    for row in a_list:
        row_lengths.append(len(row))
    
    max_length = max(row_lengths)
    
    for row in a_list:
        while len(row) < max_length:
            row.append(None)
    
    balanced_array = np.array([np.asarray(a_list[0]),np.asarray(a_list[1]),np.asarray(a_list[2])])
    df = df_main[30:700]

    u, v, r, hdg = df.loc[df.index[0], 'u'],df.loc[df.index[0], 'v'], df.loc[df.index[0], 'r'], df.loc[df.index[0], 'hdg']
    ship_it_model = ship_model(df.loc[df.index[0], 'u_dot'],df.loc[df.index[0], 'v_dot'], df.loc[df.index[0], 'r_dot'], ship_it, balanced_array)
    
    df_input = df[['rpm', 'rsa']]
    # print(u, v, r, hdg)
    df_sim = pd.DataFrame([])
    
    for i in df[:-1].index:
        u, v, r, hdg, delta_x_0, delta_y_0, delta_r_0, u_dot, v_dot, r_dot = ship_it_model.manoeuvre_model_rpa_3(u, v, r, hdg,
                                                                                                       df.loc[i, 'rpm'],
                                                                                                       df.loc[i, 'rsa'], 
                                                                                                       df.loc[i, 'delta_time'],                                                                                     
                                                                                                       )
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
    
    df['x_real'] = df.x.cumsum()
    df['y_real'] = df.y.cumsum()
    
    # calculate rho
    df['x_sim_diff_avg'] = df['x_real_sim'] - df.x_delta_sim.mean()
    df['y_sim_diff_avg'] = df['y_real_sim'] - df.y_delta_sim.mean()
    
    df['x_real_diff_avg'] = df['x_real'] - df.x.mean()
    df['y_real_diff_avg'] = df['y_real'] - df.y.mean()
    
    rho_x = (df['x_sim_diff_avg']*df['x_real_diff_avg']).sum()/np.sqrt((df['x_sim_diff_avg']**2).sum()*(df['x_real_diff_avg']**2).sum() )
    rho_y = (df['y_sim_diff_avg']*df['y_real_diff_avg']).sum()/np.sqrt((df['y_sim_diff_avg']**2).sum()*(df['y_real_diff_avg']**2).sum() )
    
    df['traj_error'] = (np.sqrt((df['x_real_sim'] - df['x_real'])**2 + (df['y_real_sim'] - df['y_real'])**2)).cumsum()
    # plt.plot(df.traj_error)
    plt.plot(df.x_real.tolist()[:],df.y_real.tolist()[:])
    plt.plot(df.x_real_sim.tolist()[:],df.y_real_sim.tolist()[:])
    plt.show()
    # value = df.loc[df.index[-1], 'traj_error']
    # if np.isnan(value):
    #     value = 0.00001
    print(rho_x*rho_y)
    del df
    return abs(rho_x*rho_y)

    
#------------- end objective function 
    
bounds=[(0.7,1.2),(0.8,1.2), (0.8,1.2)]   # upper and lower bounds of variables
nv = 3                # number of variables
mm = 1                   # if minimization problem, mm = -1; if maximization problem, mm = 1
 
# THE FOLLOWING PARAMETERS ARE OPTINAL.
particle_size=3    # number of particles
iterations= 2        # max number of iterations
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
