import random
import math
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from ship_class import ship
from manoeuvre_model_borkum import ship_model
from numpy import newaxis
import matplotlib.pyplot as plt
from sklearn.model_selection import RepeatedKFold
from sklearn.model_selection import GridSearchCV
from sklearn.linear_model import Ridge


import warnings
warnings.filterwarnings("ignore")

    
class Particle:
    def __init__(self,bounds,initial_fitness,w,c1,c2,nv):
        self.particle_position=[]                     # particle position
        self.particle_velocity=[]                     # particle velocity
        self.local_best_particle_position=[]          # best position of the particle
        self.fitness_local_best_particle_position= initial_fitness  # initial objective function value of the best particle position
        self.fitness_particle_position=initial_fitness             # objective function value of the particle position
        w = w
        c1 = c1
        c2 = c2
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
    def __init__(self,objective_function,bounds,particle_size,iterations,initial_fitness,w,c1,c2,nv):
 
        fitness_global_best_particle_position=initial_fitness
        self.global_best_particle_position=[]
 
        swarm_particle=[]
        for i in range(particle_size):
            swarm_particle.append(Particle(bounds,initial_fitness,w,c1,c2,nv))
        A=[]
         
        for i in range(iterations):
            print('New round of iteration yo')
            for j in range(particle_size):
                swarm_particle[j].evaluate(objective_function)
 
                if mm ==-1:
                    if swarm_particle[j].fitness_particle_position < fitness_global_best_particle_position:
                        self.global_best_particle_position = list(swarm_particle[j].particle_position)
                        fitness_global_best_particle_position = float(swarm_particle[j].fitness_particle_position)
                if mm ==1:
                    if swarm_particle[j].fitness_particle_position > fitness_global_best_particle_position:
                        self.global_best_particle_position = list(swarm_particle[j].particle_position)
                        fitness_global_best_particle_position = float(swarm_particle[j].fitness_particle_position)
            for j in range(particle_size):
                swarm_particle[j].update_velocity(self.global_best_particle_position)
                swarm_particle[j].update_position(bounds)
                 
            A.append(fitness_global_best_particle_position) # record the best fitness
        print('Optimal solution:', self.global_best_particle_position)
        print('Objective function value:', fitness_global_best_particle_position)
        print('Evolutionary process of the objective function value:')
        plt.plot(A)
        
        
    def return_opt_coef(self):
        return self.global_best_particle_position
        

mm = -1                   # if minimization problem, mm = -1; if maximization problem, mm = 1
 
# THE FOLLOWING PARAMETERS ARE OPTINAL.
particle_size= 15   # number of particles
iterations=   30 # max number of iterations
w=0.1                   # inertia constant
c1=1                    # cognative constant   #research
c2=1.5                     # social constant     # research

df_main = pd.read_csv('test_1_large_turbopolyp.csv', sep=',')
df_main = df_main.dropna()

ship_elements = [(0.02,0.8), (0.5, 1.5), (0.7,1.3), (0.1,50), (0.8, 1.2)]
#ship_it_elements= beta_coef, mass, i_e, x12, y12, x0
bounds = ship_elements
# for coefficient in ship_it_elements:
#     coef_bounds = (coefficient -abs(coefficient/100.0) ,coefficient + abs(coefficient/100.0))
#     bounds.append(coef_bounds)
    


nv = len(bounds)   
initial_fitness = float("inf")


def traj_error(x):
    ship_it = ship()
    
    ship_it.man_eng = ship_it.man_eng * x[0]
    ship_it.x_g = ship_it.x_g * x[1]
    ship_it.y_1, ship_it.y_2 = ship_it.y_1*x[2], ship_it.y_2*x[2]
    ship_it.I_e = ship_it.I_e * x[3]
    ship_it.Mass = ship_it.Mass * x[4]
    
    
    print(x)
    df = pd.read_csv('test_1_large_turbopolyp.csv', sep=',')[2200:]
    df = df.dropna()
    # df = df_main[1:]
    
    
    df.rpm_0 = df.rpm_0/600.0
    df.rpm_1 = df.rpm_1/600.0
    df.rpm_2 = df.rpm_2/600.0
    
    df['u_a_2'] = (1-ship_it.w)*((df.u+df.r*abs(ship_it.y_2))*-1*np.cos(np.deg2rad(df.rsa_2)) + (df.v+df.r*abs(ship_it.x_2))*-1*np.sin(np.deg2rad(df.rsa_2))) #(1-ship.w)*
    df['u_a_1'] = (1-ship_it.w)*((df.u-df.r*abs(ship_it.y_1))*-1*np.cos(np.deg2rad(df.rsa_1)) + (-df.v-df.r*abs(ship_it.x_1))*-1*np.sin(np.deg2rad(df.rsa_1))) #(1-ship.w)*
    df['u_a_0'] = (1-ship_it.w)*((df.u)*-1*np.cos(np.deg2rad(df.rsa_0)) + ((-df.v + df.r*abs(ship_it.x_0))*-1*np.sin(np.deg2rad(df.rsa_0))) ) #(1-ship.w)*

    df['beta_2'] = np.rad2deg(np.arctan((df.u_a_2)/(0.7*np.pi*df.rpm_2*ship_it.D_p)))
    df['beta_2'] = df.beta_2.apply(lambda x: x+360 if x<0 else x)
    
    df['beta_1'] = np.rad2deg(np.arctan((df.u_a_1)/(0.7*np.pi*df.rpm_1*ship_it.D_p)))
    df['beta_1'] = df.beta_1.apply(lambda x: x+360 if x<0 else x)
    
    df['beta_0'] = np.rad2deg(np.arctan((df.u_a_0)/(0.7*np.pi*df.rpm_0*ship_it.D_p)))
    df['beta_0'] = df.beta_0.apply(lambda x: x+360 if x<0 else x)
    
    df['f_p_40_2'] = 1.*((1-ship_it.t)*ship_it.beta_coef(df.beta_2)*0.5*ship_it.rho*(((((1-ship_it.w)*df.u_a_2)**2)+(0.7*np.pi*df.rpm_2*ship_it.D_p)**2))*np.pi/4*ship_it.D_p**2)  #(1-df['t_21_phi'])*(1-df['t_20_phi'])*
    df['f_p_40_1'] = 1.*((1-ship_it.t)*ship_it.beta_coef(df.beta_1)*0.5*ship_it.rho*(((((1-ship_it.w)*df.u_a_1)**2)+(0.7*np.pi*df.rpm_1*ship_it.D_p)**2))*np.pi/4*ship_it.D_p**2) #(1-df['t_12_phi'])*(1-df['t_10_phi'])*
    df['f_p_40_0'] = 1.*((1-ship_it.t)*ship_it.beta_coef(df.beta_0)*0.5*ship_it.rho*(((((1-ship_it.w)*df.u_a_0)**2)+(0.7*np.pi*df.rpm_0*ship_it.D_p)**2))*np.pi/4*ship_it.D_p**2)#.rolling(20).mean()  #(1-df['t_02_phi'])*(1-df['t_01_phi'])*



    df = df[20:-7]
    
    u = df.u.to_numpy()[:, newaxis]
    v = df.v.to_numpy()[:, newaxis]
    r = df.r.to_numpy()[:, newaxis]
    
    u_a_2 = df.u_a_2.to_numpy()[:, newaxis]
    u_a_1 = df.u_a_1.to_numpy()[:, newaxis]
    u_a_0 = df.u_a_0.to_numpy()[:, newaxis]
    
    u_dot = df.u_dot.to_numpy()[:, newaxis]
    
    v_dot = df.v_dot.to_numpy()[:, newaxis]
    
    r_dot = df.r_dot.to_numpy()[:, newaxis]
    
    rsa_0 = df.rsa_0.to_numpy()[:, newaxis]
    rsa_1 = df.rsa_1.to_numpy()[:, newaxis]
    rsa_2 = df.rsa_2.to_numpy()[:, newaxis]
    
    f_p_40_0 = df.f_p_40_0.to_numpy()[:, newaxis]
    f_p_40_2 = df.f_p_40_2.to_numpy()[:, newaxis]
    f_p_40_1 = df.f_p_40_1.to_numpy()[:, newaxis]

    X = np.concatenate([u_dot, u, u*u, u*u*u, v*v, r*r, v*r, u*v*v, r*v*u, u*r*r,
                        ], axis=1)
    Y = np.concatenate([v_dot, v, v*v, u*v, u*r, u*u*r, u*u*v, v*v*v, r*r*r, r*r*v, v*v*r , abs(v)*v, abs(r)*v, r*abs(v), abs(r)*r], axis=1) #, abs(v)*v, abs(r)*v, r*abs(v), abs(r)*r
    N = np.concatenate([r_dot, r, r*r, v*r, u*r, u*u*r, u*u*v, v*v*v, r*r*r, r*r*v, v*v*r, abs(v)*v, abs(r)*v, r*abs(v), abs(r)*r, 
                        ], axis=1)
    
    y_x = ship_it.Mass*(u_dot-r*v)-1.0*(-np.cos(np.deg2rad(rsa_0))*(f_p_40_0)-np.cos(np.deg2rad(rsa_1))*(f_p_40_1)-np.cos(np.deg2rad(rsa_2))*(f_p_40_2))
    y_y = ship_it.Mass*(v_dot+r*u)-1.0*(-np.sin(np.deg2rad(rsa_0))*(f_p_40_0)-np.sin(np.deg2rad(rsa_1))*(f_p_40_1)-np.sin(np.deg2rad(rsa_2))*(f_p_40_2)) #np.sin(rsa_0)*abs(f_p_40_0)+np.sin(rsa_1)*abs(f_p_40_1)+
    y_r = ship_it.I_e*r_dot-1*(ship_it.x_0*-1*np.sin(np.deg2rad(rsa_0))*(f_p_40_0)+ship_it.x_2*-1*np.sin(np.deg2rad(rsa_2))*(f_p_40_2) + ship_it.x_1*-1*np.sin(np.deg2rad(rsa_1))*(f_p_40_1) - ship_it.y_2*-1*np.cos(np.deg2rad(rsa_2))*(f_p_40_2) - ship_it.y_1*-1*np.cos(np.deg2rad(rsa_1))*(f_p_40_1))

    
    model = Ridge(fit_intercept=False)
    cv = RepeatedKFold(n_splits=5, n_repeats=10, random_state=25)
    grid = dict()
    grid['alpha'] = np.logspace(1.0, -4.0, num=10)
    search = GridSearchCV(model, grid, scoring='neg_mean_squared_error', cv=cv, n_jobs=-1)
    results_x = search.fit(X, y_x)
    one_mse_value = search.cv_results_['mean_test_score'][-1] - search.cv_results_['mean_test_score'].std()
    a = [i for i in range(len(search.cv_results_['mean_test_score'])) if search.cv_results_['mean_test_score'][i] > one_mse_value]
    # search.param_grid['alpha'][0][int(a[0])]
    clf_x = Ridge(alpha=search.param_grid['alpha'][int(a[0])])
    clf_x.fit(X, y_x)
    # clf.score(X, y_x)
    
    
    model = Ridge(fit_intercept=False)
    cv = RepeatedKFold(n_splits=5, n_repeats=10, random_state=25)
    grid = dict()
    grid['alpha'] = np.logspace(5.0, -4.0, num=200)
    search = GridSearchCV(model, grid, scoring='neg_mean_squared_error', cv=cv, n_jobs=-1)
    results_x = search.fit(Y, y_y)
    one_mse_value = search.cv_results_['mean_test_score'][-1] - search.cv_results_['mean_test_score'].std()
    a = [i for i in range(len(search.cv_results_['mean_test_score'])) if search.cv_results_['mean_test_score'][i] > one_mse_value]
    # search.param_grid['alpha'][0][int(a[0])]
    clf_y = Ridge(alpha=search.param_grid['alpha'][int(a[0])])
    clf_y.fit(Y, y_y)
    
    model = Ridge(fit_intercept=False)
    cv = RepeatedKFold(n_splits=5, n_repeats=10, random_state=25)
    grid = dict()
    grid['alpha'] = np.logspace(1.0, -4.0, num=10)
    search = GridSearchCV(model, grid, scoring='neg_mean_squared_error', cv=cv, n_jobs=-1)
    results_x = search.fit(N, y_r)
    one_mse_value = search.cv_results_['mean_test_score'][-1] - search.cv_results_['mean_test_score'].std()
    a = [i for i in range(len(search.cv_results_['mean_test_score'])) if search.cv_results_['mean_test_score'][i] > one_mse_value]
    # search.param_grid['alpha'][0][int(a[0])]
    clf_n = Ridge(alpha=search.param_grid['alpha'][int(a[0])])
    clf_n.fit(N, y_r)
    
    coef__ = np.array([np.asarray(clf_x.coef_[0]),np.asarray(clf_y.coef_[0]),np.asarray(clf_n.coef_[0])])
    # print(coef__)
    u, v, r, hdg = df.loc[df.index[0], 'u'],df.loc[df.index[0], 'v'], df.loc[df.index[0], 'r'], df.loc[df.index[0], 'hdg']
    ship_it_model = ship_model(df.loc[df.index[0], 'u_dot'],df.loc[df.index[0], 'v_dot'], df.loc[df.index[0], 'r_dot'], ship_it, coef__)
    df_sim = pd.DataFrame([])
    for i in df[:-1].index:
        if i%50==0:
            u, v, r, hdg = df.loc[i,'u'],df.loc[i,'v'], df.loc[i,'r'], df.loc[i, 'hdg']
        u, v, r, hdg, delta_x_0, delta_y_0, delta_r_0, u_dot, v_dot, r_dot = ship_it_model.manoeuvre_model_borkum(u, v, r, hdg,
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
    
    df['x_diff'] = abs(df.x_delta_sim - df.x)
    df['y_diff'] = abs(df.y_delta_sim - df.y)
    df['error'] = np.sqrt(df.x_diff**2 + df.y_diff**2)
    
    
    plt.plot(df.x_real.tolist()[:],df.y_real.tolist()[:])
    plt.plot(df.x_real_sim.tolist()[:],df.y_real_sim.tolist()[:])
    # plt.plot(df.traj_error)
    plt.show()
    # print(df)
    feature = df.error.cumsum().iloc[-1]
    # print(feature)
    # print(feature)
    print(feature)
    if feature==np.inf or math.isnan(feature):
        feature=1000000000000000000000000000000000
    
    return feature

    
PSO_run = PSO(traj_error,bounds,particle_size,iterations,initial_fitness ,w, c1, c2,nv)
it_coef = PSO_run.return_opt_coef()
    

#seperate trajectories
#analyze delta uu!!!!!!!!!!



