import random
import math
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from ship_class import ship
from manoeuvre_model_evo import ship_model
import matplotlib.pyplot as plt




    
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
        
        
# change objective function-->fitting score
# 


# # r_coef = coef__i[2][~np.isnan(coef__i[2])]
# # r_coef = [8895951.144180702, -890876.6472610383, 793959.3557495456, 113197.4501268888, -26952.345777803515, -15362.303933270758, -1603.9340931816955, 8181.51410608622, -465359.2633082309, 3044.5158291071148, -1958898.2235264275, -31736.99398919771, -503580.4226463014, 1478084.7049691107, -4335781.63727453]
# r_coef = [13343926.716271054, -445438.32363051915, 423230.53141400067, 56598.7250634444, -13476.172888901758, -7681.151966635379, -801.9670465908478, 5660.876609517085, -512436.4650278686, 4566.773743660672, -1837287.2122747388, -15868.496994598854, -435621.79309854744, 739042.3524845553, -2167890.818637265]

# bounds = []
# for coef in r_coef:
#     # print(coef)
#     coef_bounds = (coef - abs(coef/2) ,coef + abs(coef/2))#abs(coef/1000.0))
#     bounds.append(coef_bounds)

# print(bounds)
# bounds=[(0.8,1.2),(0.8,1.2),(0.8,1.2),(0.8,1.2)]   # upper and lower bounds of variables
            # number of variables
mm = -1                   # if minimization problem, mm = -1; if maximization problem, mm = 1
 
# THE FOLLOWING PARAMETERS ARE OPTINAL.
particle_size=30   # number of particles
iterations=   10 # max number of iterations
w=0.2                    # inertia constant
c1=1                    # cognative constant   #research
c2=1.5                     # social constant     # research
# if mm == -1:
#     initial_fitness = float("inf") # for minimization problem
# if mm == 1:
#     initial_fitness = -float("inf") # for maximization problem
#------------------------------------------------------------------------------   
# Main PSO         
# it_coef = PSO(traj_error,bounds,particle_size,iterations,initial_fitness = float("inf"),w=0.2, c1=1, c2=1.5)


df_main = pd.read_csv('test_1_large.csv', sep=',')
df_main = df_main.dropna()[300:1500]
coef__ = np.genfromtxt('foo_evo_general.csv', delimiter=',')
acc_lim = np.genfromtxt('acc_limits.csv', delimiter=',')


first_round_coef = coef__[~np.isnan(coef__)].tolist()
bounds = []
for coefficient in first_round_coef:
    coef_bounds = (coefficient -abs(coefficient/100.0) ,coefficient + abs(coefficient/100.0))
    bounds.append(coef_bounds)
    

traj_seg_len = [500, 1000,2000]
nv = len(bounds)   
initial_fitness = float("inf")
for traj_len in traj_seg_len:
    def traj_error(x):
        ship_it = ship()
        df = df_main

        coef__ = np.array([np.asarray(x[:10]),np.asarray(x[10:25]),np.asarray(x[25:40])])
        # print(coef__)
        u, v, r, hdg = df.loc[df.index[0], 'u'],df.loc[df.index[0], 'v'], df.loc[df.index[0], 'r'], df.loc[df.index[0], 'hdg']
        ship_it_model = ship_model(df.loc[df.index[0], 'u_dot'],df.loc[df.index[0], 'v_dot'], df.loc[df.index[0], 'r_dot'], ship_it, coef__, acc_lim)
        df_sim = pd.DataFrame([])
        for i in df[:-1].index:
            if i%traj_len==0:
                u, v, r, hdg = df.loc[i,'u'],df.loc[i,'v'], df.loc[i,'r'], df.loc[i, 'hdg']
            u, v, r, hdg, delta_x_0, delta_y_0, delta_r_0, u_dot, v_dot, r_dot = ship_it_model.manoeuvre_model_rt_evolution(u, v, r, hdg,
                                                                                                           df.loc[i, 'rpm_0']/60., df.loc[i, 'rpm_1']/60., df.loc[i, 'rpm_2']/60.,
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
        
        del ship_it_model
        if feature==np.inf or math.isnan(feature):
            feature=10000000000000000
        print(feature)
        del df
        return feature
    
    PSO_run = PSO(traj_error,bounds,particle_size,iterations,initial_fitness ,w, c1, c2,nv)
    it_coef = PSO_run.return_opt_coef()
    # print(it_coef)
    for i_coef in range(len(it_coef)):
        
        coef_delta = abs(first_round_coef[i_coef]-it_coef[i_coef])
        coef_bounds = (it_coef[i_coef] - coef_delta*1.1,it_coef[i_coef] + coef_delta*1.1 )#abs(coef/1000.0))
        bounds.append(coef_bounds)
    first_round_coef = it_coef

#seperate trajectories
#analyze delta uu!!!!!!!!!!



