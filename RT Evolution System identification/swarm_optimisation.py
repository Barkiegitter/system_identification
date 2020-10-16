import random
import math
import matplotlib.pyplot as plt
#------------------------------------------------------------------------------
# TO CUSTOMIZE THIS PSO CODE TO SOLVE UNCONSTRAINED OPTIMIZATION PROBLEMS, CHANGE THE PARAMETERS IN THIS SECTION ONLY:
# THE FOLLOWING PARAMETERS MUST BE CHANGED.



#-------------- objective function
# def objective_function(x):
#     y = 3*(1-ship.D_p)**2*math.exp(-ship.D_p**2 - (x[1]+1)**2) - 10*(ship.D_p/5 - ship.D_p**3 - x[1]**5)*math.exp(-ship.D_p**2 - x[1]**2) -1/3*math.exp(-(ship.D_p+1)**2 - x[1]**2);
#     return y

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
ship = ship()
df_main = pd.read_csv('test_1.csv', sep=',')
# df_main = df_main[30400:31600].reset_index(inplace=False)
df_main.timestamp = pd.to_datetime(df_main.timestamp, format='%Y-%m-%d %H:%M:%S.%f')
df_main = df_main.dropna()
df_main.rpm_0 = df_main.rpm_0/60.0
df_main.rpm_1 = df_main.rpm_1/60.0
df_main.rpm_2 = df_main.rpm_2/60.0
# df_main = df_main.iloc[::10]

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

def thruster_interaction_coefficient(x_eng, y_eng, az_eng, cone_deg, flow_distance, x_down, y_down, az_down, prop_d):  # give engine number: shipclass engine number
    thrust_cone_boolean = thrust_cone(x_eng, y_eng, az_eng, cone_deg, flow_distance, x_down, y_down)
    x_d_ratio = np.sqrt((x_down-x_eng)**2+abs(y_down-y_eng)) / prop_d
    t_engine = (1 - 0.75**(x_d_ratio**(2/3)))/(thrust_cone_boolean)
    # print(abs(az_eng-az_down), 1)
    t = t_engine+(1-t_engine)*(((abs(az_eng-az_down))**3)/(((130.)/(t_engine**3)) + ((abs(az_eng-az_down))**3)))
    if math.isnan(t):
        return 0
    else:
        return 1-t

def objective_function_thrust_parameters(x):                # define which engine parameters are being tuned.   x[propeller diameter, fourier series multiplication constant]
    #azimuth 2 port side
    df_main['u_a_2'] = (1-ship.w)*((-df_main.u-df_main.r*abs(ship.y_2))*np.cos(np.deg2rad(df_main.rsa_2)) + (-df_main.v-df_main.r*abs(ship.x_2))*np.sin(np.deg2rad(df_main.rsa_2))) #(1-ship.w)*
    df_main['u_a_1'] = (1-ship.w)*((-df_main.u+df_main.r*abs(ship.y_1))*np.cos(np.deg2rad(df_main.rsa_1)) + (-df_main.v-df_main.r*abs(ship.x_1))*np.sin(np.deg2rad(df_main.rsa_1))) #(1-ship.w)*
    df_main['u_a_0'] = (1-ship.w)*((df_main.u)*-1*np.cos(np.deg2rad(df_main.rsa_0)) + ((-df_main.v + df_main.r*abs(ship.x_0))*np.sin(np.deg2rad(df_main.rsa_0))) ) #(1-ship.w)*
    
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
    df_main['t_21_phi'] = df_main.apply(lambda row: thruster_interaction_coefficient(ship.x_1, ship.y_1, row['rsa_1'], 25.0, 100.0, ship.x_2, ship.y_2, row['rsa_2'], ship.D_p), axis=1)
    df_main['t_20_phi'] = df_main.apply(lambda row: thruster_interaction_coefficient(ship.x_0, ship.y_0, row['rsa_0'], 25.0, 100.0, ship.x_2, ship.y_2, row['rsa_2'], ship.D_p), axis=1)
    df_main['f_p_40_2'] = x[0]*(1-df_main['t_21_phi'])*(1-df_main['t_20_phi'])*((1-ship.t)*ship.beta_coef(df_main.beta_2)*0.5*ship.rho*(((((1-ship.w)*df_main.u_a_2)**2)+ (0.7*np.pi*df_main.rpm_2*ship.D_p)**2))*np.pi/4*ship.D_p**2)  #(1-df_main['t_21_phi'])*(1-df_main['t_20_phi'])*
    ##
    df_main['t_12_phi'] = df_main.apply(lambda row: thruster_interaction_coefficient(ship.x_2, ship.y_2, row['rsa_2'], 25.0, 100.0, ship.x_1, ship.y_1, row['rsa_1'], ship.D_p), axis=1)
    df_main['t_10_phi'] = df_main.apply(lambda row: thruster_interaction_coefficient(ship.x_0, ship.y_0, row['rsa_0'], 25.0, 100.0, ship.x_1, ship.y_1, row['rsa_1'], ship.D_p), axis=1)
    df_main['f_p_40_1'] = x[0]*(1-df_main['t_12_phi'])*(1-df_main['t_10_phi'])*((1-ship.t)*ship.beta_coef(df_main.beta_1)*0.5*ship.rho*(((((1-ship.w)*df_main.u_a_1)**2)+ (0.7*np.pi*df_main.rpm_1*ship.D_p)**2))*np.pi/4*ship.D_p**2) #(1-df_main['t_12_phi'])*(1-df_main['t_10_phi'])*
    
    df_main['t_02_phi'] = df_main.apply(lambda row: thruster_interaction_coefficient(ship.x_2, ship.y_2, row['rsa_2'], 25.0, 100.0, ship.x_0, ship.y_0, row['rsa_0'], ship.D_p), axis=1)
    df_main['t_01_phi'] = df_main.apply(lambda row: thruster_interaction_coefficient(ship.x_1, ship.y_1, row['rsa_1'], 25.0, 100.0, ship.x_0, ship.y_0, row['rsa_0'], ship.D_p), axis=1)
    df_main['f_p_40_0'] = x[0]*(1-df_main['t_02_phi'])*(1-df_main['t_01_phi'])*((1-ship.t)*ship.beta_coef(df_main.beta_0)*0.5*ship.rho*(((((1-ship.w)*df_main.u_a_0)**2)+ (0.7*np.pi*df_main.rpm_0*ship.D_p)**2))*np.pi/4*ship.D_p**2)  #(1-df_main['t_02_phi'])*(1-df_main['t_01_phi'])*
    
    u = df_main.u.to_numpy()[:,newaxis]
    v = df_main.v.to_numpy()[:,newaxis]
    r = df_main.r.to_numpy()[:,newaxis]
    
    u_dot = df_main.u_dot.to_numpy()[:,newaxis]
    v_dot = df_main.v_dot.to_numpy()[:,newaxis]
    r_dot = df_main.r_dot.to_numpy()[:,newaxis]
    
    rsa_0 = df_main.rsa_0.to_numpy()[:, newaxis]
    rsa_1 = df_main.rsa_1.to_numpy()[:, newaxis]
    rsa_2 = df_main.rsa_2.to_numpy()[:, newaxis]
    
    f_p_40_0 = df_main.f_p_40_0.to_numpy()[:,newaxis]
    f_p_40_2 = df_main.f_p_40_2.to_numpy()[:,newaxis]
    f_p_40_1 = df_main.f_p_40_1.to_numpy()[:,newaxis]
    
    X = np.concatenate([u_dot, u, u*u, u*u*u, v*v, r*r, v*r, u*v*v, r*v*u, u*r*r], axis=1)
    Y = np.concatenate([v_dot, u*v, u*r, u*u*r, u*u*v, v*v*v, r*r*r, r*r*v, v*v*r, abs(v)*v, abs(r)*v, r*abs(v), abs(r)*r], axis=1)
    N = np.concatenate([r_dot, u*v, u*r, u*u*r, u*u*v, v*v*v, r*r*r, r*r*v, v*v*r, abs(v)*v, abs(r)*v, r*abs(v), abs(r)*r], axis=1)
    
    y_x = ship.Mass*(u_dot-r*v)-1*(np.cos(np.deg2rad(rsa_0))*abs(f_p_40_0)+np.cos(np.deg2rad(rsa_1))*abs(f_p_40_1)+np.cos(np.deg2rad(rsa_2))*abs(f_p_40_2))
    y_y = ship.Mass*(v_dot+r*u)-1*(np.sin(np.deg2rad(rsa_0))*abs(f_p_40_0)+np.sin(np.deg2rad(rsa_1))*abs(f_p_40_1)+np.sin(np.deg2rad(rsa_2))*abs(f_p_40_2)) #np.sin(rsa_0)*abs(f_p_40_0)+np.sin(rsa_1)*abs(f_p_40_1)+
    y_r = ship.I_e*r_dot + 1*(abs(ship.x_0)*np.sin(np.deg2rad(rsa_0))*abs(f_p_40_0) - abs(ship.x_2)*np.sin(np.deg2rad(rsa_0))*abs(f_p_40_2) - abs(ship.x_1)*np.sin(np.deg2rad(rsa_1))*abs(f_p_40_1) - abs(ship.y_2)*np.cos(np.deg2rad(rsa_2))*abs(f_p_40_2) + abs(ship.y_1)*np.cos(np.deg2rad(rsa_1))*abs(f_p_40_1))
    
    array_export = pairs = [(X, y_x, 'X'), (Y, y_y, 'Y'), (N, y_r, 'N')]
    lasso_X = RidgeCV()
    lasso_X.fit(X, y_x)
    lasso_Y = RidgeCV()
    lasso_Y.fit(Y, y_y)
    lasso_N = RidgeCV()
    lasso_N.fit(N, y_r)
    # print(lasso_X.score(X, y_x) + lasso_Y.score(Y, y_y) + lasso_N.score(N, y_r))
    
    score_x, score_y, score_r = lasso_X.score(X, y_x), lasso_Y.score(Y, y_y), lasso_N.score(N, y_r)
    score = (score_x + score_y + score_r)/3
    print(x[0], score)
    return score

    
#------------- end objective function 
    
bounds=[(0.,1.)]   # upper and lower bounds of variables
nv = 1                   # number of variables
mm = 1                   # if minimization problem, mm = -1; if maximization problem, mm = 1
 
# THE FOLLOWING PARAMETERS ARE OPTINAL.
particle_size=30         # number of particles
iterations=10           # max number of iterations
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
PSO(objective_function_thrust_parameters,bounds,particle_size,iterations)
