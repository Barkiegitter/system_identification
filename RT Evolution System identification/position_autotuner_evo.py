import random
import math
import matplotlib.pyplot as plt
#------------------------------------------------------------------------------
# TO CUSTOMIZE THIS PSO CODE TO SOLVE UNCONSTRAINED OPTIMIZATION PROBLEMS, CHANGE THE PARAMETERS IN THIS SECTION ONLY:
# THE FOLLOWING PARAMETERS MUST BE CHANGED.



import numpy as np
import time
from manoeuvre_model_evo import ship_model
from ship_class import ship
from pid_small import PID
import matplotlib.pyplot as plt

coef_ = np.genfromtxt('foo_evo_general.csv', delimiter=',')

# P_range = np.arange(0,10.2,0.2)
# I_range = np.arange(0,10.2,0.2)
# D_range = np.arange(0,10.2,0.2)


x = np.arange(0,90.5, 0.5)
y = 1+np.log(x**0.5)

coefs = np.polynomial.polynomial.polyfit(x[1:],y[1:],3)
ffit = np.polynomial.polynomial.polyval(x, coefs)
acc_lim = np.genfromtxt('acc_limits.csv', delimiter=',')


fit_function = np.polynomial.polynomial.Polynomial(coefs)    # instead of np.poly1d
plt.plot(y)
plt.plot(fit_function(x))

def PID_tuner(x):
    #initialise ship
    u, v, r, hdg = 0,0,0,0
    ship_ = ship()
    ship_model_ = ship_model(0,0,0, ship_, coef_,acc_lim)
    
    max_rpm_second = 0.25
    rpm = 0 
    
    
    
    
    # (distance, time)
    speed_u = [(0, 0), (10, 40)]
    end_input = 0
    speed_u_position = 0
    current_speed_setting = 0
    t = 0
    dt = 0.4 #future noise
    t_end = 400
    pid_position = PID(P=x[0], I=x[1], D=x[2], Ibounds_speed=(-90,90))
    
    #
    u_ref = []
    u_real = []
    x = 0
    while t<t_end:
        # calculate speed input
        if end_input==0:
            if speed_u[speed_u_position][1] <= t:
                
                current_speed_setting = speed_u[speed_u_position][0]
                pid_position.setPoint_speed(current_speed_setting)
                speed_u_position += 1
                if speed_u_position==len(speed_u):
                    end_input  = 1
            
            
            
        control_input = pid_position.update(x)
        sign_control_input = np.sign(control_input)
        # calculate speed control iput
        if abs(abs(control_input)-abs(rpm))/dt>max_rpm_second:
            
            rpm = dt*sign_control_input*max_rpm_second + rpm
            if abs(rsa_1)>abs(pid_position.Integrator_max):
                rpm = np.sign(rpm)*abs(pid_position.Integrator_max)
        # print(rpm)
        # model
        u, v, r, hdg, delta_x_0, delta_y_0, delta_r_0, u_dot, v_dot, r_dot = ship_model.manoeuvre_model_rt_evolution(u, v, r, hdg,
                                                                                                   0, rpm_1, rpm_2,
                                                                                                   180, rsa_1, rsa_2,
                                                                                                   dt,
                                                                                                   # u_dot_arr, v_dot_arr, r_rot_arr
                                                                                                   )
        x = x + delta_y_0
        u_ref.append(current_speed_setting)
        u_real.append(u)
        t = t + dt
    
    u_ref_array = np.asarray(u_ref)
    u_real_array = np.asarray(u_real)
    
    u_score = abs(u_ref_array-u_real_array)
    u_score = u_score.sum()
    
    print(u_score)
    return u_score

    
#------------- end objective function 
    
bounds=[(-4.,4.),(-2.,2.), (-1.,10.)]   # upper and lower bounds of variables
nv = 3                # number of variables
mm = -1                   # if minimization problem, mm = -1; if maximization problem, mm = 1
 
# THE FOLLOWING PARAMETERS ARE OPTINAL.
particle_size=20    # number of particles
iterations= 50      # max number of iterations
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
# PSO(PID_tuner,bounds,particle_size,iterations)
