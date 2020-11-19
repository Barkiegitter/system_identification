import numpy as np
import time
from manoeuvre_model_evo import ship_model
from ship_class import ship
from pid_small import PID
import matplotlib.pyplot as plt

def azimuth(point1, point2):
    angle = np.arctan2(point2[0] - point1[0], point2[1] - point1[1])
    return np.degrees(angle) if angle >= 0 else np.degrees(angle) + 360
    #add thrust fraction here
    
coef_ = np.genfromtxt('foo_evo_general.csv', delimiter=',')
acc_lim = np.genfromtxt('acc_limits.csv', delimiter=',')

# P_range = np.arange(0,10.2,0.2)
# I_range = np.arange(0,10.2,0.2)
# D_range = np.arange(0,10.2,0.2)

x = np.arange(0,90.5, 0.5)
y = 2.5+np.log(x**0.6)
coefs = np.polynomial.polynomial.polyfit(x[1:],y[1:],3)
ffit = np.polynomial.polynomial.polyval(x, coefs)


fit_function = np.polynomial.polynomial.Polynomial(coefs)    # instead of np.poly1d
# plt.plot(y)
# plt.plot(fit_function(x))

#initialise ship
u, v, r, hdg = 0,0,0,0
ship_ = ship()
ship_model_ = ship_model(0,0,0, ship_, coef_,acc_lim)

# (distance, time)
#(zometeen x,y,time)
input_ = [(0,0,0), (20,20, 1), (30, 30, 60)]#, (20,70), (10, 150)]
end_input = 0
input_position = 0
current_input_setting = 0
t = 0
dt = 0.4 #future noise
t_end = 200
pid_position = PID(P=1.0, I=.0, D=10., Ibounds_speed=(-90,90))

x = 0
y = 0

rpm_1, rpm_2 = 0,0
#
input_ref = []
position_real = []
traj_x, traj_y = [], []
x = 0
while t<t_end:
    # calculate speed input
    if end_input==0:
        if input_[input_position+1][2] <= t:
            # print(input_position)
            current_input_setting = np.sqrt((input_[input_position][0]-x)**2 + (input_[input_position][1]-y)**2)
            # print('a', current_input_setting, input_position,x,y)
            # print(input_[input_position][0] - x)
            pid_position.setPoint_position(current_input_setting)
            input_position += 1
            if input_position==len(input_)-1:
                end_input  = 1
             
    # print(np.sqrt(((input_[input_position][0] - x)**2)+((input_[input_position][1] - y)**2)))    
    # if t%10==0:
    #     pid_position.setPoint_position(np.sqrt((input_[input_position][0]-x)**2 + (input_[input_position][1]-y)**2))
        
    control_input = pid_position.update(np.sqrt(((input_[input_position][0] - x)**2)+((input_[input_position][1] - y)**2)), dt)
    print(control_input, x, input_[input_position][0])
    sign_control_input = np.sign(control_input)
    
    direction_gen = (azimuth((x,y), (input_[input_position][0],input_[input_position][1])))
    # print(direction_gen)
    az_deflection = control_input
    rpm_1 = fit_function(abs(az_deflection))
    rpm_2 = fit_function(abs(az_deflection))
    rsa_1 = (direction_gen-hdg) + 90 + 180 + az_deflection 
    rsa_2 = (direction_gen-hdg) - 90 + 180 - az_deflection 
    
    # print(az_deflection,x)
    if abs(az_deflection)>abs(pid_position.Integrator_max):
        az_deflection = np.sign(az_deflection)*abs(pid_position.Integrator_max)
        rsa_1 = (direction_gen-hdg) + 90  +180 + az_deflection 
        rsa_2 = (direction_gen-hdg) - 90 +180 -  az_deflection 
        rpm_1 = fit_function(abs(az_deflection))
        rpm_2 = fit_function(abs(az_deflection))
    
    
    # print(control_input)
    if rsa_1>360.0:
        rsa_1 = rsa_1 - 360
    elif rsa_1<0.0 :
        rsa_1 = rsa_1 + 360
    if rsa_2>360.0:
        rsa_2 = rsa_2 - 360
    elif rsa_2<0.0 :
        rsa_2 = rsa_2 + 360
    # calculate speed control iput
    # if abs(abs(control_input)-abs(rpm))/dt>max_rpm_second:
        
    #     rpm = dt*sign_control_input*max_rpm_second + rpm
    # print(np.sqrt(((input_[input_position][0] - x)**2)+((input_[input_position][1] - y)**2)), control_input, direction_gen,az_deflection, hdg)

    print(hdg)
    # model
    u, v, r, hdg, delta_x_0, delta_y_0, delta_r_0, u_dot, v_dot, r_dot = ship_model_.manoeuvre_model_rt_evolution(u, v, r, hdg,
                                                                                               0., rpm_1, rpm_2,
                                                                                               180., rsa_1, rsa_2,
                                                                                               dt
                                                                                              )
    # print(rsa_1, rsa_2, direction_gen)
    # print('-', az_deflection, direction_gen)

    x = x + delta_x_0
    y = y+ delta_y_0
    input_ref.append(current_input_setting)
    # position_real.append(np.sqrt(((input_[input_position][0] - x)**2)+((input_[input_position][1] - y)**2)))
    # position_real.append(y)
    traj_x.append(x)
    traj_y.append(y)

    t = t + dt

input_ref_array = np.asarray(input_ref)
position_real_array = np.asarray(position_real)

# control_score = abs(input_ref_array-position_real_array)
# control_score = control_score.sum()

# print(control_score)

# plt.plot(input_ref_array)
plt.plot(traj_x, traj_y)
# plt.plot(rpm_set)

# plt.savefig('tuner.png')
plt.show()
plt.close()

