import numpy as np
import time
from manoeuvre_model_evo import ship_model
from ship_class import ship
from pid_small import PID
import matplotlib.pyplot as plt

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
input_ = [(0, 0), (5, 15)]#, (20,70), (10, 150)]
end_input = 0
input_position = 0
current_input_setting = 0
t = 0
dt = 0.4 #future noise
t_end = 300
pid_position = PID(P=0.01, I=.0, D=0.005, Ibounds_speed=(-80,80))

#
input_ref = []
position_real = []
x = 0
while t<t_end:
    # calculate speed input
    if end_input==0:
        if input_[input_position][1] <= t:
            
            current_input_setting = input_[input_position][0]
            pid_position.setPoint_position(current_input_setting)
            input_position += 1
            if input_position==len(input_):
                end_input  = 1
        
        
        
    control_input = pid_position.update(x)
    sign_control_input = np.sign(control_input)
    az_deflection = control_input
    rpm_1 = fit_function(abs(az_deflection))/2
    rpm_2 = fit_function(abs(az_deflection))/2
    rsa_1 = 90  + az_deflection
    rsa_2 = 270 - az_deflection
    print(az_deflection,x)
    if abs(az_deflection)>abs(pid_position.Integrator_max):
        az_deflection = np.sign(az_deflection)*abs(pid_position.Integrator_max)
        rsa_1 = 90  + az_deflection
        rsa_2 = 270 - az_deflection
        rpm_1 = fit_function(abs(az_deflection))/2
        rpm_2 = fit_function(abs(az_deflection))/2
    
        
        
    
    # calculate speed control iput
    # if abs(abs(control_input)-abs(rpm))/dt>max_rpm_second:
        
    #     rpm = dt*sign_control_input*max_rpm_second + rpm
    
    # print(rpm)
    # model
    u, v, r, hdg, delta_x_0, delta_y_0, delta_r_0, u_dot, v_dot, r_dot = ship_model_.manoeuvre_model_rt_evolution(u, v, r, hdg,
                                                                                               0., rpm_1, rpm_2,
                                                                                               180., rsa_1, rsa_2,
                                                                                               dt
                                                                                               )
    x = x + delta_y_0
    input_ref.append(current_input_setting)
    position_real.append(x)
    t = t + dt

input_ref_array = np.asarray(input_ref)
position_real_array = np.asarray(position_real)

control_score = abs(input_ref_array-position_real_array)
control_score = control_score.sum()

print(control_score)

plt.plot(input_ref_array)
plt.plot(position_real_array)
# plt.plot(rpm_set)

# plt.savefig('tuner.png')
plt.show()

