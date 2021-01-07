import numpy as np
import time
import math
from manoeuvre_model_borkum import ship_model
from ship_class import ship
from pid_small import PID
import matplotlib.pyplot as plt

def azimuth(point1, point2):
    angle = np.arctan2(point2[0] - point1[0], point2[1] - point1[1])
    return np.degrees(angle) if angle >= 0 else np.degrees(angle) + 360
    #add thrust fraction here
    
coef_ = np.genfromtxt('borkum_general.csv', delimiter=',')

ship = ship()


input_ = [(0,0,0,0), (20.0, 20,0, 1)]#, (50., 100., 100)]#,(100., 100., 200),(100., 0., 300)]#], (0., 100., 180)]#, (0., 0., 190)]#, (20,70), (10, 150)]
end_input = 0
input_position = 0
t = 0
dt = 0.4 #future noise
t_end = 22
pid_position_x = PID(P=0.9, I=0.0, D=1., Ibounds_speed=(-90,90))
pid_position_y = PID(P=0.9, I=0.0, D=1., Ibounds_speed=(-90,90))
pid_attitude = PID(P=0.1, I=0.0, D=2.5, Ibounds_speed=(-90,90))

ship_model = ship_model(0,0,0, ship, coef_)
x, x_old = 0, 0 
y, y_old = 0, 0
u, v, r, hdg = 0.0,0,0,0
rpm_1, rpm_2 = 0,0
input_ref = []
position_real = []
traj_x, traj_y, hdg_list = [], [], []
x = 0
ref_side = 0
traj_angle_position = 0
last_coordinate = [0,0]
while t<t_end:
    # calculate speed input
    # print(end_input)
    if end_input==0:
        if input_[input_position+1][3] <= t:
            # print(t ,input_[input_position+1][1])
            last_coordinate[0] = input_[input_position][0];last_coordinate[1] = input_[input_position][1]
            # print(input_position)
            current_input_setting_x = input_[input_position+1][0]
            current_input_setting_y = input_[input_position+1][1]
            current_input_setting_attitude = input_[input_position+1][2]
            
            
        
            pid_position_x.setPoint_hdg(current_input_setting_x)
            pid_position_y.setPoint_hdg(current_input_setting_y)
            pid_attitude.setPoint_hdg(current_input_setting_attitude)
            
            
            
            input_position += 1
            if input_position==len(input_)-1:
                end_input = 1

    control_input_x = pid_position_x.update(x, dt)
    control_input_y = pid_position_y.update(y, dt)
    
    
    sign_control_input_x = np.sign(control_input_x)
    sign_control_input_y = np.sign(control_input_y)
    
    
    
    
    if math.copysign(1, control_input_x)==1:
        rsa_1 = 90
    else:
        rsa_1 = 270
    if math.copysign(1, control_input_y)==1:
        rsa_2 = 180.0
    else:
        rsa_2 = 0.0
    
    
    
    control_input_attitude = pid_attitude.update(hdg, dt)
    sign_control_input_attitude = np.sign(control_input_attitude)     
    if abs(control_input_attitude)<2.0:
        control_input_attitude=0.
    
    if math.copysign(1, control_input_attitude)==1:
        rsa_0 = 270.0
    else:
        rsa_0 = 90.0
    
    rpm_0 = abs(control_input_attitude)
    if rpm_0>5.0:
        rpm_0 = 5.0
    
    
    
    
    rsa_1 = rsa_1 + hdg 
    rsa_2 = rsa_2 + hdg    
   
        
        
    
    print(control_input_attitude, control_input_x, control_input_y,rsa_0,hdg)

    # print(az_deflection)
    rpm_1 = abs(control_input_x)
    rpm_2 = abs(control_input_y)
    
    
    if rpm_1>2.0:
        rpm_1 = 2.0
    if rpm_2>5.0:
        rpm_2 = 5.0
    
    
    
    u, v, r, hdg, delta_x_0, delta_y_0, delta_r_0, u_dot, v_dot, r_dot = ship_model.manoeuvre_model_borkum(u, v, r, hdg,
                                                                                               rpm_0, rpm_1, rpm_2,
                                                                                               rsa_0, rsa_1, rsa_2,
                                                                                               dt
                                                                                              )
    print(u,v,r)
    # print('-', az_deflection, direction_gen)

    x = x + delta_x_0
    y = y + delta_y_0
    # position_real.append(np.sqrt(((input_[input_position][0] - x)**2)+((input_[input_position][1] - y)**2)))
    # position_real.append(y)
    traj_x.append(x)
    traj_y.append(y)
    # hdg_list.append(np.sqrt(u**2+v**2))
    hdg_list.append(rsa_1)
    # print(az_deflection)
    # print(correction)
    # print(angle_resultant-hdg)
    # print(angle_resultant)
    t = t + dt
plt.plot(traj_x,traj_y)

