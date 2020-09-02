import numpy as np
import os
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

package_dir = os.path.dirname(os.path.abspath(__file__))
fourier_ct_array_x = []
fourier_ct_array_y = []
with open(os.path.join(package_dir, 'ct_fourier_thrust_coefficients')) as reader:
    line = reader.readline()
    while line != '':
        fourier_ct_array_x.append(float(line.split(',')[0].strip()))
        fourier_ct_array_y.append(float(line.split(',')[1].strip()))
        line = reader.readline()

# plt.plot(fourier_ct_array_x, fourier_ct_array_y)
#
# f2 = interp1d(fourier_ct_array_x, fourier_ct_array_y, kind='cubic')
# x = np.linspace(0,360.0, 360000)
#
# plt.plot(x, f2(x))
# plt.show()


class ship:
    def __init__(self):
        self.L_oa = 31.95
        self.L_fl = 55.8/58*31.95
        self.B = 12.6
        self.Disp = 581.14
        self.T = 6.25/58*31.95
        self.rho = 1029
        self.v_max = 12.5*0.5144
        self.C_b = self.Disp/(self.L_fl*self.T*self.B)
        self.Mass = self.Disp*self.rho
        self.I_e = 1/120*3.14*self.rho*self.L_fl*self.B*self.T*(self.B**2+self.L_fl**2)
        self.I_z = self.Mass*(0.25*self.L_fl)**2
        self.w = 2*(self.C_b**5)*(1-self.C_b) + 0.1
        self.D_p = 2.36
        self.t = 0.075
        self.pr = 0.69

        #dimensions
        self.x_g = 30.8/58*31.95
        self.x_0 =  10/58*31.95 - self.x_g
        self.y_0 = 0.0
        self.x_1 = 44/58*31.95 - self.x_g
        self.y_1 = 5.2/58*31.95
        self.x_2 = 44/58*31.95 - self.x_g
        self.y_2 = -5.2/58*31.95

        self.x_0_pos = 10 / 58 * 31.95 - self.x_g
        self.y_0_pos = 0.0
        self.x_1_pos = 44 / 58 * 31.95 - self.x_g
        self.y_1_pos = 5.2 / 58 * 31.95
        self.x_2_pos = 44 / 58 * 31.95 - self.x_g
        self.y_2_pos = -5.2 / 58 * 31.95

        self.Z = 5.

        self.x_12 = abs(-5.2/58*31.95) +  abs(5.2/58*31.95)
        self.x_21 = abs(-5.2/58*31.95) +  abs(5.2/58*31.95)
        self.x_10 = abs(self.x_g - 10/58*31.95)+  abs(44/58*31.95 - self.x_g)
        self.x_01 = abs(self.x_g - 10/58*31.95)+  abs(44/58*31.95 - self.x_g)
        self.x_20 = abs(self.x_g - 10/58*31.95)+  abs(44/58*31.95 - self.x_g)
        self.x_02 = abs(self.x_g - 10/58*31.95)+  abs(44/58*31.95 - self.x_g)

        self.alpha_20 = abs(np.arctan(np.rad2deg(self.y_2/(abs(self.x_2)+abs(self.x_0)))))
        self.alpha_10 = abs(np.arctan(np.rad2deg(self.y_2/(abs(self.x_2)+abs(self.x_0)))))
        self.alpha_01 = abs(np.arctan(np.rad2deg(self.y_2/(abs(self.x_2)+abs(self.x_0)))))
        self.alpha_02 = abs(np.arctan(np.rad2deg(self.y_2/(abs(self.x_2)+abs(self.x_0)))))

        self.t_12 = 1 - (1 - 0.75 ** ((self.x_12 / self.D_p) ** (2 / 3)))
        self.t_21 = 1 - (1 - 0.75 ** ((self.x_21 / self.D_p) ** (2 / 3)))
        self.t_20 = 1 - (1 - 0.75 ** ((self.x_20 / self.D_p) ** (2 / 3))) / (1 - np.deg2rad(self.alpha_20))
        self.t_02 = 1 - (1 - 0.75 ** ((self.x_02 / self.D_p) ** (2 / 3))) / (1 - np.deg2rad(self.alpha_02))
        self.t_10 = 1 - (1 - 0.75 ** ((self.x_10 / self.D_p) ** (2 / 3))) / (1 - np.deg2rad(self.alpha_10))
        self.t_01 = 1 - (1 - 0.75 ** ((self.x_01 / self.D_p) ** (2 / 3))) / (1 - np.deg2rad(self.alpha_01))


        self.fourier_x = fourier_ct_array_x
        self.fourier_y = fourier_ct_array_y
        self.interpolate_f = interp1d(fourier_ct_array_x, fourier_ct_array_y, kind='linear')

    def beta_coef(self, beta):
        return self.interpolate_f(beta)




    #def calc_four

# add fourier series coefficients

