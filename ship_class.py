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
        self.L_oa = 17.59
        self.L_fl = 17.26
        self.B = 4.75
        self.Disp = 37.875
        self.T = 1.297
        self.rho = 1029
        self.v_max = 10.29
        self.C_b = self.Disp/(self.L_fl*self.T*self.B)
        self.Mass = self.Disp*self.rho
        self.A_r = 2* 0.305
        self.I_e = 1/120*3.14*self.rho*self.L_fl*self.B*self.T*(self.B**2+self.L_fl**2)
        self.x_r = 6.5280
        self.AR = 1.42
        self.y_th = 0.9
        self.I_z = self.Mass*(0.25*self.L_fl)**2
        self.w = 0.1069
        self.D_p = 0.9
        self.t = 0.075
        self.fourier_x = fourier_ct_array_x
        self.fourier_y = fourier_ct_array_y
        self.interpolate_f = interp1d(fourier_ct_array_x, fourier_ct_array_y, kind='linear')

    def beta_coef(self, beta):
        return self.interpolate_f(beta)




    #def calc_four

# add fourier series coefficients

