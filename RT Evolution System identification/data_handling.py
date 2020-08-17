
import numpy as np
import pandas as pd
import csv
import time
import matplotlib.pyplot as plt



class data_handling:
    def __init__(self): #, something_like_path, ship, date, session, integers_showing_specific

        #gps
        self.df_gps = pd.read_csv('./RPA3 trials/gpgga.csv', error_bad_lines=True)
        self.df_gps.content = self.df_gps.content.str.split(',')
        self.df_gps['lat'] = self.df_gps.content.apply(lambda x: float(x[2])/100)
        self.df_gps['lon'] = self.df_gps.content.apply(lambda x: float(x[4])/100)
        self.df_gps = self.df_gps[['timestamp', 'lat', 'lon']]
        self.df_gps['time'] = pd.to_datetime(self.df_gps.timestamp, format='%Y-%m-%d %H:%M:%S.%f')
        print(self.df_gps.head())

        #hdg
        self.df_hdg = pd.read_csv('./RPA3 trials/hehdt.csv')
        self.df_hdg.content = self.df_hdg.content.str.split(',')
        self.df_hdg['hdg'] = self.df_hdg.content.apply(lambda x: float(x[1]) )
        self.df_hdg = self.df_hdg[['timestamp', 'hdg']]
        self.df_hdg['time'] = pd.to_datetime(self.df_hdg.timestamp, format='%Y-%m-%d %H:%M:%S.%f')
        print(self.df_hdg.head())


        #rpm
        self.df_rpm = pd.read_csv('./RPA3 trials/aiadc.csv')
        print(self.df_rpm.head())
        self.df_rpm.content = self.df_rpm.content.str.split(',')
        self.df_rpm['rpm'] = self.df_rpm.content.apply(lambda x: float(x[1]))
        self.df_rpm = self.df_rpm[['timestamp', 'rpm']]
        self.df_rpm['time'] = pd.to_datetime(self.df_rpm.timestamp, format='%Y-%m-%d %H:%M:%S.%f')
        print(self.df_rpm.head())


        #rsa
        self.df_rsa = pd.read_csv('./RPA3 trials/yxrsa.csv')#, skiprows=2, error_bad_lines=False
        # self.df_rsa = self.df_rsa.rename(
        #     columns={self.df_rsa.columns[0]: 'timestampcontent', self.df_rsa.columns[1]: 'rsa'})
        # self.df_rsa.timestampcontent = self.df_rsa.timestampcontent.apply(lambda x: x[0:26])
        # self.df_rsa = self.df_rsa[['timestampcontent', 'rsa']]
        # self.df_rsa['time'] = pd.to_datetime(self.df_rsa.timestampcontent, format='%Y-%m-%d %H:%M:%S.%f')
        self.df_rsa.content = self.df_rsa.content.str.split(',')
        self.df_rsa['rsa'] = self.df_rsa.content.apply(lambda x: float(x[1]) )
        self.df_rsa = self.df_rsa[['timestamp', 'rsa']]
        self.df_rsa['time'] = pd.to_datetime(self.df_rsa.timestamp, format='%Y-%m-%d %H:%M:%S.%f')
        print(self.df_rsa.head())

    def printstuff(self):
        print(self.df_rpm)
start = time.time()
main = data_handling()

def convert_float(val):
    try:
        return float(val)
    except ValueError:
        return np.nan

df_main = main.df_gps[20000:]
# df_main = df_main.reset_index(inplace=False)
df_main = df_main.drop_duplicates(subset=['time'], keep='first')

df_main['hdg'] = 0.0
df_main['rpm'] = 0.0
df_main['rsa'] = 0.0
df_rsa = main.df_rsa
df_rpm = main.df_rpm
df_hdg = main.df_hdg
integer_loop = 1
for i in df_main[1:].index:
    for k in df_rsa[integer_loop:].index:
        if df_rsa.loc[k,'time']>df_main.loc[i,'time']:
            # print(((main.df_rpm.loc[k,'time']-df_main.loc[i,'time']).total_seconds()) )
            rsa_inp = df_rsa.loc[k, 'rsa'] - \
                      ((df_rsa.loc[k, 'rsa'] - df_rsa.loc[k-1, 'rsa'])/((df_rsa.loc[k, 'time'] - df_rsa.loc[k-1, 'time']).total_seconds())) * \
                      ((df_rsa.loc[k, 'time']-df_main.loc[i, 'time']).total_seconds())
            # print(rsa_inp)
            if rsa_inp>0.0:
                df_main.loc[i, 'rsa'] = rsa_inp * 35 / 66
            else:
                df_main.loc[i, 'rsa'] = rsa_inp * 35 / 58
            integer_loop = k - 1
            break
        else:
            None



integer_loop = 1
for i in df_main.index: #[df_main[df_main.time>df_rpm.time[0]].index[0]-1:]
    for k in df_rpm[integer_loop:].index:
        if df_rpm.loc[k, 'time']>df_main.loc[i, 'time']:
            # print(k)
            rpm_inp = df_rpm.loc[k, 'rpm'] - \
                      ((df_rpm.loc[k, 'rpm'] - df_rpm.loc[k-1, 'rpm'])/((df_rpm.loc[k, 'time'] - df_rpm.loc[k-1, 'time']).total_seconds())) * \
                      ((df_rpm.loc[k, 'time']-df_main.loc[i, 'time']).total_seconds())
            df_main.loc[i, 'rpm'] = rpm_inp
            integer_loop = k - 2
            break
        else:
            continue


integer_loop = 1
for i in df_main[1:].index:
    for k in df_hdg[integer_loop:].index:
        if df_hdg.loc[k,'time']>df_main.loc[i,'time']:
            if abs(df_hdg.loc[k, 'hdg'] - df_hdg.loc[k - 1, 'hdg']) > 300.0:
                if df_hdg.loc[k, 'hdg'] > df_hdg.loc[k - 1, 'hdg']:
                    delta_hdg = df_hdg.loc[k, 'hdg'] - 360 - df_hdg.loc[k - 1, 'hdg']
                else:
                    delta_hdg = df_hdg.loc[k, 'hdg'] + 360 - df_hdg.loc[k - 1, 'hdg']
            else:
                delta_hdg = df_hdg.loc[k, 'hdg'] - df_hdg.loc[k - 1, 'hdg']
            # print(k)
            hdg_inp = df_hdg.loc[k, 'hdg'] - \
                      ((delta_hdg)/((df_hdg.loc[k, 'time'] - df_hdg.loc[k-1, 'time']).total_seconds())) * \
                      ((df_hdg.loc[k, 'time']-df_main.loc[i, 'time']).total_seconds())
            df_main.loc[i, 'hdg'] = hdg_inp
            if hdg_inp > 360.0:
                df_main.loc[i, 'hdg'] = hdg_inp - 360  #
            elif hdg_inp < 0.0:
                df_main.loc[i, 'hdg'] = hdg_inp + 360
            integer_loop = k - 2
            break
        else:
            continue
end = time.time()
print(end-start)
plt.plot(df_main.rsa.tolist()[:])#, df_main.rsa.tolist()[:])
# plt.plot(df_rsa.time.tolist()[:],df_rsa.rsa.tolist()[:])
plt.show()
df_main.reset_index(inplace=True)
df_main.to_csv('test.csv', index =False)






df.drop_duplicates(subset=['A', 'C'], keep=False)









