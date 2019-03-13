import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("log_turnover.csv", header = 0)
print(df.describe())
plt.rcParams['xtick.direction'] = 'in'
plt.rcParams['ytick.direction'] = 'in'
font = {'family' : 'Times New roman','size'   : 20}
plt.rc('font', **font)
plt.rc('mathtext', **{'rm': 'serif',
                      'it': 'serif:itelic',
                      'bf': 'serif:bold',
                      'fontset': 'cm'})
x_lim_start = 25.5
x_lim_end = 26.5


plt.subplot(3,1,1)
plt.plot(df['t'],df['p1'])
plt.ylabel('x[m]')
plt.xlim([x_lim_start,x_lim_end])

plt.subplot(3,1,2)
plt.plot(df['t'],df['p2'])
plt.ylabel('y[m]')
plt.xlim([x_lim_start,x_lim_end])

plt.subplot(3,1,3)
plt.plot(df['t'],df['p3'])
plt.xlabel('time[sec]')
plt.ylabel('z[m]')
plt.xlim([x_lim_start,x_lim_end])


plt.show()
