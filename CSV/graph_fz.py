import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("log_turnover.csv", header = 0)
print(df.describe())
x_lim_start = 6
x_lim_end = 8.5
plt.rcParams['xtick.direction'] = 'in'
plt.rcParams['ytick.direction'] = 'in'
font = {'family' : 'Times New roman','size'   : 16}
plt.rc('font', **font)
plt.rc('mathtext', **{'rm': 'serif',
                      'it': 'serif:itelic',
                      'bf': 'serif:bold',
                      'fontset': 'cm'})

plt.figure(figsize=(25,10))

plt.subplot(2,2,1)
plt.plot(df['t'],df['p2'],label = 'res')
plt.plot(df['t'],df['pc2'],label = 'cmd')
plt.xlabel('time[sec]')
plt.ylabel('$y$[m]')
plt.legend()
plt.xlim([x_lim_start,x_lim_end])
plt.ylim([0.05,-0.05])

plt.subplot(2,2,3)
plt.plot(df['t'],df['p3'], label = 'res')
plt.plot(df['t'],df['pc3'],label = 'cmd')
plt.xlabel('time[sec]')
plt.ylabel('$z$[m]')
plt.legend()
plt.xlim([x_lim_start,x_lim_end])
plt.ylim([-0.1,0.2])


plt.subplot(2,2,2)
plt.plot(df['t'],df['Fy'])
plt.xlabel('time[sec]')
plt.ylabel('Fy[N]')
plt.xlim([20,35])
plt.xlim([x_lim_start,x_lim_end])


plt.subplot(2,2,4)
plt.plot(df['t'],df['Fz'])
plt.xlabel('time[sec]')
plt.ylabel('Fz[N]')
plt.xlim([20,35])
plt.xlim([x_lim_start,x_lim_end])


plt.savefig("stiff_relust_paper_pybullet.eps")

plt.show()
