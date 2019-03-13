import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

df1 = pd.read_csv("log_turnover.csv", header = 0)
x_lim_start = 7
x_lim_end = 10
plt.rcParams['xtick.direction'] = 'in'
plt.rcParams['ytick.direction'] = 'in'
font = {'family' : 'Times New roman','size'   : 40}
plt.rc('font', **font)
plt.rc('mathtext', **{'rm': 'serif',
                      'it': 'serif:itelic',
                      'bf': 'serif:bold',
                      'fontset': 'cm'})

plt.figure(figsize=(12,12))

plt.plot(df1['t'],df1['Fz'], linewidth = 5.0)
#plt.plot(df2['t'],df2['Fz'] ,linewidth = 5.0)
plt.xlabel('time[sec]')
plt.ylabel('Fz[N]')
plt.xlim([20,35])
plt.xlim([x_lim_start,x_lim_end])
#plt.legend(fontsize = 30)


plt.savefig("draw_fz.eps", bbox_inches="tight", pad_inches=0.001)

plt.show()
