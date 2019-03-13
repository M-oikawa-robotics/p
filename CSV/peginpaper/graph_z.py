import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

df1 = pd.read_csv("log_turnover.csv", header = 0)
x_lim_start = 7
x_lim_end = 10
plt.rcParams['xtick.direction'] = 'in'
plt.rcParams['ytick.direction'] = 'in'
font = {'family' : 'Times New roman','size'   : 30}
plt.rc('font', **font)
plt.rc('mathtext', **{'rm': 'serif',
                      'it': 'serif:itelic',
                      'bf': 'serif:bold',
                      'fontset': 'cm'})

plt.figure(figsize=(12,12))

plt.plot(df1['t'],df1['p3'],label = 'res', linewidth = 5.0)
plt.plot(df1['t'],df1['pc3'],label = 'cmd', linewidth = 3.0,  linestyle='dashed')
plt.xlabel('time[sec]')
plt.ylabel('$z$[m]')
plt.legend(fontsize = 30)
plt.xlim([x_lim_start,x_lim_end])
plt.ylim([-0.07,0.1])


plt.savefig("draw_z.eps", bbox_inches="tight", pad_inches=0.1)

plt.show()
