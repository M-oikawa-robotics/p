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

#plt.figure(figsize=(16,12))
plt.figure(figsize=(12,12))
plt.plot(df1['t'],df1['p2'],label = 'res$_{conv}$', linewidth = 5.0)
plt.plot(df1['t'],df1['pc2'],label = 'cmd$_{conv}$', linewidth = 5.0,  linestyle='dashed')
plt.plot(df2['t'],df2['p2'],label = 'res$_{prop}$', linewidth = 5.0)
plt.plot(df2['t'],df2['pc2'],label = 'cmd$_{prop}$', linewidth = 5.0,  linestyle='dashed')
plt.xlabel('time[sec]')
plt.ylabel('$y$[m]')
plt.legend(fontsize = 30)
plt.xlim([x_lim_start,x_lim_end])
plt.ylim([-0.007,0.02])




plt.savefig("draw_x.eps", bbox_inches="tight", pad_inches=0.0)

plt.show()
