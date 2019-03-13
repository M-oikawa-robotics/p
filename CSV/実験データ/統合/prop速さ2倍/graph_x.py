import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("log_turnover.csv", header = 0)
print(df.describe())
x_lim_start = 6.5
x_lim_end = 7.5
plt.rcParams['xtick.direction'] = 'in'
plt.rcParams['ytick.direction'] = 'in'
font = {'family' : 'Times New roman','size'   : 40}
plt.rc('font', **font)
plt.rc('mathtext', **{'rm': 'serif',
                      'it': 'serif:itelic',
                      'bf': 'serif:bold',
                      'fontset': 'cm'})

plt.figure(figsize=(16,12))
plt.plot(df['t'],df['p2'],label = 'res', linewidth = 5.0)
plt.plot(df['t'],df['pc2'],label = 'cmd', linewidth = 5.0)
#plt.plot(df['t'],df['Fz'], label = ' ',linewidth = 2.0)
plt.xlabel('time[sec]')
plt.ylabel('$y$[m]')
plt.legend()
plt.xlim([x_lim_start,x_lim_end])
plt.ylim([-0.02,0.02])




plt.savefig("draw_x.pdf", bbox_inches="tight", pad_inches=0.001)

plt.show()
