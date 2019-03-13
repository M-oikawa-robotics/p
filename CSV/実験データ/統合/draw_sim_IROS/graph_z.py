import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

df1 = pd.read_csv("conv.csv", header = 0)
df2 = pd.read_csv("prop.csv", header = 0)
x_lim_start = 6.6
x_lim_end = 7.2
plt.rcParams['xtick.direction'] = 'in'
plt.rcParams['ytick.direction'] = 'in'
font = {'family' : 'Times New roman','size'   : 30}
plt.rc('font', **font)
plt.rc('mathtext', **{'rm': 'serif',
                      'it': 'serif:itelic',
                      'bf': 'serif:bold',
                      'fontset': 'cm'})

plt.figure(figsize=(12,12))

plt.plot(df1['t'],df1['p3'],label = 'res$_{conv.}$', linewidth = 5.0)
plt.plot(df1['t'],df1['pc3'],label = 'des$_{conv.}$', linewidth = 3.0,  linestyle='dashed')
plt.plot(df2['t'],df2['p3'],label = 'res$_{prop.}$', linewidth = 5.0)
plt.plot(df2['t'],df2['pc3'],label = 'des$_{prop.}$', linewidth = 5.0,  linestyle='dashed')
plt.vlines([6.74], -1000, 1000, "blue", linestyles='dashdot', linewidth = 3.0)
plt.xlabel('time[sec]')
plt.ylabel('$z$[m]')
plt.legend(fontsize = 30)
plt.xlim([x_lim_start,x_lim_end])
plt.ylim([0.01,0.12])


plt.savefig("draw_z.eps", bbox_inches="tight", pad_inches=0.01)

plt.show()
