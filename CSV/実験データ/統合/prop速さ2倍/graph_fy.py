import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

df1 = pd.read_csv("conv.csv", header = 0)
df2 = pd.read_csv("prop.csv", header = 0)
#print(df.describe())
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

plt.plot(df1['t'],df1['Fy'], label = 'conv',linewidth = 5.0)
plt.plot(df2['t'],df2['Fy'], label = 'prop',linewidth = 5.0)
plt.xlabel('time[sec]')
plt.ylabel('Fx[N]')
plt.xlim([x_lim_start,x_lim_end])
plt.ylim([-1,2])


plt.savefig("draw_fx.pdf", bbox_inches="tight", pad_inches=0.001)

plt.show()
