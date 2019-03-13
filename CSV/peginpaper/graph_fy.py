import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

df1 = pd.read_csv("log_turnover.csv", header = 0)
#print(df.describe())
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

plt.plot(df1['t'],df1['Fx'],linewidth = 5.0)
#plt.plot(df2['t'],df2['Fy'], label = 'prop.',linewidth = 5.0)
plt.xlabel('time[sec]')
plt.ylabel('Fx[N]')
plt.xlim([x_lim_start,x_lim_end])
#plt.legend(fontsize = 30)
plt.ylim([-10,10])


plt.savefig("pegin_fx.eps", bbox_inches="tight", pad_inches=0.001)

plt.show()
