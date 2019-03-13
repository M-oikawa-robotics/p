import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("log_turnover.csv", header = 0)
print(df.describe())
x_lim_start = 6.5
x_lim_end = 8.5
plt.rcParams['xtick.direction'] = 'in'
plt.rcParams['ytick.direction'] = 'in'
font = {'family' : 'Times New roman','size'   : 40}
plt.rc('font', **font)
plt.rc('mathtext', **{'rm': 'serif',
                      'it': 'serif:itelic',
                      'bf': 'serif:bold',
                      'fontset': 'cm'})

plt.figure(figsize=(12,12))

plt.plot(df['t'],df['p2'],label = 'res', linewidth = 5.0)
plt.plot(df['t'],df['pc2'],label = 'cmd', linewidth = 5.0,  linestyle='dashed')
plt.xlabel('time[sec]')
plt.ylabel('$y$[m]')
plt.legend()
plt.xlim([x_lim_start,x_lim_end])
plt.ylim([-0.005,0.04])



plt.savefig("pegin_x.pdf", bbox_inches="tight", pad_inches=0.01)


plt.show()
