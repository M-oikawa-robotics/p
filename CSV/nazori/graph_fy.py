import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("log_turnover.csv", header = 0)
print(df.describe())
x_lim_start = 6
x_lim_end = 9.5
plt.rcParams['xtick.direction'] = 'in'
plt.rcParams['ytick.direction'] = 'in'
font = {'family' : 'Times New roman','size'   : 40}
plt.rc('font', **font)
plt.rc('mathtext', **{'rm': 'serif',
                      'it': 'serif:itelic',
                      'bf': 'serif:bold',
                      'fontset': 'cm'})

plt.figure(figsize=(16,12))

plt.plot(df['t'],df['Fy'])
plt.xlabel('time[sec]')
plt.ylabel('Fx[N]')
plt.xlim([x_lim_start,x_lim_end])
plt.ylim([-11,10])


plt.savefig("draw_fz.pdf", bbox_inches="tight", pad_inches=0.0)

plt.show()