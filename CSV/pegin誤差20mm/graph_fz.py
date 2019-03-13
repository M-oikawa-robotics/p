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

plt.plot(df['t'],df['Fz'], linewidth = 5.0)
plt.xlabel('time[sec]')
plt.ylabel('Fz[N]')
plt.xlim([20,35])
plt.xlim([x_lim_start,x_lim_end])


plt.savefig("pegin_fz.pdf", bbox_inches="tight", pad_inches=0.01)

plt.show()
