import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("admcon.csv", header = 0)
print(df.describe())

plt.figure(figsize=(10,10))
plt.subplot(3,1,1)
plt.plot(df['t'],df['Fx'])
plt.ylabel('Fx[N]',fontsize=18)
plt.legend()
plt.xlim([30,50])
plt.tick_params(labelsize=15)



plt.subplot(3,1,2)
plt.plot(df['t'],df['Fz'])

plt.ylabel('Fz[N]',fontsize=18)
plt.legend()
plt.xlim([30,50])
plt.tick_params(labelsize=15)


plt.subplot(3,1,3)
plt.plot(df['t'],df['My'])
plt.xlabel('time[sec]')
plt.ylabel('My[N/m]',fontsize=18)
plt.legend()
plt.xlim([30,50])
plt.tick_params(labelsize=15)


plt.show()
