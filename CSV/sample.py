import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("log_turnover.csv", header = 0)
print(df.describe())

plt.figure(figsize=(20,10))
plt.subplot(3,2,1)
plt.plot(df['t'],df['p1'],label="ref")
plt.plot(df['t'],df['pc1'],label="cmd")
plt.xlabel('time[sec]')
plt.ylabel('x[m]')
plt.legend()

 
plt.subplot(3,2,2)
plt.plot(df['t'],df['p2'],label="ref")
plt.plot(df['t'],df['pc2'],label="cmd")
plt.xlabel('time[sec]')
plt.ylabel('y[m]')
plt.legend()

plt.subplot(3,2,3)
plt.plot(df['t'],df['p3'],label="ref")
plt.plot(df['t'],df['pc3'],label="cmd")
plt.xlabel('time[sec]')
plt.ylabel('z[m]')
plt.legend()

 
plt.subplot(3,2,4)
plt.plot(df['t'],df['p4'],label="ref")
plt.plot(df['t'],df['pc4'],label="cmd")
plt.xlabel('time[sec]')
plt.ylabel('roll[rad]')
plt.legend()


plt.subplot(3,2,5)
plt.plot(df['t'],df['p5'],label="ref")
plt.plot(df['t'],df['pc5'],label="cmd")
plt.xlabel('time[sec]')
plt.ylabel('pitch[rad]')
plt.legend()


plt.subplot(3,2,6)
plt.plot(df['t'],df['p6'],label="ref")
plt.plot(df['t'],df['pc6'],label="cmd")
plt.xlabel('time[sec]')
plt.ylabel('yaw[rad]')
plt.legend()


plt.show()
