from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

df = pd.read_csv("log_turnover.csv", header = 0)


fig = plt.figure()
ax = Axes3D(fig)
ax.plot(df['p1'],df['p2'],df['p3']) #<---ここでplot
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')


plt.savefig('figure.png')
plt.show()
