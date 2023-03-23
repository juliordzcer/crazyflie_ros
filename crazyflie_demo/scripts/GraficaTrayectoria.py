import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

h = 0.6
r = 0
w = np.pi/9.
p = 15

t = np.linspace(0, 100, 1000)

x = r * (np.arctan(p) + np.arctan(t - p)) * np.cos(w * t) - 0.5
y =  r * (np.arctan(p) + np.arctan(t - p)) * np.sin(w * t)
z = (h/2) * (1 + np.tanh(t-7.5))
yaw = 0

fig = plt.figure()
ax = fig.add_subplot(111, projection=Axes3D.name)

ax.plot(x, y, z)

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

plt.show()
