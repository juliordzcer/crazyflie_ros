import numpy as np
import matplotlib.pyplot as plt

h = 1
r = .2
w = np.pi/9.
p = 8

t = np.linspace(0, 100, 1000)

# Cálculo de las coordenadas x, y, z y sus derivadas
x = r * (np.arctan(p) + np.arctan(t - p)) * np.cos(w * t)
y =  r * (np.arctan(p) + np.arctan(t - p)) * np.sin(w * t)
z = 1.5 * t/t #(h/2) * (1 + np.tanh(t-7.5))

dx = -r * (np.arctan(p) + np.arctan(t - p)) * w * np.sin(w * t)
dy = r * (np.arctan(p) + np.arctan(t - p)) * w * np.cos(w * t)
dz = (h/2) * np.tanh(t-7.5) * (1 - np.tanh(t-7.5))

ddx = -r * (np.arctan(p) + np.arctan(t - p)) * w**2 * np.cos(w * t)
ddy = -r * (np.arctan(p) + np.arctan(t - p)) * w**2 * np.sin(w * t)
ddz = (h/2) * np.tanh(t-7.5) * (1 - np.tanh(t-7.5)) * (1 - 2*np.tanh(t-7.5)) + 9.8

# Creación de las figuras
fig1, ax1 = plt.subplots()
ax1.plot(t, x, label='x')
ax1.plot(t, y, label='y')
ax1.plot(t, z, label='z')
ax1.set_xlabel('Tiempo')
ax1.set_ylabel('Posición')
ax1.legend()

fig2, ax2 = plt.subplots()
ax2.plot(t, dx, label='dx')
ax2.plot(t, dy, label='dy')
ax2.plot(t, dz, label='dz')
ax2.set_xlabel('Tiempo')
ax2.set_ylabel('Velocidad')
ax2.legend()

fig3, ax3 = plt.subplots()
ax3.plot(t, ddx, label='ddx')
ax3.plot(t, ddy, label='ddy')
ax3.plot(t, ddz, label='ddz')
ax3.set_xlabel('Tiempo')
ax3.set_ylabel('Aceleración')
ax3.legend()

plt.show()
