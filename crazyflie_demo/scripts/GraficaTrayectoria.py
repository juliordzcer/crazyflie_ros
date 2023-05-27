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
z = (h/2) * (1 + np.tanh(t-2.5))

dx = -r * (np.arctan(p) + np.arctan(t - p)) * w * np.sin(w * t)
dy = r * (np.arctan(p) + np.arctan(t - p)) * w * np.cos(w * t)
dz = (h/2) * np.tanh(t-2.5) * (1 - np.tanh(t-2.5))

ddx = -r * (np.arctan(p) + np.arctan(t - p)) * w**2 * np.cos(w * t)
ddy = -r * (np.arctan(p) + np.arctan(t - p)) * w**2 * np.sin(w * t)
ddz = (h/2) * np.tanh(t-2.5) * (1 - np.tanh(t-2.5)) * (1 - 2*np.tanh(t-2.5)) 

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
# ax3.plot(t, ddz, label='ddz')
ax3.set_xlabel('Tiempo')
ax3.set_ylabel('Aceleración')
ax3.legend()



plt.show()

# import numpy as np
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D

# # Generar puntos en el tiempo
# t = np.linspace(0, 10, 100)  # Intervalo de tiempo de 0 a 10 segundos

# # Calcular las coordenadas x, y, z
# x = 0.3 * np.cos(2 * np.pi * 0.1 * t)
# y = 0.3 * np.sin(2 * np.pi * 0.1 * t)
# z = 0.7 * np.ones_like(t)  # Coordenada z constante en 0.7

# # Calcular la velocidad y la aceleración
# dx_dt = -0.3 * 2 * np.pi * 0.1 * np.sin(2 * np.pi * 0.1 * t)
# dy_dt = 0.3 * 2 * np.pi * 0.1 * np.cos(2 * np.pi * 0.1 * t)
# dz_dt = np.zeros_like(t)  # La velocidad en z es siempre cero

# d2x_dt2 = -0.3 * (2 * np.pi * 0.1) ** 2 * np.cos(2 * np.pi * 0.1 * t)
# d2y_dt2 = -0.3 * (2 * np.pi * 0.1) ** 2 * np.sin(2 * np.pi * 0.1 * t)
# d2z_dt2 = np.zeros_like(t)  # La aceleración en z es siempre cero

# # Crear subgráficos
# fig, axs = plt.subplots(3, 1, figsize=(8, 12))

# # Gráfico de posición
# axs[0].plot(t, x, label='x')
# axs[0].plot(t, y, label='y')
# axs[0].plot(t, z, label='z')
# axs[0].set_xlabel('Tiempo [s]')
# axs[0].set_ylabel('Posición [m]')
# axs[0].set_title('Posición vs. Tiempo')
# axs[0].legend()

# # Gráfico de velocidad
# axs[1].plot(t, dx_dt, label='Vx')
# axs[1].plot(t, dy_dt, label='Vy')
# axs[1].plot(t, dz_dt, label='Vz')
# axs[1].set_xlabel('Tiempo [s]')
# axs[1].set_ylabel('Velocidad [m/s]')
# axs[1].set_title('Velocidad vs. Tiempo')
# axs[1].legend()

# # Gráfico de aceleración
# axs[2].plot(t, d2x_dt2, label='Ax')
# axs[2].plot(t, d2y_dt2, label='Ay')
# axs[2].plot(t, d2z_dt2, label='Az')
# axs[2].set_xlabel('Tiempo [s]')
# axs[2].set_ylabel('Aceleración [m/s²]')
# axs[2].set_title('Aceleración vs. Tiempo')
# axs[2].legend()

# # Crear la figura 3D
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')

# # Graficar la trayectoria
# ax.plot(x, y, z)

# # Configurar etiquetas y título del gráfico
# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('Z')
# ax.set_title('Trayectoria en 3D')

# # Ajustar el espaciado entre subgráficos
# plt.tight_layout()

# # Mostrar los gráficos
# plt.show()
