import numpy as np
import math
import os
# Nombre del archivo a crear
archivo = "trajectory.py"

r = 0.15
h = 0.5
w = np.pi/9 
T = 100 
p = 15
n = 2       # Numero de digitos redondeados. 
nT = 50000  # Cantidad de muestras.
dt = T / nT


x_prev  = 0
xp_prev = 0
y_prev  = 0
yp_prev = 0
z_prev  = 0
zp_prev = 0
yaw_prev = 0
yawp_prev = 0


with open(archivo, "w") as f:
    f.write("#!/usr/bin/env python3\nfrom RunTraj import Demo\nif __name__ == '__main__':\n    demo = Demo(\n        [\n" )

    for t in np.linspace(0, T, nT):
        x = r * (np.arctan(p) + np.arctan(t - p)) * np.cos(w * t)
        y = r * (np.arctan(p) + np.arctan(t - p)) * np.sin(w * t)
        z = (h/2) * (1 + np.tanh(t-7.5))
        yaw = 0
        
        xp = (x - x_prev) / dt
        yp = (y - y_prev) / dt
        zp = (z - z_prev) / dt
        yawp = (yaw - yaw_prev) / dt

        xpp = (xp - xp_prev) / dt
        ypp = (yp - yp_prev) / dt
        zpp = (zp - zp_prev) / dt
        yawpp = (yawp - yawp_prev) / dt

        f.write("           [{},{},{},{},{},{},{},{},{},{},{},{},{}],\n".format( round(x, n), round(y, n), round(z, n), round(yaw, n),round(xp, n), round(yp, n), round(zp, n), round(yawp, n),round(xpp, n), round(ypp, n), round(zpp, n), round(yawpp, n), dt))

        x_prev  = x
        xp_prev = xp
        y_prev  = y
        yp_prev = yp
        z_prev  = z
        zp_prev = zp
        yaw_prev = yaw
        yawp_prev = yawp

    f.write("        ]\n    )\n    demo.run()")

os.chmod(archivo, 0o755)