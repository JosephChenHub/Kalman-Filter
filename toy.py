
import numpy as np
import matplotlib.pyplot as plt

dt = 0.02
sensor_std = 0.08
speed_std = 0.2

t = np.arange(0, 10, dt)
x = 0.05 * t * np.cos(2*t)
n = x.shape[0]

u = np.insert(np.diff(x)/dt, 0, 0) + speed_std * np.random.randn(n)

z = x - sensor_std * np.random.randn(n)

plt.plot(t, x, 'r')
plt.plot(t, z, 'g')
plt.show()


out = ""
for i in range(t.shape[0]):
    out += str(t[i]) + ","
out = out[:-1]
out += "\n"
for i in range(u.shape[0]):
    out += str(u[i]) + ","
out = out[:-1]
out += "\n"
for i in range(x.shape[0]):
    out += str(x[i]) + ","
out = out[:-1]
out += "\n"
for i in range(z.shape[0]):
    out += str(z[i]) + ","
out = out[:-1]

with open("data.txt", "w") as fout:
    fout.writelines(out)
