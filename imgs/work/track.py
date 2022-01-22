import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Arc

plt.rcParams.update({'font.size': 15, 'mathtext.fontset': 'cm'})

track = pd.read_csv('../../data/wp_yaw_const.csv', names=['x', 'y', 'z', 'yaw'])

fig, ax = plt.subplots(figsize=(10, 10), dpi=60)

ax.scatter(track["x"], track["y"], color='blue', s=1)
ax.scatter(track["x"][0], track["y"][0], color='red', s=50)

# ax.set_xlim(0, 2500)
# ax.set_ylim(800, 3300)
# ax.set_aspect(1.0 / ax.get_data_ratio(), adjustable='box')
ax.set_xlabel("x [m]")
ax.set_ylabel("y [m]")
# ax.legend()

# axi = ax.inset_axes([0.25, 0.25, 0.5, 0.5])
# axi.set_xlim(2220, 2320)
# axi.set_ylim(2920, 3020)
# axi.set_xticks([])
# axi.set_yticks([])
# ax.indicate_inset_zoom(axi)

# axi.plot(x, y, 'r', zorder=0)
# axi.plot(x - 10 * dx - 0.001 * (s - 6100) * dx,
#          y - 10 * dy - 0.055 * (s - 6100) * dy,
#          c='g', ls='--', zorder=0)

# Reference Line
# t = 6100
# axi.arrow(x[t], y[t], dx[t] * 20, dy[t] * 20, head_width=2, head_length=4,
#           fc='k')
# axi.annotate(r'$\vec{n_r}$', xy=(x[t] + dx[t] * 16, y[t] + dy[t] * 8))
# axi.arrow(x[t], y[t], sx[t] * 20, sy[t] * 20, head_width=2, head_length=4,
#           fc='k')
# axi.annotate(r'$\vec{t_r}$', xy=(x[t] + sx[t] * 30, y[t] + sy[t] * 10))
# theta
# dia = dx[t] * 25
# arc = Arc((x[t], y[t]), dia, dia, angle=0,
#           theta1=0, theta2=np.rad2deg(theta[t]),
#           linestyle="-", color='k')
# axi.add_patch(arc)
# axi.arrow(x[t] + dia / 2 * np.cos(theta[t]),
#           y[t] + dia / 2 * np.sin(theta[t]) - 0.3,
#           -1e-6 * dia / 2 * np.sin(theta[t]),
#           +1e-6 * dia / 2 * np.cos(theta[t]),
#           head_width=2, head_length=3, fc='k', length_includes_head=True)
# axi.hlines(y[t], x[t] + dia / 2 - 1, x[t] + dia / 2 + 1)
# axi.annotate(r'${\theta_r}$', xy=(x[t] - 5, y[t] + 12))
# s(t)
# axi.plot(x[t], y[t], 'o', c='gray', ms=10,
#          markeredgecolor='white', markeredgewidth=2, zorder=10)
# axi.annotate(r'$s(t)$', xy=(x[t] - 12, y[t] - 5))

# trajectory
# x_t = x[t] + dx[t] * 46.5
# y_t = y[t] + dy[t] * 46.5
# axi.plot(x_t, y_t, 'o', c='gray', ms=10,
#          markeredgecolor='white', markeredgewidth=2, zorder=10)
# axi.annotate(r'$\vec{x}(s, d)$', xy=(x_t + 3, y_t - 5))
# rot = -22
# sx_t = (sx[t] * 20 * np.cos(np.deg2rad(rot))
#         - sy[t] * 20 * np.sin(np.deg2rad(rot)))
# sy_t = (sx[t] * 20 * np.sin(np.deg2rad(rot))
#         + sy[t] * 20 * np.cos(np.deg2rad(rot)))
# axi.arrow(x_t, y_t, sx_t, sy_t, head_width=2, head_length=4, fc='k')
# axi.annotate(r'$\vec{n_x}$', xy=(x_t + 15, y_t + 12))
# axi.arrow(x_t, y_t, sy_t, -sx_t, head_width=2, head_length=4, fc='k')
# axi.annotate(r'$\vec{t_x}$', xy=(x_t - 5, y_t + 20))

# connection
# axi.plot([x[t], x_t], [y[t], y_t], c='gray', zorder=0)
# axi.annotate(r'$d(t)$', xy=(x[t] + dx[t] * 16, y[t] + dy[t] * 35))

# axi.annotate('center line', xy=(2225, 2973), c='r', size=12)
# axi.annotate('trajectory\ncandidate', xy=(2244, 3008), c='g', size=12)

# plt.show()
plt.savefig('track.png')
