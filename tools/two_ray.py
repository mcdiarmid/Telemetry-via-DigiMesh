from numpy import cos, sin, pi, sqrt, arctan, exp, log10
import numpy as np

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter


c = 299792458


def R_val(er, theta):
    z = sqrt(er-cos(theta)**2)/er  # Vertical Polarization
    s = sin(theta)
    return (s-z)/(s+z)


def nlos_path(d, ht, hr):
    return sqrt((ht+hr)**2 + d**2)


def los_path(d, ht, hr):
    return sqrt((ht-hr)**2 + d**2)


def delta_phi(d, ht, hr, wl):
    nlos = nlos_path(d, ht, hr)
    los = los_path(d, ht, hr)
    return 2*pi*(nlos-los)/wl


def delta_phi_approx(d, ht, hr, wl):
    return 4*pi*ht*hr/(wl*d)


def theta(d, ht, hr):
    return arctan((hr+ht)/d)


def wavelength(f):
    return c/f


def two_ray_loss(d, ht, hr, f, er):
    th = theta(d, ht, hr)
    R = R_val(er, th)
    wl = wavelength(f)
    l = los_path(d, ht, hr)
    rr = nlos_path(d, ht, hr)
    d_phi = delta_phi(d, ht, hr, wl)
    return wl/(4*pi) * np.abs((1/l)+(R*exp(d_phi*complex(0,1))/rr))


def fs_path_loss(d, ht, hr, f):
    wl = wavelength(f)
    l = los_path(d, ht, hr)
    return wl/(4*pi*l)



# Known Constants
gcs_height = 15
carrier_freq = 915e6
wave_len = wavelength(carrier_freq)
er = 3.14
Gtx = 2.1  # dBi gain of tx antenna
Grx = 2.1  # dBi gain of rx antenna
Ltx = 0  # Tx losses
Lrx = 0  # Rx losses
Gl = Gtx+Grx-Ltx-Lrx  # Total hardware gain (dBi)
Ptx = 24  # dBm tx power
Prx = -101  # Receiver sensitivity dBm

# Variables
relay_height = 200.727

d_start = 10**1
d_end = 10**5
d_arr = np.logspace(log10(d_start), log10(d_end), 4000, base=10)

h_start = 10**1
h_end = 10**3
h_arr = np.logspace(log10(h_start), log10(h_end), 1000, base=10)
l_arr = los_path(d_arr, gcs_height, relay_height)


# Calculate
fspl = fs_path_loss(d_arr, gcs_height, relay_height, carrier_freq)
loss_2r = two_ray_loss(d_arr, gcs_height, relay_height, carrier_freq, er)


fspl_h = fs_path_loss(2000, gcs_height, h_arr, carrier_freq)
loss_2r_h = two_ray_loss(2000, gcs_height, h_arr, carrier_freq, er)


# Convert to dB link budget
lb_fspl = 20*log10(fspl) + Gl + Ptx
lb_2ray = 20*log10(loss_2r) + Gl + Ptx

lb_fspl_h = 20*log10(fspl_h) + Gl + Ptx
lb_2ray_h = 20*log10(loss_2r_h) + Gl + Ptx


# Plot everything
fig = plt.figure(1)
fig.suptitle(f'Simulated Channel Response, $f_c$ = {carrier_freq//1e6}MHz', fontsize=14)
ax1 = plt.subplot(311)
ax1.plot(d_arr, lb_2ray)
ax1.plot(d_arr, lb_fspl, '--')
ax1.plot((d_start, d_end), (Prx, Prx), ':k')
ax1.set_xscale('log')
ax1.set_ylabel('Received Power, $P_{Rx}$ (dB)')
ax1.legend(['Two-Ray Model', 'Free-Space Path-Loss', 'Receiver Sensitivity'])
ax1.set_xlim(d_start, d_end)
ax1.set_ylim(-110, -40)
ax1.grid()

ax2 = plt.subplot(312)
ax2.plot(d_arr, delta_phi(d_arr, gcs_height, relay_height, wavelength(carrier_freq)))
ax2.set_xscale('log')
ax2.set_ylabel('Phase Difference, $\Delta\phi$ (rad)')
ax2.set_xlabel('Horizontal Distance, $d$ (m)')
ax2.set_xlim(d_start, d_end)
ax2.set_ylim(0)

ax3 = plt.subplot(313)
ax3.plot(h_arr, lb_2ray_h)
ax3.plot(h_arr, lb_fspl_h, '--')
ax3.plot((h_start, h_end), (Prx, Prx), ':k')
ax3.set_xscale('log')
ax3.set_ylabel('Received Power, $P_{Rx}$ (dB)')
ax3.set_xlabel('Relay Height, $h_r$ (m)')
ax3.legend(['Two-Ray Model', 'Free-Space Path-Loss', 'Receiver Sensitivity'])
ax3.set_xlim(h_start, h_end)
ax3.set_ylim(-110, -40)
ax3.grid()

# 3-D Graphic of Two-Ray model
x,y = np.meshgrid(d_arr, h_arr)
z = two_ray_loss(x, gcs_height, y, carrier_freq, er)
z = 20*log10(z) + Gl + Ptx

fig2 = plt.figure(2)
ax_3d = fig2.gca(projection='3d')
surf = ax_3d.plot_surface(log10(x), log10(y), z,
                          cmap='jet', linewidth=0, antialiased=False)
ax_3d.set_zlim(-120, -20)
ax_3d.zaxis.set_major_locator(LinearLocator(10))
fig2.colorbar(surf, shrink=0.5, aspect=5)

plt.show(block=False)
