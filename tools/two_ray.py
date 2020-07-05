from numpy import cos, sin, pi, sqrt, arctan, exp, log10
import numpy as np

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter


##from sys import version_info
##if version_info[0] < 3:
##    import pyface.qt

from mayavi import mlab


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


def local_maxima_indicies(arr):
    max_inds = []
    for i, val in enumerate(arr[1:-2], start=1):
        if val > arr[i-1] and val > arr[i+1]:
            max_inds.append(i)
    return max_inds


# Known Constants
h_d = 5
carrier_freq = 915e6
wave_len = wavelength(carrier_freq)
er = 3.17
Gtx = 2.1  # dBi gain of tx antenna
Grx = 2.1  # dBi gain of rx antenna
Ltx = 0  # Tx losses
Lrx = 0  # Rx losses
Gl = Gtx+Grx-Ltx-Lrx  # Total hardware gain (dBi)
Ptx = 24  # dBm tx power
Prx = -101  # Receiver sensitivity dBm
surf_res = 5

# Variables
relay_height = 250
d_const = 2000
n_y = 2000
n_x = 4000

d_start = 10**1
d_end = 10**5
d_arr = np.logspace(log10(d_start), log10(d_end), n_x, base=10)

h_start = 10**1
h_end = 10**3
h_arr = np.logspace(log10(h_start), log10(h_end), n_y, base=10)
l_arr = los_path(d_arr, h_d, relay_height)


# Calculate
fspl = fs_path_loss(d_arr, h_d, relay_height, carrier_freq)
loss_2r = two_ray_loss(d_arr, h_d, relay_height, carrier_freq, er)


fspl_h = fs_path_loss(d_const, h_d, h_arr, carrier_freq)
loss_2r_h = two_ray_loss(d_const, h_d, h_arr, carrier_freq, er)


# Convert to dB link budget
lb_fspl = 20*log10(fspl) + Gl + Ptx
lb_2ray = 20*log10(loss_2r) + Gl + Ptx

lb_fspl_h = 20*log10(fspl_h) + Gl + Ptx
lb_2ray_h = 20*log10(loss_2r_h) + Gl + Ptx


# Plot everything
fig = plt.figure(1, constrained_layout=True)
ax1, ax2 = fig.subplots(2,1)
ax1.plot(d_arr, lb_2ray)
ax1.plot(d_arr, lb_fspl, '--')
ax1.plot((d_start, d_end), (Prx, Prx), ':k')
ax1.set_xlabel('Horizontal Distance, $d$ (m)')
ax1.set_xlim(d_start, d_end)
ax1.set_title(f'Varying $d$, $h_r={relay_height}$ m') 

ax2.plot(h_arr, lb_2ray_h)
ax2.plot(h_arr, lb_fspl_h, '--')
ax2.plot((h_start, h_end), (Prx, Prx), ':k')
ax2.set_xlabel('Relay Height, $h_r$ (m)')
ax2.set_xlim(h_start, h_end)
ax2.set_title(f'Varying $h_r$, $d={d_const}$ m')

title = fig.suptitle(f'Simulated Channel Response, $f_c$ = {carrier_freq//1e6}MHz, $h_d={h_d}$ m', fontsize=17)
for ax in (ax1, ax2):
    ax.set_xscale('log')
    ax.set_ylim(-110, -40)
    ax.set_ylabel('Received Power, $P_{Rx}$ (dB)')
    ax.legend(['Two-Ray Model', 'Free-Space Path-Loss', 'Receiver Sensitivity'])
    ax.grid()


# 3-D Graphic of Two-Ray model
x,y = np.meshgrid(d_arr, h_arr)
z = two_ray_loss(x, h_d, y, carrier_freq, er)
z = 20*log10(z) + Gl + Ptx

figs = []
palettes = [#'gnuplot', 'gnuplot2', 'terrain', 'CMRmap', 'ocean', 'cool',
            'viridis']
            
for i, palette in enumerate(palettes):
    figs.append(plt.figure(i+2, facecolor=(1, 1, 1)))
    title2 = figs[i].suptitle(f'Simulated Two-Ray Model, $f_c$ = {carrier_freq//1e6}MHz, $h_d={h_d}$ m', fontsize=14)
    ax_3d = figs[i].gca(projection='3d')

    # Plot everything
    surf = ax_3d.plot_surface(
        log10(x), log10(y), z, alpha=0.7,
        cmap=palette, linewidth=0, antialiased=True,
        rstride=surf_res, cstride=surf_res,
        vmin=-120, vmax=-30)

    ax_3d.set_zlim(-120, -30)
    ax_3d.zaxis.set_major_locator(LinearLocator(10))
    ax_3d.set_xlabel('Horizontal Distance Exponent ($d = 10^{x}$m)')
    ax_3d.set_ylabel('Relay Height Exponent ($h_r = 10^{y}$m)')
    ax_3d.set_zlabel('Received Power, $P_{Rx}$ (dB)')
    colorbar_3d = figs[i].colorbar(surf, alpha=1)
    colorbar_3d.set_label('Received Power, $P_{Rx}$ (dB)')
    ax_3d.view_init(42, -105)

# Selected Scatter points
start_inds_x = local_maxima_indicies(z[0, :])
start_inds_y = local_maxima_indicies(z[:, 0])
end_inds_x = local_maxima_indicies(z[-1,:])
end_inds_y = local_maxima_indicies(z[:,-1])
dis_pairs = [-(x+1) for x in range(len(start_inds_x)+len(start_inds_y))]
ind_pairs = []
scatters = np.zeros((3, len(dis_pairs)*2))

for ind, pair_i in enumerate(dis_pairs):
    if pair_i >= 0:
        starts = [start_inds_y[::-1], start_inds_x]
        ends = [end_inds_x, end_inds_y]
        op = -1
    else:
        starts = [start_inds_x, start_inds_y[::-1]]
        ends = [end_inds_y, end_inds_x]
        op = 1

    for start, arrs in ((True, starts), (False, ends)):
        try:
            i, j = (arrs[0][pair_i], 0 if start else -1)[::-op* 1 if start else -1]
            if not start and op == 1:
                i, j = j, i

        except IndexError:
            i, j = (arrs[1][pair_i+op*len(arrs[0])], 0 if start else -1)[::-op* 1 if start else -1]
            if start and op == 1:
                i, j = j, i

        xs, ys, zs = x[i, j], y[i, j], z[i, j]
        ind_pairs.append([(i, j), xs, ys, zs])
        scatters[:, ind + start*len(dis_pairs)] = log10(xs), log10(ys), zs


# Max lines
# Sccatter on Surface
##ax_3d.scatter(*scatters, color='k', linewidth=1, alpha=1, marker='^')


# Heatmap and local max lines
fig_grads = plt.figure(len(figs)+2, constrained_layout=True)
ax_grads = fig_grads.subplots(2, 1)
ax_grads_color, ax_grads_lines = ax_grads
cmap2d = ax_grads_color.pcolormesh(
    x[::5, ::5], y[::5, ::5], z[::5, ::5],
    alpha=1, cmap=palette, vmin=-120, vmax=-30)
colorbar_2d = fig_grads.colorbar(cmap2d, ax=ax_grads, alpha=1)
colorbar_2d.set_label('Received Power, $P_{Rx}$ (dB)')

# Local Maxima Calcs
for i in range(0, n_y, n_y//20):
    max_inds = local_maxima_indicies(z[i,:])
    y_i = y[i, 0]
    x_vals =  x[0, max_inds]
    y_vals = y_i * np.ones(x_vals.shape)
    ax_grads_lines.plot(x_vals, y_vals, 'ko', ms=0.75 , alpha=0.9)


# Approximation
for i in range(len(ind_pairs)//2):
	start = ind_pairs[2*i]
	end = ind_pairs[2*i+1]
	_, x1, y1, z1 = start
	_, x2, y2, z2 = end
	m = (y2-y1)/(x2-x1)
	c = y1-m*x1
	ax_grads_color.plot([x1, x2], [y1,y2], 'k:', alpha=0.2)
	ax_grads_lines.plot([x1, x2], [y1,y2], 'k-', alpha=0.8)


fig_grads.suptitle(f'Local Optima Map, $f_c={carrier_freq//1e6}$ MHz, $h_d={h_d}$ m', fontsize=17)
ax_grads_color.set_title('Received Power Heatmap')
ax_grads_lines.set_title('Maximum Constructive Interference Approximation')
for axis in ax_grads:
    axis.set_xlabel('Horizontal Distance $d$ (m)')
    axis.set_ylabel('Relay Height $h_r$ (m)')
    axis.set_xlim([d_start, d_end])
    axis.set_ylim([h_start, h_end])
    axis.set_xscale('log')
    axis.set_yscale('log')

plt.show(block=False)
