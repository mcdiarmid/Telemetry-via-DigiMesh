from numpy import cos, sin, pi, sqrt, arctan, exp, log10
import numpy as np

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
from matplotlib.lines import Line2D

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


def azimuth(d, ht, hr):
    return arctan((hr-ht)/d)


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
h_g = 15
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
surf_res = 100
phi_bws = np.array([12.5, 30, 50, 70])*pi/180
hatches = ('+', '*','\\', 'o')

# Variables
relay_height = 250
d_const = 2000
n_y = 4001
n_x = 10001

d_start, d_end = 10**1, 10**5
d_arr = np.logspace(log10(d_start), log10(d_end), n_x, base=10)

h_start, h_end = 10**1, 10**3
h_arr = np.logspace(log10(h_start), log10(h_end), n_y, base=10)

x,y = np.meshgrid(d_arr, h_arr)
i_xc = np.argmax(d_arr>=d_const)
i_yc = np.argmax(h_arr>=relay_height)

# Calculate
fspl_d = fs_path_loss(d_arr, h_d, relay_height, carrier_freq)
loss_2r_d = two_ray_loss(d_arr, h_d, relay_height, carrier_freq, er)


fspl_h = fs_path_loss(d_const, h_d, h_arr, carrier_freq)
loss_2r_h = two_ray_loss(d_const, h_d, h_arr, carrier_freq, er)


# Convert to dB link budget
lb_fspl_d = 20*log10(fspl_d) + Gl + Ptx
lb_2ray_d = 20*log10(loss_2r_d) + Gl + Ptx

lb_fspl_h = 20*log10(fspl_h) + Gl + Ptx
lb_2ray_h = 20*log10(loss_2r_h) + Gl + Ptx

z = two_ray_loss(x, h_d, y, carrier_freq, er)
z = 20*log10(z) + Gl + Ptx

# Plot everything
fig = plt.figure(1, constrained_layout=True)
ax1, ax2 = fig.subplots(2,1)
ax1.plot(d_arr, lb_2ray_d)
ax1.plot(d_arr, lb_fspl_d, '--')
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
crop = 0
zc = z[crop:, :]
yc = y[crop:, :]
xc = x[crop:, :]

start_inds_x = local_maxima_indicies(zc[0, :])
start_inds_y = local_maxima_indicies(zc[:, 0])
end_inds_x = local_maxima_indicies(zc[-1,:])
end_inds_y = local_maxima_indicies(zc[:,-1])
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

        xs, ys, zs = xc[i, j], yc[i, j], zc[i, j]
        ind_pairs.append([(i, j), xs, ys, zs])
        scatters[:, ind + start*len(dis_pairs)] = log10(xs), log10(ys), zs


# Max lines
# Sccatter on Surface
##ax_3d.scatter(*scatters, color='k', linewidth=1, alpha=1, marker='^')


# Heatmap and local max lines
fig_grads = plt.figure(len(figs)+2, constrained_layout=True, figsize=(10.5, 13))
ax_grads = fig_grads.subplots(3, 1)
ax_grads_color, ax_grads_log, ax_grads_linear = ax_grads
cmap2d = ax_grads_color.pcolormesh(
    x[::5, ::5], y[::5, ::5], z[::5, ::5],
    alpha=1, cmap=palette, vmin=-120, vmax=-30)
colorbar_2d = fig_grads.colorbar(cmap2d, ax=ax_grads_color, alpha=1, aspect=10)
colorbar_2d.set_label('Received Power, $P_{Rx}$ (dB)')

fig_gcs_relay = plt.figure(len(figs)+3, constrained_layout=True)
axis_gcs_relay = fig_gcs_relay.subplots(1, 2)

# Local Maxima Calcs
for i in range(0, n_y, n_y//20):
    max_inds = local_maxima_indicies(z[i,:])
    y_i = y[i, 0]
    x_vals =  x[0, max_inds]
    y_vals = y_i * np.ones(x_vals.shape)
    for axis in (ax_grads_log, ax_grads_linear):
        line, = axis.plot(x_vals, y_vals, 'ko', ms=0.75 , alpha=0.9)
        if not i:
            line.set_label('Local Maxima')

# Approximation
for i in range(len(ind_pairs)//2):
    start = ind_pairs[2*i]
    end = ind_pairs[2*i+1]
    _, x1, y1, z1 = start
    _, x2, y2, z2 = end

    b = log10(y2) - log10(x2)
    m = 10**b
    ax_grads_color.plot([x1, x2], [y1,y2], 'k:', alpha=0.2)
    
    for axis in (ax_grads_log, ax_grads_linear):
        line, = axis.plot([x1, x2], [y1,y2], 'k-', alpha=0.7)
        if not i:
            line.set_label('Approximation')


# Draw threshold lines for different azimuth angles
for scale, axis in (('linear', ax_grads_linear), ('logarithmic', ax_grads_log)):
    legend = axis.legend(
        loc='lower right', fontsize='medium',
        labelspacing=1, handletextpad=1)

    axis.set_title(f'Maximum Constructive Interference Approximation ({scale} scale)')

fig_grads.suptitle(f'Local Optima Map, $f_c={carrier_freq//1e6}$ MHz, $h_d={h_d}$ m', fontsize=17)
ax_grads_color.set_title('Received Power Heatmap')
for i, axis in enumerate(ax_grads):
    axis.set_xlabel('Horizontal Distance $d$ (m)')
    axis.set_ylabel('Relay Height $h_r$ (m)')
    axis.set_ylim([h_start, h_end])
    if i < 2:
        axis.set_xlim([d_start, d_end])
        axis.set_xscale('log')
        axis.set_yscale('log')
    else:
        axis.set_xlim([d_start, 4000])


# GCS -- Relay stuff
phi_i  = theta(x, h_g, y)
phi_az = azimuth(x, h_g, y)

th_ys = []
th_checks = []


for phi_bwv in phi_bws:
    y_tmp = sin(phi_bwv)*d_arr + h_g
    th_ys.append(y_tmp)
    
    th_tmp = phi_i - phi_bwv/2  + phi_az <= 1e-4
    th_d = np.zeros(h_arr.shape)
    for iy in range(n_y):
        ix = th_tmp[iy, :].argmax(axis=0)
        th_d[iy] = d_arr[ix]
    th_checks.append(th_d)

axis_gcs_relay[0].fill_between(
    [d_start, d_end], [h_g, h_g], [0, 0],
    color='k', alpha=0.7, hatch='.',
    label='$\phi_{az} \leq 0^\circ$')

axis_gcs_relay[1].fill_betweenx(
    h_arr, th_checks[0], np.ones(h_arr.shape)*d_end,
    color='k',
    alpha=0.25,
    hatch=hatches[0],
    label='$\phi_{bw, v}= 'f'{phi_bws[0]*180/pi:.1f}^\circ$')

for i in range(len(th_ys)-1):
    angle = phi_bws[i]*180/pi
    angle2 = phi_bws[i+1]*180/pi
    axis_gcs_relay[0].fill_between(
        d_arr, th_ys[i], th_ys[i+1],
        color='k', alpha=0.3+0.25*(i%2), hatch=hatches[i],
        label='$\phi_{bw, v} =' f'{angle:.1f}^\circ$')

    axis_gcs_relay[1].fill_betweenx(
        h_arr, th_checks[i+1], th_checks[i],
        color='k', alpha=0.5-0.25*(i%2), hatch=hatches[i+1],
        label='$\phi_{bw, v} =' f'{angle2:.1f}^\circ$')

axis_gcs_relay[0].fill_between(
    d_arr, th_ys[-1], np.ones(d_arr.shape)*d_end,
    color='k',
    alpha=0.3+0.25*((i+1)%2),
    hatch=hatches[-1],
    label='$\phi_{bw, v} =' f'{phi_bws[i+1]*180/pi:.1f}^\circ$')


for axis, scale, name, loc in zip(axis_gcs_relay,
                                  ('log', 'log'),
                                  ('Fixed Elevation Angle $\phi_{e} = \phi_{bw, v} \div 2$', 'Antenna Tracking $\phi_{e} = \phi_{az}$'),
                                  ('center right', 'upper left')):
    axis.set_xscale(scale)
    axis.set_yscale(scale)
    axis.set_title(name)
    axis.set_xlim([d_start, d_end])    
    axis.set_ylim([h_start, h_end])
    
    axis.set_xlabel('Horizontal Distance $d$ (m)')
    axis.set_ylabel('Relay Height $h_r$ (m)')
    
    legend2 = axis.legend(
        loc='center right', fontsize='medium',
        labelspacing=1, handletextpad=1)

    for handle in legend2.legendHandles:
        handle.set_height(8)

title = fig_gcs_relay.suptitle('Spatial Constraints of Directional Antenna Use', fontsize=17)


# GCS -- Relay FSPL
Gl_24 = 3 + 6 + 0 + 0
fspl_2ghz = 20*log10(fs_path_loss(x, h_g, y, 2.4e9))
fig_2ghz = plt.figure(len(figs)+4, constrained_layout=True)
sp_2ghz = fig_2ghz.subplots(1,1)

MCS = [
    (28, -96), # 0
    (28, -95), # 1
    (28, -92), # 2
    (28, -90), # 3
    (27, -86), # 4
    (25, -83), # 5
    (23, -77), # 6
    (22, -74), # 7
    
    (28, -95), # 8
    (28, -93), # 9
    (28, -90), # 10
    (28, -87), # 11
    (27, -84), # 12
    (25, -79), # 13
    (23, -78), # 14
    (22, -75), # 15
]

labels = {
    'd': 'Horizontal Distance',
    'h_r': 'Relay Height'
    }
consts = {
    'h_r': relay_height,
    'd': d_const,
}


x_arr, y_arr = d_arr, fspl_2ghz[i_yc, :]
var, con = 'd', 'h_r'
axis = sp_2ghz
legend_items = []
tbl_list = []

for i, (tx, rx) in enumerate(MCS):
    # Calculate P_rx and identify intersection
    p_rx_arr = y_arr + Gl_24 + tx
    j = np.argmax(p_rx_arr<rx)

    # Plot stuff
    color = f'C{i%8}'
    if i < 8:
        axis.plot(x_arr, p_rx_arr, f'C{i%8}-')
        legend_items.append(Line2D([0], [0], color=color, ls='-', label=f'MCS {i}, {i+8}'))
        marker = ':'
    else:
        marker = '-.'
        
    axis.plot([x_arr[0], x_arr[-1]], [rx, rx], f'{color}{marker}')
    
    # Determine more accurate estimate for exact $d$
    x_min, x_max = int(x_arr[j-1]), int(x_arr[j+1])+1
    d_tmp = np.arange(x_min, x_max)
    p_tmp = 20*log10(fs_path_loss(d_tmp, h_g, relay_height, 2.4e9)) + Gl_24 + tx
    k = np.argmax(p_tmp<rx)
    tbl_list.append(d_tmp[k])
    
# Print out for latex table format
for i in range(len(tbl_list)//2):
    print(f'\t\t{i} & {tbl_list[i]:>5} m & {i+8:>2} & {tbl_list[i+8]:>5} m\\\\')

legend_items.extend([
    Line2D([0], [0], color='k', ls=':', label='$P_{RS}$, MCS0-7'),
    Line2D([0], [0], color='k', ls='-.', label='$P_{RS}$, MCS8-15'),
    ])

axis.set_xlim([x_arr[0], x_arr[-1]])
axis.set_xscale('log')
axis.set_xlabel(f'{labels[var]} ${var}$ (m)')
axis.set_ylabel('Received Power, $P_{Rx}$ (dB)')
axis.set_title(f'Varying ${var}$, ${con}$ = {consts[con]} m')
axis.legend(handles=legend_items)
axis.set_ylim([-100, -40])
axis.grid()

fig_2ghz.suptitle(f'FSPL for GCS - Relay Link, $f_c = 2.4$ GHz', fontsize=17)
plt.show(block=False)
