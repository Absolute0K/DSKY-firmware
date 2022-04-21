""" AGC Simulation ###
# @author Jae Choi
# 3 Bodies simulation inspired by 
# https://github.com/zaman13/Three-Body-Problem-Gravitational-System
"""

from matplotlib.widgets import Slider, Button, RadioButtons
from matplotlib import animation, rc
import multiprocessing as mp
import io
import serial
import os
import pylab as py
import numpy as np
from time import sleep
import matplotlib

matplotlib.rcParams["toolbar"] = "None"


def force_bodies(r_1, r_2, body_1, body_2):
    M1 = 0
    M2 = 0
    F = np.zeros(2)
    r = np.zeros(2)
    if body_1 == "earth" and body_2 == "commanche":
        M1 = Mc
        M2 = Mp
    elif body_1 == "earth" and body_2 == "moon":
        M1 = Ml
        M2 = Mp
    elif body_1 == "moon" and body_2 == "commanche":
        M1 = Mc
        M2 = Ml
    # Get vector difference between the two bodies
    r[0] = r_2[0] - r_1[0]
    r[1] = r_2[1] - r_1[1]
    # Get magnitude of the force and angle
    Fmag = GG * M1 * M2 / (np.linalg.norm(r) + 1e-20)**2
    theta = np.arctan(np.abs(r[1]) / (np.abs(r[0]) + 1e-20))
    F[0] = Fmag * np.cos(theta)
    F[1] = Fmag * np.sin(theta)
    if r[0] > 0:
        F[0] = -F[0]
    if r[1] > 0:
        F[1] = -F[1]
    return F


# Force on the body
def force(r, body, ro, vo):
    if body == "commanche":
        return force_bodies([0, 0], r, "earth", "commanche") + force_bodies(
            ro, r, "moon", "commanche")
    if body == "moon":
        return force_bodies([0, 0], r, "earth", "moon") - force_bodies(
            ro, r, "moon", "commanche")


# Acceleration
def dv_dt(t, r, v, body, ro, vo):
    F = force(r, body, ro, vo)
    if body == "commanche":
        a = F / Mc
    if body == "moon":
        a = F / Ml
    return a


# Differential equation solvers
# ===================================================================
def RK4Solver(t, r, v, dt, body, ro, vo):
    k11 = v
    k21 = dv_dt(t, r, v, body, ro, vo)
    k12 = v + dt * k21 / 2.0
    k22 = dv_dt(t + dt / 2.0, r + dt * k11 / 2.0, v + dt * k21 / 2.0, body, ro,
                vo)
    k13 = v + dt * k22 / 2.0
    k23 = dv_dt(t + dt / 2.0, r + dt * k12 / 2.0, v + dt * k22 / 2.0, body, ro,
                vo)
    k14 = v + dt * k23
    k24 = dv_dt(t + dt, r + dt * k13, v + dt * k23, body, ro, vo)

    y0 = r + dt * (k11 + 2.0 * k12 + 2.0 * k13 + k14) / 6.0
    y1 = v + dt * (k21 + 2.0 * k22 + 2.0 * k23 + k24) / 6.0
    z = np.zeros([2, 2])
    z = [y0, y1]
    return z


# =====================================================================
# Constants
Me = 6e24  # Mass of Earth (kg)
Mm = 6.4e23  # Mass of Mars (kg)
Mj = 1.9e27  # Mass of Jupiter (kg)
Ms = 5.7e26  # Mass of Saturn (kg)
Ms = 2e30  # Mass of Sun (kg)
Mcsm = 14e3  # Mass of Command Service Module (approx 14,000 kg)
Mll = 7.347e22  # Mass of Luna (kg)
G = 6.673e-11  # Gravitational Constant
Re = 1.496e11  # 1 AU - Distance from Earth to the Sun
Rm = 3.844e5  # 1 AU - Distance from Earth to the Sun
# Setting bodies' mass values
Mp = Me
Mc = Mcsm
Ml = Mll
SS = 10e3
# Normalization parameters
RR = Rm * SS
# RR = Re
MM = Mp * SS
TT = 365 * 24 * 60 * 60.0
FF = (G * MM**2) / RR**2  # Unit force
GG = (MM * G * TT**2) / (RR**3)
Mc = Mc / MM  # Normalized mass of CSM
Mp = Mp / MM  # Normalized mass of Planet
Ml = Ml / MM  # Normalized mass of Moon

t_i = 0  # initial time = 0
t_f = 1200  # final time = ?

N = 100 * t_f  # Max number of points for the array - 100 points per year
t = np.linspace(t_i, t_f, N)  # time array from t_i to t_f with N points
dt = t[2] - t[1]  # time step (uniform)

# Initial Conditions
ri = [1.0, 0]  # initial position of Command Service Module
rm_i = [5.0, 0]  # initial position of Moon
# Magnitude of Command Service Module's initial velocity
v_i_abs = np.sqrt(Mp * GG / ri[0])
# Magnitude of Command Service Module's initial velocity
vm_i_abs = np.sqrt(Mp * GG / (rm_i[0]))

print("GG: {} FF: {} vi: {} vm_i: {} Mp: {} Ml: {}".format(
    GG, FF, v_i_abs, vm_i_abs, Mp, Ml))

# Initial velocity vector for Command Service Module - along y.
vi = [0, v_i_abs * 1.0]
vmi = [0, vm_i_abs * 1.0]  # Initial velocity vector for Moon

# Initialization
r = np.zeros([N, 2])  # position vector of Command Service Module
v = np.zeros([N, 2])  # velocity vector of Command Service Module
rm = np.zeros([N, 2])  # position vector of Moon
vm = np.zeros([N, 2])  # velocity vector of Moon

# Initializing the arrays with initial values.
t[0] = t_i
r[0, :] = ri
v[0, :] = vi
rm[0, :] = rm_i
vm[0, :] = vmi

# Function for setting up the animation
py.style.use("dark_background")
fig, ax = py.subplots()

py.subplots_adjust(left=-0.15)
ax.axis("square")
ax.set_xlim((-7.2, 7.2))
ax.set_ylim((-7.2, 7.2))
ax.get_xaxis().set_ticks([])  # enable this to hide x axis ticks
ax.get_yaxis().set_ticks([])  # enable this to hide y axis ticks

ax.plot(0,
        0,
        "o",
        markersize=9,
        markerfacecolor="#0077BE",
        markeredgecolor="#d2eeff")
(line1, ) = ax.plot([], [],
                    "o-",
                    color="#FDB813",
                    markevery=10000,
                    markerfacecolor="#FD7813",
                    lw=2)  # line for Earth
(line2, ) = ax.plot(
    [],
    [],
    "o-",
    color="#f2f2f2",
    markersize=8,
    markerfacecolor="#595959",
    lw=2,
    markevery=10000,
)  # line for Jupiter

# Title and legends
ttl = ax.text(-6.2,
              6.3,
              r"3-Body Problem: Luna, Earth, Spacecraft",
              fontweight="bold")
# Orbital bodies legends at the bottom
ax.plot(-6, -6.2, "o", color="#FDB813", markerfacecolor="#FD7813")
ax.text(-5.5, -6.4, "Apollo Command Module")
ax.plot(2.4,
        -6.2,
        "o",
        color="#f2f2f2",
        markersize=8,
        markerfacecolor="#595959")
ax.text(2.9, -6.4, "Moon")
ax.plot(5,
        -6.2,
        "o",
        markersize=9,
        markerfacecolor="#0077BE",
        markeredgecolor="#d2eeff")
ax.text(5.5, -6.4, "Earth")
py.title(
    "Celestial Mechanics Simulator - 3 Body Problem Solved Using Runge-Kutta 4"
)

speed_i = 1
radius_i = 1
delta_f = 0.05
a_0 = 0.2
axspeed = py.axes([0.70, 0.43, 0.2, 0.03])
axfinalrad = py.axes([0.70, 0.48, 0.2, 0.03])
axtransfer = py.axes([0.70, 0.53, 0.2, 0.03])
axradius = py.axes([0.70, 0.58, 0.2, 0.03])
sspeed = Slider(axspeed,
                "Simulation Speed",
                0.0,
                5.0,
                valinit=speed_i,
                valstep=delta_f)
sfinalrad = Slider(
    axfinalrad,
    "Final Orbit Radius",
    0.1,
    0.5,
    valinit=a_0,
    valstep=delta_f,
    color="gray",
)
stransfrad = Slider(
    axtransfer,
    "Transfer Orbit Radius",
    0.2,
    4.5,
    valinit=a_0,
    valstep=delta_f,
    color="blue",
)
sradius = Slider(axradius,
                 "Orbit Radius",
                 0.5,
                 4.5,
                 valinit=radius_i,
                 valstep=delta_f,
                 color="red")

# Global variables for setting velocity
delta_vc1 = 0.0
delta_vc2 = 0.0
acm = 0.0
target_r2 = 0.0
set_delta_vc1 = False
set_delta_vc2 = False
set_lunar_inject = False
current_vc = np.zeros(2)
current_rc = np.zeros(2)
current_rm = np.zeros(2)
index = 0
offset = 0
reset = False


def arctan(val):
    ret = np.abs(np.arctan(val[1] / val[0] + 1.0e-20))
    if val[0] > 0.0 and val[1] > 0.0:
        return ret
    elif val[0] < 0.0 and val[1] > 0.0:
        return np.pi - ret
    elif val[0] < 0.0 and val[1] < 0.0:
        return np.pi + ret
    elif val[0] > 0.0 and val[1] < 0.0:
        return 2.0 * np.pi - ret
    return ret


def set_vector(mag, arg):
    return [mag * np.cos(arg), mag * np.sin(arg)]


# Animation function. Reads out the positon coordinates sequentially
def animate(i):
    global set_delta_vc1
    global set_delta_vc2
    global set_lunar_inject
    global delta_vc2
    global index
    global offset
    global index
    global reset

    index = i
    i = i - offset
    if reset == True:
        reset = False
    comanche_trail = 80
    moon_trail = 200
    current_vc[:] = v[i, :]
    current_rc[:] = r[i, :]
    current_rm[:] = rm[i, :]
    line1.set_data(
        r[i:max(1, i - comanche_trail):-1, 0],
        r[i:max(1, i - comanche_trail):-1, 1],
    )
    line2.set_data(rm[i:max(1, i - moon_trail):-1, 0],
                   rm[i:max(1, i - moon_trail):-1, 1])
    # Calculate the next point
    [r[i + 1, :],
     v[i + 1, :]] = RK4Solver(t[i], r[i, :], v[i, :], dt * sspeed.val,
                              "commanche", rm[i, :], vm[i, :])
    [rm[i + 1, :],
     vm[i + 1, :]] = RK4Solver(t[i], rm[i, :], vm[i, :], dt * sspeed.val,
                               "moon", r[i, :], v[i, :])
    theta1 = arctan(r[i])
    theta2 = arctan(rm[i])
    dr = np.subtract(rm[i], r[i])
    if (set_delta_vc1 and np.isclose(abs(theta2 - theta1), abs(acm), atol=0.05)
            and theta2 > theta1):
        v[i + 1, :] = v[i, :] + [
            delta_vc1 * np.cos(theta1 + np.pi / 2),
            delta_vc1 * np.sin(theta1 + np.pi / 2),
        ]
        set_delta_vc1 = False
        set_delta_vc2 = True
    if set_lunar_inject == True and np.isclose(
            sfinalrad.val, np.linalg.norm(dr), atol=0.01):
        theta2 = arctan(dr)
        delta_vc2 = np.sqrt(Mp * GG / np.linalg.norm(rm[i]))
        v[i + 1, :] = [
            delta_vc2 * np.cos(theta1 + np.pi / 2),
            delta_vc2 * np.sin(theta1 + np.pi / 2),
        ]
        delta_vc2 = np.sqrt(Ml * GG / sfinalrad.val)
        v[i + 1, :] += [
            delta_vc2 * np.cos(theta2 + np.pi / 2),
            delta_vc2 * np.sin(theta2 + np.pi / 2),
        ]
        set_lunar_inject = False
        set_delta_vc2 = False
        print("Lunar injection")
    elif (set_lunar_inject == False and set_delta_vc2 == True
          and np.isclose(np.linalg.norm(r[i]), target_r2, atol=0.01)):
        v[i + 1, :] = v[i, :] + [
            delta_vc2 * np.cos(theta1 + np.pi / 2),
            delta_vc2 * np.sin(theta1 + np.pi / 2),
        ]
        set_delta_vc2 = False
        print("Hohman")
    tm_yr = "Elapsed time = {:.2f} units".format(t[i] * sspeed.val)
    ttl.set_text(tm_yr)
    return (line1, line2, ttl)


def init():
    line1.set_data([], [])
    line2.set_data([], [])
    ttl.set_text("")
    return (line1, line2, ttl)


# Call animation function
anim = animation.FuncAnimation(fig,
                               animate,
                               init_func=init,
                               frames=None,
                               interval=5,
                               blit=True)


def set_orbits(event):
    global r
    global v
    global rm
    global vm
    global line1
    global line2
    global offset
    global reset
    anim.pause()
    ri = [sradius.val, 0]
    v_i_abs = np.sqrt(Mp * GG / ri[0])
    vm_i_abs = np.sqrt(Mp * GG / (rm_i[0]))
    # Initialization
    vi = [0, v_i_abs * 1.0]
    vmi = [0, vm_i_abs * 1.0]
    r = np.zeros([N, 2])
    v = np.zeros([N, 2])
    rm = np.zeros([N, 2])
    vm = np.zeros([N, 2])
    r[0, :] = ri
    v[0, :] = vi
    rm[0, :] = rm_i
    vm[0, :] = vmi
    offset = index + 1
    reset = True
    anim.resume()


def reset(event):
    sspeed.reset()
    sradius.reset()
    radio.set_active(0)
    set_orbits(event)


def hohmann_transfer(event):
    global target_r2
    global theta1
    global theta2
    global delta_vc1
    global delta_vc2
    global acm
    global set_delta_vc1
    # current_dr = np.subtract(current_rm, current_rc)
    r1 = np.linalg.norm(current_rc)
    target_r2 = r2 = np.linalg.norm(current_rm)
    theta1 = arctan(current_rc)
    theta2 = arctan(current_rm)
    delta_vc1 = np.sqrt(GG * Mp / r1) * (np.sqrt(2.0 * r2 / (r1 + r2)) - 1.0)
    delta_vc2 = np.sqrt(GG * Mp / r2) * (-1.0 * np.sqrt(2.0 * r1 /
                                                        (r1 + r2)) + 1.0)
    acm = np.pi * (1 - np.sqrt((r1 / r2 + 1)**3 / 8.0))
    print("R1:{:.5f} R2:{:.5f} Delta-V: {:.5f} then {:.5f} at {:.5f} rad\n".
          format(r1, target_r2, delta_vc1, delta_vc2, acm))

    GGMp = GG * Mp / 10.0
    rr0 = 1.0 / (sfinalrad.val * 10.0)
    rr1 = 1.0 / (r1 * 10.0)
    rr2 = 1.0 / (r2 * 10.0)
    rr3 = 1.0 / (10.0 * (r1 + target_r2) / 2.0)
    va_sqr_2 = GGMp * rr1
    vb_sqr_2 = GGMp * rr2
    vatx_sqr = GGMp * (2 * rr1 - rr3)
    vbtx_sqr = GGMp * (2 * rr2 - rr3)

    print(
        "GMp:{:.5f} 1/R1N:{:.5f} 1/R2N:{:.5f} ATX:{:.5f}\nVa:{:.5f}({:.5f}) Vb:{:.5f}({:.5f}) Vatx:{:.5f}({:.5f}) Vbtx:{:.5f}({:.5f}) V-delta:{:.10f} -> {:.10f}\n"
        .format(
            GGMp,
            rr1,
            rr2,
            rr3,
            va_sqr_2,
            np.sqrt(va_sqr_2),
            vb_sqr_2,
            np.sqrt(vb_sqr_2),
            vatx_sqr,
            np.sqrt(vatx_sqr),
            vbtx_sqr,
            np.sqrt(vbtx_sqr),
            np.sqrt(vatx_sqr) - np.sqrt(va_sqr_2),
            np.sqrt(vb_sqr_2) - np.sqrt(vbtx_sqr),
        ))
    print("Vbtx_L:{:.5f}({:.5f}) Vbtx_M:{:.5f}({:.5f}) arg:{:.5f}({:.5f},{:.5f})\n"
          .format(np.sqrt(GGMp * rr0), GGMp * rr0, np.sqrt(GGMp * rr2), GGMp * rr2, 1.0 - np.sqrt(((r1/r2)/2.0+1.0/2.0)**3), ((r1/r2)/2.0+1.0/2.0)**3, ((r1/r2)/2.0+1.0/2.0)))
    # print("GMp:{:.3f} 1/R1N:{:.3f} 1/R2N:{:.3f} ATX:{:.3f}\nVa:{:.3f}({:.3f}) Vb:{:.3f}({:.3f}) Vatx:{:.3f}({:.3f}) Vbtx:{:.3f}({:.3f}) V-delta:{:.3f} -> {:.3f}\n"\
    #       .format(GGMp*(2**14), rr1*(2**14), rr2*(2**14), rr3*(2**14), va_sqr_2*(2**14), np.sqrt(va_sqr_2)*(2**14), vb_sqr_2*(2**14), np.sqrt(vb_sqr_2)*(2**14), vatx_sqr*(2**14), \
    #               np.sqrt(vatx_sqr)*(2**14), vbtx_sqr*(2**14), np.sqrt(vbtx_sqr)*(2**14), (np.sqrt(vatx_sqr)-np.sqrt(va_sqr_2))*(2**14), (np.sqrt(vb_sqr_2)-np.sqrt(vbtx_sqr))*(2**14)))
    # print("GMp: {} 1/R1N: {} 1/R2N: {} ATX: {}\nVa: {}({}) Vb: {}({}) Vatx: {}({}) Vbtx: {}({}) V-delta: {} -> {}\n"\
    #       .format(int(np.round(GGMp*(2**14))), int(np.round(rr1*(2**14))), int(np.round(rr2*(2**14))), int(np.round(rr3*(2**14))), int(np.round(va_sqr_2*(2**14))), int(np.round(np.sqrt(va_sqr_2)*(2**14))),\
    #               int(np.round(vb_sqr_2*(2**14))), int(np.round(np.sqrt(vb_sqr_2)*(2**14))), int(np.round(vatx_sqr*(2**14))), int(np.round(np.sqrt(vatx_sqr)*(2**14))), int(np.round(vbtx_sqr*(2**14))), \
    #               int(np.round(np.sqrt(vbtx_sqr)*(2**14))), int(np.round((np.sqrt(vatx_sqr)-np.sqrt(va_sqr_2))*(2**14))), int(np.round((np.sqrt(vb_sqr_2)-np.sqrt(vbtx_sqr))*(2**14)))))
    print("<+{:05o}+{:05o}+{:05o}+{:05o} 10 123 1234>\n".format(
        int(np.round(GGMp * (2**14))),
        int(np.round(rr1 * (2**14))),
        int(np.round(rr2 * (2**14))),
        int(np.round(rr3 * (2**14))),
    ))

    set_delta_vc1 = True


def lunar_injection(event):
    global set_lunar_inject
    hohmann_transfer(event)
    set_lunar_inject = True


resetax = py.axes([0.78, 0.37, 0.07, 0.04])
buttonrst = Button(resetax, "Reset", hovercolor="gray", color="black")
buttonrst.on_clicked(reset)

setax = py.axes([0.70, 0.37, 0.07, 0.04])
buttonset = Button(setax, "Set", hovercolor="gray", color="black")
buttonset.on_clicked(lunar_injection)
sradius.on_changed(set_orbits)

rax = py.axes([0.60, 0.65, 0.23, 0.2], facecolor="#1a1a1a")
radio = RadioButtons(
    rax,
    ("Luna - Earth", "Phobos - Mars", "Europa - Jupiter", "Titan - Saturn"),
    active=0,
    activecolor="white",
)


def spplistener():
    print("Creating Bluetooth SPP connection")
    btConn = serial.Serial("/dev/ttyS7", 115200, timeout=1)
    btConn.flushInput()
    sio = io.TextIOWrapper(io.BufferedRWPair(btConn, btConn, 1),
                           encoding="utf-8")

    print("Connection is Sucessful. Attempting to listen")
    while btConn.is_open:
        try:
            ss = sio.readline()
            print("Received: " + ss)
        except:
            print("Error reading line")
            break
        try:
            # ss = sio.write("<+{:05o}+{:05o}+{:05o}+{:05o} 10 123 1234>\n"\
            #      .format(int(np.round(GGMp*(2**14))), int(np.round(rr1*(2**14))), \
            #              int(np.round(rr2*(2**14))), int(np.round(rr3*(2**14)))))
            l = sio.write("<+26336+03146+00510+01042 10 123 1234>\n")
            print("Wrote: " + str(l))
        except:
            print("Error reading line")
            break
        sleep(1)


p = mp.Process(target=spplistener, args=())
p.start()

py.show()
