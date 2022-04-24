""" AGC Simulation ###
# @author Jae Choi
# 3 Bodies simulation inspired by 
# https://github.com/zaman13/Three-Body-Problem-Gravitational-System
"""

from matplotlib.widgets import Slider, Button, RadioButtons
from matplotlib import animation, rc
import threading as th
import io
import serial
import os
import pylab as py
import numpy as np
from time import sleep
import matplotlib
import string

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
    Fmag = GG * M1 * M2 / (np.linalg.norm(r) + 1e-20) ** 2
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
            ro, r, "moon", "commanche"
        )
    if body == "moon":
        return force_bodies([0, 0], r, "earth", "moon") - force_bodies(
            ro, r, "moon", "commanche"
        )


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
    k22 = dv_dt(t + dt / 2.0, r + dt * k11 / 2.0, v + dt * k21 / 2.0, body, ro, vo)
    k13 = v + dt * k22 / 2.0
    k23 = dv_dt(t + dt / 2.0, r + dt * k12 / 2.0, v + dt * k22 / 2.0, body, ro, vo)
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
MM = Mp
TT = 365 * 24 * 60 * 60.0
FF = (G * MM**2) / RR**2  # Unit force
GG = (MM * G * TT**2) / (RR**3)
Mc = Mc / MM  # Normalized mass of CSM
Mp = Mp / MM  # Normalized mass of Planet
Ml = Ml / MM  # Normalized mass of Moon
SPSCALE = 2**14  # Conversion factor between Single Precision to Floats
SIMAGCSCALE = 10.0  # Normalization factor for between -1 and +1

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

print(
    "GG: {} FF: {} vi: {} vm_i: {} Mp: {} Ml: {}".format(
        GG, FF, v_i_abs, vm_i_abs, Mp, Ml
    )
)

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
fig, ax = py.subplots(figsize=(8, 4.5))

py.subplots_adjust(left=-0.25)
ax.axis("square")
ax.set_xlim((-7.2, 7.2))
ax.set_ylim((-7.2, 7.2))
ax.get_xaxis().set_ticks([])  # enable this to hide x axis ticks
ax.get_yaxis().set_ticks([])  # enable this to hide y axis ticks

ax.plot(0, 0, "o", markersize=9, markerfacecolor="#0077BE", markeredgecolor="#d2eeff")
(line1,) = ax.plot(
    [], [], "o-", color="#FDB813", markevery=10000, markerfacecolor="#FD7813", lw=2
)  # line for CSM
(line2,) = ax.plot(
    [],
    [],
    "o-",
    color="#f2f2f2",
    markersize=8,
    markerfacecolor="#595959",
    lw=2,
    markevery=10000,
)  # line for Moon

# Title and legends
ttl = ax.text(-6.2, 6.3, r"3-Body Problem: Luna, Earth, Spacecraft", fontweight="bold")
# Orbital bodies legends at the bottom
ax.plot(-6, -6.2, "o", color="#FDB813", markerfacecolor="#FD7813")
ax.text(-5.5, -6.4, "Apollo Command Module")
ax.plot(2.4, -6.2, "o", color="#f2f2f2", markersize=8, markerfacecolor="#595959")
ax.text(2.9, -6.4, "Moon")
ax.plot(
    5, -6.2, "o", markersize=9, markerfacecolor="#0077BE", markeredgecolor="#d2eeff"
)
ax.text(5.5, -6.4, "Earth")
py.title("Celestial Mechanics Simulator - 3 Body Problem Solved Using Runge-Kutta 4\n")

speed_i = 1.5
radius_i = 1
delta_f = 0.05
a_0 = 0.2
axspeed = py.axes([0.70, 0.43, 0.2, 0.03])
axfinalrad = py.axes([0.70, 0.48, 0.2, 0.03])
axtransfer = py.axes([0.70, 0.53, 0.2, 0.03])
axradius = py.axes([0.70, 0.58, 0.2, 0.03])
sspeed = Slider(axspeed, "Simulation Speed", 0.0, 2.5, valinit=speed_i, valstep=delta_f)
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
    valinit=radius_i,
    valstep=delta_f,
    color="orange",
)
sradius = Slider(
    axradius,
    "Orbit Radius",
    0.5,
    4.5,
    valinit=radius_i,
    valstep=delta_f,
    color="#FDB813",
)

axstatus = py.axes([0.60, 0.12, 0.3, 0.2])
axstatus.patch.set_edgecolor("white")
axstatus.set_yticks([])
axstatus.set_xticks([])
tprogl = axstatus.text(0.05, 0.8, r"$\bf{Current\ Program:}$")
tprog = axstatus.text(0.50, 0.8, r"01 Lunar Injection", color="coral")
tconnl = axstatus.text(0.05, 0.63, r"$\bf{Connection\ Status:}$")
tconn = axstatus.text(0.50, 0.63, r"Disconnected", color="red")
tdata0l = axstatus.text(0.05, 0.46, r"$\bf{Initial\ Delta-V:}$")
tdata0 = axstatus.text(0.50, 0.46, r"+0.00000")
tdata1l = axstatus.text(0.05, 0.29, r"$\bf{Final\ Delta-V:}$")
tdata1 = axstatus.text(0.50, 0.29, r"+0.00000")
tdata2l = axstatus.text(0.05, 0.12, r"$\bf{Angle\ of\ Bodies:}$")
tdata2 = axstatus.text(0.50, 0.12, r"+0.00000")

# Global variables for setting velocity
delta_vc1 = 0.0
delta_vc2 = 0.0
target_vc1 = 0.0
target_vc2 = 0.0
target_vc2l = 0.0
target_vc2p = 0.0
acm = 0.0
target_r2 = 0.0
past_prog = 0
set_delta_vc1 = False
set_delta_vc2 = False
set_homann_transfer = False
set_lunar_inject_va = False
set_lunar_inject_vb = False
set_lunar_inject = False
current_vc = np.zeros(2)
current_rc = np.zeros(2)
current_rm = np.zeros(2)
index = 0
offset = 0
reset = False
set_flags = False
do_lunar_injection = False


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
    global set_homann_transfer
    global delta_vc2
    global index
    global offset
    global index
    global reset
    global acm

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
        r[i : max(1, i - comanche_trail) : -1, 0],
        r[i : max(1, i - comanche_trail) : -1, 1],
    )
    line2.set_data(
        rm[i : max(1, i - moon_trail) : -1, 0], rm[i : max(1, i - moon_trail) : -1, 1]
    )
    # Calculate the next point
    [r[i + 1, :], v[i + 1, :]] = RK4Solver(
        t[i], r[i, :], v[i, :], dt * sspeed.val, "commanche", rm[i, :], vm[i, :]
    )
    [rm[i + 1, :], vm[i + 1, :]] = RK4Solver(
        t[i], rm[i, :], vm[i, :], dt * sspeed.val, "moon", r[i, :], v[i, :]
    )
    theta1 = arctan(r[i])
    theta2 = arctan(rm[i])
    dr = np.subtract(rm[i], r[i])
    if set_homann_transfer and set_delta_vc1:
        v[i + 1, :] = v[i, :] + [
            target_vc1 * np.cos(theta1 + np.pi / 2),
            target_vc1 * np.sin(theta1 + np.pi / 2),
        ]
        # Fire Delta-V normal to the orbit or straight ahead
        # v[i + 1, :] = set_vector(target_vc1, theta1 + np.pi / 2)
        set_delta_vc1 = False
        set_delta_vc2 = True
        acm = acm + theta1
        if acm >= 2.0 * np.pi:
            acm = acm - 2.0 * np.pi
        line1.set_color("red")
        print("Doing first Hohmann burn: {:.5f} {:.5f}".format(target_vc1, acm))
    elif (
        set_lunar_inject
        and set_delta_vc1
        and theta2 > theta1
        and np.isclose(abs(theta2 - theta1), abs(acm), atol=0.05)
    ):
        print("Angle: {:.4f} =?= {:.4f}", acm, theta2 - theta1)
        v[i + 1, :] = [
            target_vc1 * np.cos(theta1 + np.pi / 2),
            target_vc1 * np.sin(theta1 + np.pi / 2),
        ]
        set_delta_vc1 = False
        set_delta_vc2 = True
        line1.set_color("red")
    elif (
        set_lunar_inject
        and set_delta_vc2
        and np.isclose(sfinalrad.val, np.linalg.norm(dr), atol=0.01)
    ):
        theta2 = arctan(np.subtract(rm[i], r[i]))
        # delta_vc2 = np.sqrt(Mp * GG / np.linalg.norm(rm[i]))
        v[i + 1, :] = [
            target_vc2p * np.cos(theta1 + np.pi / 2),
            target_vc2p * np.sin(theta1 + np.pi / 2),
        ]
        # delta_vc2 = np.sqrt(Ml * GG / sfinalrad.val)
        v[i + 1, :] += [
            target_vc2l * np.cos(theta2 + np.pi / 2),
            target_vc2l * np.sin(theta2 + np.pi / 2),
        ]
        set_lunar_inject = False
        set_delta_vc2 = False
        line1.set_color("#FDB813")
        print("Lunar injection")
    elif (
        set_homann_transfer
        and set_delta_vc2
        and np.isclose(theta1, acm, atol=0.01)
        # and np.isclose(np.linalg.norm(r[i]), target_r2, atol=0.01)
    ):
        v[i + 1, :] = v[i, :] + [
            target_vc2 * np.cos(theta1 + np.pi / 2),
            target_vc2 * np.sin(theta1 + np.pi / 2),
        ]
        set_delta_vc2 = False
        set_homann_transfer = False
        print("Doing second Hohmann burn: {:.5f} {:.5f}".format(target_vc2, theta1))
        line1.set_color("#FDB813")
    tm_yr = "Elapsed time = {:.2f} units {:.5f} =?= {:.5f}".format(
        t[i] * sspeed.val, theta1, acm
    )
    ttl.set_text(tm_yr)
    return (line1, line2, ttl)


def init():
    line1.set_data([], [])
    line2.set_data([], [])
    ttl.set_text("")
    return (line1, line2, ttl)


# Call animation function
anim = animation.FuncAnimation(
    fig, animate, init_func=init, frames=None, interval=5, blit=True
)


def spplistener():
    global target_vc1
    global target_vc2
    global target_vc2l
    global target_vc2p
    global acm
    global target_r2
    global past_prog
    global set_delta_vc1
    global set_delta_vc2
    global set_homann_transfer
    global set_lunar_inject_va
    global set_lunar_inject_vb
    global set_lunar_inject
    global current_rc
    global current_rm
    global btConn
    global set_flags
    global counter

    print("Connection is Sucessful. Attempting to listen")
    counter = 0
    while btConn.is_open:
        ss = btConn.readline()
        if len(ss) < 5:
            continue
        try:
            # Read parameters from the ESP32
            str_bytes = ss.decode("utf-8")
            str_in = str_bytes.replace("\x00", "")
            str_in = str_in.split(" ")
            res_0 = int(str_in[0])
            res_1 = int(str_in[1])
            res_2 = int(str_in[2])
            res_3 = int(str_in[3])
            verb = int(str_in[4])
            dsky_prog = int(str_in[5])
        except:
            print("Error in decoding")
            continue
        # print(
        #     "Lunar: {} {} {} {} {}".format(
        #         set_lunar_inject,
        #         set_delta_vc1,
        #         set_lunar_inject_vb,
        #         set_lunar_inject_va,
        #         acm,
        #     )
        # )
        # print(
        #     "Got: {:.5f} {:.5f} {:.5f} {:.5f}".format(
        #         float(res_0) / SPSCALE,
        #         float(res_1) / SPSCALE,
        #         float(res_2) / SPSCALE,
        #         float(res_3) / SPSCALE,
        #     )
        # )
        if verb == 39 and set_flags:

            if dsky_prog == 0 and do_lunar_injection == False:
                # Escape velocity
                target_vc1 = float(res_0) / SPSCALE * SIMAGCSCALE
                target_vc2 = 0.0
                acm = 0
                set_homann_transfer = True
                set_delta_vc1 = True
                set_flags = False
            elif dsky_prog == 1 and do_lunar_injection == False:
                # Homann Transfer
                target_vc1 = float(res_1 - res_0) / SPSCALE * SIMAGCSCALE
                target_vc2 = float(res_2 - res_3) / SPSCALE * SIMAGCSCALE
                acm = np.pi
                set_homann_transfer = True
                set_delta_vc1 = True
                set_flags = False
            elif dsky_prog == 2 and do_lunar_injection:
                if dsky_prog != past_prog:
                    set_lunar_injection()
                # print("Read first lunar inject")
                # Lunar Injection Initial Velocity Calculation
                target_vc1 = float(res_1) / SPSCALE * SIMAGCSCALE
                set_lunar_inject_va = True
            elif dsky_prog == 3 and do_lunar_injection:
                # print("Read second lunar inject")
                if dsky_prog != past_prog:
                    set_lunar_injection()
                # Lunar Injection Final Velocity Calculation
                target_vc1 = float(res_1) / SPSCALE * SIMAGCSCALE
                target_vc2l = float(res_2) / SPSCALE * SIMAGCSCALE
                target_vc2p = float(res_3) / SPSCALE * SIMAGCSCALE
                set_lunar_inject_vb = set_lunar_inject_va
                # set_lunar_injection()
                # set_flags = False
            elif dsky_prog == 4 and do_lunar_injection:
                counter = counter + 1
                if dsky_prog != past_prog:
                    set_lunar_injection()
                if counter > 5:
                    # print("Read final lunar inject")
                    # Lunar Injection Angle Calculation
                    target_vc1 = float(res_1) / SPSCALE * SIMAGCSCALE
                    target_vc2l = float(res_2) / SPSCALE * SIMAGCSCALE
                    target_vc2p = float(res_3) / SPSCALE * SIMAGCSCALE
                    acm = np.pi * float(res_0) / SPSCALE
                    set_lunar_inject = set_lunar_inject_vb
                    set_delta_vc1 = set_lunar_inject_vb
                    set_lunar_inject_vb = False
                    set_lunar_inject_va = False
                    set_flags = False
            # print(
            #     "AGC PROGRAM {}: VC1={:.5f} VC2={:.5f} ACM={:.5f}".format(
            #         dsky_prog, target_vc1, target_vc2, acm
            #     )
            # )

        tconn.set_text(r"Connected")
        tconn.set_color("springgreen")

        past_prog = dsky_prog
        sleep(0.1)


btConn = serial
print("Creating Bluetooth SPP connection")
try:
    btConn = serial.Serial("/dev/ttyS7", 115200, timeout=10)
except:
    print("Not connected")
    tconn.set_text(r"Disconnected")
    tconn.set_color("red")
btConn.flushInput()
# sio = io.TextIOWrapper(io.BufferedRWPair(btConn, btConn, 1), encoding="utf-8")
serial_thread = th.Thread(target=spplistener, args=())

serial_thread.start()


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
    line1.set_color("#FDB813")
    anim.resume()


def reset(event):
    set_orbits(event)


def homannn_transfer(event):
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
    delta_vc2 = np.sqrt(GG * Mp / r2) * (-1.0 * np.sqrt(2.0 * r1 / (r1 + r2)) + 1.0)
    acm = np.pi * (1 - np.sqrt((r1 / r2 + 1) ** 3 / 8.0))
    print(
        "R1:{:.5f} R2:{:.5f} Delta-V: {:.5f} then {:.5f} at {:.5f} rad\n".format(
            r1, target_r2, delta_vc1, delta_vc2, acm
        )
    )

    GGMp = GG * Mp / 10.0
    GGMl = GG * Ml / 10.0
    rr0 = 1.0 / (sfinalrad.val * 10.0)
    rr1 = 1.0 / (r1 * 10.0)
    rr2 = 1.0 / (r2 * 10.0)
    rr3 = 1.0 / (10.0 * (r1 + target_r2) / 2.0)
    va_sqr_2 = GGMp * rr1
    vb_sqr_2 = GGMp * rr2
    vatx_sqr = GGMp * (2 * rr1 - rr3)
    vbtx_sqr = GGMp * (2 * rr2 - rr3)

    print(
        "GMp:{:.5f} 1/R1N:{:.5f} 1/R2N:{:.5f} ATX:{:.5f}\nVa:{:.5f}({:.5f}) Vb:{:.5f}({:.5f}) Vatx:{:.5f}({:.5f}) Vbtx:{:.5f}({:.5f}) V-delta:{:.10f} -> {:.10f}\n".format(
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
        )
    )
    print(
        "Vbtx_L:{:.5f}({:.5f}) Vbtx_M:{:.5f}({:.5f}) arg:{:.5f}({:.5f},{:.5f})\n".format(
            np.sqrt(GGMp * rr0),
            GGMp * rr0,
            np.sqrt(GGMp * rr2),
            GGMp * rr2,
            1.0 - np.sqrt(((r1 / r2) / 2.0 + 1.0 / 2.0) ** 3),
            ((r1 / r2) / 2.0 + 1.0 / 2.0) ** 3,
            ((r1 / r2) / 2.0 + 1.0 / 2.0),
        )
    )
    # print("GMp:{:.3f} 1/R1N:{:.3f} 1/R2N:{:.3f} ATX:{:.3f}\nVa:{:.3f}({:.3f}) Vb:{:.3f}({:.3f}) Vatx:{:.3f}({:.3f}) Vbtx:{:.3f}({:.3f}) V-delta:{:.3f} -> {:.3f}\n"\
    #       .format(GGMp*(2**14), rr1*(2**14), rr2*(2**14), rr3*(2**14), va_sqr_2*(2**14), np.sqrt(va_sqr_2)*(2**14), vb_sqr_2*(2**14), np.sqrt(vb_sqr_2)*(2**14), vatx_sqr*(2**14), \
    #               np.sqrt(vatx_sqr)*(2**14), vbtx_sqr*(2**14), np.sqrt(vbtx_sqr)*(2**14), (np.sqrt(vatx_sqr)-np.sqrt(va_sqr_2))*(2**14), (np.sqrt(vb_sqr_2)-np.sqrt(vbtx_sqr))*(2**14)))
    # print("GMp: {} 1/R1N: {} 1/R2N: {} ATX: {}\nVa: {}({}) Vb: {}({}) Vatx: {}({}) Vbtx: {}({}) V-delta: {} -> {}\n"\
    #       .format(int(np.round(GGMp*(2**14))), int(np.round(rr1*(2**14))), int(np.round(rr2*(2**14))), int(np.round(rr3*(2**14))), int(np.round(va_sqr_2*(2**14))), int(np.round(np.sqrt(va_sqr_2)*(2**14))),\
    #               int(np.round(vb_sqr_2*(2**14))), int(np.round(np.sqrt(vb_sqr_2)*(2**14))), int(np.round(vatx_sqr*(2**14))), int(np.round(np.sqrt(vatx_sqr)*(2**14))), int(np.round(vbtx_sqr*(2**14))), \
    #               int(np.round(np.sqrt(vbtx_sqr)*(2**14))), int(np.round((np.sqrt(vatx_sqr)-np.sqrt(va_sqr_2))*(2**14))), int(np.round((np.sqrt(vb_sqr_2)-np.sqrt(vbtx_sqr))*(2**14)))))
    print(
        "<+{:05o}+{:05o}+{:05o}+{:05o} 10 10 10 123 1234>\n".format(
            int(np.round(GGMp * (2**14))),
            int(np.round(rr1 * (2**14))),
            int(np.round(rr2 * (2**14))),
            int(np.round(rr3 * (2**14))),
        )
    )

    print(
        "<+{:05f}+{:05f}+{:05f}+{:05f} 10 10 10 123 1234>".format(
            (GGMp),
            (rr0),
            (rr2),
            (GGMl),
        )
    )
    print(
        "<+{:05o}+{:05o}+{:05o}+{:05o} 10 10 10 123 1234>".format(
            int(np.round(GGMp * (2**14))),
            int(np.round(rr0 * (2**14))),
            int(np.round(rr2 * (2**14))),
            int(np.round(GGMl * (2**14))),
        )
    )
    print(
        "Vm={:05d} Vp={:05d} V-d={:.5f}".format(
            int(np.round(np.sqrt(GGMl * rr0) * (2**14))),
            int(np.round(np.sqrt(GGMp * rr2) * (2**14))),
            (np.sqrt(GGMp * rr0 + GGMp * rr2) - np.sqrt(vbtx_sqr)),
        )
    )
    print(
        "GGMl={:5f} Vm={:5f} Vp={:5f} V-final={:.5f}\n".format(
            (GGMl),
            (np.sqrt(GGMl * rr0)),
            (np.sqrt(GGMp * rr2)),
            (np.sqrt(GGMp * rr0 + GGMp * rr2)),
        )
    )

    print(
        "<+{:05f}+{:05f}+{:05f}+{:05f} 10 10 1 123 1234>".format(
            (r1 / r2),
            (0),
            (0),
            (0),
        )
    )
    print(
        "<+{:05o}+{:05o}+{:05o}+{:05o} 10 10 1 123 1234>".format(
            int(np.round(r1 / r2 * (2**14))),
            int(np.round(r1 / r2 * (2**14))),
            int(np.round(r1 / r2 * (2**14))),
            int(np.round(r1 / r2 * (2**14))),
        )
    )
    print(
        "result octal={:05o} result bef sqrt={:5f} result bef. minus={:.5f} result={:.5f}".format(
            int(np.round((1 - np.sqrt((r1 / r2 + 1) ** 3 / 8.0)) * (2**14))),
            (r1 / r2) / 2 + 1 / 2,
            np.sqrt((r1 / r2 + 1) ** 3 / 8.0),
            1 - np.sqrt((r1 / r2 + 1) ** 3 / 8.0),
        )
    )

    set_delta_vc1 = True


# p.start()


def set_lunar_injection():
    global past_prog
    global set_lunar_inject
    global set_lunar_inject_va
    global set_lunar_inject_vb
    global btConn
    global set_flags

    r1 = np.linalg.norm(current_rc)
    target_r2 = r2 = np.linalg.norm(current_rm)

    if r1 == 0.0 or r2 == 0.0:
        return

    GGMp = GG * Mp / SIMAGCSCALE
    GGMl = GG * Ml / SIMAGCSCALE
    rr0 = 1.0 / (sfinalrad.val * SIMAGCSCALE)
    rr1 = 1.0 / (r1 * SIMAGCSCALE)
    rr2 = 1.0 / (r2 * SIMAGCSCALE)
    rr3 = 1.0 / (SIMAGCSCALE * (r1 + target_r2) / 2.0)  # Apogee

    if (
        not set_lunar_inject
        and not set_lunar_inject_va
        and not set_lunar_inject_vb
        and past_prog != 4
    ):
        # print("Wrote first lunar inject")
        ss = btConn.write(
            "<+{:05o}+{:05o}+{:05o}+{:05o} 10 10 1 123 1234>\n".format(
                int(np.round(GGMp * (2**14))),
                int(np.round(rr1 * (2**14))),
                int(np.round(rr2 * (2**14))),
                int(np.round(rr3 * (2**14))),
            ).encode()
        )
    elif set_lunar_inject_va and not set_lunar_inject and not set_lunar_inject_vb:
        # print("Wrote second lunar inject")
        ss = btConn.write(
            "<+{:05o}+{:05o}+{:05o}+{:05o} 10 10 1 123 1234>\n".format(
                int(np.round(GGMp * (2**14))),
                int(np.round(rr0 * (2**14))),
                int(np.round(rr2 * (2**14))),
                int(np.round(GGMl * (2**14))),
            ).encode()
        )
    elif set_lunar_inject_vb and set_lunar_inject_va and not set_lunar_inject:
        # print("Wrote third lunar inject")
        ss = btConn.write(
            "<+{:05o}+{:05o}+{:05o}+{:05o} 10 10 1 123 1234>\n".format(
                int(np.round(r1 / r2 * (2**14))),
                int(np.round(r1 / r2 * (2**14))),
                int(np.round(r1 / r2 * (2**14))),
                int(np.round(r1 / r2 * (2**14))),
            ).encode()
        )
    # set_flags = True
    # set_lunar_inject = True


def send_lunar(event):
    global set_flags
    global set_lunar_inject
    global set_delta_vc1
    global set_lunar_inject_vb
    global set_lunar_inject_va
    global counter
    global do_lunar_injection
    do_lunar_injection = True
    counter = 0
    set_lunar_inject = False
    set_delta_vc1 = False
    set_lunar_inject_vb = False
    set_lunar_inject_va = False
    set_lunar_injection()
    set_flags = True


def send_hohmann(event):
    global btConn
    global set_flags
    global do_lunar_injection
    do_lunar_injection = False
    r1 = np.linalg.norm(current_rc)
    target_r2 = r2 = stransfrad.val

    if r1 == 0.0 or r2 == 0.0:
        return

    GGMp = GG * Mp / SIMAGCSCALE
    rr1 = 1.0 / (r1 * SIMAGCSCALE)
    rr2 = 1.0 / (r2 * SIMAGCSCALE)
    rr3 = 1.0 / (SIMAGCSCALE * (r1 + target_r2) / 2.0)  # Apogee

    ss = btConn.write(
        "<+{:05o}+{:05o}+{:05o}+{:05o} 10 10 1 123 1234>\n".format(
            int(np.round(GGMp * (2**14))),
            int(np.round(rr1 * (2**14))),
            int(np.round(rr2 * (2**14))),
            int(np.round(rr3 * (2**14))),
        ).encode()
    )
    print(
        "<+{:05o}+{:05o}+{:05o}+{:05o} 10 10 1 123 1234>\n".format(
            int(np.round(GGMp * (2**14))),
            int(np.round(rr1 * (2**14))),
            int(np.round(rr2 * (2**14))),
            int(np.round(rr3 * (2**14))),
        )
    )
    set_flags = True

    # target_r2 = r2 = 5
    # va_sqr_2 = GGMp * rr1
    # vb_sqr_2 = GGMp * rr2
    # vatx_sqr = GGMp * (2 * rr1 - rr3)
    # vbtx_sqr = GGMp * (2 * rr2 - rr3)
    # acm = np.pi * (1 - np.sqrt((r1 / r2 + 1) ** 3 / 8.0))
    # print(
    #     "Lunar R1:{:.5f} R2:{:.5f} Delta-V: {:.5f} then {:.5f} at {:.5f} rad".format(
    #         r1,
    #         r2,
    #         np.sqrt(vatx_sqr) * SIMAGCSCALE,
    #         np.sqrt(vbtx_sqr) * SIMAGCSCALE,
    #         acm,
    #     )
    # )
    # target_r2 = r2 = stransfrad.val
    # va_sqr_2 = GGMp * rr1
    # vb_sqr_2 = GGMp * rr2
    # vatx_sqr = GGMp * (2 * rr1 - rr3)
    # vbtx_sqr = GGMp * (2 * rr2 - rr3)
    # acm = np.pi
    # print(
    #     "Hohmann R1:{:.5f} R2:{:.5f} Vatx: {:.5f} then Vbtx {:.5f} at {:.5f} rad".format(
    #         r1,
    #         r2,
    #         (np.sqrt(vatx_sqr) - np.sqrt(va_sqr_2)) * SIMAGCSCALE,
    #         np.sqrt(vb_sqr_2) - np.sqrt(vbtx_sqr) * SIMAGCSCALE,
    #         acm,
    #     )
    # )

    # delta_vc1 = np.sqrt(GG * Mp / r1) * (np.sqrt(2.0 * r2 / (r1 + r2)) - 1.0)
    # delta_vc2 = np.sqrt(GG * Mp / r2) * (-1.0 * np.sqrt(2.0 * r1 / (r1 + r2)) + 1.0)
    # acm = np.pi * (1 - np.sqrt((r1 / r2 + 1) ** 3 / 8.0))
    # print(
    #     "R1:{:.5f} R2:{:.5f} Delta-V: {:.5f} then {:.5f} at {:.5f} rad\n".format(
    #         r1, target_r2, delta_vc1, delta_vc2, acm
    #     )
    # )


resetax = py.axes([0.78, 0.37, 0.07, 0.04])
buttonrst = Button(resetax, "Reset", hovercolor="gray", color="black")
buttonrst.on_clicked(reset)

setax = py.axes([0.70, 0.37, 0.07, 0.04])
buttonsethohm = Button(setax, "Hohmann", hovercolor="gray", color="black")
buttonsethohm.on_clicked(send_hohmann)

setax = py.axes([0.62, 0.37, 0.07, 0.04])
buttonsetlunar = Button(setax, "Lunar Inject", hovercolor="gray", color="black")
buttonsetlunar.on_clicked(send_lunar)


sradius.on_changed(set_orbits)

rax = py.axes([0.60, 0.65, 0.3, 0.2], facecolor="#1a1a1a")
radio = RadioButtons(
    rax,
    ("Luna - Earth", "Phobos - Mars", "Europa - Jupiter", "Titan - Saturn"),
    active=0,
    activecolor="white",
)

fig.set_dpi(156)

py.show()
