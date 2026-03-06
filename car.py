from cmath import cos, pi, sin
from math import atan2
import time
import sys

from simple_pid import PID
import busio
from adafruit_pca9685 import PCA9685
import RPi.GPIO as GPIO
import matplotlib.pyplot as plt
import numpy as np

SCL = 3
SDA = 2

GPIO.setmode(GPIO.BCM)
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 1000

# CONSTANTS
ROBOT_WIDTH = 0.21      # m, distance between left and right wheels
ROBOT_LENGTH = 0.095    # m, distance between front and rear wheels
M_PER_REV = 0.25
VELOCITY_SCALE_SEMICIRCLE = 0.8

# PID constants, to be tuned
fl_PID_gains = (800.0, 300.0, 5)
fr_PID_gains = (800.0, 300.0, 5)
rl_PID_gains = (800.0, 300.0, 5)
rr_PID_gains = (800.0, 300.0, 5)
PID_OUTPUT_LIMITS = (-100, 100)
PID_INTERVAL = 0.001  # seconds between PID updates

# MOTORS — front controller, PCA channel
ENAFR = 0
IN1FR = 1
IN2FR = 2

IN3FL = 5
IN4FL = 6
ENBFL = 4

# rear controller, PCA channel
ENARR = 8
IN1RR = 9
IN2RR = 10

IN3RL = 13
IN4RL = 14
ENBRL = 12

# ENCODERS, GPIO.board pin
S1FR = 17   # pin 11
S2FR = 27   # pin 13

S1FL = 22   # pin 15
S2FL = 10   # pin 19

S1RR = 9    # pin 21
S2RR = 11   # pin 23

S1RL = 5    # pin 29
S2RL = 6    # pin 31
perRev = 155

# PCA9685
PWMOEN = 4  # pin 7
pwmOEn = GPIO.setup(PWMOEN, GPIO.OUT)

# push button
pushButton = 26     # pin 37, GPIO 26
GPIO.setup(pushButton, GPIO.IN)
oldPushb = 0


def readPush():
    global oldPushb
    pushb = GPIO.input(pushButton)
    if pushb != oldPushb:
        oldPushb = pushb
        return True, pushb
    else:
        return False, pushb


def valmap(value, istart, istop, ostart, ostop):
    return ostart + (ostop - ostart) * ((value - istart) / (istop - istart))


def getPWMPer(value):
    return int(valmap(value, 0, 100, 0, 0xFFFF))


high = 0xFFFF
low = 0

_fig = None
_axd = None
_ax_traj = None
_traj_line = None

_robot_x = 0.0
_robot_y = 0.0
_robot_heading = 0.0   # radians, 0 = facing +Y (forward)

_ARROW_MAX_MPS = 1.0   # m/s that maps to a full-length wheel arrow


def create_display():
    global _fig, _axd, _ax_traj, _traj_line
    global _robot_x, _robot_y, _robot_heading

    _robot_x = 0.0
    _robot_y = 0.0
    _robot_heading = 0.0

    plt.ion()
    _fig, _axd = plt.subplot_mosaic(
        [['traj', 'FL', 'FR'],
         ['traj', 'RL', 'RR']],
        figsize=(12, 5),
        layout='constrained'
    )
    _fig.suptitle('Live Robot Telemetry', fontsize=11)

    _ax_traj = _axd['traj']
    _ax_traj.set_aspect('equal')
    _ax_traj.set_xlim(-2, 2)
    _ax_traj.set_ylim(-2, 2)
    _ax_traj.grid(True)
    _ax_traj.set_title('Robot Trajectory (encoder dead-reckoning)', fontsize=9)
    _ax_traj.set_xlabel('X (m)')
    _ax_traj.set_ylabel('Y (m)')
    _ax_traj.plot(0, 0, 'go', markersize=8, label='start')
    _ax_traj.legend(fontsize=7, loc='upper right')

    _traj_line, = _ax_traj.plot([], [], '-', color='steelblue', linewidth=1.5)

    # Draw zero-speed arrows on startup
    _draw_wheel_panels(0.0, 0.0, 0.0, 0.0)
    plt.pause(0.001)


def _draw_wheel_panel(ax, name, spd_mps):
    ax.cla()
    ax.set_xlim(-1.5, 1.5)
    ax.set_ylim(-1.5, 1.5)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_title(name, fontsize=9, pad=2)

    norm = max(_ARROW_MAX_MPS, 0.001)
    length = max(min(abs(spd_mps) / norm, 1.0), 0.05)
    direction = 1 if spd_mps >= 0 else -1
    color = 'royalblue' if spd_mps >= 0 else 'tomato'

    ax.quiver(0, 0, 0, direction * length,
              color=color, width=0.04, pivot='mid',
              angles='uv', scale_units='height', scale=1.0)
    ax.text(0, -1.3, f'{spd_mps:+.3f} m/s',
            ha='center', va='bottom', fontsize=8)


def _draw_wheel_panels(spd_fl, spd_fr, spd_rl, spd_rr):
    """Redraw all four wheel subplots from measured encoder speeds."""
    if _axd is None:
        return
    _draw_wheel_panel(_axd['FL'], 'FL', spd_fl)
    _draw_wheel_panel(_axd['FR'], 'FR', spd_fr)
    _draw_wheel_panel(_axd['RL'], 'RL', spd_rl)
    _draw_wheel_panel(_axd['RR'], 'RR', spd_rr)


def _update_trajectory(spd_fl, spd_fr, spd_rl, spd_rr):
    global _robot_x, _robot_y, _robot_heading

    if _ax_traj is None:
        return

    half = (ROBOT_WIDTH + ROBOT_LENGTH) / 2.0
    vy = (spd_fl + spd_fr + spd_rl + spd_rr) / 4.0
    vx = (spd_fl - spd_fr - spd_rl + spd_rr) / 4.0
    omega = (-spd_fl + spd_fr + spd_rl - spd_rr) / (4.0 * half)

    # Rotate body-frame velocity into world frame using current heading
    c = np.cos(_robot_heading)
    s = np.sin(_robot_heading)
    world_vx = vx * c - vy * s
    world_vy = vx * s + vy * c

    _robot_x += world_vx * PID_INTERVAL
    _robot_y += world_vy * PID_INTERVAL
    _robot_heading += omega * PID_INTERVAL

    # Extend trajectory line
    xs = np.append(_traj_line.get_xdata(), _robot_x)
    ys = np.append(_traj_line.get_ydata(), _robot_y)
    _traj_line.set_data(xs, ys)

    # Auto-expand axes if robot drifts out of view
    xlim = _ax_traj.get_xlim()
    ylim = _ax_traj.get_ylim()
    margin = 0.3
    if _robot_x < xlim[0] + margin or _robot_x > xlim[1] - margin:
        _ax_traj.set_xlim(min(xlim[0], _robot_x) - 0.5,
                          max(xlim[1], _robot_x) + 0.5)
    if _robot_y < ylim[0] + margin or _robot_y > ylim[1] - margin:
        _ax_traj.set_ylim(min(ylim[0], _robot_y) - 0.5,
                          max(ylim[1], _robot_y) + 0.5)

    # Draw a quiver arrow only when meaningfully moving
    speed_mag = np.hypot(world_vx, world_vy)
    if speed_mag > 0.005:
        scale = 0.12 / max(speed_mag, 0.001)
        _ax_traj.quiver(_robot_x, _robot_y,
                        world_vx * scale, world_vy * scale,
                        color='tomato', width=0.004,
                        pivot='mid', angles='xy',
                        scale_units='xy', scale=1.0)


def update_display(spd_fl, spd_fr, spd_rl, spd_rr):
    """
    Refresh all display panels with real encoder-measured speeds (m/s).
    Called automatically from set_powers() every PID tick.
    """
    if _fig is None:
        return
    _draw_wheel_panels(spd_fl, spd_fr, spd_rl, spd_rr)
    _update_trajectory(spd_fl, spd_fr, spd_rl, spd_rr)
    plt.pause(0.001)


def close_display():
    """Hold the plot open until Enter, then close."""
    global _fig
    if _fig is not None:
        plt.ioff()
        input("Press Enter to close the plot...")
        plt.close(_fig)
        _fig = None
        print("Plot closed")


class Wheel:
    def __init__(self, name, enCh, in1Ch, in2Ch, pid_gains, output_limits,
                 encoder):
        self.name = name
        self.en = pca.channels[enCh]
        self.in1 = pca.channels[in1Ch]
        self.in2 = pca.channels[in2Ch]
        self.k_p = pid_gains[0]
        self.k_i = pid_gains[1]
        self.k_d = pid_gains[2]
        self.pid = PID(self.k_p, self.k_i, self.k_d, setpoint=0,
                       output_limits=output_limits,
                       sample_time=PID_INTERVAL)
        self.encoder = encoder
        self.setpoints = []
        self.speeds = []
        self.powers = []

    def move(self, velocity):
        if velocity == 0:
            self.brake()
        else:
            self.pid.setpoint = velocity
            spd = self.encoder.readSpeed()
            power = self.pid(spd)
            print(self.name, "setp:", velocity, "spd:", spd, "pwr:", power)
            self.setpoints.append(self.pid.setpoint)
            self.speeds.append(spd)
            self.powers.append(power)
            self.in1.duty_cycle = high if power > 0 else low
            self.in2.duty_cycle = low if power > 0 else high
            self.en.duty_cycle = getPWMPer(abs(power))

    def get_speed(self):
        """Return the most recently measured encoder speed (m/s)."""
        return self.encoder.readSpeed()

    def brake(self):
        self.in1.duty_cycle = low
        self.in2.duty_cycle = low


class Encoder:
    def __init__(self, name, S1, S2, side):
        self.name = name
        self.s1 = S1
        GPIO.setup(S1, GPIO.IN)
        self.s2 = S2
        GPIO.setup(S2, GPIO.IN)
        self.aState = 0
        self.bState = 0
        self.aLastState = 0
        self.bLastState = 0
        self.counter = 0
        self.lastCounter = 0
        self.aturn = 0
        self.bturn = 0
        self.speed = 0
        self.time = time.perf_counter_ns()
        self.lastTime = self.time
        self.side = side

    def read(self):
        self.aState = GPIO.input(self.s1)
        self.bState = GPIO.input(self.s2)
        self.time = time.perf_counter_ns()
        return self.aState, self.bState

    def read_turn(self):
        return self.aturn, self.bturn

    def read_name(self):
        return self.name

    def readEncoder(self):
        aState, bState = self.read()
        if aState != self.aLastState:
            if bState != aState:
                self.counter += 1
            else:
                self.counter -= 1
        if bState != self.bLastState:
            self.bturn += 1
        self.aLastState = aState
        self.bLastState = bState

    def readEncoderTest(self):
        self.readEncoder()

    def callback_encoder(self, channel):
        self.readEncoder()

    def readSpeed(self):
        if self.time != 0 and self.time != self.lastTime:
            self.speed = (
                self.side * (self.counter - self.lastCounter)
                / (self.time - self.lastTime)
            )
        else:
            self.speed = 0
        self.lastTime = self.time
        self.lastCounter = self.counter
        return self.speed / perRev / 4 * 1e9 * M_PER_REV

    def resetSpeed(self):
        self.speed = 0
        self.counter = 0
        self.lastCounter = 0
        self.time = time.perf_counter_ns()
        self.lastTime = time.perf_counter_ns()


# Set up Encoder instances
sfl = Encoder("sfl", S1FL, S2FL, 1)
sfr = Encoder("sfr", S1FR, S2FR, -1)
srl = Encoder("srl", S1RL, S2RL, 1)
srr = Encoder("srr", S1RR, S2RR, -1)

# Set up Wheel instances
rl = Wheel("rl", ENBRL, IN3RL, IN4RL, rl_PID_gains, PID_OUTPUT_LIMITS, srl)
rr = Wheel("rr", ENARR, IN1RR, IN2RR, rr_PID_gains, PID_OUTPUT_LIMITS, srr)
fl = Wheel("fl", ENBFL, IN3FL, IN4FL, fl_PID_gains, PID_OUTPUT_LIMITS, sfl)
fr = Wheel("fr", ENAFR, IN1FR, IN2FR, fr_PID_gains, PID_OUTPUT_LIMITS, sfr)


def test_speed():
    print('fl: ', sfl.readSpeed())
    print('fr: ', sfr.readSpeed())
    print('rl: ', srl.readSpeed())
    print('rr: ', srr.readSpeed())


def test_Encoders():
    sfl.readEncoderTest()
    sfr.readEncoderTest()
    srl.readEncoderTest()
    srr.readEncoderTest()


GPIO.add_event_detect(sfl.s1, GPIO.BOTH, callback=sfl.callback_encoder)
GPIO.add_event_detect(sfr.s1, GPIO.BOTH, callback=sfr.callback_encoder)
GPIO.add_event_detect(srl.s1, GPIO.BOTH, callback=srl.callback_encoder)
GPIO.add_event_detect(srr.s1, GPIO.BOTH, callback=srr.callback_encoder)


def stop_car():
    rl.brake()
    rr.brake()
    fl.brake()
    fr.brake()
    time.sleep(1)
    sfl.resetSpeed()
    srr.resetSpeed()
    srl.resetSpeed()
    sfr.resetSpeed()


def set_powers(fl_power, fr_power, rl_power, rr_power):
    fl.move(fl_power)
    fr.move(fr_power)
    rl.move(rl_power)
    rr.move(rr_power)
    # Read actual encoder speeds and push them to the live display


def update_display_all():
    spd_fl = fl.get_speed()
    spd_fr = fr.get_speed()
    spd_rl = rl.get_speed()
    spd_rr = rr.get_speed()
    update_display(spd_fl, spd_fr, spd_rl, spd_rr)


def moveAllPID(vx, vy, omega, forSecs):
    # omega in rad/s, positive is CCW, negative is CW
    fl_target = vy + vx - omega * (ROBOT_WIDTH + ROBOT_LENGTH) / 2.0
    fr_target = vy - vx + omega * (ROBOT_WIDTH + ROBOT_LENGTH) / 2.0
    rl_target = vy - vx - omega * (ROBOT_WIDTH + ROBOT_LENGTH) / 2.0
    rr_target = vy + vx + omega * (ROBOT_WIDTH + ROBOT_LENGTH) / 2.0

    start_time = time.perf_counter_ns()
    while time.perf_counter_ns() - start_time < forSecs * 1e9:
        set_powers(fl_target, fr_target, rl_target, rr_target)
        update_display_all()
        time.sleep(PID_INTERVAL)


def moveFromTo(startx, starty, endx, endy, forSecs):
    dx = endx - startx
    dy = endy - starty
    distance = (dx**2 + dy**2)**0.5
    velocity = distance / forSecs
    vx = velocity * cos(atan2(dy, dx)).real
    vy = velocity * sin(atan2(dy, dx)).real
    print("Moving from (" + str(startx) + ", " + str(starty) + ") to ("
          + str(endx) + ", " + str(endy) + ") at velocity " + str(velocity)
          + " m/s for " + str(forSecs) + " seconds.")
    print(vx, vy)
    moveAllPID(vx, vy, 0, forSecs)


def turn_right(speed, degrees):
    forSecs = (pi * ROBOT_WIDTH * degrees / 360.0) / speed
    omega = -speed / (ROBOT_WIDTH / 2.0)
    moveAllPID(0, 0, omega, forSecs)


def turn_left(speed, degrees):
    forSecs = (pi * ROBOT_WIDTH * degrees / 360.0) / speed
    omega = speed / (ROBOT_WIDTH / 2.0)
    moveAllPID(0, 0, omega, forSecs)


def coastAll(forSecs):
    print("Coast forSecs " + str(forSecs) + " secs.")
    rl.move(0)
    rr.move(0)
    fr.move(0)
    fl.move(0)
    time.sleep(forSecs)
    stop_car()


def turn_semicircle(radius, forSecs):
    velocity = (pi * abs(radius)) / forSecs
    omega = velocity / abs(radius)
    if radius < 0:
        omega = -omega
    print("Turning semicircle with radius " + str(radius) + " m at velocity "
          + str(velocity) + " m/s for " + str(forSecs) + " seconds.")
    moveAllPID(0, velocity * VELOCITY_SCALE_SEMICIRCLE, omega, forSecs)


def test_readPush():
    changed, state = readPush()
    if changed:
        print("button changed to " + str(state))


def destroy():
    GPIO.output(PWMOEN, 1)
    GPIO.cleanup()


def main():
    print("starting main, using file list of functions")

    if len(sys.argv) == 1:
        myfile = 'instructionsCar.txt'
    else:
        myfile = sys.argv[1]
    print("reading file ", myfile)

    with open(myfile, encoding="utf-8") as myf:
        actionList = myf.readlines()

    GPIO.output(PWMOEN, 0)
    create_display()    # open the live telemetry figure

    for x in actionList:
        print(x, end='')
        if '#' not in x:
            exec(x)

    stop_car()
    close_display()     # hold plot open until Enter is pressed
    destroy()
    print("\nStopped and cleanup done")


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        stop_car()
        close_display()
        destroy()
        print("\nStopped and cleanup done")
