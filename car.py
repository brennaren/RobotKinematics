from cmath import cos, pi, sin
from math import atan2
import time
import sys

from simple_pid import PID
import busio
from adafruit_pca9685 import PCA9685
import RPi.GPIO as GPIO

SCL = 3
SDA = 2

GPIO.setmode(GPIO.BCM)
# I2C for PCS9685 and Gyro
# create i2c bus interface to access PCA9685, for example
i2c = busio.I2C(SCL, SDA)   # busio.I2C(board.SCL, board.SDA) create i2c bus
pca = PCA9685(i2c)  # adafruit_pca9685.PCA9685(i2c) instance PCA9685 on bus
pca.frequency = 1000        # set pwm clock in Hz (debug 60 was 1000)
# usage: pwm_channel = pca.channels[0] instance example
#        pwm_channel.duty_cycle = speed (0 .. 100)  speed example

# CONSTANTS
ROBOT_WIDTH = 0.21      # m, distance between left and right wheels
ROBOT_LENGTH = 0.095    # m, distance between front and rear wheels
M_PER_REV = 0.25
VELOCITY_SCALE_SEMICIRCLE = 0.8   # adjust velocity for semicircles

# PID constants, to be tuned
fl_PID_gains = (800.0, 300.0, 5)
fr_PID_gains = (800.0, 300.0, 5)
rl_PID_gains = (800.0, 300.0, 5)
rr_PID_gains = (800.0, 300.0, 5)
PID_OUTPUT_LIMITS = (-100, 100)  # limit correction to +/- 30% power
PID_INTERVAL = 0.001  # seconds between PID updates 20 ms

# MOTORS
# front controller, PCA channel
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
perRev = 155    # estimate A type motors 155 #2024 motors 750

# PCA9685
PWMOEN = 4  # pin 7 # PCA9685 OEn pin
pwmOEn = GPIO.setup(PWMOEN, GPIO.OUT)   # enable PCA outputs

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


# equivalent of Arduino map()
def valmap(value, istart, istop, ostart, ostop):
    return ostart + (ostop - ostart) * ((value - istart) / (istop - istart))


# for 0 to 100, % speed as integer, to use for PWM
# full range 0xFFFF, but PCS9685 ignores last Hex digit as only 12 bit res
def getPWMPer(value):
    return int(valmap(value, 0, 100, 0, 0xFFFF))


# for IN1, IN2, define 1  and 0 settings
high = 0xFFFF   # 1 was True
low = 0        # 0 was False


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
        if (velocity == 0):
            self.brake()
        else:
            self.pid.setpoint = velocity
            spd = self.encoder.readSpeed()
            power = self.pid(spd)
            print(self.name, "setpoint:", velocity, "spd:", spd, "power:", power)  # debug
            self.setpoints.append(self.pid.setpoint)
            self.speeds.append(spd)
            self.powers.append(power)
            self.in1.duty_cycle = high if power > 0 else low
            self.in2.duty_cycle = low if power > 0 else high
            self.en.duty_cycle = getPWMPer(abs(power))

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
        # Reads the "current" state of the encoders
        aState, bState = self.read()
        # If the previous and the current state are different,
        # a Pulse has occurred
        if aState != self.aLastState:
            # If the outputB state != outputA state, rotating clockwise
            if bState != aState:
                self.counter += 1
            else:  # rotating counter clockwise
                self.counter -= 1

        if bState != self.bLastState:
            self.bturn += 1

        self.aLastState = aState
        self.bLastState = bState

    def readEncoderTest(self):
        self.readEncoder()

    def callback_encoder(self, channel):
        self.readEncoder()


    # returns speed in rev/s
    def readSpeed(self):
        # correct for side of car left goes - otherwise
        if self.time != 0 and self.time != self.lastTime:
            # store speed in clicks/nS
            self.speed = (
                self.side * (self.counter - self.lastCounter)
                / (self.time - self.lastTime)
            )
        else:
            self.speed = 0

        # lastTime and lastCounter were set at last call to this function
        self.lastTime = self.time
        self.lastCounter = self.counter

        # Scale by encoder tick/ns to rev/s
        return self.speed / perRev / 4 * 1e9 * M_PER_REV

    def resetSpeed(self):
        self.speed = 0
        self.counter = 0
        self.lastCounter = 0
        self.time = time.perf_counter_ns()
        self.lastTime = time.perf_counter_ns()


# Set up Encoder instances with connections, GPIO.board (swheel),
# side = -1 if speed reported negative
sfl = Encoder("sfl", S1FL, S2FL, 1)
sfr = Encoder("sfr", S1FR, S2FR, -1)
srl = Encoder("srl", S1RL, S2RL, 1)
srr = Encoder("srr", S1RR, S2RR, -1)


# Set up Wheel instances with connections, ch 0 is left end,
# leaving one pin per quad for future
rl = Wheel("rl", ENBRL, IN3RL, IN4RL, rl_PID_gains, PID_OUTPUT_LIMITS,
           srl)   # Rear-left wheel
rr = Wheel("rr", ENARR, IN1RR, IN2RR, rr_PID_gains, PID_OUTPUT_LIMITS,
           srr)   # Rear-right wheel
fl = Wheel("fl", ENBFL, IN3FL, IN4FL, fl_PID_gains, PID_OUTPUT_LIMITS,
           sfl)   # Front-left wheel
fr = Wheel("fr", ENAFR, IN1FR, IN2FR, fr_PID_gains, PID_OUTPUT_LIMITS,
           sfr)   # Front-right wheel


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


# set up interrupts on A encoders, channel needs GPIO channel,
# callback has no parameters defined here,
# automatically gets self if object, + channel
GPIO.add_event_detect(sfl.s1, GPIO.BOTH, callback=sfl.callback_encoder)
GPIO.add_event_detect(sfr.s1, GPIO.BOTH, callback=sfr.callback_encoder)
GPIO.add_event_detect(srl.s1, GPIO.BOTH, callback=srl.callback_encoder)
GPIO.add_event_detect(srr.s1, GPIO.BOTH, callback=srr.callback_encoder)


def stop_car():
    # brakes all 4 wheels
    rl.brake()
    rr.brake()
    fl.brake()
    fr.brake()
    time.sleep(1)   # allow time to halt, then reset all speeds to 0
    sfl.resetSpeed()
    srr.resetSpeed()
    srl.resetSpeed()
    sfr.resetSpeed()


def set_powers(fl_power, fr_power, rl_power, rr_power):
    fl.move(fl_power)
    fr.move(fr_power)
    rl.move(rl_power)
    rr.move(rr_power)


def moveAllPID(vx, vy, omega, forSecs):
    # omega in rad/s, positive is CCW, negative is CW
    fl_target = vy + vx - omega * (ROBOT_WIDTH + ROBOT_LENGTH) / 2.0
    fr_target = vy - vx + omega * (ROBOT_WIDTH + ROBOT_LENGTH) / 2.0
    rl_target = vy - vx - omega * (ROBOT_WIDTH + ROBOT_LENGTH) / 2.0
    rr_target = vy + vx + omega * (ROBOT_WIDTH + ROBOT_LENGTH) / 2.0

    start_time = time.perf_counter_ns()
    while time.perf_counter_ns() - start_time < forSecs * 1e9:
        # Targets passed directly into each wheel's persistent PID
        set_powers(fl_target, fr_target, rl_target, rr_target)
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
    # Duration calculated the same way, now driven by PID
    forSecs = (pi * ROBOT_WIDTH * degrees / 360.0) / speed
    omega = -speed / (ROBOT_WIDTH / 2.0)   # CW → negative omega
    moveAllPID(0, 0, omega, forSecs)


def turn_left(speed, degrees):
    forSecs = (pi * ROBOT_WIDTH * degrees / 360.0) / speed
    omega = speed / (ROBOT_WIDTH / 2.0)    # CCW → positive omega
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

    # Uses PID loop instead of open-loop moveAll
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
    for x in actionList:
        print(x, end='')
        if '#' not in x:
            exec(x)

    stop_car()
    destroy()
    print("\nStopped and cleanup done")


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        stop_car()
        destroy()
        print("\nStopped and cleanup done")