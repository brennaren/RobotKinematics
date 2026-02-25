from cmath import cos, pi, sin
from math import atan2
import time
import sys
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
MPS_TO_POWER_TURN = 240.0   # convert from m/s to power percentage
MPS_TO_POWER_LINEAR = 150.0     # convert from m/s to power percentage
VELOCITY_SCALE_SEMICIRCLE = 0.8   # adjust velocity for semicircles

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
perRev = 750    # estimate A type motors 155 #2024 motors 750

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
    def __init__(self, name, enCh, in1Ch, in2Ch):
        self.name = name
        self.en = pca.channels[enCh]
        self.in1 = pca.channels[in1Ch]
        self.in2 = pca.channels[in2Ch]

    def move(self, power):
        self.in1.duty_cycle = high if power > 0 else low
        self.in2.duty_cycle = low if power > 0 else high
        self.en.duty_cycle = getPWMPer(abs(power))

    def brake(self):
        self.in1.duty_cycle = low
        self.in2.duty_cycle = low


# Set up Wheel instances with connections, ch 0 is left end,
# leaving one pin per quad for future
rl = Wheel("rl", ENBRL, IN3RL, IN4RL)   # Rear-left wheel
rr = Wheel("rr", ENARR, IN1RR, IN2RR)   # Rear-right wheel
fl = Wheel("fl", ENBFL, IN3FL, IN4FL)   # Front-left wheel
fr = Wheel("fr", ENAFR, IN1FR, IN2FR)   # Front-right wheel


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

        return self.speed

    def resetSpeed(self):
        self.speed = 0
        self.counter = 0
        self.lastCounter = 0
        self.time = time.perf_counter_ns()
        self.lastTime = time.perf_counter_ns()


# Set up Encoder instances with connections, GPIO.board (swheel),
# side = -1 if speed reported negative
sfl = Encoder("sfl", S1FL, S2FL, -1)
sfr = Encoder("sfr", S1FR, S2FR, 1)
srl = Encoder("srl", S1RL, S2RL, -1)
srr = Encoder("srr", S1RR, S2RR, 1)


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


def moveAll(vx, vy, omega, forSecs):
    # omega in rad/s, positive is CCW, negative is CW
    fl_power = vy + vx - omega * (ROBOT_WIDTH + ROBOT_LENGTH) / 2.0
    fr_power = vy - vx + omega * (ROBOT_WIDTH + ROBOT_LENGTH) / 2.0
    rl_power = vy - vx - omega * (ROBOT_WIDTH + ROBOT_LENGTH) / 2.0
    rr_power = vy + vx + omega * (ROBOT_WIDTH + ROBOT_LENGTH) / 2.0

    # Scale power to convert from m/s to power percentage
    # and also to ensure that the maximum power does not exceed 100%
    max_power = max(abs(fl_power), abs(fr_power),
                    abs(rl_power), abs(rr_power)) or 1  # avoid division by 0
    scale_factor = min(1, 100.0 / max_power)
    fl_power = int(fl_power * MPS_TO_POWER_LINEAR * scale_factor)
    fr_power = int(fr_power * MPS_TO_POWER_LINEAR * scale_factor)
    rl_power = int(rl_power * MPS_TO_POWER_LINEAR * scale_factor)
    rr_power = int(rr_power * MPS_TO_POWER_LINEAR * scale_factor)

    print(fl_power)
    print(fr_power)
    print(rl_power)
    print(rr_power)
    print(forSecs)

    # set wheel speeds
    fl.move(fl_power)
    fr.move(fr_power)
    rl.move(rl_power)
    rr.move(rr_power)
    time.sleep(forSecs)


def moveFromTo(startx, starty, endx, endy, forSecs):
    # velocity in m/s, positive is forward, negative is backward, 0 is stop
    # compute the distance to move and the angle to move in
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

    moveAll(vx, vy, 0, forSecs)


def go_ahead(speed, distance):
    forSecs = distance / speed
    power = speed * MPS_TO_POWER_LINEAR
    print(forSecs)
    print(power)
    rl.move(power)
    rr.move(power)
    fl.move(power)
    fr.move(power)
    time.sleep(forSecs)


def go_back(speed, distance):
    forSecs = distance / speed
    power = speed * MPS_TO_POWER_LINEAR
    rr.move(-power)
    rl.move(-power)
    fr.move(-power)
    fl.move(-power)
    time.sleep(forSecs)


# Making right turn on spot (tank turn)
def turn_right(speed, degrees):
    forSecs = (pi * ROBOT_WIDTH * degrees/360.0) / (speed)
    power = speed * MPS_TO_POWER_TURN
    rl.move(power)
    rr.move(-power)
    fl.move(power)
    fr.move(-power)
    time.sleep(forSecs)


# Making left turn on spot (tank turn)
def turn_left(speed, degrees):
    forSecs = (pi * ROBOT_WIDTH * degrees/360.0) / (speed)
    power = speed * MPS_TO_POWER_TURN

    print(forSecs)
    print(power)

    rr.move(power)
    rl.move(-power)
    fr.move(power)
    fl.move(-power)
    time.sleep(forSecs)


# parallel left shift (crab left)
def shift_left(power, forSecs):
    fr.move(power)
    rr.move(-power)
    rl.move(power)
    fl.move(-power)
    time.sleep(forSecs)


# parallel right shift (crab right)
def shift_right(power, forSecs):
    fr.move(-power)
    rr.move(power)
    rl.move(-power)
    fl.move(power)
    time.sleep(forSecs)


# Diagonal forward and right @45
def upper_right(power, forSecs):
    rr.move(power)
    fl.move(power)
    time.sleep(forSecs)


# Diagonal back and left @45
def lower_left(power, forSecs):
    rr.move(-power)
    fl.move(-power)
    time.sleep(forSecs)


# Diagonal forward and left @45
def upper_left(power, forSecs):
    fr.move(power)
    rl.move(power)
    time.sleep(forSecs)


# Diagonal back and right @45
def lower_right(power, forSecs):
    fr.move(-power)
    rl.move(-power)
    time.sleep(forSecs)


# Front left only
def front_left(power, forSecs):
    print("Front left ahead @ " + str(power) + "% for " + str(forSecs)
          + " secs.")
    fl.move(power)
    time.sleep(forSecs)
    test_speed()
    stop_car()  # will set speed to 0


# Front right only
def front_right(power, forSecs):
    print("Front right ahead @ " + str(power) + "% for " + str(forSecs)
          + " secs.")
    fr.move(power)
    time.sleep(forSecs)
    test_speed()
    stop_car()  # will set speed to 0


# Rear left only
def rear_left(power, forSecs):
    print("Rear left ahead @ " + str(power) + "% for " + str(forSecs)
          + " secs.")
    rl.move(power)
    time.sleep(forSecs)
    test_speed()
    stop_car()  # will set speed to 0


# Rear right only
def rear_right(power, forSecs):
    print("Rear right ahead @ " + str(power) + "% for " + str(forSecs)
          + " secs.")
    rr.move(power)
    time.sleep(forSecs)
    test_speed()
    stop_car()  # will set speed to 0


def coastAll(forSecs):
    print("Coast forSecs " + str(forSecs) + " secs.")
    rl.move(0)
    rr.move(0)
    fr.move(0)
    fl.move(0)
    time.sleep(forSecs)
    stop_car()  # will set speed to 0


def turn_semicircle(radius, forSecs):
    # radius in m, forSecs in s, positive radius is left, negative is right
    # compute the velocity needed to make the turn in the given time
    velocity = (pi * abs(radius)) / forSecs

    # compute the angular velocity needed to make the turn in the given time
    omega = velocity / abs(radius)
    if radius < 0:
        omega = -omega

    print("Turning semicircle with radius " + str(radius) + " m at velocity "
          + str(velocity) + " m/s for " + str(forSecs) + " seconds.")

    moveAll(0, velocity * VELOCITY_SCALE_SEMICIRCLE, omega, forSecs)


def test_readPush():
    changed, state = readPush()
    if changed:
        print("button changed to " + str(state))


def destroy():
    # pwmOEn=1 # disable outputs of PCA9685
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

    GPIO.output(PWMOEN, 0)  # enable PWM outputs
    for x in actionList:
        print(x, end='')
        if '#' not in x:
            exec(x)

    stop_car()  # stop movement

    destroy()   # clean up GPIO
    print("\nStopped and cleanup done")


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        stop_car()  # stop movement
        destroy()   # clean up GPIO
        print("\nStopped and cleanup done")
