# test_hiwonder_motora.py
# Test the Oled display driver using I2C
#
# MicroPython v1.20.0
# verified by k. winter
#
from machine import Pin, I2C, SoftI2C
#from pyb import Pin, I2C # not available: SoftI2C
import time, gc

##### flash drive ################
# Files pca9685.py and servo.py (within the pca9685 folder) must reside on the BlackPill.
import pca9685
from servo import Servos
# Files including folders ir_rx/ir_tx must reside on BlackPill
from ir_rx.nec import NEC_8, NEC_16, SAMSUNG
# **** PLUG IN the OLED as it has pull-up resistors for the I2C lines
##### Constants and Pin Definitions ################
I2C_PORT = 'I2C1'
I2C_SDA_PIN = 'PB7'
I2C_SCL_PIN = 'PB6'
I2C_FREQ = 40000

# using PCA9685
HW_SERVO_PORT = 0
SERVO_STRAIGHT_ANGLE = 100

# Motor Controller
I2C_ADDR = 0x34
ADC_BAT_ADDR = 0
MOTOR_TYPE_ADDR = 20
MOTOR_ENCODER_POLARITY_ADDR = 21
MOTOR_FIXED_PWM_ADDR = 31
MOTOR_FIXED_SPEED_ADDR = 51
MOTOR_ENCODER_TOTAL_ADDR = 60

#Motor Type
MOTOR_TYPE_WITHOUT_ENCODER = 0
MOTOR_TYPE_TT = 1
MOTOR_TYPE_N20 = 2
MOTOR_TYPE_JGB37_520_12V_110RPM = 3

# IR 1838 board
IR_PIN = "PB0"
ir1838 = Pin(IR_PIN, Pin.IN, Pin.PULL_UP)

#### Functions #############
# IR1838 callback function
def cb(data, addr, ctrl):
    if data < 0:  # NEC protocol sends repeat codes.
        print("Repeat code.")
    else:
        #print(f"Data 0x{data:02x} Addr 0x{addr:04x} Ctrl 0x{ctrl:02x}")
        setHeading(data)

#
# this is the HiWonder Test Program
#
car_forward = bytearray(4)
car_stop = bytearray(4)
car_forward = [-10,0,10,0]
car_retreat = [8,0,-8,0]
car_stop = [0,0,0,0]

def goForward():
    print("forward")
    servoboard.position(0, SERVO_STRAIGHT_ANGLE)
    i2c.writeto_mem(I2C_ADDR, MOTOR_FIXED_SPEED_ADDR, bytearray(car_forward))
    time.sleep(1)

def goReverse():
    print("reverse")
    servoboard.position(0, SERVO_STRAIGHT_ANGLE)
    i2c.writeto_mem(I2C_ADDR, MOTOR_FIXED_SPEED_ADDR, bytearray(car_retreat))
    time.sleep(1)

def goRight():
    print("right")
    servoboard.position(0, SERVO_STRAIGHT_ANGLE + 45)
    i2c.writeto_mem(I2C_ADDR, MOTOR_FIXED_SPEED_ADDR, bytearray(car_forward))
    time.sleep(1)
    servoboard.position(0, SERVO_STRAIGHT_ANGLE + 0)
    i2c.writeto_mem(I2C_ADDR, MOTOR_FIXED_SPEED_ADDR, bytearray(car_forward))
    time.sleep(0.2)

def goLeft():
    print("left")
    servoboard.position(0, SERVO_STRAIGHT_ANGLE - 45)
    i2c.writeto_mem(I2C_ADDR, MOTOR_FIXED_SPEED_ADDR, bytearray(car_forward))
    time.sleep(1)
    servoboard.position(0, SERVO_STRAIGHT_ANGLE)
    i2c.writeto_mem(I2C_ADDR, MOTOR_FIXED_SPEED_ADDR, bytearray(car_forward))
    time.sleep(0.2)

def motorStop():
    print("stop")
    i2c.writeto_mem(I2C_ADDR, MOTOR_FIXED_SPEED_ADDR, bytearray(car_stop))
    time.sleep(1)

def setHeading(data):
    global heading, mspeed, done
    heading = "S"
    val = str(data)
    print("heading: ",val)
    if val == "13":
        mspeed = 0
        heading = "Exit"
        done = True
    if val == "24":
        mspeed = 90
        heading = "Fwd"
        goForward()
    if val == "28":
        mspeed = 0
        heading = "Stop"
        motorStop()
    if val == "90":
        mspeed = 60
        heading = "Right"
        goRight()
    if val == "82":
        mspeed = 70
        heading = "Reverse"
        goReverse()
    if val == "8":
        mspeed = 60
        heading = "Left"
        goLeft()
    print("Keypress code:{} Heading={}.".format(val,heading))
##### Setup ################
i2c = SoftI2C(scl=Pin(I2C_SCL_PIN), sda=Pin(I2C_SDA_PIN), freq=I2C_FREQ)
servoboard = Servos(i2c, address=0x40, freq=50, min_us=600, max_us=2400,
                 degrees=180)
classes = (NEC_8, NEC_16)
ir = classes[0](ir1838, cb)  # Instantiate receiver
##### Configure the Motor Control Board ################
MotorType =  MOTOR_TYPE_JGB37_520_12V_110RPM
MotorEncoderPolarity = 0
temp = bytearray(1)
temp[0] = MotorType
i2c.writeto_mem(I2C_ADDR, MOTOR_TYPE_ADDR, temp)
time.sleep(0.1)

temp[0] = MotorEncoderPolarity
i2c.writeto_mem(I2C_ADDR, MOTOR_ENCODER_POLARITY_ADDR,temp)
time.sleep(0.1)

##### Loop ################


print("servo 0 pan")
for i in range(-45,45,15):
    print("pan {} angle".format(i))
    servoboard.position(0, SERVO_STRAIGHT_ANGLE + i)
    time.sleep(0.1)
    
servoboard.position(0, SERVO_STRAIGHT_ANGLE)
done = False
try:
    print("running")
    while not done:
        print(".", end="" )
        time.sleep(5)
        gc.collect()
except KeyboardInterrupt:
    ir.close()
    print()
print("Finished.")

