#sonic.py
from pyb import Pin
from machine import Pin, I2C
import time

##### flash drive ################
#driver files must be on flash drive of BlackPill
# ultraSonic device is HC-SR04 (or HC-SR05)
from hcsr04 import HCSR04
#servoboard is the PCA9685 device
import pca9685
from servo import Servos

# Servo Zero assumes:
# far right is 0 degrees
# straight ahead (front position) is 90 degrees
# far left is 180 degrees
FRONTPOSITION = 90

#Servo One assumes:
# bottom (level and straight-ahead) is 0 degrees
# highest is 90 degrees


#23456789012345678901234567890123456789012345678901234567890123456789012345678901234567890
#        1         2         3         4         5         6         7         8  

class Sonic():
    
    def __init__(self, SONIC_TRIG_PIN, SONIC_ECHO_PIN):
        self.sonic = HCSR04(trigger_pin=SONIC_TRIG_PIN, echo_pin=SONIC_ECHO_PIN,echo_timeout_us=1000000)
        self.has_servos = False
        self.distance = 0
        self.detect_warn = 29
        self.detect_alert = 24
        self.warn = False
        self.alert = False
        self.front_bad = False
        self.right_bad = False
        self.left_bad = False
        self.servo1st_port = 0  #modified by setupServos
        self.servo2nd_port = 1  #modified by setupServos
        self.zero_front = 90    #modified by setupServos
        self.offset = 90 - self.zero_front
        return
    
    def resetAlerts(self):
        self.front_bad = False
        self.right_bad = False
        self.left_bad = False
        self.servoboard.position(self.servo1st_port,90+self.offset)
        self.servoboard.position(self.servo2nd_port,0)
        
    def setZeroFront(self, Front):
        self.zero_front = Front
        return
    
    def obstacle(self, Dist):
        self.warn = False
        self.alert = False
        if Dist < self.detect_warn:
            self.warn = True
        if Dist < self.detect_alert:
            self.alert = True
        return self.warn or self.alert
        
    def setupServos(self, I2C_PORT, I2C_SCL_PIN, I2C_SDA_PIN, I2C_FREQ, First_Port, Second_Port, Front=90):
        self.zero_farright = 0
        self.zero_front = Front
        self.zero_farleft = 180
        self.servo1st_port = First_Port
        self.servo2nd_port = Second_Port       
        self.i2c = I2C(scl=Pin(I2C_SCL_PIN), sda=Pin(I2C_SDA_PIN), freq=I2C_FREQ)
        #servoboard is the PCA9685 device
        self.servoboard = Servos(self.i2c, address=0x40, freq=50, min_us=600, max_us=2400,degrees=180)
        self.has_servos = True
        return
    
    def scan(self):
        self.servoboard.position(self.servo1st_port,90+self.offset)
        self.servoboard.position(self.servo2nd_port,0)
        #find obstacles
#         self.front_bad = False
#         self.right_bad = False
#         self.left_bad = False
        self.servoboard.position(self.servo1st_port, degrees=(90 + self.offset))
        time.sleep(0.2)
        self.distance = self.sonic.distance_cm()
        if self.obstacle(self.distance):
	    if self.alert:
                self.front_bad = True
                print("Stop. Front distance=",self.distance)
        if self.front_bad:
            #check right
            self.servoboard.position(self.servo1st_port, degrees=(30 + self.offset))
            time.sleep(0.2)
            self.distance = self.sonic.distance_cm()
            if self.obstacle(self.distance):
                self.right_bad = True 
                print("right distance=",self.distance)
            #check left
            self.servoboard.position(self.servo1st_port, degrees=(150 + self.offset))
            time.sleep(0.2)
            self.distance = self.sonic.distance_cm()
            if self.obstacle(self.distance):
                self.left_bad = True
                print("left distance=",self.distance)  

