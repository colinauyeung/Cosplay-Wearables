import math
from scipy import signal
import copy
import csv
import time
import RPi.GPIO as GPIO

import smbus
import imufusion
import numpy as np


import board
import neopixel


import patterns
import mpu6050

import statistics

#Break
GPIO.setup(9,GPIO.OUT)
servo2 = GPIO.PWM(9,50) # pin 11 for servo1, pulse 50Hz
GPIO.setup(11,GPIO.OUT)
servo1 = GPIO.PWM(11,50) # pin 11 for servo1, pulse 50Hz

# Start PWM running, with value of 0 (pulse off)
servo1.start(0)
servo2.start(0)


def moveservo(servo, angle):
    if(servo == 1):
        servo1.ChangeDutyCycle(2+(angle/18))  
    else:
        swap = 180-angle
        servo2.ChangeDutyCycle(2+(swap/18))

mpu = mpu6050.MPU6050(0x68)
mpu2 = mpu6050.MPU6050(0x69)
    

ahrs = imufusion.Ahrs()
sample_rate = 40
ahrs.settings = imufusion.Settings(0.5,  # gain
                                10,  # acceleration rejection
                                20,  # magnetic rejection
                                5 * sample_rate)  # rejection timeout = 5 seconds

ahrs2  = imufusion.Ahrs()
ahrs2.settings = imufusion.Settings(0.5,  # gain
                                10,  # acceleration rejection
                                20,  # magnetic rejection
                                5 * sample_rate)  # rejection timeout = 5 seconds

pixels = neopixel.NeoPixel(board.D10, 6)

	
pixels[0] = (0, 0, 0)
pixels[1] = (0, 0, 0)
pixels[2] = (0, 0, 0)
pixels[3] = (0, 0, 0)
pixels[4] = (0, 0, 0)
pixels[5] = (0, 0, 0)

def color(r,g,b):
    pixels[0] = (r, g, b)
    pixels[1] = (r, g, b)
    pixels[2] = (r, g, b)
    pixels[3] = (r, g, b)
    pixels[4] = (r, g, b)
    pixels[5] = (r, g, b)

moveservo(1, 90)
moveservo(2, 90)
color(255, 0, 0)
time.sleep(1)
color(255,255,0)

moveservo(1,0)
moveservo(0,0)

cl = patterns.corrolation(10, 0.4)
cs = patterns.corrolation(10, 0.4)

model_l = []
with open("walkingsample.csv") as m:
    csv_reader = csv.reader(m)
    for row in csv_reader:
        model_l.append([float(row[0]), float(row[1])])
        
cl.prerendermodels(model_l, 0.75, 1.25)

model_s = []
with open("squatsample.csv") as m:
    csv_reader = csv.reader(m)
    for row in csv_reader:
        model_s.append([float(row[0]), float(row[1])])
        
cs.prerendermodels(model_s, 0.75, 1.25)




print("Prerendering")

data = []
counter = 0
stateq = []
diff = []
inoutmode = True
prevsquat = False
stateqdiff = 10
countdown = 0
base = 0

t2 = time.perf_counter()
time.sleep( 0.005 )
while(True):

    if(counter == stateqdiff):
        m = statistics.mode(stateq)
        if(inoutmode):
            if(m == 0):
                moveservo(1, 90)
                moveservo(2, 90)   
            elif (m==1):
                moveservo(1, 50)
                moveservo(2, 90)
            else:
                moveservo(1, 90)
                moveservo(2, 50)
        counter = 0
    counter += 1
    # print(counter)
    t1 = t2




    ax, ay, az, gx, gy, gz, temp = mpu.readAllData()
    ahrs.update_no_magnetometer(np.array([gx, gy, gz]), np.array([ax,ay,az]), 1 / sample_rate )  # 100 Hz sample rate

    euler = ahrs.quaternion.to_euler()

    ax2, ay2, az2, gx2, gy2, gz2, temp = mpu2.readAllData()
    ahrs2.update_no_magnetometer(np.array([gx2, gy2, gz2]), np.array([ax2,ay2,az2]), 1 / sample_rate )  # 100 Hz sample rate

    euler2 = ahrs2.quaternion.to_euler()



    data.append([euler[1], euler2[1]])

    maxangle = 10
    
    if(not inoutmode):
        if(countdown > 0):
            countdown = countdown - 1
            if(countdown == 0):
                base = euler2[1] - maxangle
        else:
            current = euler2[1] - maxangle
            if(current > base):
                current = base
            if(current < 0): 
                current = 0
            percentcur = current/base
            angle = 90*percentcur
            moveservo(1, angle)
            moveservo(2, angle)
    # t2 = time.perf_counter()
    # print( (t2 - t1) )

    resl = cl.getresults(data)
    cresl = cl.convertresults(resl[0], resl[1], resl[2])

    ress = cs.getresults(data)
    cress = cs.convertresults(ress[0], ress[1], ress[2])


    diff.append(euler[1] + euler2[1])
    if(len(diff) > 10):
        diff.pop(0)

    upper = 0.6
    lower = 0.1

    if(cresl[0] > cress[0]):
        prevsquat = False
        motion = cresl[1]/cresl[2]
        if((max(diff) - min(diff)) < 2):
            stateq.append(0)
        else:   
            if(motion >= lower and motion < upper):
                stateq.append(1)
            elif(motion >= upper or motion < lower):
                stateq.append(2)
      

    else:
        stateq.append(0)
        if(not prevsquat and euler[1] <= 50):
            print(inoutmode)
            inoutmode = not inoutmode
            countdown = 80
        # moveservo(1, 90)
        # moveservo(2, 90)
        prevsquat = True

    if(len(stateq) > stateqdiff):
        stateq.pop(0)

    if(len(data) > 100):
        data.pop(0)




  

    t2 = time.perf_counter()
    # print( 1/(t2 - t1) )
    time.sleep( max( 0, ((1/sample_rate)  - (t2 - t1)) ) )
    


servo1.stop()
GPIO.cleanup()
print("Goodbye!")

