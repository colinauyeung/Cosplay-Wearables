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
sample_rate = 20
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

cl = patterns.corrolation(5, 0.4)
# cr = patterns.corrolation(2, 0.4)
# cs = patterns.corrolation(5, 0.4)

model_l = []
with open("test2.csv") as m:
    csv_reader = csv.reader(m)
    for row in csv_reader:
        # print( "gx:{:0.2f} gy:{:0.2f} gz:{:0.2f}".format( float(row[0]), float(row[1]), float(row[2]) ) )
        # time.sleep(0.01)
        model_l.append([float(row[0]), float(row[1]), float(row[2])])
        
cl.prerendermodels(model_l, 0.5, 1.5)


# model_r = []
# with open("rightleg.csv") as m2:
#     csv_reader = csv.reader(m2)
#     for row in csv_reader:
#         # print( "gx:{:0.2f} gy:{:0.2f} gz:{:0.2f}".format( float(row[0]), float(row[1]), float(row[2]) ) )
#         # time.sleep(0.01)
#         model_r.append([float(row[0]), float(row[1]), float(row[2]), float(row[3])])
        
# cr.prerendermodels(model_r, 0.5, 1.5)


# model_s = []
# with open("squat.csv") as m3:
#     csv_reader = csv.reader(m3)
#     for row in csv_reader:
#         # print( "gx:{:0.2f} gy:{:0.2f} gz:{:0.2f}".format( float(row[0]), float(row[1]), float(row[2]) ) )
#         # time.sleep(0.01)
#         model_s.append([float(row[0]), float(row[1]), float(row[2]), float(row[3])])
        
# cs.prerendermodels(model_s, 0.5, 1.5)


print("Prerendering")

data = []
periods = []
prev = 0
last = (0,0,0,0)
counter = 0
timedown = 0
mode = False
cooldown = 100
coolcount = 0

t1 = time.perf_counter()
time.sleep( 0.005 )
while(True):
    t2 = time.perf_counter()
    ax, ay, az, gx, gy, gz, temp = mpu.readAllData()
    ahrs.update_no_magnetometer(np.array([gx, gy, gz]), np.array([ax,ay,az]), 1 / sample_rate )  # 100 Hz sample rate

    euler = ahrs.quaternion.to_euler()
    # print( "gx:{:0.2f} gy:{:0.2f} gz:{:0.2f}".format( euler[0], euler[1], euler[2] ) )

    ax2, ay2, az2, gx2, gy2, gz2, temp = mpu2.readAllData()
    ahrs2.update_no_magnetometer(np.array([gx2, gy2, gz2]), np.array([ax2,ay2,az2]), 1 / sample_rate )  # 100 Hz sample rate

    euler2 = ahrs2.quaternion.to_euler()
    # print( "gx2:{:0.2f} gy2:{:0.2f} gz2:{:0.2f}".format( euler2[0], euler2[1], euler2[2] ) )

    # if(counter < 100):
    #     counter +=1
    #     print( 1 / (t2 - t1) )
    #     t1 = t2
    #     time.sleep( max( 0, 1 / sample_rate  - (time.perf_counter() - t2) ) )
    #     continue

    data.append([euler[0], euler[1], euler2[0], euler2[1]])
    # diff  = abs(last[0] - euler[0]) +  abs(last[1] - euler[1]) +  abs(last[2] - euler2[0]) +  abs(last[3] - euler2[1])
    # if(diff < 5):
    #     counter +=1
    #     print( 1 / (t2 - t1) )
    #     t1 = t2
    #     time.sleep( max( 0, 1 / sample_rate  - (time.perf_counter() - t2) ) )

    #     continue


    if(coolcount >0):
        coolcount = coolcount - 1

    print(euler[1],euler2[1])
    if(mode):
        if(euler[1]<=10 and euler2[1]<=10 and coolcount == 0):
            mode = False
        else:
            setting = euler[1]
            if(setting < 0):
                setting = 0
            if(setting > 70):
                setting = 70

            angle = 70 + (70-setting)
            moveservo(1, angle)
            moveservo(2, angle)
            intensity = setting/70
            color(255 * intensity, 255 * intensity, 0)
    else:
        if(euler[1]<=10):
            if(euler2[1]<=10):
                if(coolcount == 0):
                    moveservo(1, 90)
                    moveservo(2, 90)
                    color(0,0,255)
                    mode = True
            else:
                moveservo(1, 70)
                moveservo(2, 110)
                color(0, 255, 0)
            

                
        elif(euler2[1]<=10):
            color(255, 0, 0)
            moveservo(1, 110)
            moveservo(2, 70)
      



    # cresl = cl.getresults(data)
    # cresr = cr.getresults(data)
    # cress = cs.getresults(data)
    # print(cresl[0], cresr[0], cress[0])


    # cresl = cl.convertresults(resl[0], resl[1], resl[2])
    # cresr = cr.convertresults(resr[0], resr[1], resr[2])
    # cress = cl.convertresults(ress[0], ress[1], ress[2])

    # if(cresl[0] > cresr[0] and cresl[0] >cress[0]):
    #     if(cresl[0]> 0.7):
    #         color(255, 0, 0)
    # elif(cresr[0] > cress[0]):
    #     if(cresr[0]>0.7):
    #         color(0, 255, 0)
    # else:
    #     if(cress[0] > 0.7):
    #         color(0,0,255)


    # if(cres[0] >= 0.85):
    #     pixels[0] = (0, 255, 0)
    #     pixels[1] = (0, 255, 0)
    #     pixels[2] = (0, 255, 0)
    #     pixels[3] = (0, 255, 0)
    #     pixels[4] = (0, 255, 0)
    #     pixels[5] = (0, 255, 0)
    #     timedown = 100

    # if(timedown>0):
    #     timedown = timedown - 1
    #     if(timedown == 0):
    #         pixels[0] = (0, 0, 0)
    #         pixels[1] = (0, 0, 0)
    #         pixels[2] = (0, 0, 0)
    #         pixels[3] = (0, 0, 0)
    #         pixels[4] = (0, 0, 0)
    #         pixels[5] = (0, 0, 0)
    

    # print(counter, c.convertresults(res[0], res[1], res[2]))
    if(len(data) > 100):
        data.pop(0)

    # last = (euler[0], euler[1], euler2[0], euler2[1])

    counter += 1
    # writer.writerow([euler[0], euler[1], euler2[0], euler2[1]])


    # print( 1 / (t2 - t1) )
    t1 = t2
    time.sleep( max( 0, 1 / sample_rate  - (time.perf_counter() - t2) ) )



# with open("test4.csv") as d:
#     csv_reader = csv.reader(d)
#     counter = 0
#     for row in csv_reader:
#         # print( "gx:{:0.2f} gy:{:0.2f} gz:{:0.2f}".format( float(row[0]), float(row[1]), float(row[2]) ) )
#         # time.sleep(0.01)

#         if(counter < 300):
#             counter +=1
#             continue

#         t1 = time.perf_counter()
#         data.append([float(row[0]), float(row[1]), float(row[2]), float(row[3])])
#         diff  = abs(last[0] - float(row[0])) +  abs(last[1] - float(row[1])) +  abs(last[2] - float(row[2])) +  abs(last[3] - float(row[3]))
#         if(diff < 10):
#             counter +=1
#             continue
#         if(prev == 0):
#             res = c.getresults(data, 0, 0, 0)
#         else:
#             res = c.getresults(data, prev[0], prev[1], prev[2])

#         print(counter, c.convertresults(res[0], res[1], res[2]))
#         prev = res
#         t2 = time.perf_counter()
#         periods.append(t2-t1)

#         if(len(data) > 111):
#             data.pop(0)

#         last = (float(row[0]), float(row[1]), float(row[2]), float(row[3]))

#         counter += 1
#         # if(counter > 200):
#         #     break
#         # if(res[0] > 0.8):
#         #     q = res[1]/res[2]
#         #     angle = 180 * q
#         #     servo1.ChangeDutyCycle(2+(angle/18))
#         #     time.sleep(0.5)
#         #     servo1.ChangeDutyCycle(0)

# total = 0
# for i in range(len(periods)):
#     total = total + periods[i]
# print(total/len(periods))

servo1.stop()
GPIO.cleanup()
print("Goodbye!")





# print(data)


# print(c.dotproduct([1,2,3], [1,5,7]))
# print(c.normalize([[1,2],[1,2],[1,2]]))

# f = [1,2,3]
# c.rotate(f, 2)
# print(f)

# print(signal.resample([1,2,3,2], 8))