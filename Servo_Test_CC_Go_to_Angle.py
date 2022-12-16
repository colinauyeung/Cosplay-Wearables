# Import libraries
import RPi.GPIO as GPIO
import time
import board
import neopixel


pixels = neopixel.NeoPixel(board.D10, 6)

	
pixels[0] = (0, 255, 0)
pixels[1] = (0, 255, 0)
pixels[2] = (0, 255, 0)
pixels[3] = (0, 255, 0)
pixels[4] = (0, 255, 0)
pixels[5] = (0, 255, 0)



# Set GPIO numbering mode
# GPIO.setmode(GPIO.BOARD)

# Set pin 11 as an output, and define as servo1 as PWM pin
GPIO.setup(9,GPIO.OUT)
servo2 = GPIO.PWM(9,50) # pin 11 for servo1, pulse 50Hz
GPIO.setup(11,GPIO.OUT)
servo1 = GPIO.PWM(11,50) # pin 11 for servo1, pulse 50Hz

# Start PWM running, with value of 0 (pulse off)
servo1.start(0)
servo2.start(0)

# Loop to allow user to set servo angle. Try/finally allows exit
# with execution of servo.stop and GPIO cleanup :)

try:
    while True:
        #Ask user for angle and turn servo to it
        angle = float(input('Enter angle between 0 & 180: '))
        pixels[0] = (255, 255, 0)
        pixels[1] = (255, 255, 0)
        pixels[2] = (255, 255, 0)
        pixels[3] = (255, 255, 0)
        pixels[4] = (255, 255, 0)
        pixels[5] = (255, 255, 0)
        servo1.ChangeDutyCycle(2+(angle/18))  
        swap = 180-angle
        servo2.ChangeDutyCycle(2+(swap/18))
        # time.sleep(0.5)
        # servo1.ChangeDutyCycle(0)
        # servo2.ChangeDutyCycle(0)
   

finally:
    #Clean things up at the end
    servo1.stop()
    servo2.stop()
    GPIO.cleanup()
    print("Goodbye!")




