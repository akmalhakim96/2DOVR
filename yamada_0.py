#!/usr/bin/python3
# vl53_mlt_tsXX.py  Yasushi Honda 2020 5.18


import time
import VL53L0X
import RPi.GPIO as GPIO
import keyin
import motor
import math

period=0.05

mL=motor.Lmotor(17)
mR=motor.Rmotor(18)

# GPIO for Sensor 1 shutdown pin
sensor1_shutdown = 22
# GPIO for Sensor 2 shutdown pin
sensor2_shutdown = 23

GPIO.setwarnings(False)

# Setup GPIO for shutdown pins on each VL53L0X
GPIO.setmode(GPIO.BCM)
GPIO.setup(sensor1_shutdown, GPIO.OUT)
GPIO.setup(sensor2_shutdown, GPIO.OUT)

# Set all shutdown pins low to turn off each VL53L0X
GPIO.output(sensor1_shutdown, GPIO.LOW)
GPIO.output(sensor2_shutdown, GPIO.LOW)

# Keep all low for 500 ms or so to make sure they reset
time.sleep(0.50)

# Create one object per VL53L0X passing the address to give to
# each.
tof = VL53L0X.VL53L0X(address=0x2B)
tof1 = VL53L0X.VL53L0X(address=0x2D)

# Set shutdown pin high for the first VL53L0X then 
# call to start ranging 
GPIO.output(sensor1_shutdown, GPIO.HIGH)
time.sleep(0.50)
#tof.start_ranging(VL53L0X.VL53L0X_BETTER_ACCURACY_MODE)
tof.start_ranging(VL53L0X.VL53L0X_LONG_RANGE_MODE)

# Set shutdown pin high for the second VL53L0X then 
# call to start ranging 
GPIO.output(sensor2_shutdown, GPIO.HIGH)
time.sleep(0.50)
#tof1.start_ranging(VL53L0X.VL53L0X_BETTER_ACCURACY_MODE)
tof1.start_ranging(VL53L0X.VL53L0X_LONG_RANGE_MODE)

timing = tof.get_timing()
if (timing < 20000):
    timing = 20000
print ("Timing %d ms" % (timing/1000))

kbd=keyin.Keyboard()

now=time.time()
start=now
init=now
rate=0

ganma = 5
delta = 0.1
p = 300
q = 0


print("Input 'q' to stop this program")
key='c'
while key!='q':
   try:
       distance = tof.get_distance()
       distance1 = tof1.get_distance()
       now = time.time()
       rate+=1
       #print (" %6.2f %d %d mm" % (now-start, distance, distance1) )
       #time.sleep(timing/1000000.00)
       #time.sleep(0.01)

       left = ganma*(math.tanh(delta*(distance-p))+q)
    
       if now-init>period: 
          rate=rate/period
          print ("\r time=%6.2f %4d %4d mm rate=%3d" % (now-start, left, distance1,rate), end=' ')
          rate=0
          init=now
        
       mL.run(left)
       mR.run(left)
        
   except :
       break

   key=kbd.read()

tof1.stop_ranging()
GPIO.output(sensor2_shutdown, GPIO.LOW)
tof.stop_ranging()
GPIO.output(sensor1_shutdown, GPIO.LOW)

mL.run(0)
mR.run(0)