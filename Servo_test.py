import time
from adafruit_servokit import ServoKit

kit     = ServoKit(channels=16)
seconds = 3

print("start")

kit.servo[8].angle = 180 #degree
print("Direction = y-180")
time.sleep(seconds)

kit.servo[10].angle = 0 #degree
print("Direction = x-0")
time.sleep(seconds)

      
print("finish")

