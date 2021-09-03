# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import RPi.GPIO as GPIO
import time
import cv2
import numpy as np

from adafruit_servokit import ServoKit
kit     = ServoKit(channels=16)

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 2
rawCapture = PiRGBArray(camera, size=(640, 480))
# allow the camera to warmup
time.sleep(0.1)

# define the range of red and the parameter for drawing the blobs
lower_red_0 = np.array([0, 70, 0]) 
upper_red_0 = np.array([5, 255, 255])
lower_red_1 = np.array([175, 70, 0]) 
upper_red_1 = np.array([180, 255, 255])
avg_x = []
avg_y = []
cX = 0
cY = 0

# define the GPIO for L298n
ENB = 21
ENA = 5
IN1 = 19
IN2 = 26
IN3 = 13
IN4 = 6
tim1 = 0.04
tim2 = 0.2

# define the GPIO for ultrasonic sensor
Trig = 14
Echo = 15
dis = 0

# for grab or drop
condition = 0

#set the mode of GPIO
GPIO.setmode(GPIO.BCM)

'''
GPIO.cleanup()
print("GPIO clean up")
time.sleep(5)
'''

# set the category of L298n GPIO pin
GPIO.setup(ENA, GPIO.OUT)       
GPIO.setup(IN1, GPIO.OUT)       
GPIO.setup(IN2, GPIO.OUT)      
GPIO.setup(ENB, GPIO.OUT)       
GPIO.setup(IN3, GPIO.OUT)       
GPIO.setup(IN4, GPIO.OUT)

# set the category of ultrasonic sensor GPIO pin
GPIO.setup(Trig, GPIO.OUT, initial = GPIO.LOW)
GPIO.setup(Echo, GPIO.IN)

# use the ultrasonic sensor to check distance
def checkdist():
    GPIO.output(Trig, GPIO.HIGH)
    time.sleep(0.00015)
    GPIO.output(Trig, GPIO.LOW)
    while not GPIO.input(Echo):
        pass
    t1 = time.time()
    while GPIO.input(Echo):
        pass
    t2 = time.time()
    return (t2-t1)*340*100/2

# let the car go ahead
def go_ahead():
    print("Start go ahead")
    GPIO.output(IN1, True)     
    GPIO.output(IN2, False)     
    GPIO.output(ENA, True)      
    GPIO.output(IN3, False)     
    GPIO.output(IN4, True)  
    GPIO.output(ENB, True)
    time.sleep(0.1)         
    GPIO.output(ENA, False)
    GPIO.output(ENB, False)
    print("End go ahead ")
    time.sleep(tim2)

# let the car back off
def back_off():
    print("Start back off")
    GPIO.output(IN1, False)     
    GPIO.output(IN2, True)     
    GPIO.output(ENA, True)      
    GPIO.output(IN3, True)     
    GPIO.output(IN4, False)  
    GPIO.output(ENB, True)
    time.sleep(tim1)         
    GPIO.output(ENA, False)
    GPIO.output(ENB, False)
    print("End back off")
    time.sleep(tim2)

# let the car turn right
def turn_left():
    print("Start turn left")
    GPIO.output(IN1, False)     
    GPIO.output(IN2, True)      
    GPIO.output(ENA, True)      
    GPIO.output(IN3, False)     
    GPIO.output(IN4, True)      
    GPIO.output(ENB, True)
    time.sleep(tim1)               
    GPIO.output(ENA, False)
    GPIO.output(ENB, False)
    print("End turn left")
    time.sleep(tim2)

# let the car turn left
def turn_right():
    print("Star turn right")
    GPIO.output(IN1, True)     
    GPIO.output(IN2, False)    
    GPIO.output(ENA, True)     
    GPIO.output(IN3, True)     
    GPIO.output(IN4, False)    
    GPIO.output(ENB, True)
    time.sleep(tim1)              
    GPIO.output(ENA, False)
    GPIO.output(ENB, False)
    print("End turn right")
    time.sleep(tim2)

# let the car stop and clean GPIO
def car_stop():
    GPIO.output(ENA, False)
    GPIO.output(ENB, False)
    print("Car stop!")
    time.sleep(1)

# clean the GPIO
def clean_gpio():
    GPIO.cleanup()
    print("GPIO clean up!")
    time.sleep(1)

# standard the position of robotic arm
def arm_standard():
    kit.servo[8].angle = 130 #degree
    print("Direction = y-130")
    time.sleep(0.5)
    kit.servo[10].angle = 45 #degree
    print("Direction = x-45")
    time.sleep(0.5)
    kit.servo[10].angle = 0 #degree
    print("Direction = x-0")
    time.sleep(1)

# move the arm to grab the cup
def cup_grab():
    print("Start to grab")
    kit.servo[8].angle = 140 #degree
    print("Direction = y-140")
    time.sleep(0.5)
    kit.servo[8].angle = 150 #degree
    print("Direction = y-150")
    time.sleep(0.5)
    kit.servo[8].angle = 160 #degree
    print("Direction = y-160")
    time.sleep(0.5)
    kit.servo[8].angle = 170 #degree
    print("Direction = y-170")
    time.sleep(0.5)
    kit.servo[8].angle = 180 #degree
    print("Direction = y-180")
    time.sleep(0.5)
    kit.servo[10].angle = 45 #degree
    print("Direction = x-45")
    time.sleep(1)
    kit.servo[8].angle = 150 #degree
    print("Direction = y-150")
    time.sleep(0.5)
    kit.servo[8].angle = 120 #degree
    print("Direction = y-120")
    print("Grab end")
    print("Start turn back")
    GPIO.output(IN1, True)     
    GPIO.output(IN2, False)    
    GPIO.output(ENA, True)     
    GPIO.output(IN3, True)     
    GPIO.output(IN4, False)    
    GPIO.output(ENB, True)
    time.sleep(0.7)              
    GPIO.output(ENA, False)
    GPIO.output(ENB, False)
    print("End turn back")
    time.sleep(tim2)
    time.sleep(3)

# move the arm to drop the cup
def cup_drop():
    kit.servo[8].angle = 140 #degree
    print("Direction = y-180")
    time.sleep(0.5)
    kit.servo[8].angle = 160 #degree
    print("Direction = y-180")
    time.sleep(0.5)
    kit.servo[8].angle = 180 #degree
    print("Direction = y-180")
    time.sleep(0.5)
    kit.servo[10].angle = 0 #degree
    print("Direction = x-0")
    print("End cup drop")
    time.sleep(8)

    

arm_standard()

# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	# grab the raw NumPy array representing the image, then initialize the timestamp
	# and occupied/unoccupied text
        image = frame.array
        # draw the center of the image frame
        cv2.circle(image, (320, 240), 3, (0,255,0), -1)
        # convert image from BGR to HSV
        hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # Set the range of red in HSV
        red_mask0 = cv2.inRange(hsv_img, lower_red_0, upper_red_0)
        red_mask1 = cv2.inRange(hsv_img, lower_red_1, upper_red_1)
        red_mask = cv2.bitwise_or(red_mask0, red_mask1)
        # find and draw contours
        Red_cnts = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        cv2.drawContours(image, Red_cnts, -1, (0, 0, 255), 2)
        
        # deal with every contour in one frame
        #print(" r = ", len(Red_cnts))
        for cnt in range(len(Red_cnts)):
                # take the contour first and measure the area
                cnts = Red_cnts[cnt]
                area = cv2.contourArea(cnts)
                #print("cnt = ", cnt)
                if area > 2000:
                        # find and draw the mean point
                        M = cv2.moments(cnts)
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                        cv2.circle(image, (cX, cY), 3, (255,0,0), -1)
                        cv2.circle(red_mask, (cX, cY), 3, (255,0,0), -1)
                        
                else:
                        pass
               
        # measure the distance of x-axis                
        x_dis = cX-320
        #y_dis = cY-320
        print("The x-distance is ", x_dis)
        #print("The y-distance is ", y_dis)

        # show the frame
        cv2.imshow("Frame", image)
        #cv2.imshow("Red Detection", red_mask)
        key = cv2.waitKey(1) & 0xFF
        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)
        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
                break
        
        # use robotic arm to grap cup
        if x_dis <= 10:
            turn_left()
            print("x_dis < 0")
        elif x_dis >= 20:
            turn_right()
            print("x_dis > 30")
        elif x_dis > 10 and x_dis < 20:
            print("0 < x_dis < 30")
            #check distance
            dis = checkdist()
            print(int(dis))
            if dis >= 9.8:
                go_ahead()
            elif dis <= 9.2:
                back_off()
            elif dis > 9.2 and dis < 9.8:
                print("x_dis and checkdis are both correct")
                if condition == 0:
                    cup_grab()
                    condition = condition + 1
                    time.sleep(1)
                elif condition == 1:
                    cup_drop()
                    condition = condition - 1
                    time.sleep(1)
                else:
                    print("condition error")
            else:
                print("checkdis error")
        else:
            print("x_dis error")
         
camera.close()
time.sleep(1.5)
car_stop()
clean_gpio()
cv2.destroyWindow("Frame")
#cv2.destroyWindow("Red Detection")

