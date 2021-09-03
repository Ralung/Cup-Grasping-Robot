# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np

from adafruit_servokit import ServoKit
kit     = ServoKit(channels=16)

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
# allow the camera to warmup
time.sleep(0.1)

# define the range of red
lower_red_0 = np.array([0, 70, 0]) 
upper_red_0 = np.array([5, 255, 255])
lower_red_1 = np.array([175, 70, 0]) 
upper_red_1 = np.array([180, 255, 255])
avg_x = []
avg_y = []
conter = 0
cX = 0
cY = 0
index = -1
maxm = 0

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
        
        # find and draw the center for every red contours
        for c in Red_cnts:
                M  = cv2.moments(c)
                if M["m00"]!=0.0 :
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                        cv2.circle(image, (cX, cY), 3, (255,0,0), -1)
                        cv2.circle(red_mask, (cX, cY), 3, (255,0,0), -1)
                        conter = conter + 1
                else:
                        print("Failed to draw a center")     
        
        # measure the distance of x-axis                
        x_dis = cY-320
        print("The x-distance is ", x_dis)        
        # show the frame
        cv2.imshow("Frame", image)
        #cv2.imshow("Red Detection", red_mask)
        key = cv2.waitKey(1) & 0xFF
        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)
        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
                break

camera.close()
time.sleep(1.5)
cv2.destroyWindow("Frame")
#cv2.destroyWindow("Red Detection")
print(conter)
