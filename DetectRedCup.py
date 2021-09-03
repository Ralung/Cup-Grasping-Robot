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
camera.framerate = 2
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
                # print("cnt = ", cnt)
                # choose the contour which area is more than 1000
                if area > 1000:
                        # find and draw the center
                        M= cv2.moments(cnts)
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                        cv2.circle(image, (cX, cY), 3, (255,0,0), -1)
                        cv2.circle(red_mask, (cX, cY), 3, (255,0,0), -1)
						# conter is used to check how many time the loop runs
                        conter = conter + 1
                else:
                        pass
                
        # measure the distance of x-axis and y-axis              
        x_dis = cX-320
        y_dis = cY-240
        print("The x-distance is ", x_dis)
        print("The y-distance is ", y_dis)
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
