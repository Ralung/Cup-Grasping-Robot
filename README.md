# Cup-Grasping-Robot  
In this project, there is a robot with a Raspberry pi 4, a picamera, a PCA9685 chip, an ultrasonic sensor and two servo motors was built to compete a automatic cup grasping mission with high success rate.  
Content  
1. Servo motors  
2. Picamera installation  
3. Image processing with OpenCV & Python  
4. Cup Grasping Robot  

### 1. Servo motors  
PCA9685 is used to control servo moters in this project and servo motors are used to build a robotic arm and robotic paw.  
PCA9685: PCA9685 is a driver control module chip, a 16-channel controller with I2C bus interface. The PWM controller of PCA9685 can control the servo motor more accurately than the Raspberry Pi.  
![image](https://user-images.githubusercontent.com/76200428/131968645-332bd432-6920-4b39-9c1b-b48336ce82e6.png)
![image](https://user-images.githubusercontent.com/76200428/131968674-e7d33386-63d8-41c2-be0d-a3808b1c9da0.png)

The two servo motors are connected with one PCA9685 and they could be controled by different channels.  
In "Servo_test.py", channel 8 and channel 10 control different servo moters and the value of each channel represent the degree of servo motor.  
Note: It is needed to give the external power to the servo motors. As a result, there are totally two battery on this robot, one for raspberry pi and the other for servo motors  
![image](https://user-images.githubusercontent.com/76200428/131970315-95c0115a-6723-494e-9ad3-39c973d94a58.png)

### 2. Picamera installation
##### Step 1: Go to the raspberry pi configuration and enable the camera.  
![image](https://user-images.githubusercontent.com/76200428/131970940-1e6c2d3e-8612-4d1d-a5fa-7e3ebef88330.png)
##### Step 2: Connect the camera with the raspberry pi and confirm the Picamera with this command $vcgencmd get_camera. If the result is like the following picture, it works.  
![image](https://user-images.githubusercontent.com/76200428/131970958-9a1ae91c-b8a3-45ab-9c95-b1ba8b663247.png)

### 3. Image processing with OpenCV & Python
In this project, the target is a cup with red color and there are some steps neededs to be done.  
##### Step 1: Transmit the frames in real time.  
In this step, "test_video.py" is used to ensure turn on the Picamera, transmit the frames in real time and set various parameters of the frames.  
Transmitting the frames in real time is the pivotal factor. When the frames can be transmitted in real time, the comming feature extraction function can be successed.  
##### Step 2: Color detection  
In this step, "ColorDetect_Red.py" would process the image and find out where the red color parts are.  
Depends on the different environment, the paremeters of HSV values need to be modified.  
##### Step 3: Contours detection  
Finding out the contours of red objects give a foundation of comming step which allow the robot can filter out the target cup from red objects and the central points for distance detection.  
Please check "Red_contours.py".  
##### Step 4: Center points of the window and the mass of contours  
The blue points represent the center point of red objects and the green point represents the center point of the window.  
![image](https://user-images.githubusercontent.com/76200428/131975242-720754fe-879f-4b41-95cf-8a675f90f736.png)
Please check "DrawRedCenter.py".  
##### Step 5: Filter out the red contours whose size are too small
In this step, all the contours of red object are be drawn but only the biggest red contour would be drawn on its center point.  
The biggest red contour is the target cup of this project.  
![image](https://user-images.githubusercontent.com/76200428/131976258-610ef61c-de3f-43e2-bc52-29e6c6e4539a.png)
Please check "DetectRedCup.py".  

### 4. Cup Grasping Robot
After the setting of Picamera, servo motors and feature extraction, it is needed to integrate them together to complete the robot.  
The ultrasonic sensor also needs to be set to detect the distance between the robot and object in front of it.  
The following diagram is the logic flow chart of this project.  
![image](https://user-images.githubusercontent.com/76200428/131976923-39ae1bd9-8ffb-4a7a-b3b0-a5e93f6b7d7d.png)
Please check "Cup_Grasping_Robot".  

### Demostration
![image](https://user-images.githubusercontent.com/76200428/131977283-2d5b1e7c-6928-4d06-a7c9-9cee4c2ccb1c.png)
![image](https://user-images.githubusercontent.com/76200428/131977306-9bebd5e3-e1da-4bcd-ab6d-d2abf22ad6e9.png)
![image](https://user-images.githubusercontent.com/76200428/131977326-abe73a0e-e38d-4c39-b148-3299ba2259d5.png)
![image](https://user-images.githubusercontent.com/76200428/131977342-347fdb34-cc4d-458e-8771-6ed903eeab23.png)

