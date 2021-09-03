# Cup-Grasping-Robot  
In this project, there is a robot with a Raspberry pi 4, a picamera, a PCA9685 chip, an ultrasonic sensor and two servo motors was built to compete a automatic cup grasping mission.  
Content  
1. Servo motors  
2. Picamera installation  
3. Image processing with OpenCV & Python  
4. Cup Grasping Robot  

### Servo motors  
PCA9685 is used to control servo moters in this project and servo motors are used to build a robotic arm and robotic paw.  
PCA9685: PCA9685 is a driver control module chip, a 16-channel controller with I2C bus interface. The PWM controller of PCA9685 can control the servo motor more accurately than the Raspberry Pi.  
![image](https://user-images.githubusercontent.com/76200428/131968645-332bd432-6920-4b39-9c1b-b48336ce82e6.png)
![image](https://user-images.githubusercontent.com/76200428/131968674-e7d33386-63d8-41c2-be0d-a3808b1c9da0.png)

The two servo motors are connected with one PCA9685 and they could be controled by different channels.  
In "Servo_test.py", channel 8 and channel 10 control different servo moters and the value of each channel represent the degree of servo motor.  
Note: It is needed to give the external power to the servo motors. As a result, there are totally two battery on this robot, one for raspberry pi and the other for servo motors  
![image](https://user-images.githubusercontent.com/76200428/131970315-95c0115a-6723-494e-9ad3-39c973d94a58.png)

### Picamera installation
Step 1: Go to the raspberry pi configuration and enable the camera.
Step 2: Connect the camera with the raspberry pi and confirm the Picamera with this command $vcgencmd get_camera. If the result is like the following picture, it works.  
![image](https://user-images.githubusercontent.com/76200428/131970940-1e6c2d3e-8612-4d1d-a5fa-7e3ebef88330.png)
![image](https://user-images.githubusercontent.com/76200428/131970958-9a1ae91c-b8a3-45ab-9c95-b1ba8b663247.png)

### Image processing with OpenCV & Python
In this project, the target is a cup with red color and there are some steps neededs to be done.  
Step 1: Transmit the frames in real time.  
In this step, "test_video.py" is used to ensure turn on the Picamera, transmit the frames in real time and set various parameters of the frames.  
Transmitting the frames in real time is the pivotal factor. When the frames can be transmitted in real time, the comming feature extraction function can be successed.  
Step 2: Color detection  
In this step, "ColorDetect_Red.py" would process the image and find out where the red color parts are.  
Depends on the different environment, the paremeters of HSV values need to be modified.  
Step 3: Contours detection  
Finding out the contours of red objects give a foundation of comming step which allow the robot can filter out the target cup from red objects and the central points for distance detection.  
