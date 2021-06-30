# ProjectRobo-A
## Introductory 
As the name for this project suggest, this robot has 6 legs, each function with 3 servos for movement. This requires us 18 servos. The types of servos used for this robot is 
MG996R Servo Motor. The MG996R is essentially an upgraded version of the famous MG995 servo, and features upgraded shock-proofing and a redesigned PCB and IC control system that 
make it much more accurate than its predecessor. With that being said, this servo also has tail, and a head. This makes it look much like a real ant. The head is capable of seeing 
with help of an ultrasonic sensor placed in the head. It's also capable of moving up and down, while moving left and right too. Moreover, with those movements, the head will be 
able to rotate itself as well. That adds up 3 more servos. And one final servo is needed for the tail.  That brings up all the servos to a total of 23 servos. Though, the one 
of these servos for the head rotation unlike the other servos, is a SG90 Micro Servo Motor. The brain for this robot is a Mega Arduino. The Mega Arduino would be the only one that 
would be able to support this many servos. For powering this robot, a 3S LiPo Battery. This battery has a voltage of around 12v. It's capable of powering all the servos even if 
they were to run all at the same time. However, the servos operating voltage is limited from 4.8 to 7.2V, which means that we need to use a DC-DC buck converter to convert the 12V 
to 5V. The buck converter that is used for this project can handle up to 8 amps of current. For the control of this robot. I will be using an app to connect with a Wifi Shield. 
This gives the robot to be control via an IOS device, but just to back up this idea, having a Bluetooth Module isn't a bad idea. Although, the Bluetooth Module would be connected 
with an Android device. If not one, the other will work for sure. But our main goal here is to make it work with an IOS device. Last but not least, the body parts are going to be 
3D printed.





![alt text](https://github.com/afaqirz67/ProjectRobo-A/blob/main/images/Arduino-Ant-Robot-3D-Model-768x432.jpg?raw=true)


![alt text](https://github.com/afaqirz67/ProjectRobo-A/blob/main/images/Hexapod-3D-Model-768x432.jpg?raw=true)

![alt text](https://github.com/afaqirz67/ProjectRobo-A/blob/main/images/Arduino-Hexapod-Ant-Robot-Circuit-Diagram.png?raw=true)

Visuals from:
Dejan, et al. “Arduino Ant Hexapod Robot.” HowToMechatronics, 7 Jan. 2021, howtomechatronics.com/projects/arduino-ant-hexapod-robot/. 

 ![Tux, the Linux mascot](https://github.com/afaqirz67/ProjectRobo-A/blob/main/images/pd1.jpg?raw=true)

![alt text](https://github.com/afaqirz67/ProjectRobo-A/blob/main/images/pdraw.jpg?raw=true)

## Components needed for this project:

MG996R Servo Motor X 21 

SG90 Micro Servo Motor 

Arduino Mega Board 

3S LiPo Battery 

DC-DC Buck Converter 

HC-05 Bluetooth Module  

## Deadlines

Deadlines for this project will be in two phases each serving different parts of the project, but contain many shorter and more specific deadlines within. One would be building the robot and the parts, and the other would be coding and testing. All the deadlines are referred to in the timeline bellow.

Designing the parts, 3d printing, and assembling them.


![alt text](https://github.com/afaqirz67/ProjectRobo-A/blob/main/images/Project%20Robo-A%20deadlines.png?raw=true)


## Change in plan
After some time and thinking about the desired output and the required input, I have come to the idea that my planning for the robot's interior design should be changed. The 
change is caused by the fact that using a bluetooth shield in my design would require an Android device and app to control the and send signals to the robot's reciever. I wanted 
my robot to be able to communicate with IOS devices and apps. To solve this issue, I thought of using a wifi shield and be able to control the robot with an IOS device, but it
seemed that controlling the robot with a wifi would have slowed than the process of sending and recieving signals between the robot and the controller. For this issue to be
solved, I chose to use an ESP32 microcontroller rather than the a wifi shield. I will be using using bluetooth for communication, which will provide the communication rate a 
reasonable speed. Moreover, I will still be able to use an IOS device and app to control the robot. Obviously this change in design will effect the other parts too. The ESP32
microcontroller will be replacing the Arduino Mega board, but there would be another issue caused by that. The ESP32 won't be able to replace the Mega board by itself because it 
doesn't have the capability of controlling 22 servos. In order to solve this issue, I will be using servo drivers. The specific board would be the Adafruit 16-Channel 12-bit
PWM/Servo Driver which has the capability of running 16 servos, so I'll be using two of them. A buck converter would be needed to convert the power to 3.3 for the ESP32
board. And the servos will be powerd by both the ESP board and the servo drivers which each recieve 6v of power. The last change would be the battery. For this design I would be
using 4 double A batteries to power up the servos and the boards. Moreover, the customization of the boards will cause a change in the robots inerior part design, so that the 
new boards can fit in it easily. A battery holder will be placed in the bottom part of the body, so that replacing the battries wouldn't require any extra work. The flowchart
below shows a better picture of what this design looks like. 


![alt text](https://github.com/afaqirz67/ProjectRobo-A/blob/main/images/flow-chart-planning.png?raw=true)

## New list of needed components
[Servo Driver x2](https://www.adafruit.com/product/815)

[Esp32 microcontroller](https://www.adafruit.com/product/4769)

MG996R Servo Motor X 21

SG90 Micro Servo Motor

[3.3v Voltage Regulator](https://www.adafruit.com/product/4683) 

Double a battery mount

Double A battries  x 4+ 

Flathead M3 bolts 12 or 14mm length x 200

M3 lock nuts x 200

M4 bolts and nuts and washers x 15

[Logic Level Converter](https://www.sparkfun.com/products/12009)
