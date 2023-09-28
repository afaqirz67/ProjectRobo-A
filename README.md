# ProjectRobo-A
## Introductory 
As the name of this project suggests, this robot has 6 legs, each function with 3 servos for movement. This requires us 18 servos. The types of servos used for this robot are 
MG996R Servo Motor. The MG996R is essentially an upgraded version of the famous MG995 servo, and features upgraded shock-proofing and a redesigned PCB and IC control system that 
make it much more accurate than its predecessor. With that being said, this servo also has a tail and a head. This makes it look much like a real ant. The head is capable of seeing 
with the help of an ultrasonic sensor placed in the head. It's also capable of moving up and down while moving left and right too. Moreover, with those movements, the head will be 
able to rotate itself as well. That adds up to 3 more servos. And one final servo is needed for the tail.  That brings up all the servos to a total of 23 servos. However, one 
of these servos for the head rotation unlike the other servos, is an SG90 Micro Servo Motor. The brain for this robot is a Mega Arduino. The Mega Arduino would be the only one that 
would be able to support this many servos. For powering this robot, a 3S LiPo Battery. This battery has a voltage of around 12v. It's capable of powering all the servos even if 
they were to run all at the same time. However, the servo operating voltage is limited from 4.8 to 7.2V, which means that we need to use a DC-DC buck converter to convert the 12V 
to 5V. The buck converter used for this project can handle up to 8 amps of current. For the control of this robot. I will be using an app to connect with a Wi-Fi shield. 
This allows the robot to be controlled via an IOS device, but just to back up this idea, having a Bluetooth Module isn't a bad idea. However, the Bluetooth Module would be connected 
to an Android device. If not one, the other will work for sure. But our main goal here is to make it work with an IOS device. Last but not least, the body parts are going to be 
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

Deadlines for this project will be in two phases each serving different parts of the project, but contain many shorter and more specific deadlines within. One would be building the robot and the details, and the other would be coding and testing. All the deadlines are referred to in the timeline below.

Designing the parts, 3D printing, and assembling them.


![alt text](https://github.com/afaqirz67/ProjectRobo-A/blob/main/images/Project%20Robo-A%20deadlines.png?raw=true)


## Change in plan
After some time and thinking about the desired output and the required input, I have come to the idea that my planning for the robot's interior design should be changed. The 
change is caused by the fact that using a Bluetooth shield in my design would require an Android device and app to control and send signals to the robot's receiver. I wanted 
my robot to be able to communicate with IOS devices and apps. To solve this issue, I thought of using a wifi shield and being able to control the robot with an IOS device, but it
seemed that controlling the robot with a wifi would have slowed the process of sending and receiving signals between the robot and the controller. For this issue to be
solved, I chose to use an ESP32 microcontroller rather than a Wi-Fi shield. I will be using Bluetooth for communication, which will provide the communication rate at a 
reasonable speed. Moreover, I will still be able to use an IOS device and app to control the robot. Obviously, this change in design will affect the other parts too. The ESP32
microcontroller will be replacing the Arduino Mega board, but there would be another issue caused by that. The ESP32 won't be able to replace the Mega board by itself because it 
doesn't have the capability of controlling 22 servos. In order to solve this issue, I will be using servo drivers. The specific board would be the Adafruit 16-Channel 12-bit
PWM/Servo Driver which has the capability of running 16 servos, so I'll be using two of them. A buck converter would be needed to convert the power to 3.3 for the ESP32
board. The servos will be powered by both the ESP board and the servo drivers which each receive 6v of power. The last change would be the battery. For this design, I would be
using 4 double A batteries to power up the servos and the boards. Moreover, the customization of the boards will cause a change in the robots' interior part design, so that the 
new boards can fit in it easily. A battery holder will be placed in the bottom part of the body so that replacing the batteries won't require any extra work. The flowchart
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


## Pictures of final CAD design
The design of particular parts, especially the interior, had to be changed due to the changes of microcontrollers inside. The space inside was modified to be placed in the AA 
battery pack, servo drivers, ESP-32, and the breadboard accordingly.

![Robo-A full ](https://user-images.githubusercontent.com/56890879/135955293-255680c7-2bed-497a-92ce-4f4a4916a33f.png)

![roboA](https://user-images.githubusercontent.com/56890879/135955348-a9b5378b-156d-45b3-9b89-4e940c9589e2.png)

## Change in plan
We need to switch to more powerful batteries because the AA batteries could not supply enough current to keep the servos running. Instead, we are going to use a 7.4v 
lipo battery with 5200 mah of current & a discharge rate of 50C. This battery will be able to meet the demands of the servos. The thing we had accounted for last time 
was that the servos draw current even when they are not running, but that wasn't the issue as it was drawing only 7mA. The issue was when the servos would get close
to their stall current & it would be reasonable to think that because the robot is heavy. The stall current of the servos we are using is 1100mA. Multiply that to 22 
servos and it would drastically exceed the amount of current the AA batteries can supply at once. Even if 3 legs are running at a time - the rest of the legs are still 
drawing current since they are under load. Therefore we are using a lipo battery with a higher ampere and higher discharge rate so that it meets the needs. 

Since we'll be using a 7.4v battery and we only need 5-6 volts of power, we'll be using a 6-40V to 1.2-36V 20A voltage regulator, so the battery only supplies the 
the exact amount of power needed.

[Battery](https://www.amazon.com/POVWAY-Battery-5200mAh-Trucks-Vehicles/dp/B08RYNT234/ref=sr_1_26?crid=24WYA8KKDSMCX&keywords=2s+lipo&qid=1648590463&sprefix=2s+lipo%2Caps%2C87&sr=8-26)

[Voltage Regulator](https://www.amazon.com/Anmbest-Converter-Adjustable-Regulator-Protection/dp/B07R832BRX/ref=sr_1_2?crid=SE93U8ZM74VY&keywords=24+amp+5v+regulator&qid=1648590895&sprefix=24+amps+5v+regulator%2Caps%2C104&sr=8-2)
## Code
Still in progress...

```C
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define BLYNK_PRINT Serial
#define BLYNK_USE_DIRECT_CONNECT
#include <BlynkSimpleEsp32_BLE.h>
#include <BLEDevice.h>
#include <BLEServer.h>





#define ANIM_LEN 3
#define SERVO_LEN 3


struct AnimationFrame {
  // Servo degrees
  float servos[SERVO_LEN];
};

// Transition between frames in degrees/ms
struct AnimationTransition {
  float servos[SERVO_LEN];
};

struct Animation {
  struct AnimationFrame frames[ANIM_LEN];
  int transitionTimes[ANIM_LEN];
  struct AnimationTransition transitions[ANIM_LEN];
};

void computeTransitions(struct Animation *animation) {
  for(int current = 0; current < ANIM_LEN; ++current) {
    int next = (current + 1) % ANIM_LEN;

    struct AnimationFrame currentFrame = animation->frames[current];
    struct AnimationFrame nextFrame = animation->frames[next];
    int transitionMs = animation->transitionTimes[current];

    struct AnimationTransition transition = {0,0,0};

    for(int servo = 0; servo < SERVO_LEN; ++servo) {
      transition.servos[servo] = ((float)nextFrame.servos[servo] - (float)currentFrame.servos[servo]) / (float)transitionMs;
    }

    animation->transitions[current] = transition;
  }
}

struct AnimationFrame computeState(struct Animation animation, int *ctr) {
  int total = 0;
  for(int i = 0; i < ANIM_LEN; i++) {
    total += animation.transitionTimes[i];
  }

  *ctr %= total;

  int tmpCtr = *ctr;

  int state;
  for(state = 0; state < ANIM_LEN; state++) {
    int thisTime = animation.transitionTimes[state];
    if(tmpCtr < thisTime) {
      break;
    }
    tmpCtr -= thisTime;
  }
  Serial.println(state);
  // state -> last keyframe/transition
  struct AnimationFrame lastKey = animation.frames[state];
  struct AnimationTransition trans = animation.transitions[state];

  // tmpCtr -> offset from the start of the state

  struct AnimationFrame outState;

  for(int s = 0; s < SERVO_LEN; s++) {
    outState.servos[s] = lastKey.servos[s] + trans.servos[s] * tmpCtr;
  }

  return outState;
}

struct Leg {
  // servo indices
  int servos[SERVO_LEN];

  struct Animation animation;

  int state;
  int counter;
};

struct Leg leg1 = {
  // Servo indices
  {0, 1, 2},
  // Animation
  {
    // Frames
    {
      {10, 45, 45},
      {55, 90, 90},
      {10, 45, 135},
    },
    // Timing
      {2000000, 2000000, 4000000}, 
  },
  // state
  0,
  // counter
  0,
};





// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "pcL11CQSj4YmNVoMPNCXqoMQbgf7mECT";


// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);
void findServo(int n, Adafruit_PWMServoDriver *pwm, int *i) {
  if (n < 16) {
    *pwm = pwm1;
    *i = n;
  } else {
    *pwm = pwm2;
    *i = n - 16;
  }
}

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
#define DEGREESMIN 0
#define DEGREESMAX 180 

uint8_t forwards = 0;
uint8_t backwards = 0;
uint8_t right = 0;
uint8_t left = 0;
uint8_t hup_down = 0;
uint8_t hleft_right = 0;
uint8_t p = 0;
uint8_t tail = 0;

// our servo # counter
uint8_t servonum = 0;

void setAngle(
  int servoNum,
  uint8_t degrees
) {
  Adafruit_PWMServoDriver p;
  int n;
  findServo(servoNum, &p, &n);
  uint16_t pulselength = map(degrees, 0, 180, SERVOMIN, SERVOMAX);
  p.setPWM(n , 0, pulselength);
  
}


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  computeTransitions(&leg1.animation);
  // Debug console
  Serial.begin(9600);
  Serial.println("Waiting for connections...");
  Blynk.setDeviceName("Blynk");

  Blynk.begin(auth);
   pwm1.begin();
   pwm2.begin();
  /*
   * In theory the internal oscillator (clock) is 25MHz but it really isn't
   * that precise. You can 'calibrate' this by tweaking this number until
   * you get the PWM update frequency you're expecting!
   * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
   * is used for calculating things like writeMicroseconds()
   * Analog servos run at ~50 Hz updates, It is importaint to use an
   * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
   * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
   *    the I2C PCA9685 chip you are setting the value for.
   * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
   *    expected value (50Hz for most ESCs)
   * Setting the value here is specific to each individual I2C PCA9685 chip and
   * affects the calculations for the PWM update frequency. 
   * Failure to correctly set the int.osc value will cause unexpected PWM results
   */
  pwm1.setOscillatorFrequency(27000000);
  pwm1.setPWMFreq(SERVO_FREQ);
  pwm2.setOscillatorFrequency(27000000);
  pwm2.setPWMFreq(SERVO_FREQ);// Analog servos run at ~50 Hz updates

  delay(10);
  //leg1.pwm = pwm;
  //leg1.index = 0;
  //resetLeg(&leg1);
}

BLYNK_WRITE(V1){
forwards = param.asInt();

}

BLYNK_WRITE(V2){
backwards = param.asInt();
}

BLYNK_WRITE(V3){
p = param.asInt();

}
 

// You can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. It's not precise!
//void setServoPulse(uint8_t n, double pulse) {
//  double pulselength;
  
  //pulselength = 1000000;   // 1,000,000 us per second
  //pulselength /= SERVO_FREQ;   // Analog servos run at ~60 Hz updates
  //Serial.print(pulselength); Serial.println(" us per period"); 
  //pulselength /= 4096;  // 12 bits of resolution
  //Serial.print(pulselength); Serial.println(" us per bit"); 
  //pulse *= 1000000;  // convert input seconds to us
  //pulse /= pulselength;
  //Serial.println(pulse);
  //pwm.setPWM(n, 0, pulse);
//}
  double degrees = 90;
  bool direction = false;
  
unsigned long now = 0;
unsigned long last = 0;




void outputLeg(struct Leg *leg) {
  struct AnimationFrame state = computeState(leg->animation, &leg->counter);
  for(int s = 0; s < SERVO_LEN; s++) {
      setAngle(leg->servos[s], (uint16_t)state.servos[s]);
  }
}




void loop() {
if (last == 0){
  last = micros();
  
}else {last = now;}
 now = micros();
 
int change = now - last; 
leg1.counter+=change; outputLeg(&leg1);
   
}
```



### UPDATED CODE
```c
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define BLYNK_PRINT Serial
#define BLYNK_USE_DIRECT_CONNECT
#include <BlynkSimpleEsp32_BLE.h>
#include <BLEDevice.h>
#include <BLEServer.h>





#define ANIM_LEN 3                                    
#define SERVO_LEN 3


struct AnimationFrame {
  // Servo degrees
  float servos[SERVO_LEN];
};

// Transition between frames in degrees / ms
struct AnimationTransition {
  float servos[SERVO_LEN];
};

struct Animation {
  struct AnimationFrame frames[ANIM_LEN];
  int transitionTimes[ANIM_LEN];
  struct AnimationTransition transitions[ANIM_LEN];
};

void computeTransitions(struct Animation *animation) {
  for(int current = 0; current < ANIM_LEN; ++current) {
    int next = (current + 1) % ANIM_LEN;

    struct AnimationFrame currentFrame = animation->frames[current];
    struct AnimationFrame nextFrame = animation->frames[next];
    int transitionMs = animation->transitionTimes[current];

    struct AnimationTransition transition = {0,0,0};

    for(int servo = 0; servo < SERVO_LEN; ++servo) {
      transition.servos[servo] = ((float)nextFrame.servos[servo] - (float)currentFrame.servos[servo]) / (float)transitionMs;
    }

    animation->transitions[current] = transition;
  }
}

struct AnimationFrame computeState(struct Animation animation, int *ctr) {
  int total = 0;
  for(int i = 0; i < ANIM_LEN; i++) {
    total += animation.transitionTimes[i];
  }

  *ctr %= total;

  int tmpCtr = *ctr;

  int state;
  for(state = 0; state < ANIM_LEN; state++) {
    int thisTime = animation.transitionTimes[state];
    if(tmpCtr < thisTime) {
      break;
    }
    tmpCtr -= thisTime;
  }
  Serial.println(state);
  // state -> last keyframe/transition
  struct AnimationFrame lastKey = animation.frames[state];
  struct AnimationTransition trans = animation.transitions[state];

  // tmpCtr -> offset from the start of the state

  struct AnimationFrame outState;

  for(int s = 0; s < SERVO_LEN; s++) {
    outState.servos[s] = lastKey.servos[s] + trans.servos[s] * tmpCtr;
  }

  return outState;
}

struct Leg {
  // servo indices
  int servos[SERVO_LEN];

  struct Animation animation;

  int state;
  int counter;
};

struct Leg leg1 = {
  // Servo indices
  {16, 17, 18},
  // Animation
  {
    // Frames
    {
          {90, 90, 90},
      {90, 90, 90},
      {45, 45, 180},
    },
    // Timing
        {2000000, 2000000, 3000000}, 
  },
  // state
  0,
  // counter
  0,
};





// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "pcL11CQSj4YmNVoMPNCXqoMQbgf7mECT";


// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);
void findServo(int n, Adafruit_PWMServoDriver *pwm, int *i) {
  if (n < 16) {
    *pwm = pwm1;
    *i = n;
  } else {
    *pwm = pwm2;
    *i = n - 16;
  }
}

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
#define DEGREESMIN 0
#define DEGREESMAX 180 

uint8_t forwards = 0;
uint8_t backwards = 0;
uint8_t right = 0;
uint8_t left = 0;
uint8_t hup_down = 0;
uint8_t hleft_right = 0;
uint8_t p = 0;
uint8_t tail = 0;

// our servo # counter
uint8_t servonum = 0;

void setAngle(
  int servoNum,
  uint8_t degrees
) {
  Adafruit_PWMServoDriver p;
  int n;
  findServo(servoNum, &p, &n);
  uint16_t pulselength = map(degrees, 0, 180, SERVOMIN, SERVOMAX);
  p.setPWM(n , 0, pulselength);
  
}


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  computeTransitions(&leg1.animation);
  // Debug console
  Serial.begin(9600);
  Serial.println("Waiting for connections...");
  Blynk.setDeviceName("Blynk");

  Blynk.begin(auth);
   pwm1.begin();
   pwm2.begin();
  /*
   * In theory the internal oscillator (clock) is 25MHz but it really isn't
   * that precise. You can 'calibrate' this by tweaking this number until
   * you get the PWM update frequency you're expecting!
   * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
   * is used for calculating things like writeMicroseconds()
   * Analog servos run at ~50 Hz updates, It is importaint to use an
   * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
   * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
   *    the I2C PCA9685 chip you are setting the value for.
   * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
   *    expected value (50Hz for most ESCs)
   * Setting the value here is specific to each individual I2C PCA9685 chip and
   * affects the calculations for the PWM update frequency. 
   * Failure to correctly set the int.osc value will cause unexpected PWM results
   */
  pwm1.setOscillatorFrequency(27000000);
  pwm1.setPWMFreq(SERVO_FREQ);
  pwm2.setOscillatorFrequency(27000000);
  pwm2.setPWMFreq(SERVO_FREQ);// Analog servos run at ~50 Hz updates

  delay(10);
  //leg1.pwm = pwm;
  //leg1.index = 0;
  //resetLeg(&leg1);
}

BLYNK_WRITE(V1){
forwards = param.asInt();

}

BLYNK_WRITE(V2){
backwards = param.asInt();
}

BLYNK_WRITE(V3){
p = param.asInt();

}
 

// You can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. It's not precise!
//void setServoPulse(uint8_t n, double pulse) {
//  double pulselength;
  
  //pulselength = 1000000;   // 1,000,000 us per second
  //pulselength /= SERVO_FREQ;   // Analog servos run at ~60 Hz updates
  //Serial.print(pulselength); Serial.println(" us per period"); 
  //pulselength /= 4096;  // 12 bits of resolution
  //Serial.print(pulselength); Serial.println(" us per bit"); 
  //pulse *= 1000000;  // convert input seconds to us
  //pulse /= pulselength;
  //Serial.println(pulse);
  //pwm.setPWM(n, 0, pulse);
//}
  double degrees = 90;
  bool direction = false;
  
unsigned long now = 0;
unsigned long last = 0;




void outputLeg(struct Leg *leg) {
  struct AnimationFrame state = computeState(leg->animation, &leg->counter);
  for(int s = 0; s < SERVO_LEN; s++) {
      setAngle(leg->servos[s], (uint16_t)state.servos[s]);
  }
}




void loop() {
if (last == 0){
  last = micros();
  
}else {last = now;}
 now = micros();
 
int change = now - last; 
leg1.counter+=change; outputLeg(&leg1);
   
}
```
