#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "anim.h"
#define __DEBUG
#ifdef __PROFILE
#define profile(name, thing) ({ \
  int start = micros();         \
  thing;                        \
  int end = micros();           \
  Serial.println(name);         \
  Serial.println(end - start);  \
  thing;                        \
})

#else
#define profile(name, thing) (thing)
#endif

#ifdef __DEBUG
#define debug(text, value) \
  Serial.print(text);      \
  Serial.println(value)
#else
#define debug(text, value)
#endif

#define BLYNK_PRINT Serial
#define BLYNK_USE_DIRECT_CONNECT
#include <BlynkSimpleEsp32_BLE.h>
#include <BLEDevice.h>
#include <BLEServer.h>

#define ANIM_LEN 3
#define SERVO_LEN 3

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "pcL11CQSj4YmNVoMPNCXqoMQbgf7mECT";



// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
// Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);
void findServo(int n, Adafruit_PWMServoDriver *pwm, int *i)
{
  if (n < 16)
  {
    *pwm = pwm1;
    *i = n;
  }
  else
  {
    *pwm = pwm2;
    *i = n - 16;
  }
}

// Depending on your servo make, the pulse width min and max may vary, you
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN 150  // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 600  // This is the 'maximum' pulse length count (out of 4096)
#define USMIN 600     // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX 2400    // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
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
    uint8_t degrees)

{
  debug("servo ", servoNum);
  debug("degrees ", degrees);
  Adafruit_PWMServoDriver p;
  int n;
  findServo(servoNum, &p, &n);
  uint16_t pulselength = map(degrees, 0, 180, SERVOMIN, SERVOMAX);
  p.setPWM(n, 0, pulselength);
}





void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  doLegs(initLeg);

  // Debug console
  Serial.begin(115200);

  
  
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
  pwm2.setPWMFreq(SERVO_FREQ); // Analog servos run at ~50 Hz updates

  for (int i = 0; i < 32; ++i)
  {
    Adafruit_PWMServoDriver p;
    int n;
    findServo(i, &p, &n);
    p.setPin(n, 0);
  }

  delay(10);
  // leg1.pwm = pwm;
  // leg1.index = 0;
  // resetLeg(&leg1);







  




  
}

BLYNK_WRITE(V1)
{
  forwards = param.asInt();
}

BLYNK_WRITE(V2)
{
  backwards = param.asInt();
}

BLYNK_WRITE(V3)
{
  p = param.asInt();
}

// You can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. It's not precise!
// void setServoPulse(uint8_t n, double pulse) {
//  double pulselength;

// pulselength = 1000000;   // 1,000,000 us per second
// pulselength /= SERVO_FREQ;   // Analog servos run at ~60 Hz updates
// Serial.print(pulselength); Serial.println(" us per period");
// pulselength /= 4096;  // 12 bits of resolution
// Serial.print(pulselength); Serial.println(" us per bit");
// pulse *= 1000000;  // convert input seconds to us
// pulse /= pulselength;
// Serial.println(pulse);
// pwm.setPWM(n, 0, pulse);
//}
double degrees = 90;
bool direction = false;

unsigned long now = 0;
unsigned long last = 0;

void outputLeg(int i, struct Leg *leg)
{
  leg->counter += (now - last);
  struct AnimationFrame state = computeState(leg->animation, &leg->counter);
  for (int s = 0; s < SERVO_LEN; s++)
  {
    uint16_t angle = (uint16_t)state.servos[s];
    if (leg->reflect)
    {
      angle = 180 - angle;
    }
    setAngle(leg->servos[s], angle);
  }
}



void loop()
{
  if (last == 0)
  {
    last = micros();
  }
  else
  {
    last = now;
  }
  now = micros();

  int change = now - last;
  debug("loop time", change);
  doLegs(outputLeg);
}
