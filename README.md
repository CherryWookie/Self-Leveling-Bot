# Lab8 - Final Project (Self-Leveling Robot)
Michael Sell (Lab partner: Ben Schaser)

## Table of Contents:

[`Resources`](#resources)

[`Materials`](#materials)

[`Objective`](#objectiveproposal)

[`3D Printing/Laser Cutting`](#3d-printinglaser-cutting)

[`Robot Design`](#robot-design)

[`Robot Assembly`](#robot-assembly)

[`Code`](#code)

* [PID Control](#pid-control)

[`Troubleshooting`](#troubleshooting)

[`Performance`](#performance)


## Resources
#
* [Self-Levling Bot Instructional](https://www.instructables.com/Arduino-Self-Balancing-Robot-1/)
* [Self-Leveling Bot w/ Video](https://electricdiylab.com/diy-self-balancing-robot/)
* [MotorShield Instruction](https://learn.adafruit.com/adafruit-motor-shield-v2-for-arduino/using-stepper-motors)
* [Potential Wheel Option](https://www.amazon.com/90-10mm-Black-Robot-Wheels/dp/B00T3MQG7M/ref=sr_1_33?crid=D4G5H5ZWXL8G&keywords=Plastic+Robotic+Wheel+Rubber+Tire+Wheel+100m&qid=1667595648&qu=eyJxc2MiOiIwLjk3IiwicXNhIjoiMC4wMCIsInFzcCI6IjAuMDAifQ%3D%3D&sprefix=plastic+robotic+wheel+rubber+tire+wheel+100m%2Caps%2C231&sr=8-33)
* [Accelstepper](https://github.com/adafruit/AccelStepper/blob/master/AccelStepper.cpp)
* [DC Motor that would probably work](https://www.studica.com/612rpm-6-12v-planetary-gearmotor)
* [Adafruit Motorshield Libray V_2](https://github.com/adafruit/Adafruit_Motor_Shield_V2_Library)


## Materials
#
* `Arduino UNO`

* `2 Stepper Motors (42mm High Torque Hybrid Stepping Motor)`
    * [LINK TO WEBPAGE](https://www.adafruit.com/product/324)
    * [DataSheet][def]

[def]: motordatasheet.jpeg

* `Adafruit Motorshield Kit`
    * [LINK TO WEBPAGE](https://www.adafruit.com/product/1438)
    * <a href="https://cdn-shop.adafruit.com/datasheets/TB6612FNG_datasheet_en_20121101.pdf">View DataSheet</a> 
    
* `Gyroscope Module`
    * [LINK TO WEBPAGE](https://www.amazon.com/HiLetgo-MPU-6050-Accelerometer-Gyroscope-Converter/dp/B00LP25V1A/ref=sr_1_1?crid=32FG5TYBEF8PB&keywords=MPU-6050%2BModule%2B3%2BAxis%2BGyroscope%2B%2B%2B3%2BAxis%2BAccelerometer%2FG-Sensor%2C%2BI2%2B%C2%B0C%2C%2Be.g.%2Bgenuino%2Bfor%2BArduino%2C%2BRaspberry%2BPi&qid=1664679029&qu=eyJxc2MiOiIwLjcyIiwicXNhIjoiMC4wMCIsInFzcCI6IjAuMDAifQ%3D%3D&sprefix=mpu-6050%2Bmodule%2B3%2Baxis%2Bgyroscope%2B%2B%2B3%2Baxis%2Baccelerometer%2Fg-sensor%2C%2Bi2%2Bc%2C%2Be.g.%2Bgenuino%2Bfor%2Barduino%2C%2Braspberry%2Bpi%2Caps%2C68&sr=8-1&th=1)
    * Main Chip: MPU-6050
    * Power supply: 3~5V
    * Communication mode: standard IIC communication protocol
    * Chip built-in 16bit AD converter, 16bit data output
    * Gyroscopes range: +/- 250 500 1000 2000 degree/sec
    * Acceleration range: ±2 ±4 ±8 ±16g 

* `Batteries`
    * In our case: 8 AA 1.5V rechargeable batteries (TENERGY Premium 2500mAh Ni-MH Rechargeable)

* `Breadboard/wires`

* `Wheels/Tires`
    * Anything around 10cm diameter wheels should suffice. Preferably a good amount of grip for the the bot to maintain traction with various surfaces. In our case, we will be 3D Printing the wheel rims and using rubber tape to wrap around the rims for our "tires".
    * [STL FILE](wheel-rims.stl)

* `Hardware`
    * Various screws, nuts, etc. that we have yet to determine
    * Brackets for stepper motor installation
    * 4 support beams for structural integrity

* `Structure`
    * Platforms for UNO, battery, and anything else to rest on while offering the bot support. (3 in total, in our case, we will be cutting them out of a 1/4" piece of wood with a laser cutter)


## Objective/Proposal
#
Create a robot using an Arduino Uno and the above listed materials that is able to autonomously level itself without any human interaction after the bot is turned on.

`Steps to Completion:` We will be designing the robot similar to other bots listed under the [resources](#resources) section. We will be using all the components listed above with miscellaneous hardware in order to accomplish this. As for the software, we will be using a PID controller in order to control our robot. This will depend on our robot structure and the exact dimensions. We will be 3D printing the wheel rims and using the laser cutter in order to cut our platform sectionals. 

## Testing Components
#

We started by testing the stepper motors and followed the online instructions on the webpage for the motor shield. We soldered the shield and the gyroscope.

We had to make sure the .h files were also in the arduino folder.

After putting the motors and the new wheels on our robot frame, we found that the motors were not making us very happy. For one, they didn't seem to be able to go fast enough. They produced very little torque and they were very jerky in motion as we have to step each one at a time instead of running them simultaneously.

## 3D Printing/Laser Cutting
#

`Laser Cut thickness for red line: .072pt, Red line`

Our first test of the 3D print for the wheel rims didn't turn out very well as it printed out vertical instead of horizontal. To make it better, we switched it so that it was lying horizontal on the base plate and switched from non-soluble support material to water-soluble material so that we would end up with a cleaner product. We ended up re-printing a third time and this time opted for some larger wheels to fit our bigger robot design. Our final dimensions for the wheels were as follows: 


We ended up completely rethinking our design as we kept running into the problem of our robot falling over too fast and the motors not having enough torque/speed to recover. We hoped that creating a chassis our of wood while making it smaller would aid in this problem. 

<img src="Photos-001/IMG_20221205_134104.jpg" alt="drawing" width="300"/>

<!-- ![image](Photos-001/IMG_20221205_134104.jpg) -->
Above is the image of our laser cut components for our new design.

## Robot Design
#

Platform Dimensions: 3.5 x 7 x 1/4" (x3) (Perhaps?)
(9cm x 18cm)
8.4 cm x 16 cm

For general robot design, we started out with a rough

Eventually, after a few days of adjusting parameters and trying different methods, we concluded the new stepper motors were still not fast enough and were not stepping in sync, causing the bot to shake violently, twist, and fall over. We then went to Dr. Kuehns laboratory and got two of his motors. They turned out to be

## Robot Assembly
#

`Note - Black M3 screws used to hold motors in brackets are Professor Brewsters! (Now the silver M3 screws*)`

## Code
#

For the software, we used the instructional resources above as a rough guide of what to do. We started by using the accelerometer along with the ```atan2(x,y)``` function in order to get an angle from the acceleration. We then gathered the data from the gyroscope. The angle from the acceleration is very consistent but has to deal with gravity and acceleration in other directions while the gyroscope module has some float or accumulated error. Both of these methods have certain drawbacks, so, in order to remedy this, we applied both through a complementary filter. This allowed us to modify our output to optimize the accuracy and stability of our output angle.
<font size = "1">

```C++
// Michael Sell and Ben Schaser
// Full code:

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MPU6050 mpu;
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_StepperMotor *leftMotor = AFMS.getStepper(600, 2);
Adafruit_StepperMotor *rightMotor = AFMS.getStepper(600, 1);

// Angle of Inclination
double accX, accZ;
double accAngle;
int gyroY, gyroRate;
double gyroAngle = 0;
double gyroAngleNext = 0;
double gyroAnglePrev = 0;
unsigned long currTime, prevTime = 0;
double loopTime;
double currentAngle = 0;
double prevAngle = 0;
double prevPrevAngle = 0;
double nextAngle = 0;
double tau = 0.7;
double dt = 0.004;
double alpha = tau / (tau + dt);

// PID Constants
double Kp = 0.0;
double Ki = 0.0;
double Kd = 0.0;

double targetAngle = 0;
double difference = 0;
double prevDif = 0;
double difSum = 0;
double motorSpeed = 0;
double RtoD = 57.2957795131;

void setup() {
  Serial.begin(9600);
  mpu.begin();

  // set accelerometer range to +-8G
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  // set gyro range to +- 500 deg/s
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  // set filter bandwidth to 21 Hz
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  //  mpu.setXAccelOffset(315);
  //  mpu.setYAccelOffset(993);
  //  mpu.setZAccelOffset(415);
  //  mpu.setXGyroOffset(93);
  //  mpu.setYGyroOffset(-23);
  //  mpu.setZGyroOffset(-11);

  AFMS.begin();
  TWBR = ((F_CPU/400000l) - 16)/2; // Change the i2c clock speed to 400KHz
}

void loop() {
  currTime = millis();
  //  Serial.println(currTime - prevTime);
  loopTime = (currTime - prevTime);
  loopTime = loopTime / 1000;
  prevTime = currTime;
  
  getCurrentAngle(loopTime);
  Serial.println(currentAngle);
  calcSpeed();
//  Serial.println(motorSpeed);
  //
//  if ((currentAngle + prevAngle + prevPrevAngle) / 3 < -1.0) {
//    leftMotor->setSpeed(motorSpeed);
//    rightMotor->setSpeed(motorSpeed);
//    leftMotor->step(1, BACKWARD, DOUBLE);
//    rightMotor->step(1, FORWARD, DOUBLE);
//  }
//  else if ((currentAngle + prevAngle + prevPrevAngle) / 3 > 1.0) {
//    leftMotor->setSpeed(motorSpeed);
//    rightMotor->setSpeed(motorSpeed);
//    leftMotor->step(1, FORWARD, DOUBLE);
//    rightMotor->step(1, BACKWARD, DOUBLE);
//  }
  prevPrevAngle = prevAngle;
  prevAngle = currentAngle;
}



void getCurrentAngle(double elapsedTime) {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  accZ = a.acceleration.z;
  accX = a.acceleration.x;
//  Serial.print(accX);
//  Serial.print("  ");
//  Serial.print(accZ);
//  Serial.print('\n');

  accAngle = atan2(accX, accZ) * RtoD;

  gyroY = g.gyro.y;

  gyroRate = map(gyroY, -32768, 32767, -250, 250);
  gyroAngle = gyroAngle + (float)gyroRate * elapsedTime;
  currentAngle = tau * (prevAngle + gyroAngle*dt) + (1 - tau) * accAngle;
//  Serial.println(accAngle);

  
//  alternative formula - leapfrog method :)
//  gyroAngleNext = (gyroAnglePrev + gyroAngle) / 2 + accAngle*elapsedTime;
//  nextAngle = currentAngle;

}
void calcSpeed() {
  difference = currentAngle - targetAngle;
  difSum += difference;
  difSum = constrain(difSum, -300, 300);
  motorSpeed = Kp * difference + Ki * difSum * loopTime + Kd * (difference - prevDif) / loopTime;
  prevDif = difference;
}
```
</font>

### PID Control
pain

## Troubleshooting
#

## Performance
#

