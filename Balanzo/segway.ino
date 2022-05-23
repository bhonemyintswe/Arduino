

////////////////////////////////////////////////////////////////////////////////
////    Some lines of program may be changed eg. PID values, pidMode, etc   ////
////////////////////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include <avr/wdt.h>
#include <PID_v1.h>
#include <LMotorController.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define MANUAL_TUNING 1
#define MOVE_BACK_FORTH 1
String stvalue = "";    // a string to hold incoming data

MPU6050 mpu;
#include <SoftwareSerial.h>

SoftwareSerial bluetooth(0, 1);

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];

// PID values to balance the segway itself
double kp = 52.5;
double kd = 4.23;
double ki = 9.54;
double MIN_ABS_SPEED = 20;
double originalSetpoint = 185.5;
double setpoint = originalSetpoint;
double leftmotorspeed = 0.65, rightmotorspeed = 0.65;

double prevKp, prevKi, prevKd;
double movingAngleOffset = 0;
double input, output;
int moveState = 0;
char inChar;
#if MANUAL_TUNING
PID pid(&input, &output, &setpoint, 0, 0, 0, DIRECT);
#else
PID pid(&input, &output, &setpoint, kp, ki, kd, DIRECT);
#endif
//MOTOR CONTROLLER
int ENA = 13;
int IN1 = 12;
int IN2 = 11;
int IN3 = 10;
int IN4 = 9;
int ENB = 8;

LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, leftmotorspeed, rightmotorspeed); //0.7 left,right

//timers
long time1Hz = 0;
long time5Hz = 0;

volatile bool mpuInterrupt = false;
void dmpDataReady() {
  mpuInterrupt = true;
}

void setup() {

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(9600);
  bluetooth.begin(9600);

  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(72);
  mpu.setYGyroOffset(-63);
  mpu.setZGyroOffset(21);
  mpu.setZAccelOffset(1111);

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-255, 255);
  }
}

void loop() {
#if MOVE_BACK_FORTH
  moveBackForth();
#endif

  if (!dmpReady) return;
  while (!mpuInterrupt && fifoCount < packetSize) {
    if (moveState == 3) {   //Right
      setpoint = originalSetpoint + 5;
      pid.Compute();
      motorController.turnLeft(output, 60);
      delay(1);
    }
    else if (moveState == 4) {    //Left
      setpoint = originalSetpoint - 5;
      pid.Compute();
      motorController.turnRight(output, 60);
      delay(1);
    }
    else if (moveState == 0 || moveState == 1 || moveState == 2 || moveState == 5 || moveState == 6) {    //Forward and Backward(both low speed and high speed)
      pid.Compute();
      motorController.move(output, MIN_ABS_SPEED);
      delay(1);
    }

    unsigned long currentMillis = millis();
    if (currentMillis - time1Hz >= 1000) {
      loopAt1Hz();
      time1Hz = currentMillis;
    }
  }

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
  }
  else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    input = ypr[1] * 180 / M_PI + 180;
    serialEvent();
  }
}

void loopAt1Hz() {
#if MANUAL_TUNING
  setPIDTuningValues();
#endif
}

void moveBackForth() {    //Forward and Backward(both low speed and high speed)
  if (moveState == 0)
    setpoint = originalSetpoint;
  else if (moveState == 1)
    setpoint = originalSetpoint + 2;
  else if (moveState == 2)
    setpoint = originalSetpoint - 2;
  else if (moveState == 5)
    setpoint = originalSetpoint + 10;
  else if  (moveState == 6)
    setpoint = originalSetpoint - 10;
}

#if MANUAL_TUNING
void setPIDTuningValues() {
  pid.SetTunings(kp, ki, kd);
  prevKp = kp; prevKi = ki; prevKd = kd;
}
#endif


void serialEvent() {
  while (Serial.available()) {
    inChar = Serial.read();
    Serial.print(inChar);
    if (inChar == 'F') {
      moveState = 2;    // Forward
    }
    else if (inChar == 'B') {
      moveState = 1;    //Backward
    }
    else if (inChar == 'L') {
      moveState = 4;    //Left
    }
    else if (inChar == 'R') {
      moveState = 3;    //Right
    }
    else if (inChar == 'f') {
      moveState = 5;    //High Speed Forward
    }
    else if (inChar == 'b') {
      moveState = 6;    //High Speed Backward
    }
    else {
      moveState = 0;
    }
  }
}
