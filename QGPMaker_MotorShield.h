/******************************************************************

 It will only work with http://www.7gp.cn
 
 ******************************************************************/

#ifndef _QGPMaker_MotorShield_h_
#define _QGPMaker_MotorShield_h_

#include <inttypes.h>
#include <Wire.h>
#include "Adafruit_MS_PWMServoDriver.h"

//#define MOTORDEBUG

#define MICROSTEPS 16 // 8 or 16

#define MOTOR1_A 2
#define MOTOR1_B 3
#define MOTOR2_A 1
#define MOTOR2_B 4
#define MOTOR4_A 0
#define MOTOR4_B 6
#define MOTOR3_A 5
#define MOTOR3_B 7

#define FORWARD 1
#define BACKWARD 2
#define BRAKE 3
#define RELEASE 4

#define SINGLE 1
#define DOUBLE 2
#define INTERLEAVE 3
#define MICROSTEP 4

class QGPMaker_MotorShield;

class QGPMaker_DCMotor
{
public:
  QGPMaker_DCMotor(void);
  friend class QGPMaker_MotorShield;
  void run(uint8_t);
  void setSpeed(uint8_t);

private:
  uint8_t _speed, IN1pin, IN2pin, MDIR;
  QGPMaker_MotorShield *MC;
  uint8_t motornum;
};

class QGPMaker_MotorShield
{
public:
  QGPMaker_MotorShield(uint8_t addr = 0x60);
  friend class QGPMaker_DCMotor;
  void begin(uint16_t freq = 1600);

  void setPWM(uint8_t pin, uint16_t val);
  void setPin(uint8_t pin, boolean val);
  QGPMaker_DCMotor *getMotor(uint8_t n);


private:
  uint8_t _addr;
  uint16_t _freq;
  QGPMaker_DCMotor dcmotors[4];

  Adafruit_MS_PWMServoDriver _pwm;

};

#endif
