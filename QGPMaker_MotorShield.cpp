/******************************************************************
  It will only work with http://www.7gp.cn
 ******************************************************************/

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include <Wire.h>

#include "QGPMaker_MotorShield.h"
#include "Adafruit_MS_PWMServoDriver.h"

#if defined(ARDUINO_SAM_DUE)
#define WIRE Wire1
#else
#define WIRE Wire
#endif

#if (MICROSTEPS == 8)
uint8_t microstepcurve[] = {0, 50, 98, 142, 180, 212, 236, 250, 255};
#elif (MICROSTEPS == 16)
uint8_t microstepcurve[] = {0, 25, 50, 74, 98, 120, 141, 162, 180, 197, 212, 225, 236, 244, 250, 253, 255};
#endif

QGPMaker_MotorShield::QGPMaker_MotorShield(uint8_t addr)
{
  _addr = addr;
  _pwm = Adafruit_MS_PWMServoDriver(_addr);
}

void QGPMaker_MotorShield::begin(uint16_t freq)
{
  // init PWM w/_freq
  WIRE.begin();
  _pwm.begin();
  _freq = freq;
  _pwm.setPWMFreq(_freq); // This is the maximum PWM frequency
  for (uint8_t i = 0; i < 16; i++)
    _pwm.setPWM(i, 0, 0);
}

void QGPMaker_MotorShield::setPWM(uint8_t pin, uint16_t value)
{
  if (value > 4095)
  {
    _pwm.setPWM(pin, 4096, 0);
  }
  else
    _pwm.setPWM(pin, 0, value);
}
void QGPMaker_MotorShield::setPin(uint8_t pin, boolean value)
{
  if (value == LOW)
    _pwm.setPWM(pin, 0, 0);
  else
    _pwm.setPWM(pin, 4096, 0);
}

QGPMaker_DCMotor *QGPMaker_MotorShield::getMotor(uint8_t num)
{
  if (num > 4)
    return NULL;

  num--;

  if (dcmotors[num].motornum == 0)
  {
    // not init'd yet!
    dcmotors[num].motornum = num;
    dcmotors[num].MC = this;
    uint8_t pwm, in1, in2;
    if (num == 0)
    {
      pwm = 8;
      in2 = 9;
      in1 = 8;
    }
    else if (num == 1)
    {
      pwm = 13;
      in2 = 11;
      in1 = 10;
    }
    else if (num == 2)
    {
      pwm = 2;
      in2 = 15;
      in1 = 14;
    }
    else if (num == 3)
    {
      pwm = 7;
      in2 = 13;
      in1 = 12;
    }
    //    dcmotors[num].PWMpin = pwm;
    dcmotors[num].IN1pin = in1;
    dcmotors[num].IN2pin = in2;
  }
  return &dcmotors[num];
}

/******************************************
               MOTORS
******************************************/

QGPMaker_DCMotor::QGPMaker_DCMotor(void)
{
  MC = NULL;
  motornum = 0;
  _speed = IN1pin = IN2pin = 0;
}

void QGPMaker_DCMotor::run(uint8_t cmd)
{
  MDIR = cmd;
  switch (cmd)
  {
  case FORWARD:
    MC->setPin(IN2pin, LOW); // take low first to avoid 'break'
    MC->setPWM(IN1pin, _speed);
    break;
  case BACKWARD:
    MC->setPin(IN1pin, LOW); // take low first to avoid 'break'
    MC->setPWM(IN2pin, _speed);
    break;
  case RELEASE:
    MC->setPin(IN1pin, LOW);
    MC->setPin(IN2pin, LOW);
    break;
  case BRAKE:
    MC->setPin(IN1pin, HIGH);
    MC->setPin(IN2pin, HIGH);
    break;
  }
}

void QGPMaker_DCMotor::setSpeed(uint8_t speed)
{
  _speed = speed*16;
  run(MDIR);
}

void QGPMaker_DCMotor::setPwm(int16_t speed)
{
  _speed = abs(speed);
  if(speed>0)run(FORWARD);
  else if(speed==0)run(RELEASE);
  else run(BACKWARD);
}
