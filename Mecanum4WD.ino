#include <Wire.h>
#include "PS2X_lib.h"
#include "QGPMaker_MotorShield.h"

QGPMaker_MotorShield AFMS = QGPMaker_MotorShield();
PS2X ps2x;

QGPMaker_DCMotor *DCMotor_2 = AFMS.getMotor(2);
QGPMaker_DCMotor *DCMotor_4 = AFMS.getMotor(4);
QGPMaker_DCMotor *DCMotor_1 = AFMS.getMotor(1);
QGPMaker_DCMotor *DCMotor_3 = AFMS.getMotor(3);

void forward() {
  DCMotor_1->setSpeed(200);
  DCMotor_1->run(FORWARD);
  DCMotor_2->setSpeed(200);
  DCMotor_2->run(FORWARD);
  DCMotor_3->setSpeed(200);
  DCMotor_3->run(FORWARD);
  DCMotor_4->setSpeed(200);
  DCMotor_4->run(FORWARD);
}

void turnLeft() {
  DCMotor_1->setSpeed(200);
  DCMotor_1->run(BACKWARD);
  DCMotor_2->setSpeed(200);
  DCMotor_2->run(BACKWARD);
  DCMotor_3->setSpeed(200);
  DCMotor_3->run(FORWARD);
  DCMotor_4->setSpeed(200);
  DCMotor_4->run(FORWARD);
}

void turnRight() {
  DCMotor_1->setSpeed(200);
  DCMotor_1->run(FORWARD);
  DCMotor_2->setSpeed(200);
  DCMotor_2->run(FORWARD);
  DCMotor_3->setSpeed(200);
  DCMotor_3->run(BACKWARD);
  DCMotor_4->setSpeed(200);
  DCMotor_4->run(BACKWARD);
}

void moveLeft() {
  DCMotor_1->setSpeed(200);
  DCMotor_1->run(BACKWARD);
  DCMotor_2->setSpeed(200);
  DCMotor_2->run(FORWARD);
  DCMotor_3->setSpeed(200);
  DCMotor_3->run(BACKWARD);
  DCMotor_4->setSpeed(200);
  DCMotor_4->run(FORWARD);
}

void moveRight() {
  DCMotor_1->setSpeed(200);
  DCMotor_1->run(FORWARD);
  DCMotor_2->setSpeed(200);
  DCMotor_2->run(BACKWARD);
  DCMotor_3->setSpeed(200);
  DCMotor_3->run(FORWARD);
  DCMotor_4->setSpeed(200);
  DCMotor_4->run(BACKWARD);
}

void backward() {
  DCMotor_1->setSpeed(200);
  DCMotor_1->run(BACKWARD);
  DCMotor_2->setSpeed(200);
  DCMotor_2->run(BACKWARD);
  DCMotor_3->setSpeed(200);
  DCMotor_3->run(BACKWARD);
  DCMotor_4->setSpeed(200);
  DCMotor_4->run(BACKWARD);
}

void stopMoving() {
  DCMotor_1->setSpeed(0);
  DCMotor_1->run(RELEASE);
  DCMotor_2->setSpeed(0);
  DCMotor_2->run(RELEASE);
  DCMotor_3->setSpeed(0);
  DCMotor_3->run(RELEASE);
  DCMotor_4->setSpeed(0);
  DCMotor_4->run(RELEASE);
}

void setup(){
  AFMS.begin(50);

  int error = 0;
  do{
    error = ps2x.config_gamepad(13,11,10,12, true, true);
    if(error == 0){
      break;
    }else{
      delay(100);
    }
  }while(1);
  Serial.begin(115200);

}

void loop(){
  ps2x.read_gamepad(false, 0);
  delay(30);
  if (ps2x.Button(PSB_PAD_UP)) {
    if (ps2x.Button(PSB_L2)) {
      DCMotor_2->setSpeed(200);
      DCMotor_2->run(FORWARD);
      DCMotor_4->setSpeed(200);
      DCMotor_4->run(FORWARD);

    } else if (ps2x.Button(PSB_R2)) {
      DCMotor_1->setSpeed(200);
      DCMotor_1->run(FORWARD);
      DCMotor_3->setSpeed(200);
      DCMotor_3->run(FORWARD);
    } else {
      forward();

    }

  } else if (ps2x.Button(PSB_PAD_DOWN)) {
    if (ps2x.Button(PSB_L2)) {
      DCMotor_2->setSpeed(200);
      DCMotor_2->run(BACKWARD);
      DCMotor_4->setSpeed(200);
      DCMotor_4->run(BACKWARD);

    } else if (ps2x.Button(PSB_R2)) {
      DCMotor_1->setSpeed(200);
      DCMotor_1->run(BACKWARD);
      DCMotor_3->setSpeed(200);
      DCMotor_3->run(BACKWARD);
    } else {
      backward();

    }
  } else if (ps2x.Button(PSB_PAD_LEFT)) {
    turnLeft();
  } else if (ps2x.Button(PSB_PAD_RIGHT)) {
    turnRight();
  } else if (ps2x.Button(PSB_L1)) {
    moveLeft();
  } else if (ps2x.Button(PSB_R1)) {
    moveRight();
  } else {
    stopMoving();

  }
  // 按下手柄X按钮，手柄震动一下
  if (ps2x.Button(PSB_CROSS)) {
    ps2x.read_gamepad(true, 200);
    delay(300);
    ps2x.read_gamepad(false, 0);

  }
  int t;
  if(Serial.available()){
    t=Serial.read();
    switch(t){
      case '1':
      forward();
      break;
      case '2':
      backward();
      break;
      case '3':
      moveLeft();
      break;
      case '4':
      moveRight();
      break;
      case '5':
      turnLeft();
      break;
      case '6':
      turnRight();
      break;
      default:
      break;
    }
  }
  
  
  delay(2);

}
