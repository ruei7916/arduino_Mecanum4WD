#include <Wire.h>
#include "PS2X_lib.h"
#include "QGPMaker_MotorShield.h"
#include "QGPMaker_Encoder.h"

QGPMaker_MotorShield AFMS = QGPMaker_MotorShield();


QGPMaker_DCMotor *DCMotor_2 = AFMS.getMotor(2);
QGPMaker_DCMotor *DCMotor_4 = AFMS.getMotor(4);
QGPMaker_DCMotor *DCMotor_1 = AFMS.getMotor(1);
QGPMaker_DCMotor *DCMotor_3 = AFMS.getMotor(3);
QGPMaker_Servo *Servo0 = AFMS.getServo(0);

QGPMaker_Encoder Encoder1(1);
QGPMaker_Encoder Encoder2(2);
QGPMaker_Encoder Encoder3(3);
QGPMaker_Encoder Encoder4(4);

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

#define FRAME_HEADER      0X7B       // Frame header
#define FRAME_TAIL        0X7D       // Frame tail
#define RECV_DATA_SIZE    10          // The length of data sent by ROS to the lower machine 


uint8_t recv_data[RECV_DATA_SIZE];
uint8_t send_data[12];
int recv_count;
bool start_frame;
float x,y,z;

float motor_1_target, motor_2_target, motor_3_target, motor_4_target;
int16_t motor_1_pwm, motor_2_pwm, motor_3_pwm, motor_4_pwm;
float m1_speed, m2_speed, m3_speed, m4_speed;
unsigned long last_time_send_data, last_time_control;
uint8_t command_pick_count, result_pick_count;

// wheel radius
#define wheel_radius (3.0/100) // unit: meter
// half of the distance between front wheels
#define wheel_spacing (19.5/2.0/100) // unit: meter
// half of the distance between front wheel and rear wheel
#define axle_spacing (15.1/2.0/100) // unit: meter

bool ps2_control;

void setup(){
  AFMS.begin(50);
  
  Serial.begin(38400);
  recv_count = 0;
  start_frame = false;
  last_time_send_data = millis();
  last_time_control = millis();
  Servo0->writeServo(10);
  command_pick_count=0;
  result_pick_count=0;
}

void loop(){
    if(Serial.available()){
      uint8_t t = Serial.read();
      if(start_frame){
        recv_data[recv_count++] = t;
      }
      else if(t==FRAME_HEADER){
        start_frame = true; // start receiving a frame
        recv_data[recv_count++] = t;
      }
      
      if(recv_count==RECV_DATA_SIZE){
        recv_count = 0;
        start_frame = false;
        if(t==FRAME_TAIL){
          // finish receiving a frame
          if(recv_data[8]==(recv_data[0]^recv_data[1]^recv_data[2]^recv_data[3]^recv_data[4]^recv_data[5]^recv_data[6]^recv_data[7])){ //檢查資料
            // received frame data correct
            x = (int16_t)((recv_data[1]<<8)|recv_data[2])/1000.0;
            y = (int16_t)((recv_data[3]<<8)|recv_data[4])/1000.0;
            z = (int16_t)((recv_data[5]<<8)|recv_data[6])/1000.0;
            if(command_pick_count!=recv_data[7]){
              //pick ball
              int degree = 10;
              for (int i=0;i<21;i++){
                degree+=6;
                Servo0->writeServo(degree);
                delay(50);
              }
              for (int i=0;i<21;i++){
                degree-=6;
                Servo0->writeServo(degree);
                delay(50);

              }
              //finish picking ball
              result_pick_count=recv_data[7];
            }
            command_pick_count=recv_data[7];
          }
        }
      }
    }
    // this runs at 100Hz
    if(millis()-last_time_control>10){

      motor_1_target = x-y-z*(wheel_spacing+axle_spacing);
      motor_2_target = x+y-z*(wheel_spacing+axle_spacing);
      motor_3_target = x-y+z*(wheel_spacing+axle_spacing);
      motor_4_target = x+y+z*(wheel_spacing+axle_spacing);

      m1_speed = Encoder1.getspeed();
      m2_speed = Encoder2.getspeed();
      m3_speed = Encoder3.getspeed();
      m4_speed = Encoder4.getspeed();
    
      motor_1_pwm=Incremental_PI_A(m1_speed*wheel_radius,motor_1_target);
      motor_2_pwm=Incremental_PI_B(m2_speed*wheel_radius,motor_2_target);
      motor_3_pwm=Incremental_PI_C(m3_speed*wheel_radius,motor_3_target);
      motor_4_pwm=Incremental_PI_D(m4_speed*wheel_radius,motor_4_target);
    
      DCMotor_1->setPwm(motor_1_pwm);
      DCMotor_2->setPwm(motor_2_pwm);
      DCMotor_3->setPwm(motor_3_pwm);
      DCMotor_4->setPwm(motor_4_pwm);
    
      last_time_control = millis();
    }
  
  // send data every 50ms
  if(millis()-last_time_send_data>50){
    
    int16_t M1, M2, M3, M4; // 把轉速變成16bits
    M1 = m1_speed*1000;
    send_data[1] = M1;     // 分成2個8bits輸出
    send_data[0] = M1>>8;
    
    M2 = m2_speed*1000;
    send_data[3] = M2;
    send_data[2] = M2>>8;
    
    M3 = m3_speed*1000;
    send_data[5] = M3;
    send_data[4] = M3>>8;
    
    M4 = m4_speed*1000;
    send_data[7] = M4;
    send_data[6] = M4>>8;
    
    Serial.write(FRAME_HEADER);
    Serial.write(send_data[0]); // 輸出4個輪子的速度
    Serial.write(send_data[1]);
    Serial.write(send_data[2]);
    Serial.write(send_data[3]);
    Serial.write(send_data[4]);
    Serial.write(send_data[5]);
    Serial.write(send_data[6]);
    Serial.write(send_data[7]);
    Serial.write(result_pick_count);
    Serial.write(FRAME_HEADER^send_data[0]^send_data[1]^send_data[2]^send_data[3]^send_data[4]^send_data[5]^send_data[6]^send_data[7]^result_pick_count);
    Serial.write(FRAME_TAIL);
    
    last_time_send_data = millis();
  }
}



/*incremental pi controllers*/
float Velocity_KP=300.0,Velocity_KI=300.0; 
int Incremental_PI_A (float Encoder,float Target)
{ 	
  static float Bias,Pwm,Last_bias;
  if(Target==0.0){
    Pwm=0.0;
    Bias=0.0;
    Last_bias=0.0;
    return Pwm;
  }
  if(Encoder>=1.0){
    return Pwm;
  }
  Bias=Target-Encoder; //Calculate the deviation //计算偏差
  Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias; 
  if(Pwm>4096.0)Pwm=4096.0;
  if(Pwm<-4096.0)Pwm=-4096.0;
  Last_bias=Bias; //Save the last deviation //保存上一次偏差 
  return Pwm;    
}
int Incremental_PI_B (float Encoder,float Target)
{  
  static float Bias,Pwm,Last_bias;
  if(Target==0.0){
    Pwm=0.0;
    Bias=0.0;
    Last_bias=0.0;
    return Pwm;
  }
  if(Encoder>=1.0){
    return Pwm;
  }
  Bias=Target-Encoder; //Calculate the deviation //计算偏差
  Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;  
  if(Pwm>4096.0)Pwm=4096.0;
  if(Pwm<-4096.0)Pwm=-4096.0;
  Last_bias=Bias; //Save the last deviation //保存上一次偏差 
  return Pwm;
}
int Incremental_PI_C (float Encoder,float Target)
{  
  static float Bias,Pwm,Last_bias;
  if(Target==0.0){
    Pwm=0.0;
    Bias=0.0;
    Last_bias=0.0;
    return Pwm;
  }
  if(Encoder>=1.0){
    return Pwm;
  }
  Bias=Target-Encoder; //Calculate the deviation //计算偏差
  Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias; 
  if(Pwm>4096.0)Pwm=4096.0;
  if(Pwm<-4096.0)Pwm=-4096.0;
  Last_bias=Bias; //Save the last deviation //保存上一次偏差 
  return Pwm; 
}
int Incremental_PI_D (float Encoder,float Target)
{  
  static float Bias,Pwm,Last_bias;
  if(Target==0.0){
    Pwm=0.0;
    Bias=0.0;
    Last_bias=0.0;
    return Pwm;
  }
  if(Encoder>=1.0){
    return Pwm;
  }
  Bias=Target-Encoder; //Calculate the deviation //计算偏差
  Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;  
  if(Pwm>4096.0)Pwm=4096.0;
  if(Pwm<-4096.0)Pwm=-4096.0;
  Last_bias=Bias; //Save the last deviation //保存上一次偏差 
  return Pwm; 
}
