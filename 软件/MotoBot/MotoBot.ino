/*
 * Author: Xuechao Zhang
 * Date: July 23rd, 2020
 * Description: 自行车机器人
 */

//#define OLED_DEBUG
#define SERIAL_PARATUNING

#include <Wire.h>
#include <Servo.h>

#define ENCODER_A 2
#define BEEP 4
#define STEER_SERVO 9
#define BALANCE_SERVO 10
#define ENCODER_B 7
#define MOTOR_F 5
#define MOTOR_B 6
#define FLYWHEEL 11
#define FLYWHEEL_DIR 14
#define BUTTON0 16
#define BUTTON1 15

float roll = 0, pitch = 0, yaw = 0, last_roll = 0;
long flywheel_position[2] = {0}; // 编码器 前一时刻和当前时刻
float flywheel_speed = 0, flywheel_target = 0;
float fw_kp = 0.02, fw_ki = 0.003, fw_kd = 0;//0.01;
float bl_kp = -1300.0, bl_ki = 0.0, bl_kd = -1800.0;
unsigned long currentTime, previousTime; // 计时
float elapsedTime;
int pwm_out = 0;
  
Servo steer_servo, balance_servo;

void BeepandBlink(int t = 100){
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(BEEP, HIGH);
  delay(t);
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  digitalWrite(BEEP, LOW);
  delay(t);
}

void forceidle(int angle = 20)
{
  if(roll>angle or roll<-angle) // 翻车 需要拨一下开关恢复
  {
    analogWrite(FLYWHEEL, 255);
    DisplayWarning(10);
    while(1)
      if(digitalRead(BUTTON1)==LOW)
        break;
    DisplayWarning(20);
    while(1)
      if(digitalRead(BUTTON1)==HIGH)
        break;
    while(roll>angle or roll<-angle)
    {
      Read_IMU(); // 大概率overflow
      delay(100);  
    }
    BeepandBlink();
  }  
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BEEP, OUTPUT);
  BeepandBlink();
  
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
  pinMode(BUTTON0, INPUT);
  pinMode(BUTTON1, INPUT);
  pinMode(FLYWHEEL_DIR, OUTPUT);
  steer_servo.attach(STEER_SERVO);
  balance_servo.attach(BALANCE_SERVO);
  attachInterrupt(0, flywheel_encoder, CHANGE);
    
  Serial.begin(115200);

  Wire.begin(); // Initialize comunication
  init_IMU(); 
  #ifdef OLED_DEBUG
    init_OLED();
  #endif
  
//  BeepandBlink();
//  while (millis() < 25000) //等待稳定读数
//  {
//    SerialPrint();
//    #ifdef OLED_DEBUG
//      display_regular();
//    #endif
//    Read_IMU();
//  }
  BeepandBlink();
}

void loop() { 
  previousTime = currentTime;
  currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000.0; // 一个循环的时间 用于测速
  Read_IMU(); // 读陀螺仪
  flywheel_readspeed(); // 读编码器  

  if (digitalRead(BUTTON1)==HIGH)//按钮测试
  {
    balance_control();    
    //servo_control();
  }
  else
  {
    delay(1000);
    BeepandBlink();
  } 
  
  //Send_wave(); 
  SerialPrint();
  #ifdef OLED_DEBUG
    display_regular();
  #endif
  
  #ifdef SERIAL_PARATUNING
    serial_paratuning();// 串口调参
  #endif
  
  forceidle(); //翻车
  
  while (millis()-currentTime<20); // 手动20ms控制周期
}
