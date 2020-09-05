/*
 * Author: Xuechao Zhang
 * Date: July 23rd, 2020
 * Description: 自行车机器人
 */

#include <Wire.h>
#include <Servo.h>
//#include <MsTimer2.h>

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

float roll, pitch, yaw;
long flywheel_position[2] = {0};
float flywheel_speed = 0;
float flywheel_pwm = 0;
float flywheel_target = 0;
float elapsedTime, currentTime, previousTime; // 计时

Servo steer_servo, balance_servo;

void BeepandBlink(int t = 100){
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(BEEP, HIGH);
  delay(t);
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  digitalWrite(BEEP, LOW);
  delay(t);
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
    
  Serial.begin(19200);

  Wire.begin(); // Initialize comunication
  init_IMU(); 
  init_OLED();
  
//  MsTimer2::set(50, timerIsr); // 50ms period
//  MsTimer2::start();
  
//  BeepandBlink();
}

void SerialPrint()
{ 
    Serial.print(elapsedTime);
    Serial.print(",");  
    Serial.print(roll);
    Serial.print(",");
    Serial.print(pitch);
    Serial.print(",");
    Serial.print(yaw);
    Serial.print(",");

//    Serial.print(flywheel_pwm);
//    Serial.print(",");
//    Serial.print(flywheel_target);
//    Serial.print(",");
//    Serial.print(flywheel_speed);

    Serial.println("");
}

void loop() { 
  previousTime = currentTime;
  currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000;
  
  if (digitalRead(BUTTON1)==HIGH)//按钮测试
  {
    flywheel_target = millis() / 10 % 2000 - millis() / 10 % 100; 
    
    Read_IMU();
    flywheel_readspeed();
    flywheel();
    
    //Motion_control();
    display_demo();
    
    //Send_wave();
    SerialPrint();
  }
  else
  {
    delay(1000);
    BeepandBlink();
  }
}
