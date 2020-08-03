/*
 * Author: Xuechao Zhang
 * Date: July 23rd, 2020
 * Description: 自行车机器人
 */

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
#define BUTTON1 16
#define BUTTON2 15

float roll, pitch, yaw;
long flywheel_position = 0;

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
  
  for (int i = 0; i < 4; i++)
    BeepandBlink();
}

void loop() { 
  if (digitalRead(BUTTON1)==HIGH)//按钮测试
  {
    Read_IMU();
    flywheel();
    Motion_control();
    Send_wave();
    display_demo();
  }
  else
  {
    BeepandBlink();
    delay(1000);
  }
  delay(50);
}
