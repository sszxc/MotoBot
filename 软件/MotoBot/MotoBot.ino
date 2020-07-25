/*
 * Author: Xuechao Zhang
 * Date: July 23rd, 2020
 * Description: 自行车机器人
 */

#include <Wire.h>
#include <Servo.h>

#define ENCODER_A 2
#define BEEP 4
#define STEER_SERVO 5
#define BALANCE_SERVO 6
#define ENCODER_B 7
#define MOTOR_F 9
#define MOTOR_B 10
#define FLYWHEEL 11

float roll, pitch, yaw;
int flywheel_position = 0;

Servo steer_servo, balance_servo;
int pos = 0; // 存储伺服电机角度信息的变量

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
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
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
  Read_IMU();
//  Send_wave();
//  Motion_control();

  display_demo();
  delay(50);
}
