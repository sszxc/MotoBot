/*
 * Author: Xuechao Zhang
 * Date: July 23rd, 2020
 * Description: 自行车机器人
 */

#include <Wire.h>

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

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
  attachInterrupt(0, flywheel_encoder, CHANGE);

  Serial.begin(19200);

  init_IMU(); 
  
  for (int i = 0; i < 4; i++)
  {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(100);                        // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(100);                        // wait for a second  
  }  
}

void loop() { 
  Read_IMU();
  Send_wave();
  Motion_control();
}
