/*
 * Author: Xuechao Zhang
 * Date: July 24th, 2020
 * Description: 转向、平衡、飞轮、驱动电机控制
 */

void Motion_control(){
  // 双舵机测试
  for (int pos = 45; pos <= 135; pos += 30)
  {
    steer_servo.write(pos);
    balance_servo.write(pos);
    Serial.println(pos);
    delay(1000);
  }
  for (int pos = 135; pos >= 45; pos -= 30)
  {
    steer_servo.write(pos);
    balance_servo.write(pos);
    Serial.println(pos);
    delay(1000);
  }
}

void servo(){
  
}

void flywheel(){
  //飞轮测试
//  flywheel_pwm += 0.05*(flywheel_target - flywheel_speed);
  
  flywheel_PID();
  
  flywheel_pwm = constrain(flywheel_pwm, -100, 100);
  if (flywheel_pwm > 0)
  {
    digitalWrite(FLYWHEEL_DIR, LOW);
    analogWrite(FLYWHEEL, 255.0 - flywheel_pwm);
  }
  else
  {
    digitalWrite(FLYWHEEL_DIR, HIGH);
    analogWrite(FLYWHEEL, 255.0 + flywheel_pwm);
  }
}

void flywheel_PID()
{  
  flywheel_pwm_d = 1.4 * pitch + 35.0 * (pitch - last_pitch);
  flywheel_pwm += flywheel_pwm_d;
  last_pitch = pitch;
}

void flywheel_encoder(){ //感觉可以减少一半的中断触发 只看下降沿
  // ENA脚下降沿中断触发
  if (digitalRead(ENCODER_A) == LOW)
  {
    // 查询ENB的电平以确认是顺时针还是逆时针旋转
    if (digitalRead(ENCODER_B) == LOW)
      flywheel_position[1]++;
  }
  // ENA脚上升沿中断触发
  else
  {
    // 查询ENB的电平以确认是顺时针还是逆时针旋转
    if (digitalRead(ENCODER_B) == LOW)
      flywheel_position[1]--;
  }
}

void flywheel_readspeed()//定时器中断处理函数
{
  flywheel_speed = (flywheel_position[1] - flywheel_position[0])/elapsedTime;
  flywheel_position[0] = flywheel_position[1];
}
