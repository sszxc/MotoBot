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
  int pwmout = constrain(pitch, -30, 30)*8;
  if (pwmout > 0)
  {
    digitalWrite(FLYWHEEL_DIR, HIGH);
    analogWrite(FLYWHEEL, 255 - pwmout);
  }
  else
  {
    digitalWrite(FLYWHEEL_DIR, LOW);
    analogWrite(FLYWHEEL, 255 + pwmout);
  }
}

void flywheel_encoder(){ //感觉可以减少一半的中断触发 只看下降沿
  // ENA脚下降沿中断触发
  if (digitalRead(ENCODER_A) == LOW)
  {
    // 查询ENB的电平以确认是顺时针还是逆时针旋转
    if (digitalRead(ENCODER_B) == LOW)
      flywheel_position++;
  }
  // ENA脚上升沿中断触发
  else
  {
    // 查询ENB的电平以确认是顺时针还是逆时针旋转
    if (digitalRead(ENCODER_B) == LOW)
      flywheel_position--;
  }  
}
