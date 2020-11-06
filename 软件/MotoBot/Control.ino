/*
 * Author: Xuechao Zhang
 * Date: July 24th, 2020
 * Description: 转向、平衡、飞轮、驱动电机控制
 */

void servo_control(){
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

float ct_cirlce = 0;//舵机控制周期
char steer_acc = 0;//转向增量
//遥控舵机
void direction_telecontrol(){
  ct_cirlce += elapsedTime;
  // 0.02 s 控制一次
  if (ct_cirlce >= 0.02) 
  {
    ct_cirlce = 0;//重新计数控制周期
    if(steer_acc != 0)
    {
      //上一时刻角度变化方向与当前要求不一致，立刻角度制动
      if ((steer_acc < 0 && BT_char == 'R')||(steer_acc > 0 && BT_char == 'L'))
      {
        steer_acc = 0;//角度增量置 0
      } 
      steer_angle += steer_acc;//改变角度 
      //steer_angle = constrain(steer_angle,21,170);//输出限幅
      if (steer_angle == 21 || steer_acc == 170) 
      {
        steer_acc = 0;//舵机打死，角度增量置 0
      }
    }
    steer_servo.write(steer_angle); 
  }
}

//舵机PID
float roll_pre = 0;
void direction_control(){
  #ifdef TELE_MODE //遥控模式
    direction_telecontrol();
    //steer_servo.write(steer_angle); 
    return;
  #endif
  // PD 控制
    steer_angle = steer_kp * (roll - roll_offset) + steer_kd * (roll - roll_pre);
    roll_pre = roll;
    steer_angle = constrain(steer_angle, -65, 65); //输出限幅
    steer_servo.write(steer_angle + 90); 
}

//电机PID
void speed_control(){
  // #ifndef TELE_MODE //PID模式
  //   // P 控制
  //   motor_speed = speed_kp*roll;
  // #endif
  motor_speed = constrain(motor_speed,-255,255);//输出限幅
  if (motor_speed > 0)
  {
    analogWrite(MOTOR_F, motor_speed); 
    analogWrite(MOTOR_B, 0); 
  }
  else
  {
    analogWrite(MOTOR_F, 0); 
    analogWrite(MOTOR_B, -motor_speed); 
  }
}

//直立PID
void balance_control(){
  static float last_roll;
  static float roll_sum;

  roll_sum += roll;
  roll_sum = constrain(roll_sum, -1000, 1000); // 限幅

  // flywheel_target = bl_kp * (roll - roll_offset) + bl_ki * roll_sum + bl_kd * (roll - last_roll);

  // 串级一下
  flywheel_target = bl_kp * (roll - roll_offset - flywheel_speed * sp_kp) + bl_ki * roll_sum + bl_kd * (roll - last_roll);

  // flywheel_target = millis() / 10 % 1000 - millis() / 10 % 200; // 速度环调试
  
  // 速度闭环 or 开环
  if (digitalRead(BUTTON1)==HIGH) 
    pwm_out = flywheel_PID(flywheel_target);
  else
    pwm_out = flywheel_target / 20;
    
  pwm_out = constrain(pwm_out, -254, 254); // 限幅
  if (pwm_out < 5 && pwm_out > -5) pwm_out = 0; //防止烧电机
  if (pwm_out > 0) // 方向控制
  {
    digitalWrite(FLYWHEEL_DIR, LOW);
    analogWrite(FLYWHEEL, 255.0 - pwm_out);
  }
  else
  {
    digitalWrite(FLYWHEEL_DIR, HIGH);
    analogWrite(FLYWHEEL, 255.0 + pwm_out);
  }
  last_roll = roll;
}

//飞轮电机速度环
int flywheel_PID(int target) 
{ 
  static int fw_pwm, error, error_last = 0, error_sum;
  error = target - flywheel_speed;
  error_sum += error;
  error_sum = constrain(error_sum, -10, 10); // 限幅
  fw_pwm += fw_kp * error + fw_ki * error_sum + fw_kd * (error - error_last);
  error_last = error;
  return fw_pwm;
}

//编码器读数，中断回调函数
void flywheel_encoder(){ // 感觉可以减少一半的中断触发 只看下降沿
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

// 主循环调用
void flywheel_readspeed() // 作差计算速度
{
  flywheel_speed = (flywheel_position[1] - flywheel_position[0])/elapsedTime;
  flywheel_position[0] = flywheel_position[1];
}
