/*
 * Author: Xuechao Zhang
 * Date: July 24th, 2020
 * Description: 转向、平衡、飞轮、驱动电机控制
 */

void Motion_control(){
  
}

void servo(){
  
}

void flywheel(){

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