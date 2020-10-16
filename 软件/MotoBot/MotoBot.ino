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
#define BUTTON1 16 //拨码开关 1,靠近电源开关,按下是 LOW
#define BUTTON2 15 //拨码开关 2

float roll = 0, pitch = 0, yaw = 0;
long flywheel_position[2] = {0}; // 编码器 前一时刻和当前时刻
float flywheel_speed = 0, flywheel_target = -1.5;
// float fw_kp = 0.02, fw_ki = 0.003, fw_kd = 0;//0.01; //速度环
// float bl_kp = -1300.0, bl_ki = 0.0, bl_kd = -1800.0; //角度环节
float fw_kp = 0.0, fw_ki = 0.0, fw_kd = 0;//0.01; //速度环
float bl_kp = -65.0, bl_ki = 0.0, bl_kd = -25.0; //角度环节 //-46 0 -25 
unsigned long currentTime, previousTime = 0; // 计时
float elapsedTime;
int pwm_out = 0;
  
Servo steer_servo, balance_servo;

void BeepandBlink(bool Bee_En = true){
  int t = 100;
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  if(Bee_En) digitalWrite(BEEP, HIGH);
  delay(t);
  if(Bee_En) digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  digitalWrite(BEEP, LOW);
  delay(t);
}

//翻车则强制停止
void forceidle(int angle = 40)
{
  if(roll>angle or roll<-angle) // 翻车 需要拨一下开关恢复
  {
    analogWrite(FLYWHEEL, 255);
    DisplayWarning(10);
    while(1)
      if(digitalRead(BUTTON2)==LOW)
        break;
    DisplayWarning(20);
    while(1)
      if(digitalRead(BUTTON2)==HIGH)
        break;
    while(roll>angle or roll<-angle)
    {
      Read_IMU(); // 大概率overflow
      delay(100);  
    }
    BeepandBlink();
  }  
}

//初始化函数
void setup() {
  //配置IO为输出 
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BEEP, OUTPUT);
  //蜂鸣器响
  BeepandBlink(false);
  //配置编码器
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
  //配置拨码开关
  pinMode(BUTTON1, INPUT);
  pinMode(BUTTON2, INPUT);
  // 配置飞轮
  pinMode(FLYWHEEL_DIR, OUTPUT);
  // 配置转向舵机
  steer_servo.attach(STEER_SERVO);
  // 配置平衡舵机
  balance_servo.attach(BALANCE_SERVO);
  // 设置飞轮编码器的中断为边沿触发
  attachInterrupt(0, flywheel_encoder, CHANGE);
  // 配置波特率
  Serial.begin(115200);
  // 开始通信
  Wire.begin(); // Initialize comunication
  // 初始化陀螺仪
  init_IMU(); 
  // 初始化OLED
  #ifdef OLED_DEBUG
    init_OLED();
  #endif
/*
  //  BeepandBlink();
  //  while (millis() < 25000) //等待稳定读数
  //  {
  //    SerialPrint();
  //    #ifdef OLED_DEBUG
  //      display_regular();
  //    #endif
  //    Read_IMU();
  //  }
*/
  // 蜂鸣器响
  BeepandBlink();
}

// 主循环
void loop() { 
  // 获取当前运行时间
  currentTime = millis();
  // 一个循环的时间, 单位：s, 用于测速
  elapsedTime = (currentTime - previousTime) / 1000.0; 
  // 读陀螺仪
  Read_IMU(); 
  // 读编码器 
  flywheel_readspeed();  
  //拨码开关 2 未按下就进行直立控制
  if (digitalRead(BUTTON2)==HIGH)
  {
    balance_control_v2();    
    //servo_control();
  }
  else
  {
    delay(1000);
    BeepandBlink();
  } 
  //Send_wave(); 
  //串口打印
  SerialPrint();
  #ifdef OLED_DEBUG
    display_regular();
  #endif
  // 串口调参
  #ifdef SERIAL_PARATUNING
    serial_paratuning();
  #endif
  forceidle(); //翻车检测
  // 手动控制周期为 20ms
  while (millis()-currentTime<20); 
  // 更新上一时刻
  previousTime = currentTime;
}
