/*
 * Author: Xuechao Zhang
 * Date: July 23rd, 2020
 * Description: 自行车机器人
 */

// #define OLED_DEBUG        //OLED调参模式
#define SERIAL_PARATUNING //串口调试模式
// #define TELE_MODE         //遥控模式

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
float flywheel_speed = 0, flywheel_target = 0;
float fw_kp = 0.02, fw_ki = 0, fw_kd = 0.04;       //飞轮电机PID
// float bl_kp = -450.0, bl_ki = 0.0, bl_kd = -500.0; //直立角度环(测试架)
// float sp_kp = 0.0035;                              //直立速度环(测试架)
float bl_kp = -1400.0, bl_ki = 0.0, bl_kd = -100.0; //直立角度环
float sp_kp = 0.0003;                               //直立速度环
unsigned long currentTime, previousTime = 0; // 计时
float elapsedTime;
int pwm_out = 0;
char BT_char = '0';//蓝牙控制字,默认为 '0'
int steer_angle = 90; //±65°
float steer_kp = -3.0; //打角 P
float steer_kd = 0.5;
int motor_speed = 170;//电机速度，(-255,+255)
float speed_kp = 0.0; //速度 P
float roll_offset = -2.3;

Servo steer_servo, balance_servo;

void BeepandBlink(){
  bool Bee_En = true;
  int t = 100;
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  if(Bee_En) digitalWrite(BEEP, HIGH);
  delay(t);
  if(Bee_En) digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  digitalWrite(BEEP, LOW);
  delay(t);
}

//翻车则强制停止
void forceidle(int angle = 23)
{
  if(roll>angle || roll< -angle) // 翻车 需要拨一下开关恢复 大概率影响IMU读数
  {
    analogWrite(FLYWHEEL, 255);
    analogWrite(MOTOR_F, 0);
    analogWrite(MOTOR_B, 0);
    steer_servo.write(90);
    // DisplayWarning(10);
    while(1)
      if(digitalRead(BUTTON2)==LOW)
        break;
   // DisplayWarning(20);
    while(1)
      if(digitalRead(BUTTON2)==HIGH)
        break;
    while(roll>angle || roll<-angle)
    {
      Read_IMU();
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
  BeepandBlink();
  //配置编码器
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
  //配置拨码开关
  pinMode(BUTTON1, INPUT);
  pinMode(BUTTON2, INPUT);
  // 配置飞轮
  pinMode(FLYWHEEL_DIR, OUTPUT);
  //配置电机
  pinMode(MOTOR_F,OUTPUT);
  pinMode(MOTOR_B,OUTPUT);
  // 配置转向舵机
  steer_servo.attach(STEER_SERVO);
  steer_servo.write(steer_angle);
  // 配置平衡舵机
  balance_servo.attach(BALANCE_SERVO);
  balance_servo.write(steer_angle);
  // 设置飞轮编码器的中断为边沿触发
  attachInterrupt(0, flywheel_encoder, CHANGE);
  // 配置波特率
  //Serial.begin(115200);
  Serial.begin(9600);

  #ifdef SERIAL_PARATUNING
    Serial.println("waiting for para!");
    while (!serial_paratuning())
    ; // 串口调参
  #endif
  
  // 开始通信
  Wire.begin(); // Initialize comunication
  // 初始化陀螺仪
  init_IMU(); 
  // 初始化OLED
  #ifdef OLED_DEBUG
    init_OLED();
  #endif

  // 蜂鸣器响
  BeepandBlink();
  SerialPrint_init();
  previousTime = millis();
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
  //拨码开关 2 未按下 进行直立控制
  if (digitalRead(BUTTON2)==HIGH)
  {
    balance_control();  //平衡控制
    direction_control();//方向控制
    speed_control();    //速度控制
  }
  else
  {
    delay(1000);
    BeepandBlink();
  } 
  // 山外示波器
  //Send_wave(); 

  // 串口打印
  SerialPrint();
  SerialRead();
  // OLED调参模式
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
