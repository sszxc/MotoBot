/*
 * Author: Xuechao Zhang
 * Date: July 24th, 2020
 * Description: 用于山外上位机的示波器；串口调参
 *    帧协议参考https://www.iambigboss.top/post/37201_1_1.html
 */

#define Channel 4 //通道数

void vcan_sendware(uint8_t *wareaddr, uint32_t waresize)
{
#define CMD_WARE 3                         //帧协议
  uint8_t cmdf[2] = {CMD_WARE, ~CMD_WARE}; //串口调试 使用的前命令
  uint8_t cmdr[2] = {~CMD_WARE, CMD_WARE}; //串口调试 使用的后命令

  Serial.write(CMD_WARE);
  Serial.write(~CMD_WARE);
  while (waresize--)
  {
    Serial.write(*wareaddr);
    wareaddr++;
  }
  Serial.write(~CMD_WARE);
  Serial.write(CMD_WARE);
  //Serial.write(cmdr[2]); //这种发送方法不对？？
  //Serial.write(cmdr[2]);
}

float var[Channel]; //示波器通道数
void Send_wave()
{
  // 山外虚拟示波器
  var[0] = roll;
  var[1] = pitch;
  var[2] = yaw;
  var[3] = flywheel_speed;
  vcan_sendware((uint8_t *)var, sizeof(var));
}

// 用于Arduino串口绘图器
void SerialPrint_init()
{
  Serial.println("target,speed");
  // Serial.println("roll*1000");
}
void SerialPrint()
{
  // Serial.print(steer_angle);
  // Serial.print(",");
  // Serial.println(roll * 1000);
  // Serial.print(",");
  // Serial.print(pitch);
  // Serial.print(",");
  // Serial.print(yaw);
  Serial.print(flywheel_target);
  Serial.print(",");
  Serial.print(flywheel_speed);
  Serial.println();
}

void SerialRead()
{
  if(Serial.available()>0)
  {
    BT_char = Serial.read();
    Serial.println(BT_char);
    switch (BT_char)
    {
      case 'U':
        /* Speed Up */
        motor_speed += 10;
        Serial.println(motor_speed);
        break;
      case 'D':
        /* Slow down */
        motor_speed -= 10;
        Serial.println(motor_speed);
        break;
      case 'L':
        /*Turn Left */
        steer_acc -= 1;
        // steer_angle -= 1;
        Serial.println(steer_angle);
        break;
      case 'R':
        /*Turn Right */
        steer_acc += 1;
        // steer_angle += 1;
        Serial.println(steer_angle);
        break;
      case 'S':
        /* Stop*/
        motor_speed = 0;
        Serial.println(motor_speed);
        break;
      case '1':
        /*bw_kp += ? ; */
        bl_kp += 10;
        break;
      case '2':
        /*bw_kp -= ? ; */
        bl_kp -= 10;
        break;
      case '3':
        /*bw_kd += ? ; */
        bl_kd += 10;
        break;
      case '4':
        /*bw_kd -= ? ; */
        bl_kd -= 10;
        break;
      case '5':
        /*sp_kp += ? ; */
        sp_kp += 0.0001;
        break;
      case '6':
        /*sp_kp -= ? ; */
        sp_kp -= 0.0001;
        break;
      default:
        break;
    }
}
}

// 串口调参
const int bufferLength = 50;     // 定义缓存大小为50个字节
char serialBuffer[bufferLength]; // 建立字符数组用于缓存

void split(char *src, const char *separator, char **dest, int *num)
{
  /*
    src 源字符串的首地址(buf的地址) 
    separator 指定的分割字符
    dest 接收子字符串的数组
    num 分割后子字符串的个数
  */
  char *pNext;
  int count = 0;
  if (src == NULL || strlen(src) == 0) //如果传入的地址为空或长度为0，直接终止
    return;
  if (separator == NULL || strlen(separator) == 0) //如未指定分割的字符串，直接终止
    return;
  pNext = (char *)strtok(src, separator); //必须使用(char *)进行强制类型转换(虽然不写有的编译器中不会出现指针错误)
  while (pNext != NULL)
  {
    *dest++ = pNext;
    ++count;
    pNext = (char *)strtok(NULL, separator); //必须使用(char *)进行强制类型转换
  }
  *num = count;
}

bool serial_paratuning()
{
  if (Serial.available())
  { 
    // 当串口接收到信息后
    //Serial.println("Received Serial Data:");
    Serial.readBytes(serialBuffer, bufferLength); // 将接收到的信息使用readBytes读取

    char *revbuf[10] = {0};                     // 存放分割后的子字符串
    int Datanum = 0;                            // 分割后子字符串的个数
    split(serialBuffer, ",", revbuf, &Datanum); // 调用函数进行分割//Datanum从1开始

    // for (int i = 0; i < Datanum; i++)
    // {
    //   Serial.print(i);
    //   Serial.print(":");
    //   //Serial.println(revbuf[i]);
    //   float test = atof(revbuf[i]);
    //   Serial.println(test, 5);
    // }

    // 变量赋值
    if (Datanum == 3)
    {
      // motor_speed = atof(revbuf[0]);
      // steer_kp = atof(revbuf[1]);
      // steer_kd = atof(revbuf[2]);
      bl_kp = atof(revbuf[0]);
      bl_kd = atof(revbuf[1]);
      sp_kp = atof(revbuf[2]);
    }

    memset(serialBuffer, 0, 50);
    // Serial.println("Finished Printing Recevied Data.");
    return 1;
  }
  else
    return 0;
}
