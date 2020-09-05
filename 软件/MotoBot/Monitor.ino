/*
 * Author: Xuechao Zhang
 * Date: July 24th, 2020
 * Description: 用于山外上位机的示波器
 *    帧协议参考https://www.iambigboss.top/post/37201_1_1.html
 */

#define Channel 4 //通道数

void vcan_sendware(uint8_t *wareaddr, uint32_t waresize)
{
#define CMD_WARE 3 //帧协议
  uint8_t cmdf[2] = {CMD_WARE, ~CMD_WARE};    //串口调试 使用的前命令
  uint8_t cmdr[2] = {~CMD_WARE, CMD_WARE};    //串口调试 使用的后命令

  Serial.write(CMD_WARE);
  Serial.write(~CMD_WARE);
  while(waresize--)
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
