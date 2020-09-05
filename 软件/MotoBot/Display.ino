/*
 * Author: Xuechao Zhang
 * Date: July 25th, 2020
 * Description: OLED 0.96 I2C
 */

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // 设置OLED宽度,单位:像素
#define SCREEN_HEIGHT 64 // 设置OLED高度,单位:像素

// const int OLED = 0x3C; // OLED I2C address

// 自定义重置引脚,虽然教程未使用,但却是Adafruit_SSD1306库文件所必需的
#define OLED_RESET 17
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void init_OLED(){
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  
}

void display_demo()
{
  // 清除屏幕
  display.clearDisplay();
 
  // 设置字体颜色,白色可见
  display.setTextColor(WHITE);
 
  //设置字体大小
  display.setTextSize(1.5);
 
  //设置光标位置
  display.setCursor(0, 0);
  display.print("MotoBot");
 
  display.setCursor(0, 10);
  display.print("time: ");
  //打印自开发板重置以来的秒数：
  display.print(millis() / 1000);
  display.print(" s");
 
  display.setCursor(0, 20);
  display.print("Author: ");
  display.print("Henry Beta");

  display.setCursor(0, 30);
  display.print(roll);
  display.print("/");
  display.print(pitch);
  display.print("/");
  display.println(yaw);

  display.setCursor(0, 40);
  display.print("PWM: ");
  display.print(flywheel_pwm);
  display.setCursor(0, 50);
  display.print("Speed: ");
  display.print(flywheel_speed);
  
  display.display(); //刷新
}

void OLED_refresh(){

}
