/*
 * Author: Xuechao Zhang
 * Date: July 24th, 2020
 * Description: MPU6050的初始化、读取、零点漂移、滤波
 *    解算参考https://zhuanlan.zhihu.com/p/100740936（弃）
 *    卡尔曼滤波参考https://github.com/jjundot/MPU6050_Kalman
 *    Sept 19th，使用DMP模式 去除了外部中断
 *    Oct 25th，移植ICM20602 移除MPU6050 DMP模式代码
 */

// #define ICM20602 // 选择ICM20602 or MPU6050

#ifdef ICM20602
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"

#define     ICM20602_WHO_AM_I           0x75
#define     ICM20602_DEV_ADDR           0x69 //SA0接地：0x68   SA0上拉：0x69     0X69 逐飞，0X68 龙邱 
#define     ICM20602_PWR_MGMT_1         0x6B
#define     ICM20602_PWR_MGMT_2         0x6C
#define     ICM20602_CONFIG             0x1A
#define     ICM20602_SMPLRT_DIV         0x19
#define     ICM20602_GYRO_CONFIG        0x1B
#define     ICM20602_ACCEL_CONFIG       0x1C
#define     ICM20602_ACCEL_CONFIG_2     0x1D
#define     ICM20602_ACCEL_XOUT_H       0x3B
#define     ICM20602_GYRO_XOUT_H        0x43

int16_t ax, ay, az, gx, gy, gz;             //加速度计陀螺仪原始数据
float aax=0, aay=0,aaz=0, agx=0, agy=0, agz=0;//角度变量
long axo = 0, ayo = 0, azo = 0;             //加速度计偏移量
long gxo = 0, gyo = 0, gzo = 0;             //陀螺仪偏移量

float pi = 3.1415926;
float AcceRatio = 16384.0;                  //加速度计比例系数
float GyroRatio = 131.0;                    //陀螺仪比例系数

uint8_t n_sample = 8;                       //加速度计滤波算法采样个数
float aaxs[8] = {0}, aays[8] = {0}, aazs[8] = {0};         //x,y轴采样队列
long aax_sum, aay_sum,aaz_sum;                      //x,y轴采样和

float a_x[10]={0}, a_y[10]={0},a_z[10]={0} ,g_x[10]={0} ,g_y[10]={0},g_z[10]={0}; //加速度计协方差计算队列
float Px=1, Rx, Kx, Sx, Vx, Qx;             //x轴卡尔曼变量
float Py=1, Ry, Ky, Sy, Vy, Qy;             //y轴卡尔曼变量
float Pz=1, Rz, Kz, Sz, Vz, Qz;             //z轴卡尔曼变量

void init_IMU(){
    //自检
    uint8_t data_tmp = 0x00;
    while(0x12 != data_tmp)
    {
        I2Cdev::readByte(ICM20602_DEV_ADDR,ICM20602_WHO_AM_I,&data_tmp);
        //卡在这里原因有以下几点
        //1 MPU6050坏了，如果是新的这样的概率极低
        //2 接线错误或者没有接好
        //3 可能你需要外接上拉电阻，上拉到3.3V
    }

    //复位设备
    I2Cdev::writeByte(ICM20602_DEV_ADDR, ICM20602_PWR_MGMT_1,0x80);
    delay(2);//延时2ms
    data_tmp = 0x00;
    while(0x80 & data_tmp) //等待复位完成
    {
        I2Cdev::readByte(ICM20602_DEV_ADDR,ICM20602_PWR_MGMT_1,&data_tmp);
    }
    I2Cdev::writeByte(ICM20602_DEV_ADDR,ICM20602_PWR_MGMT_1,0x01);//时钟设置
    I2Cdev::writeByte(ICM20602_DEV_ADDR,ICM20602_PWR_MGMT_2,0x00);               //开启陀螺仪和加速度计
    I2Cdev::writeByte(ICM20602_DEV_ADDR,ICM20602_CONFIG,0x01);                   //176HZ 1KHZ
    I2Cdev::writeByte(ICM20602_DEV_ADDR,ICM20602_SMPLRT_DIV,0x07);               //采样速率 SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV)
    I2Cdev::writeByte(ICM20602_DEV_ADDR,ICM20602_GYRO_CONFIG,0x18);              //±2000 dps
                                                                                 //00 -> 250
                                                                                 //08 -> 500      
                                                                                 //10 -> 1000
                                                                                 //18 -> 2000
    
    I2Cdev::writeByte(ICM20602_DEV_ADDR,ICM20602_ACCEL_CONFIG,0x08);             //±2g  
                                                                                 //00 -> 2g 
                                                                                 //08 -> 4g  
                                                                                 //10 -> 8g 
                                                                                 //18 -> 16g
    I2Cdev::writeByte(ICM20602_DEV_ADDR,ICM20602_ACCEL_CONFIG_2,0x03);           //Average 8 samples   44.8HZ             

}

void calculate_ICM_error(){
    unsigned short times = 200;             //采样次数
    for(int i=0;i<times;i++)
    {
        Read_ICM_raw(&ax, &ay, &az, &gx, &gy, &gz); //读取六轴原始数值
        axo += ax; ayo += ay; azo += az;      //采样和
        gxo += gx; gyo += gy; gzo += gz;
    
    }
    
    axo /= times; ayo /= times; azo /= times; //计算加速度计偏移
    gxo /= times; gyo /= times; gzo /= times; //计算陀螺仪偏移
}

void Read_ICM_raw(int16_t* ax_t, int16_t* ay_t, int16_t* az_t, int16_t* gx_t, int16_t* gy_t, int16_t* gz_t){
    uint8_t acc_dat[6];
    I2Cdev::readBytes(ICM20602_DEV_ADDR, ICM20602_ACCEL_XOUT_H, 6,acc_dat); 
    //读取六轴原始数值
    *ax_t = ((int16_t)(acc_dat[0]<<8) | acc_dat[1]);
    *ay_t = ((int16_t)(acc_dat[2]<<8 | acc_dat[3]));
    *az_t = ((int16_t)(acc_dat[4]<<8 | acc_dat[5]));

    uint8_t gyro_dat[6];
    I2Cdev::readBytes(ICM20602_DEV_ADDR, ICM20602_GYRO_XOUT_H, 6,gyro_dat);  
    *gx_t = ((int16_t)(gyro_dat[0]<<8 | gyro_dat[1]));
    *gy_t = ((int16_t)(gyro_dat[2]<<8 | gyro_dat[3]));
    *gz_t = ((int16_t)(gyro_dat[4]<<8 | gyro_dat[5]));
}

// float K_com = 0.23; // 互补滤波
// void Read_ICM(){
//     Read_ICM_raw(&ax, &ay, &az, &gx, &gy, &gz); //读取六轴原始数值
//     float accx = ax / AcceRatio;              //x轴加速度
//     float accy = ay / AcceRatio;              //y轴加速度
//     float accz = az / AcceRatio;              //z轴加速度

//     aax = atan(accy / accz) * (-57.3);//(-180) / pi;    //y轴对于z轴的夹角，角度制
//     // aay = atan(accx / accz) * 57.3;   //   180 / pi;    //x轴对于z轴的夹角，角度制
//     // aaz = atan(accz / accy) * 57.3;   //   180 / pi;    //z轴对于y轴的夹角，角度制

//     agx += - (gx-gxo) / GyroRatio * elapsedTime; //角速度积分 单位：度
//     // agy += - (gy-gyo) / GyroRatio * elapsedTime; //角速度积分 单位：度
//     // agz += - (gz-gzo) / GyroRatio * elapsedTime; //角速度积分 单位：度

//     roll_new = K_com * aax +(1 - K_com) * agx;
// }

// 卡尔曼滤波
void Read_IMU()
{
    Read_ICM_raw(&ax, &ay, &az, &gx, &gy, &gz); //读取六轴原始数值

    float accx = ax / AcceRatio;              //x轴加速度
    float accy = ay / AcceRatio;              //y轴加速度
    float accz = az / AcceRatio;              //z轴加速度

    aax = atan(accy / accz) * (-180) / pi;    //y轴对于z轴的夹角
    aay = atan(accx / accz) * 180 / pi;       //x轴对于z轴的夹角
    aaz = atan(accz / accy) * 180 / pi;       //z轴对于y轴的夹角

    aax_sum = 0;                              // 对于加速度计原始数据的滑动加权滤波算法
    aay_sum = 0;
    aaz_sum = 0;
  
    for(int i=1;i<n_sample;i++)
    {
        aaxs[i-1] = aaxs[i];
        aax_sum += aaxs[i] * i;
        aays[i-1] = aays[i];
        aay_sum += aays[i] * i;
        aazs[i-1] = aazs[i];
        aaz_sum += aazs[i] * i;    
    }
    
    aaxs[n_sample-1] = aax;
    aax_sum += aax * n_sample;
    aax = (aax_sum / (11*n_sample/2.0)) * 9 / 7.0; //角度调幅至0-90°
    aays[n_sample-1] = aay;                        //此处应用实验法取得合适的系数
    aay_sum += aay * n_sample;                     //本例系数为9/7
    aay = (aay_sum / (11*n_sample/2.0)) * 9 / 7.0;
    aazs[n_sample-1] = aaz; 
    aaz_sum += aaz * n_sample;
    aaz = (aaz_sum / (11*n_sample/2.0)) * 9 / 7.0;

    float gyrox = - (gx-gxo) / GyroRatio * elapsedTime; //x轴角速度
    float gyroy = - (gy-gyo) / GyroRatio * elapsedTime; //y轴角速度
    float gyroz = - (gz-gzo) / GyroRatio * elapsedTime; //z轴角速度
    agx += gyrox;                             //x轴角速度积分
    agy += gyroy;                             //x轴角速度积分
    agz += gyroz;
    
    /* kalman start */
    Sx = 0; Rx = 0;
    Sy = 0; Ry = 0;
    Sz = 0; Rz = 0;
    
    for(int i=1;i<10;i++)
    {                 //测量值平均值运算
        a_x[i-1] = a_x[i];                      //即加速度平均值
        Sx += a_x[i];
        a_y[i-1] = a_y[i];
        Sy += a_y[i];
        a_z[i-1] = a_z[i];
        Sz += a_z[i];    
    }
    
    a_x[9] = aax;
    Sx += aax;
    Sx /= 10;                                 //x轴加速度平均值
    a_y[9] = aay;
    Sy += aay;
    Sy /= 10;                                 //y轴加速度平均值
    a_z[9] = aaz;
    Sz += aaz;
    Sz /= 10;

    for(int i=0;i<10;i++)
    {
        Rx += sq(a_x[i] - Sx);
        Ry += sq(a_y[i] - Sy);
        Rz += sq(a_z[i] - Sz);    
    }
    
    Rx = Rx / 9;                              //得到方差
    Ry = Ry / 9;                        
    Rz = Rz / 9;
  
    Px = Px + 0.0025;                         // 0.0025在下面有说明...
    Kx = Px / (Px + Rx);                      //计算卡尔曼增益
    agx = agx + Kx * (aax - agx);             //陀螺仪角度与加速度计速度叠加
    Px = (1 - Kx) * Px;                       //更新p值

    Py = Py + 0.0025;
    Ky = Py / (Py + Ry);
    agy = agy + Ky * (aay - agy); 
    Py = (1 - Ky) * Py;
  
    Pz = Pz + 0.0025;
    Kz = Pz / (Pz + Rz);
    agz = agz + Kz * (aaz - agz); 
    Pz = (1 - Kz) * Pz;

    /* kalman end */
    roll = agx;
    pitch = agy;
    yaw = agz;
}

#else

#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 accelgyro;

int16_t ax, ay, az, gx, gy, gz;                             //加速度计陀螺仪原始数据
float aax = 0, aay = 0, aaz = 0, agx = 0, agy = 0, agz = 0; //角度变量
long axo = 0, ayo = 0, azo = 0;                             //加速度计偏移量
long gxo = 0, gyo = 0, gzo = 0;                             //陀螺仪偏移量

float pi = 3.1415926;
float AcceRatio = 16384.0; //加速度计比例系数
float GyroRatio = 131.0;   //陀螺仪比例系数

uint8_t n_sample = 8;                              //加速度计滤波算法采样个数
float aaxs[8] = {0}, aays[8] = {0}, aazs[8] = {0}; //x,y轴采样队列
long aax_sum, aay_sum, aaz_sum;                    //x,y轴采样和

float a_x[10] = {0}, a_y[10] = {0}, a_z[10] = {0}, g_x[10] = {0}, g_y[10] = {0}, g_z[10] = {0}; //加速度计协方差计算队列
float Px = 1, Rx, Kx, Sx, Vx, Qx;                                                               //x轴卡尔曼变量
float Py = 1, Ry, Ky, Sy, Vy, Qy;                                                               //y轴卡尔曼变量
float Pz = 1, Rz, Kz, Sz, Vz, Qz;                                                               //z轴卡尔曼变量

void init_IMU()
{
    //陀螺仪初始化
    accelgyro.initialize();

    delay(100);
    calculate_IMU_error();
    delay(100);
}

void calculate_IMU_error()
{
    unsigned short times = 200; //采样次数
    for (int i = 0; i < times; i++)
    {
        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); //读取六轴原始数值
        axo += ax;
        ayo += ay;
        azo += az; //采样和
        gxo += gx;
        gyo += gy;
        gzo += gz;
    }

    axo /= times;
    ayo /= times;
    azo /= times; //计算加速度计偏移
    gxo /= times;
    gyo /= times;
    gzo /= times; //计算陀螺仪偏移
}

void Read_IMU()
{
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); //读取六轴原始数值

    float accx = ax / AcceRatio; //x轴加速度
    float accy = ay / AcceRatio; //y轴加速度
    float accz = az / AcceRatio; //z轴加速度

    aax = atan(accy / accz) * (-180) / pi; //y轴对于z轴的夹角
    aay = atan(accx / accz) * 180 / pi;    //x轴对于z轴的夹角
    aaz = atan(accz / accy) * 180 / pi;    //z轴对于y轴的夹角

    aax_sum = 0; // 对于加速度计原始数据的滑动加权滤波算法
    aay_sum = 0;
    aaz_sum = 0;

    for (int i = 1; i < n_sample; i++)
    {
        aaxs[i - 1] = aaxs[i];
        aax_sum += aaxs[i] * i;
        aays[i - 1] = aays[i];
        aay_sum += aays[i] * i;
        aazs[i - 1] = aazs[i];
        aaz_sum += aazs[i] * i;
    }

    aaxs[n_sample - 1] = aax;
    aax_sum += aax * n_sample;
    aax = (aax_sum / (11 * n_sample / 2.0)) * 9 / 7.0; //角度调幅至0-90°
    aays[n_sample - 1] = aay;                          //此处应用实验法取得合适的系数
    aay_sum += aay * n_sample;                         //本例系数为9/7
    aay = (aay_sum / (11 * n_sample / 2.0)) * 9 / 7.0;
    aazs[n_sample - 1] = aaz;
    aaz_sum += aaz * n_sample;
    aaz = (aaz_sum / (11 * n_sample / 2.0)) * 9 / 7.0;

    float gyrox = -(gx - gxo) / GyroRatio * elapsedTime; //x轴角速度
    float gyroy = -(gy - gyo) / GyroRatio * elapsedTime; //y轴角速度
    float gyroz = -(gz - gzo) / GyroRatio * elapsedTime; //z轴角速度
    agx += gyrox;                                        //x轴角速度积分
    agy += gyroy;                                        //x轴角速度积分
    agz += gyroz;

    /* kalman start */
    Sx = 0;
    Rx = 0;
    Sy = 0;
    Ry = 0;
    Sz = 0;
    Rz = 0;

    for (int i = 1; i < 10; i++)
    {                        //测量值平均值运算
        a_x[i - 1] = a_x[i]; //即加速度平均值
        Sx += a_x[i];
        a_y[i - 1] = a_y[i];
        Sy += a_y[i];
        a_z[i - 1] = a_z[i];
        Sz += a_z[i];
    }

    a_x[9] = aax;
    Sx += aax;
    Sx /= 10; //x轴加速度平均值
    a_y[9] = aay;
    Sy += aay;
    Sy /= 10; //y轴加速度平均值
    a_z[9] = aaz;
    Sz += aaz;
    Sz /= 10;

    for (int i = 0; i < 10; i++)
    {
        Rx += sq(a_x[i] - Sx);
        Ry += sq(a_y[i] - Sy);
        Rz += sq(a_z[i] - Sz);
    }

    Rx = Rx / 9; //得到方差
    Ry = Ry / 9;
    Rz = Rz / 9;

    Px = Px + 0.0025;             // 0.0025在下面有说明...
    Kx = Px / (Px + Rx);          //计算卡尔曼增益
    agx = agx + Kx * (aax - agx); //陀螺仪角度与加速度计速度叠加
    Px = (1 - Kx) * Px;           //更新p值

    Py = Py + 0.0025;
    Ky = Py / (Py + Ry);
    agy = agy + Ky * (aay - agy);
    Py = (1 - Ky) * Py;

    Pz = Pz + 0.0025;
    Kz = Pz / (Pz + Rz);
    agz = agz + Kz * (aaz - agz);
    Pz = (1 - Kz) * Pz;

    /* kalman end */
    roll = agx;
    pitch = agy;
    yaw = agz;
}

#endif