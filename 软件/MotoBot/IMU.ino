/*
 * Author: Xuechao Zhang
 * Date: Sept 19th, 2020
 * Description: MPU6050 使用DMP模式 去除了外部中断
 */
 
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"

MPU6050 mpu;

#define OVERFLOW_WARNING

//#define OUTPUT_READABLE_QUATERNION // 返回四元数
//#define OUTPUT_READABLE_EULER // 返回欧拉角
#define OUTPUT_READABLE_YAWPITCHROLL // 返回YPR角度
//#define OUTPUT_READABLE_REALACCEL // 返回加速度 去掉重力
//#define OUTPUT_READABLE_WORLDACCEL // 返回加速度 去掉重力 补偿旋转

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


// INTERRUPT DETECTION ROUTINE
volatile bool mpuInterrupt = true; //去除中断 false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



void init_IMU(){
  // 设备初始化 initialize device 
    mpu.initialize();

    // 确认连接 verify connection 
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    //mpu.setXGyroOffset(51);
    //mpu.setYGyroOffset(8);
    //mpu.setZGyroOffset(21);
    //mpu.setXAccelOffset(1150); 
    //mpu.setYAccelOffset(-50); 
    //mpu.setZAccelOffset(1060); 

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // 校准 Calibration Time: generate offsets and calibrate our MPU6050
        //mpu.CalibrateAccel(6);
        //mpu.CalibrateGyro(6);
        //Serial.println();
        //mpu.PrintActiveOffsets();
        // 开启DMP turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

       mpuIntStatus = mpu.getIntStatus();

        // 准备就绪标志
        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // 错误！ ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
        while(1);  //别走了      
    }
}

void Read_IMU(){
    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        if (mpuInterrupt && fifoCount < packetSize) {
          // try to get out of the infinite loop 
          fifoCount = mpu.getFIFOCount();
        }  
    }
    
    // 重置中断flag 获取INT_STATUS
    // reset interrupt flag and get INT_STATUS byte
    // mpuInterrupt = false; //去除中断
    mpuIntStatus = mpu.getIntStatus();
    
    // 获取FIFO存储器计数
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
    
    // 检查溢出 一般不会
    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & (0x01 << MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) { // 这一行就没怎么懂
        // reset so we can continue cleanly
        mpu.resetFIFO();
        fifoCount = mpu.getFIFOCount();
        #ifdef OVERFLOW_WARNING
          Serial.println(F("FIFO overflow!"));
        #endif 
        
    // 不溢出 就检查DMP的就绪中断 应该非常频繁
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    //} else if (mpuIntStatus & (0x01 << MPU6050_INTERRUPT_DMP_INT_BIT)) { //这一行改了
    } else {
        // 等待一个有效长度 应该很快
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        
        // 一个完整的包后 读出来
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // 如果有多余一个的有效包 然后啥都不做？
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            Serial.print("quat\t");
            Serial.print(q.w);
            Serial.print("\t");
            Serial.print(q.x);
            Serial.print("\t");
            Serial.print(q.y);
            Serial.print("\t");
            Serial.println(q.z);
        #endif

        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            Serial.print("euler\t");
            Serial.print(euler[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(euler[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(euler[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//            Serial.print("ypr\t");
//            Serial.print(ypr[0] * 180/M_PI);
//            Serial.print("\t");
//            Serial.print(ypr[1] * 180/M_PI);
//            Serial.print("\t");
//            Serial.print(ypr[2] * 180/M_PI);
            yaw = ypr[0] * 180/M_PI;
            pitch = ypr[1] * 180/M_PI;
            roll = ypr[2] * 180/M_PI;
            /*
            mpu.dmpGetAccel(&aa, fifoBuffer);
            Serial.print("\tRaw Accl XYZ\t");
            Serial.print(aa.x);
            Serial.print("\t");
            Serial.print(aa.y);
            Serial.print("\t");
            Serial.print(aa.z);
            mpu.dmpGetGyro(&gy, fifoBuffer);
            Serial.print("\tRaw Gyro XYZ\t");
            Serial.print(gy.x);
            Serial.print("\t");
            Serial.print(gy.y);
            Serial.print("\t");
            Serial.print(gy.z);
            */
            //Serial.println();

        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            Serial.print("areal\t");
            Serial.print(aaReal.x);
            Serial.print("\t");
            Serial.print(aaReal.y);
            Serial.print("\t");
            Serial.println(aaReal.z);
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            Serial.print("aworld\t");
            Serial.print(aaWorld.x);
            Serial.print("\t");
            Serial.print(aaWorld.y);
            Serial.print("\t");
            Serial.println(aaWorld.z);
        #endif
    }

//    roll = agx;
//    pitch = agy;
//    yaw = agz;
//    Serial.print(agx);Serial.print(",");
//    Serial.print(agy);Serial.print(",");
//    Serial.print(agz);Serial.println();
}
