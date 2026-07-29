#ifndef __MPU6050_H
#define __MPU6050_H

#include <stdint.h>

// MPU6050 寄存器地址
#define MPU6050_ADDR        0x68
#define MPU6050_SMPLRT_DIV  0x19
#define MPU6050_CONFIG      0x1A
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_PWR_MGMT_1  0x6B
#define MPU6050_PWR_MGMT_2  0x6C
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_XOUT_L 0x3C
#define MPU6050_ACCEL_YOUT_H 0x3D
#define MPU6050_ACCEL_YOUT_L 0x3E
#define MPU6050_ACCEL_ZOUT_H 0x3F
#define MPU6050_ACCEL_ZOUT_L 0x40
#define MPU6050_TEMP_OUT_H  0x41
#define MPU6050_TEMP_OUT_L  0x42
#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_GYRO_XOUT_L 0x44
#define MPU6050_GYRO_YOUT_H 0x45
#define MPU6050_GYRO_YOUT_L 0x46
#define MPU6050_GYRO_ZOUT_H 0x47
#define MPU6050_GYRO_ZOUT_L 0x48
#define MPU6050_WHO_AM_I    0x75

// 外部变量声明
extern float Yaw;
extern float Pitch;
extern float Roll;

// 函数声明
uint8_t MPU6050_Init(void);
void MPU6050_Calibrate_Gyro(void);
void MPU6050_Read_Accel(void);
void MPU6050_Read_Gyro(void);
void MPU6050_Read_Data(void);
void MPU6050_Get_Angle(void);

// I2C 读写函数
void MPU6050_Write_Reg(uint8_t reg, uint8_t data);
uint8_t MPU6050_Read_Reg(uint8_t reg);
void MPU6050_Read_Burst(uint8_t reg, uint8_t *data, uint8_t len);

#endif
