#include "MPU6050.h"
#include "ti_msp_dl_config.h"
#include "Delay.h"
#include <math.h>

extern volatile uint32_t g_sys_tick_10ms;

// 定义原始数据变量
int16_t accel_x_raw, accel_y_raw, accel_z_raw;
int16_t gyro_x_raw, gyro_y_raw, gyro_z_raw;
int16_t temp_raw;

// 定义实际物理量
float accel_x, accel_y, accel_z;
float gyro_x, gyro_y, gyro_z;
float temp;

// 姿态角
float Yaw = 0;
float Pitch = 0;
float Roll = 0;

// 互补滤波参数
#define GYRO_SCALE 131.0f      // 陀螺仪量程±250°/s
#define ACCEL_SCALE 16384.0f   // 加速度计量程±2g
#define DT 0.01f               // 采样周期 10ms
#define ALPHA 0.98f            // 互补滤波系数

// 陀螺仪零漂校准参数
#define GYRO_CALIB_SAMPLES 200 // 校准采样次数
static float gyro_x_offset = 0;
static float gyro_y_offset = 0;
static float gyro_z_offset = 0;

// 通过加速度计计算的角度
float accel_pitch, accel_roll;

// 中断保护宏（注释掉以避免长时间关中断导致主循环饿死）
// #define I2C_ENTER_CRITICAL()    __asm(" CPSID I ")
// #define I2C_EXIT_CRITICAL()     __asm(" CPSIE I ")
#define I2C_ENTER_CRITICAL()
#define I2C_EXIT_CRITICAL()

//=====================================================================
// 软件模拟 I2C（参考嘉立创官方示例）
// SDA = PA0, SCL = PA1
//=====================================================================

#define IIC_SDA_PORT    GPIOA
#define IIC_SDA_PIN     DL_GPIO_PIN_0
#define IIC_SCL_PORT    GPIOA
#define IIC_SCL_PIN     DL_GPIO_PIN_1

static void SDA_OUT(void)
{
    DL_GPIO_initDigitalOutput(IOMUX_PINCM1);
}

static void SDA_IN(void)
{
    DL_GPIO_initDigitalInput(IOMUX_PINCM1);
}

static uint8_t SDA_GET(void)
{
    return (DL_GPIO_readPins(IIC_SDA_PORT, IIC_SDA_PIN) & IIC_SDA_PIN) ? 1 : 0;
}

static void SDA_SET(uint8_t x)
{
    if (x)
        DL_GPIO_setPins(IIC_SDA_PORT, IIC_SDA_PIN);
    else
        DL_GPIO_clearPins(IIC_SDA_PORT, IIC_SDA_PIN);
}

static void SCL_SET(uint8_t x)
{
    if (x)
        DL_GPIO_setPins(IIC_SCL_PORT, IIC_SCL_PIN);
    else
        DL_GPIO_clearPins(IIC_SCL_PORT, IIC_SCL_PIN);
}

static void IIC_Delay(void)
{
    Delay_us(5);
}

static void IIC_Start(void)
{
    SDA_OUT();
    SCL_SET(1);
    SDA_SET(1);
    IIC_Delay();
    SDA_SET(0);
    IIC_Delay();
    SCL_SET(0);
}

static void IIC_Stop(void)
{
    SDA_OUT();
    SCL_SET(0);
    SDA_SET(0);
    IIC_Delay();
    SCL_SET(1);
    IIC_Delay();
    SDA_SET(1);
    IIC_Delay();
}

static uint8_t IIC_WaitAck(void)
{
    uint8_t ack_flag = 10;
    SCL_SET(0);
    SDA_SET(1);
    SDA_IN();
    IIC_Delay();
    SCL_SET(1);
    IIC_Delay();
    while (SDA_GET() && ack_flag) {
        ack_flag--;
        IIC_Delay();
    }
    SCL_SET(0);
    SDA_OUT();
    if (ack_flag == 0) {
        IIC_Stop();
        return 1;
    }
    return 0;
}

static void IIC_SendAck(uint8_t ack)
{
    SDA_OUT();
    SCL_SET(0);
    SDA_SET(ack ? 1 : 0);
    IIC_Delay();
    SCL_SET(1);
    IIC_Delay();
    SCL_SET(0);
    SDA_SET(1);
}

static void Send_Byte(uint8_t dat)
{
    int i;
    SDA_OUT();
    SCL_SET(0);
    for (i = 0; i < 8; i++) {
        SDA_SET((dat & 0x80) >> 7);
        IIC_Delay();
        SCL_SET(1);
        IIC_Delay();
        SCL_SET(0);
        IIC_Delay();
        dat <<= 1;
    }
}

static uint8_t Read_Byte(void)
{
    uint8_t i, receive = 0;
    SDA_IN();
    for (i = 0; i < 8; i++) {
        SCL_SET(0);
        IIC_Delay();
        SCL_SET(1);
        IIC_Delay();
        receive <<= 1;
        if (SDA_GET()) {
            receive |= 1;
        }
        IIC_Delay();
    }
    SCL_SET(0);
    return receive;
}

//=====================================================================
// MPU6050 I2C 读写
//=====================================================================

static uint8_t MPU6050_WriteReg(uint8_t addr, uint8_t regaddr, uint8_t num, uint8_t *regdata)
{
    uint16_t i;
    I2C_ENTER_CRITICAL();
    IIC_Start();
    Send_Byte((addr << 1) | 0);
    if (IIC_WaitAck() == 1) { IIC_Stop(); I2C_EXIT_CRITICAL(); return 1; }
    Send_Byte(regaddr);
    if (IIC_WaitAck() == 1) { IIC_Stop(); I2C_EXIT_CRITICAL(); return 2; }
    for (i = 0; i < num; i++) {
        Send_Byte(regdata[i]);
        if (IIC_WaitAck() == 1) { IIC_Stop(); I2C_EXIT_CRITICAL(); return (3 + i); }
    }
    IIC_Stop();
    I2C_EXIT_CRITICAL();
    return 0;
}

static uint8_t MPU6050_ReadData(uint8_t addr, uint8_t regaddr, uint8_t num, uint8_t *Read)
{
    uint8_t i;
    I2C_ENTER_CRITICAL();
    IIC_Start();
    Send_Byte((addr << 1) | 0);
    if (IIC_WaitAck() == 1) { IIC_Stop(); I2C_EXIT_CRITICAL(); return 1; }
    Send_Byte(regaddr);
    if (IIC_WaitAck() == 1) { IIC_Stop(); I2C_EXIT_CRITICAL(); return 2; }
    IIC_Start();
    Send_Byte((addr << 1) | 1);
    if (IIC_WaitAck() == 1) { IIC_Stop(); I2C_EXIT_CRITICAL(); return 3; }
    for (i = 0; i < (num - 1); i++) {
        Read[i] = Read_Byte();
        IIC_SendAck(0);
    }
    Read[i] = Read_Byte();
    IIC_SendAck(1);
    IIC_Stop();
    I2C_EXIT_CRITICAL();
    return 0;
}

void MPU6050_Write_Reg(uint8_t reg, uint8_t data)
{
    MPU6050_WriteReg(MPU6050_ADDR, reg, 1, &data);
}

uint8_t MPU6050_Read_Reg(uint8_t reg)
{
    uint8_t val = 0;
    MPU6050_ReadData(MPU6050_ADDR, reg, 1, &val);
    return val;
}

void MPU6050_Read_Burst(uint8_t reg, uint8_t *data, uint8_t len)
{
    MPU6050_ReadData(MPU6050_ADDR, reg, len, data);
}

static uint8_t MPU6050_Read_Burst_Retry(uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t retry;
    for(retry = 0; retry < 3; retry++)
    {
        if(MPU6050_ReadData(MPU6050_ADDR, reg, len, data) == 0)
            return 0;  // 成功
    }
    return 1;  // 失败
}

//=====================================================================
// 陀螺仪零漂校准
//=====================================================================

void MPU6050_Calibrate_Gyro(void)
{
    int i;
    int32_t sum_x = 0, sum_y = 0, sum_z = 0;
    int16_t raw_x, raw_y, raw_z;
    uint8_t buf[6];

    for (i = 0; i < GYRO_CALIB_SAMPLES; i++) {
        MPU6050_Read_Burst(MPU6050_GYRO_XOUT_H, buf, 6);
        raw_x = (buf[0] << 8) | buf[1];
        raw_y = (buf[2] << 8) | buf[3];
        raw_z = (buf[4] << 8) | buf[5];
        sum_x += raw_x;
        sum_y += raw_y;
        sum_z += raw_z;
        Delay_ms(5);
    }

    gyro_x_offset = (float)(sum_x / GYRO_CALIB_SAMPLES) / GYRO_SCALE;
    gyro_y_offset = (float)(sum_y / GYRO_CALIB_SAMPLES) / GYRO_SCALE;
    gyro_z_offset = (float)(sum_z / GYRO_CALIB_SAMPLES) / GYRO_SCALE;
}

//=====================================================================
// 初始化
//=====================================================================

uint8_t MPU6050_Init(void)
{
    uint8_t whoami;
    uint8_t reg_val;

    // 初始化 GPIO 为开漏输出
    SDA_OUT();
    SCL_SET(1);
    SDA_SET(1);
    Delay_ms(10);

    // 复位 MPU6050
    reg_val = 0x80;
    MPU6050_WriteReg(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 1, &reg_val);
    Delay_ms(100);

    // 唤醒：选择时钟源为 X 轴陀螺仪
    reg_val = 0x01;
    MPU6050_WriteReg(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 1, &reg_val);
    Delay_ms(10);

    // 验证通信
    whoami = MPU6050_Read_Reg(MPU6050_WHO_AM_I);
    if (whoami != 0x68 && whoami != 0x69) {
        return 0;
    }

    // 配置
    reg_val = 0x00;
    MPU6050_WriteReg(MPU6050_ADDR, MPU6050_SMPLRT_DIV, 1, &reg_val);
    reg_val = 0x03;
    MPU6050_WriteReg(MPU6050_ADDR, MPU6050_CONFIG, 1, &reg_val);
    reg_val = 0x00;
    MPU6050_WriteReg(MPU6050_ADDR, MPU6050_GYRO_CONFIG, 1, &reg_val);
    reg_val = 0x00;
    MPU6050_WriteReg(MPU6050_ADDR, MPU6050_ACCEL_CONFIG, 1, &reg_val);
    reg_val = 0x00;
    MPU6050_WriteReg(MPU6050_ADDR, MPU6050_PWR_MGMT_2, 1, &reg_val);

    Delay_ms(100);

    // 零漂校准
    MPU6050_Calibrate_Gyro();

    return 1;
}

//=====================================================================
// 数据读取
//=====================================================================

void MPU6050_Read_Accel(void)
{
    uint8_t buf[6];
    MPU6050_Read_Burst(MPU6050_ACCEL_XOUT_H, buf, 6);

    accel_x_raw = (buf[0] << 8) | buf[1];
    accel_y_raw = (buf[2] << 8) | buf[3];
    accel_z_raw = (buf[4] << 8) | buf[5];

    accel_x = (float)accel_x_raw / ACCEL_SCALE;
    accel_y = (float)accel_y_raw / ACCEL_SCALE;
    accel_z = (float)accel_z_raw / ACCEL_SCALE;
}

void MPU6050_Read_Gyro(void)
{
    uint8_t buf[6];
    MPU6050_Read_Burst(MPU6050_GYRO_XOUT_H, buf, 6);

    gyro_x_raw = (buf[0] << 8) | buf[1];
    gyro_y_raw = (buf[2] << 8) | buf[3];
    gyro_z_raw = (buf[4] << 8) | buf[5];

    gyro_x = (float)gyro_x_raw / GYRO_SCALE - gyro_x_offset;
    gyro_y = (float)gyro_y_raw / GYRO_SCALE - gyro_y_offset;
    gyro_z = (float)gyro_z_raw / GYRO_SCALE - gyro_z_offset;
}

void MPU6050_Read_Data(void)
{
    uint8_t buf[14] = {0};
    MPU6050_Read_Burst_Retry(MPU6050_ACCEL_XOUT_H, buf, 14);  //带重试的I2C读取

    accel_x_raw = (buf[0] << 8) | buf[1];
    accel_y_raw = (buf[2] << 8) | buf[3];
    accel_z_raw = (buf[4] << 8) | buf[5];

    temp_raw = (buf[6] << 8) | buf[7];
    temp = (float)temp_raw / 340.0f + 36.53f;

    gyro_x_raw = (buf[8] << 8) | buf[9];
    gyro_y_raw = (buf[10] << 8) | buf[11];
    gyro_z_raw = (buf[12] << 8) | buf[13];

    accel_x = (float)accel_x_raw / ACCEL_SCALE;
    accel_y = (float)accel_y_raw / ACCEL_SCALE;
    accel_z = (float)accel_z_raw / ACCEL_SCALE;

    gyro_x = (float)gyro_x_raw / GYRO_SCALE - gyro_x_offset;
    gyro_y = (float)gyro_y_raw / GYRO_SCALE - gyro_y_offset;
    gyro_z = (float)gyro_z_raw / GYRO_SCALE - gyro_z_offset;
}

//=====================================================================
// 姿态角计算（互补滤波）
//=====================================================================

void MPU6050_Get_Angle(void)
{
    static uint32_t last_tick = 0;
    uint32_t current_tick = g_sys_tick_10ms;
    float dt = (float)(current_tick - last_tick) * 0.01f;  // 实际时间间隔（秒）
    if(dt <= 0.0f || dt > 0.1f) dt = 0.01f;                // 合理性检查
    last_tick = current_tick;

    accel_pitch = atan2(accel_y, accel_z) * 180.0f / 3.1415926f;
    accel_roll = atan2(accel_x, sqrt(accel_y * accel_y + accel_z * accel_z)) * 180.0f / 3.1415926f;

    Pitch = ALPHA * (Pitch + gyro_y * dt) + (1.0f - ALPHA) * accel_pitch;
    Roll  = ALPHA * (Roll  - gyro_x * dt) + (1.0f - ALPHA) * accel_roll;

    // Yaw 积分 + 漂移抑制：静止时缓慢衰减漂移，转动时正常积分
    float delta = gyro_z * dt;
    if(delta > 2.0f)  delta = 2.0f;    // 限幅：单次变化不超过 2°，防止 I2C 错误突变
    if(delta < -2.0f) delta = -2.0f;

    if(fabsf(gyro_z) > 0.1f)
        Yaw += delta;             // 正在转动，正常积分
    else
        Yaw *= 0.998f;            // 静止时缓慢拉回零，抵消漂移

    if (Yaw > 180.0f)      Yaw -= 360.0f;
    else if (Yaw < -180.0f) Yaw += 360.0f;
}
