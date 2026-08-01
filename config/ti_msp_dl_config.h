/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)


#define CPUCLK_FREQ                                                     80000000



/* Defines for MOTOR_PWM */
#define MOTOR_PWM_INST                                                     TIMG0
#define MOTOR_PWM_INST_IRQHandler                               TIMG0_IRQHandler
#define MOTOR_PWM_INST_INT_IRQN                                 (TIMG0_INT_IRQn)
#define MOTOR_PWM_INST_CLK_FREQ                                         40000000
/* GPIO defines for channel 0 */
#define GPIO_MOTOR_PWM_C0_PORT                                             GPIOA
#define GPIO_MOTOR_PWM_C0_PIN                                     DL_GPIO_PIN_12
#define GPIO_MOTOR_PWM_C0_IOMUX                                  (IOMUX_PINCM34)
#define GPIO_MOTOR_PWM_C0_IOMUX_FUNC                 IOMUX_PINCM34_PF_TIMG0_CCP0
#define GPIO_MOTOR_PWM_C0_IDX                                DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_MOTOR_PWM_C1_PORT                                             GPIOA
#define GPIO_MOTOR_PWM_C1_PIN                                     DL_GPIO_PIN_13
#define GPIO_MOTOR_PWM_C1_IOMUX                                  (IOMUX_PINCM35)
#define GPIO_MOTOR_PWM_C1_IOMUX_FUNC                 IOMUX_PINCM35_PF_TIMG0_CCP1
#define GPIO_MOTOR_PWM_C1_IDX                                DL_TIMER_CC_1_INDEX



/* Defines for TIMER_0 */
#define TIMER_0_INST                                                     (TIMG7)
#define TIMER_0_INST_IRQHandler                                 TIMG7_IRQHandler
#define TIMER_0_INST_INT_IRQN                                   (TIMG7_INT_IRQn)
#define TIMER_0_INST_LOAD_VALUE                                          (1562U)




/* Defines for OLED_I2C */
#define OLED_I2C_INST                                                       I2C0
#define OLED_I2C_INST_IRQHandler                                 I2C0_IRQHandler
#define OLED_I2C_INST_INT_IRQN                                     I2C0_INT_IRQn
#define OLED_I2C_BUS_SPEED_HZ                                             500000
#define GPIO_OLED_I2C_SDA_PORT                                             GPIOA
#define GPIO_OLED_I2C_SDA_PIN                                      DL_GPIO_PIN_0
#define GPIO_OLED_I2C_IOMUX_SDA                                   (IOMUX_PINCM1)
#define GPIO_OLED_I2C_IOMUX_SDA_FUNC                    IOMUX_PINCM1_PF_I2C0_SDA
#define GPIO_OLED_I2C_SCL_PORT                                             GPIOA
#define GPIO_OLED_I2C_SCL_PIN                                      DL_GPIO_PIN_1
#define GPIO_OLED_I2C_IOMUX_SCL                                   (IOMUX_PINCM2)
#define GPIO_OLED_I2C_IOMUX_SCL_FUNC                    IOMUX_PINCM2_PF_I2C0_SCL


/* Defines for K230_UART */
#define K230_UART_INST                                                     UART0
#define K230_UART_INST_FREQUENCY                                        80000000
#define K230_UART_INST_IRQHandler                               UART0_IRQHandler
#define K230_UART_INST_INT_IRQN                                   UART0_INT_IRQn
#define GPIO_K230_UART_RX_PORT                                             GPIOA
#define GPIO_K230_UART_TX_PORT                                             GPIOA
#define GPIO_K230_UART_RX_PIN                                     DL_GPIO_PIN_31
#define GPIO_K230_UART_TX_PIN                                     DL_GPIO_PIN_28
#define GPIO_K230_UART_IOMUX_RX                                   (IOMUX_PINCM6)
#define GPIO_K230_UART_IOMUX_TX                                   (IOMUX_PINCM3)
#define GPIO_K230_UART_IOMUX_RX_FUNC                    IOMUX_PINCM6_PF_UART0_RX
#define GPIO_K230_UART_IOMUX_TX_FUNC                    IOMUX_PINCM3_PF_UART0_TX
#define K230_UART_BAUD_RATE                                             (115200)
#define K230_UART_IBRD_80_MHZ_115200_BAUD                                   (43)
#define K230_UART_FBRD_80_MHZ_115200_BAUD                                   (26)





/* Port definition for Pin Group BEEP_IO */
#define BEEP_IO_PORT                                                     (GPIOA)

/* Defines for BEEP: GPIOA.18 with pinCMx 40 on package pin 11 */
#define BEEP_IO_BEEP_PIN                                        (DL_GPIO_PIN_18)
#define BEEP_IO_BEEP_IOMUX                                       (IOMUX_PINCM40)
/* Port definition for Pin Group GRAY_SENSOR */
#define GRAY_SENSOR_PORT                                                 (GPIOA)

/* Defines for GRAY_SENSOR_AD1: GPIOA.15 with pinCMx 37 on package pin 8 */
#define GRAY_SENSOR_GRAY_SENSOR_AD1_PIN                         (DL_GPIO_PIN_15)
#define GRAY_SENSOR_GRAY_SENSOR_AD1_IOMUX                        (IOMUX_PINCM37)
/* Defines for GRAY_SENSOR_AD2: GPIOA.16 with pinCMx 38 on package pin 9 */
#define GRAY_SENSOR_GRAY_SENSOR_AD2_PIN                         (DL_GPIO_PIN_16)
#define GRAY_SENSOR_GRAY_SENSOR_AD2_IOMUX                        (IOMUX_PINCM38)
/* Defines for GRAY_SENSOR_AD0: GPIOA.14 with pinCMx 36 on package pin 7 */
#define GRAY_SENSOR_GRAY_SENSOR_AD0_PIN                         (DL_GPIO_PIN_14)
#define GRAY_SENSOR_GRAY_SENSOR_AD0_IOMUX                        (IOMUX_PINCM36)
/* Defines for GRAY_SENSOR_DATA: GPIOA.17 with pinCMx 39 on package pin 10 */
#define GRAY_SENSOR_GRAY_SENSOR_DATA_PIN                        (DL_GPIO_PIN_17)
#define GRAY_SENSOR_GRAY_SENSOR_DATA_IOMUX                       (IOMUX_PINCM39)
/* Port definition for Pin Group KEY */
#define KEY_PORT                                                         (GPIOB)

/* Defines for KEY_1: GPIOB.6 with pinCMx 23 on package pin 58 */
#define KEY_KEY_1_PIN                                            (DL_GPIO_PIN_6)
#define KEY_KEY_1_IOMUX                                          (IOMUX_PINCM23)
/* Defines for KEY_2: GPIOB.7 with pinCMx 24 on package pin 59 */
#define KEY_KEY_2_PIN                                            (DL_GPIO_PIN_7)
#define KEY_KEY_2_IOMUX                                          (IOMUX_PINCM24)
/* Defines for KEY_3: GPIOB.8 with pinCMx 25 on package pin 60 */
#define KEY_KEY_3_PIN                                            (DL_GPIO_PIN_8)
#define KEY_KEY_3_IOMUX                                          (IOMUX_PINCM25)
/* Defines for KEY_4: GPIOB.9 with pinCMx 26 on package pin 61 */
#define KEY_KEY_4_PIN                                            (DL_GPIO_PIN_9)
#define KEY_KEY_4_IOMUX                                          (IOMUX_PINCM26)
/* Port definition for Pin Group MOTOR */
#define MOTOR_PORT                                                       (GPIOB)

/* Defines for STBY: GPIOB.13 with pinCMx 30 on package pin 1 */
#define MOTOR_STBY_PIN                                          (DL_GPIO_PIN_13)
#define MOTOR_STBY_IOMUX                                         (IOMUX_PINCM30)
/* Defines for AIN_1: GPIOB.15 with pinCMx 32 on package pin 3 */
#define MOTOR_AIN_1_PIN                                         (DL_GPIO_PIN_15)
#define MOTOR_AIN_1_IOMUX                                        (IOMUX_PINCM32)
/* Defines for AIN_2: GPIOB.16 with pinCMx 33 on package pin 4 */
#define MOTOR_AIN_2_PIN                                         (DL_GPIO_PIN_16)
#define MOTOR_AIN_2_IOMUX                                        (IOMUX_PINCM33)
/* Defines for BIN_1: GPIOB.2 with pinCMx 15 on package pin 50 */
#define MOTOR_BIN_1_PIN                                          (DL_GPIO_PIN_2)
#define MOTOR_BIN_1_IOMUX                                        (IOMUX_PINCM15)
/* Defines for BIN_2: GPIOB.3 with pinCMx 16 on package pin 51 */
#define MOTOR_BIN_2_PIN                                          (DL_GPIO_PIN_3)
#define MOTOR_BIN_2_IOMUX                                        (IOMUX_PINCM16)
/* Port definition for Pin Group ENCODER_READ */
#define ENCODER_READ_PORT                                                (GPIOB)

/* Defines for ENCODER_B1: GPIOB.26 with pinCMx 57 on package pin 28 */
#define ENCODER_READ_ENCODER_B1_PIN                             (DL_GPIO_PIN_26)
#define ENCODER_READ_ENCODER_B1_IOMUX                            (IOMUX_PINCM57)
/* Defines for ENCODER_A1: GPIOB.23 with pinCMx 51 on package pin 22 */
#define ENCODER_READ_ENCODER_A1_PIN                             (DL_GPIO_PIN_23)
#define ENCODER_READ_ENCODER_A1_IOMUX                            (IOMUX_PINCM51)
/* Defines for ENCODER_B2: GPIOB.24 with pinCMx 52 on package pin 23 */
#define ENCODER_READ_ENCODER_B2_PIN                             (DL_GPIO_PIN_24)
#define ENCODER_READ_ENCODER_B2_IOMUX                            (IOMUX_PINCM52)
/* Defines for ENCODER_A2: GPIOB.27 with pinCMx 58 on package pin 29 */
#define ENCODER_READ_ENCODER_A2_PIN                             (DL_GPIO_PIN_27)
#define ENCODER_READ_ENCODER_A2_IOMUX                            (IOMUX_PINCM58)
/* Port definition for Pin Group MPU6050_I2C */
#define MPU6050_I2C_PORT                                                 (GPIOA)

/* Defines for SDA: GPIOA.10 with pinCMx 21 on package pin 56 */
#define MPU6050_I2C_SDA_PIN                                     (DL_GPIO_PIN_10)
#define MPU6050_I2C_SDA_IOMUX                                    (IOMUX_PINCM21)
/* Defines for SCL: GPIOA.11 with pinCMx 22 on package pin 57 */
#define MPU6050_I2C_SCL_PIN                                     (DL_GPIO_PIN_11)
#define MPU6050_I2C_SCL_IOMUX                                    (IOMUX_PINCM22)


/* Defines for MCAN0 */
#define MCAN0_INST                                                        CANFD0
#define GPIO_MCAN0_CAN_TX_PORT                                             GPIOA
#define GPIO_MCAN0_CAN_TX_PIN                                     DL_GPIO_PIN_26
#define GPIO_MCAN0_IOMUX_CAN_TX                                  (IOMUX_PINCM59)
#define GPIO_MCAN0_IOMUX_CAN_TX_FUNC               IOMUX_PINCM59_PF_CANFD0_CANTX
#define GPIO_MCAN0_CAN_RX_PORT                                             GPIOA
#define GPIO_MCAN0_CAN_RX_PIN                                     DL_GPIO_PIN_27
#define GPIO_MCAN0_IOMUX_CAN_RX                                  (IOMUX_PINCM60)
#define GPIO_MCAN0_IOMUX_CAN_RX_FUNC               IOMUX_PINCM60_PF_CANFD0_CANRX


/* Defines for MCAN0 MCAN RAM configuration */
#define MCAN0_INST_MCAN_STD_ID_FILT_START_ADDR     (0)
#define MCAN0_INST_MCAN_STD_ID_FILTER_NUM          (0)
#define MCAN0_INST_MCAN_EXT_ID_FILT_START_ADDR     (0)
#define MCAN0_INST_MCAN_EXT_ID_FILTER_NUM          (1)
#define MCAN0_INST_MCAN_TX_BUFF_START_ADDR         (72)
#define MCAN0_INST_MCAN_TX_BUFF_SIZE               (1)
#define MCAN0_INST_MCAN_FIFO_1_START_ADDR          (192)
#define MCAN0_INST_MCAN_FIFO_1_NUM                 (0)
#define MCAN0_INST_MCAN_TX_EVENT_START_ADDR        (88)
#define MCAN0_INST_MCAN_TX_EVENT_SIZE              (1)
#define MCAN0_INST_MCAN_EXT_ID_AND_MASK            (0x1FFFFFFFU)
#define MCAN0_INST_MCAN_RX_BUFF_START_ADDR         (92)
#define MCAN0_INST_MCAN_FIFO_0_START_ADDR          (8)
#define MCAN0_INST_MCAN_FIFO_0_NUM                 (4)





/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_SYSCTL_CLK_init(void);
void SYSCFG_DL_MOTOR_PWM_init(void);
void SYSCFG_DL_TIMER_0_init(void);
void SYSCFG_DL_OLED_I2C_init(void);
void SYSCFG_DL_K230_UART_init(void);

void SYSCFG_DL_MCAN0_init(void);

bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
