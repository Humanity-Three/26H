#ifndef __GRAYSCALE_SENSOR_H
#define __GRAYSCALE_SENSOR_H

#include <stdint.h>
#include "ti_msp_dl_config.h"

#define GRAYSCALE_SENSOR_CHANNELS   8U

/*
 * The installed sensor returns high level on the black line.
 */
#define GRAYSCALE_BLACK_LEVEL       1U

#define SENSOR_AD0_PORT             GRAY_SENSOR_PORT
#define SENSOR_AD0_PIN              GRAY_SENSOR_GRAY_SENSOR_AD0_PIN
#define SENSOR_AD1_PORT             GRAY_SENSOR_PORT
#define SENSOR_AD1_PIN              GRAY_SENSOR_GRAY_SENSOR_AD1_PIN
#define SENSOR_AD2_PORT             GRAY_SENSOR_PORT
#define SENSOR_AD2_PIN              GRAY_SENSOR_GRAY_SENSOR_AD2_PIN
#define SENSOR_OUT_PORT             GRAY_SENSOR_PORT
#define SENSOR_OUT_PIN              GRAY_SENSOR_GRAY_SENSOR_DATA_PIN

#define GRAYSCALE_PIN_WRITE(port, pin, state) do { \
    if (state) { \
        DL_GPIO_setPins((port), (pin)); \
    } else { \
        DL_GPIO_clearPins((port), (pin)); \
    } \
} while (0)

#define SENSOR_AD0_WRITE(state)     GRAYSCALE_PIN_WRITE(SENSOR_AD0_PORT, SENSOR_AD0_PIN, (state))
#define SENSOR_AD1_WRITE(state)     GRAYSCALE_PIN_WRITE(SENSOR_AD1_PORT, SENSOR_AD1_PIN, (state))
#define SENSOR_AD2_WRITE(state)     GRAYSCALE_PIN_WRITE(SENSOR_AD2_PORT, SENSOR_AD2_PIN, (state))
#define SENSOR_OUT_READ_RAW()       (!!DL_GPIO_readPins(SENSOR_OUT_PORT, SENSOR_OUT_PIN))

void Grayscale_Sensor_Init(void);
uint16_t Grayscale_Sensor_Read_Single(uint8_t channel);
void Grayscale_Sensor_Read_All(uint16_t *sensor_values);
uint8_t Grayscale_Sensor_BuildMask(const uint16_t *sensor_values);

#endif
