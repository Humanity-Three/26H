#ifndef __GRAYSCALE_SENSOR_H
#define __GRAYSCALE_SENSOR_H

#include <stdint.h>
#include "ti_msp_dl_config.h"

#define GRAYSCALE_SENSOR_CHANNELS   8U

void Grayscale_Sensor_Init(void);
void Grayscale_Sensor_Update10ms(void);
uint16_t Grayscale_Sensor_Read_Single(uint8_t channel);
void Grayscale_Sensor_Read_All(uint16_t *sensor_values);
uint8_t Grayscale_Sensor_BuildMask(const uint16_t *sensor_values);
bool Grayscale_Sensor_IsValid(void);
uint8_t Grayscale_Sensor_GetMask(void);
uint32_t Grayscale_Sensor_GetRxByteCount(void);
uint32_t Grayscale_Sensor_GetFrameCount(void);
uint8_t Grayscale_Sensor_GetLastFrameLength(void);
char Grayscale_Sensor_GetLastFrameChar(uint8_t index);
uint8_t Grayscale_Sensor_GetRecentByte(uint8_t index);
uint8_t Grayscale_Sensor_GetRecentByteCount(void);

#endif
