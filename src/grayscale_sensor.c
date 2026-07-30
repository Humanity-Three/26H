#include "grayscale_sensor.h"
#include "Delay.h"

static uint16_t Grayscale_Normalize(uint16_t raw_value)
{
#if GRAYSCALE_BLACK_LEVEL
    return raw_value ? 1U : 0U;
#else
    return raw_value ? 0U : 1U;
#endif
}

void Grayscale_Sensor_Init(void)
{
    SENSOR_AD0_WRITE(0U);
    SENSOR_AD1_WRITE(0U);
    SENSOR_AD2_WRITE(0U);
    Delay_ms(10);
}

uint16_t Grayscale_Sensor_Read_Single(uint8_t channel)
{
    SENSOR_AD0_WRITE((channel >> 0) & 0x01U);
    SENSOR_AD1_WRITE((channel >> 1) & 0x01U);
    SENSOR_AD2_WRITE((channel >> 2) & 0x01U);

    /*
     * Keep the same mux settling time as the proven 25E implementation.
     * The sensor board and its output conditioning need this interval after
     * changing AD0..AD2 before DATA is sampled.
     */
    Delay_ms(1);

    return Grayscale_Normalize(SENSOR_OUT_READ_RAW());
}

void Grayscale_Sensor_Read_All(uint16_t *sensor_values)
{
    uint8_t i;

    if (sensor_values == 0) {
        return;
    }

    for (i = 0U; i < GRAYSCALE_SENSOR_CHANNELS; i++) {
        sensor_values[i] = Grayscale_Sensor_Read_Single(i);
    }
}

uint8_t Grayscale_Sensor_BuildMask(const uint16_t *sensor_values)
{
    uint8_t i;
    uint8_t mask = 0U;

    if (sensor_values == 0) {
        return 0U;
    }

    for (i = 0U; i < GRAYSCALE_SENSOR_CHANNELS; i++) {
        if (sensor_values[i] != 0U) {
            mask |= (uint8_t)(1U << i);
        }
    }

    return mask;
}
