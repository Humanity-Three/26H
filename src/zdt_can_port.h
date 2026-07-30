#ifndef ZDT_CAN_PORT_H
#define ZDT_CAN_PORT_H

#include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    ZDT_CAN_OK = 0,
    ZDT_CAN_INVALID_ARGUMENT,
    ZDT_CAN_NOT_READY,
    ZDT_CAN_TX_BUSY,
    ZDT_CAN_TX_ERROR
} ZDT_CAN_Status;

/*
 * Compatibility transport used by the vendor X_V2 library. The input is a
 * serial-format command (address + payload + checksum); this function maps it
 * to ZDT 29-bit extended CAN frames and fragments long commands.
 */
void can_SendCmd(volatile uint8_t *cmd, uint16_t len);

ZDT_CAN_Status ZDT_CAN_GetLastStatus(void);
uint8_t ZDT_CAN_GetLastErrorCode(void);
uint8_t ZDT_CAN_GetTxErrorCount(void);
uint8_t ZDT_CAN_GetBusOffStatus(void);
void ZDT_CAN_PollRx(void);
bool ZDT_CAN_GetMotorPosition(
    uint8_t address, int32_t *position_tenth_degree, uint32_t *sequence);
void ZDT_CAN_SetMotorLimits(
    uint8_t address, int32_t minimum_position, int32_t maximum_position);
bool ZDT_CAN_GetMotorLimits(
    uint8_t address, int32_t *minimum_position, int32_t *maximum_position);

#endif
