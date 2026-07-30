#include "zdt_can_port.h"

#include <stddef.h>
#include <string.h>

#define ZDT_CAN_TX_BUFFER_INDEX       (0U)
#define ZDT_CAN_FRAME_DATA_SIZE       (8U)
#define ZDT_CAN_WAIT_LIMIT            (100000UL)

static volatile ZDT_CAN_Status g_zdt_can_last_status = ZDT_CAN_OK;
static volatile uint8_t g_zdt_can_last_error_code;
static volatile uint8_t g_zdt_can_tx_error_count;
static volatile uint8_t g_zdt_can_bus_off_status;
static volatile int32_t g_zdt_motor_position[3];
static volatile uint32_t g_zdt_motor_position_sequence[3];
static int32_t g_zdt_motor_minimum[3];
static int32_t g_zdt_motor_maximum[3];
static bool g_zdt_motor_limits_valid[3];

static void ZDT_CAN_CaptureDiagnostics(void)
{
    DL_MCAN_ProtocolStatus protocol_status;
    DL_MCAN_ErrCntStatus error_counters;

    DL_MCAN_getProtocolStatus(MCAN0_INST, &protocol_status);
    DL_MCAN_getErrCounters(MCAN0_INST, &error_counters);
    g_zdt_can_last_error_code =
        (uint8_t)protocol_status.lastErrCode;
    g_zdt_can_tx_error_count =
        (uint8_t)error_counters.transErrLogCnt;
    g_zdt_can_bus_off_status =
        (uint8_t)protocol_status.busOffStatus;
}

static bool ZDT_CAN_EnsureNormalMode(void)
{
    uint32_t wait_count;

    if (DL_MCAN_getOpMode(MCAN0_INST) ==
        DL_MCAN_OPERATION_MODE_NORMAL)
    {
        return true;
    }

    /*
     * A missing ACK can drive the controller bus-off and set INIT again.
     * Request NORMAL mode before the next diagnostic transmission.
     */
    DL_MCAN_setOpMode(MCAN0_INST, DL_MCAN_OPERATION_MODE_NORMAL);
    wait_count = ZDT_CAN_WAIT_LIMIT;
    while (DL_MCAN_getOpMode(MCAN0_INST) !=
           DL_MCAN_OPERATION_MODE_NORMAL)
    {
        if (--wait_count == 0U)
        {
            return false;
        }
    }
    return true;
}

static bool ZDT_CAN_ClearPendingTx(void)
{
    uint32_t wait_count;
    uint32_t buffer_mask = (1UL << ZDT_CAN_TX_BUFFER_INDEX);

    if ((DL_MCAN_getTxBufReqPend(MCAN0_INST) & buffer_mask) == 0U)
    {
        return true;
    }

    (void)DL_MCAN_txBufCancellationReq(
        MCAN0_INST, ZDT_CAN_TX_BUFFER_INDEX);
    wait_count = ZDT_CAN_WAIT_LIMIT;
    while ((DL_MCAN_getTxBufReqPend(MCAN0_INST) & buffer_mask) != 0U)
    {
        if (--wait_count == 0U)
        {
            return false;
        }
    }
    return true;
}

static ZDT_CAN_Status ZDT_CAN_SendFrame(uint32_t identifier,
                                       volatile uint8_t *data,
                                       uint8_t length)
{
    DL_MCAN_TxBufElement tx_element;
    uint32_t wait_count;
    uint8_t index;

    if ((data == NULL) || (length == 0U) ||
        (length > ZDT_CAN_FRAME_DATA_SIZE))
    {
        return ZDT_CAN_INVALID_ARGUMENT;
    }
    if (!ZDT_CAN_EnsureNormalMode())
    {
        return ZDT_CAN_NOT_READY;
    }

    /*
     * A frame sent while the motor is still powering up can remain pending
     * because classic CAN retries automatically when nobody acknowledges it.
     * Cancel that stale request before loading the next motor command.
     */
    if (!ZDT_CAN_ClearPendingTx())
    {
        return ZDT_CAN_TX_BUSY;
    }

    (void)memset(&tx_element, 0, sizeof(tx_element));
    tx_element.id  = identifier & 0x1FFFFFFFUL;
    tx_element.rtr = 0U;
    tx_element.xtd = 1U;
    tx_element.esi = 0U;
    tx_element.dlc = length;
    tx_element.brs = 0U;
    tx_element.fdf = 0U;
    tx_element.efc = 0U;
    tx_element.mm  = 0U;
    for (index = 0U; index < length; index++)
    {
        tx_element.data[index] = data[index];
    }

    DL_MCAN_writeMsgRam(MCAN0_INST, DL_MCAN_MEM_TYPE_BUF,
                        ZDT_CAN_TX_BUFFER_INDEX, &tx_element);
    if (DL_MCAN_TXBufAddReq(MCAN0_INST, ZDT_CAN_TX_BUFFER_INDEX) != 0)
    {
        return ZDT_CAN_TX_ERROR;
    }

    wait_count = ZDT_CAN_WAIT_LIMIT;
    while ((DL_MCAN_getTxBufReqPend(MCAN0_INST) &
            (1UL << ZDT_CAN_TX_BUFFER_INDEX)) != 0U)
    {
        if (--wait_count == 0U)
        {
            ZDT_CAN_CaptureDiagnostics();
            (void)ZDT_CAN_ClearPendingTx();
            return ZDT_CAN_TX_BUSY;
        }
    }

    return ZDT_CAN_OK;
}

void can_SendCmd(volatile uint8_t *cmd, uint16_t len)
{
    uint8_t frame[ZDT_CAN_FRAME_DATA_SIZE];
    uint16_t payload_length;
    uint16_t source_index;
    uint8_t packet_index;
    uint8_t frame_length;
    ZDT_CAN_Status status;

    if ((cmd == NULL) || (len < 2U))
    {
        g_zdt_can_last_status = ZDT_CAN_INVALID_ARGUMENT;
        return;
    }

    /*
     * Global velocity-mode limit guard. Direction 0 is clockwise/increasing
     * position; direction 1 is counter-clockwise/decreasing position.
     * Before the power-on reference has been captured, motion is inhibited.
     */
    if ((cmd[0] < 3U) && (cmd[1] == 0xF6U) && (len >= 9U))
    {
        uint8_t address = cmd[0];
        bool motion_allowed =
            g_zdt_motor_limits_valid[address] &&
            (g_zdt_motor_position_sequence[address] != 0U);

        if (motion_allowed)
        {
            if ((cmd[2] == 0U) &&
                (g_zdt_motor_position[address] >=
                 g_zdt_motor_maximum[address]))
            {
                motion_allowed = false;
            }
            else if ((cmd[2] != 0U) &&
                     (g_zdt_motor_position[address] <=
                      g_zdt_motor_minimum[address]))
            {
                motion_allowed = false;
            }
        }

        if (!motion_allowed)
        {
            cmd[5] = 0U;
            cmd[6] = 0U;
        }
    }

    /*
     * Match the official ZDT CAN example exactly:
     *   ID = (address << 8) | packet_number
     *   data[0] = function code in every packet
     *   data[1..7] = at most seven new command bytes
     *
     * len includes address and function code, so payload_length excludes
     * both. source_index addresses the first byte following the function.
     */
    payload_length = len - 2U;
    source_index = 0U;
    packet_index = 0U;
    do
    {
        frame[0] = cmd[1];
        frame_length = 1U;
        while ((frame_length < ZDT_CAN_FRAME_DATA_SIZE) &&
               (source_index < payload_length))
        {
            frame[frame_length++] = cmd[source_index + 2U];
            source_index++;
        }

        status = ZDT_CAN_SendFrame((((uint32_t)cmd[0]) << 8U) |
                                      packet_index,
                                  frame, frame_length);
        if (status != ZDT_CAN_OK)
        {
            g_zdt_can_last_status = status;
            return;
        }
        packet_index++;
    } while (source_index < payload_length);

    g_zdt_can_last_status = ZDT_CAN_OK;
    ZDT_CAN_CaptureDiagnostics();
}

ZDT_CAN_Status ZDT_CAN_GetLastStatus(void)
{
    return g_zdt_can_last_status;
}

uint8_t ZDT_CAN_GetLastErrorCode(void)
{
    return g_zdt_can_last_error_code;
}

uint8_t ZDT_CAN_GetTxErrorCount(void)
{
    return g_zdt_can_tx_error_count;
}

uint8_t ZDT_CAN_GetBusOffStatus(void)
{
    return g_zdt_can_bus_off_status;
}

void ZDT_CAN_PollRx(void)
{
    DL_MCAN_RxFIFOStatus fifo_status;
    DL_MCAN_RxBufElement rx_element;

    fifo_status.num = DL_MCAN_RX_FIFO_NUM_0;
    for (;;)
    {
        uint8_t address;
        uint32_t magnitude;

        DL_MCAN_getRxFIFOStatus(MCAN0_INST, &fifo_status);
        if (fifo_status.fillLvl == 0U)
        {
            break;
        }

        DL_MCAN_readMsgRam(
            MCAN0_INST, DL_MCAN_MEM_TYPE_FIFO, 0U,
            fifo_status.num, &rx_element);
        DL_MCAN_writeRxFIFOAck(
            MCAN0_INST, fifo_status.num, fifo_status.getIdx);

        address = (uint8_t)(rx_element.id >> 8U);
        if ((rx_element.xtd != 0U) && (address < 3U) &&
            (rx_element.dlc >= 7U) &&
            (rx_element.data[0] == 0x36U) &&
            (rx_element.data[6] == 0x6BU))
        {
            magnitude =
                ((uint32_t)rx_element.data[2] << 24U) |
                ((uint32_t)rx_element.data[3] << 16U) |
                ((uint32_t)rx_element.data[4] << 8U) |
                (uint32_t)rx_element.data[5];
            g_zdt_motor_position[address] =
                (rx_element.data[1] != 0U) ?
                -(int32_t)magnitude : (int32_t)magnitude;
            g_zdt_motor_position_sequence[address]++;
        }
    }
}

bool ZDT_CAN_GetMotorPosition(
    uint8_t address, int32_t *position_tenth_degree, uint32_t *sequence)
{
    if ((address >= 3U) || (position_tenth_degree == NULL) ||
        (sequence == NULL) ||
        (g_zdt_motor_position_sequence[address] == 0U))
    {
        return false;
    }

    *position_tenth_degree = g_zdt_motor_position[address];
    *sequence = g_zdt_motor_position_sequence[address];
    return true;
}

void ZDT_CAN_SetMotorLimits(
    uint8_t address, int32_t minimum_position, int32_t maximum_position)
{
    if ((address >= 3U) || (minimum_position >= maximum_position))
    {
        return;
    }
    g_zdt_motor_minimum[address] = minimum_position;
    g_zdt_motor_maximum[address] = maximum_position;
    g_zdt_motor_limits_valid[address] = true;
}

bool ZDT_CAN_GetMotorLimits(
    uint8_t address, int32_t *minimum_position, int32_t *maximum_position)
{
    if ((address >= 3U) || (minimum_position == NULL) ||
        (maximum_position == NULL) ||
        !g_zdt_motor_limits_valid[address])
    {
        return false;
    }
    *minimum_position = g_zdt_motor_minimum[address];
    *maximum_position = g_zdt_motor_maximum[address];
    return true;
}
