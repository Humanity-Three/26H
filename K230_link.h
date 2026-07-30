#ifndef K230_LINK_H
#define K230_LINK_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    int16_t dx;
    int16_t dy;
    bool aligned;
    bool detected;
    uint32_t age_ms;
    uint32_t frame_count;
} K230_LinkData;

void K230_Link_Init(void);
void K230_Link_Update10ms(void);
const K230_LinkData *K230_Link_GetData(void);
bool K230_Link_IsValid(uint32_t timeout_ms);
uint32_t K230_Link_GetRxByteCount(void);

#endif
