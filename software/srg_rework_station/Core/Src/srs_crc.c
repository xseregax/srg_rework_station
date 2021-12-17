
#include "srs_crc.h"
#include "stm32f7xx_hal.h"

extern CRC_HandleTypeDef hcrc;

uint32_t calculate_crc(void *buffer, uint32_t size)
{
    // InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES
    return HAL_CRC_Calculate(&hcrc, (uint32_t *) buffer, size);
}
