#include "srs_setting.h"
#include "srs_crc.h"

SETTING_t setting = {0};

uint32_t setting_crc()
{
    //uint16_t size = sizeof(SETTING_t) - sizeof(uint32_t); // for packed
    uint16_t size = (uint32_t *)&setting.crc - (uint32_t *)&setting;
    return calculate_crc(&setting, size);
}


uint8_t setting_init()
{
    setting.signature = SETTING_SIGNATURE;


    setting.crc = setting_crc();

    return SETTING_OK;
}

uint8_t setting_load()
{

    return SETTING_OK;
}

uint8_t setting_save()
{

    return SETTING_OK;
}

uint8_t setting_reset()
{

    return SETTING_OK;
}
