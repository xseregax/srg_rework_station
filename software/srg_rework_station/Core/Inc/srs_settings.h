#pragma once

#include <sys/_stdint.h>

typedef struct {
    uint8_t version;



    uint16_t crc;
} srs_settings_t;

void srs_settings_load();
void srs_settings_save();
void srs_settings_reset();