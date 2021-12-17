#pragma once

#include <sys/_stdint.h>

#define SETTING_VERSION 0x01
#define SETTING_MAGIC 0x53524700

#define SETTING_MAKE_SIGNATURE(magic, version) ((magic) | ((version) & 0xFF))

#define SETTING_GET_MAGIC(signature) ((signature) & 0xFFFFFF00)
#define SETTING_GET_VERSION(signature) ((signature) & 0xFF)

#define SETTING_SIGNATURE SETTING_MAKE_SIGNATURE(SETTING_MAGIC, SETTING_VERSION)


// Error codes
#define SETTING_OK            ((uint8_t)0x00)
#define SETTING_ERROR         ((uint8_t)0x01)

//__attribute__((__packed__))
typedef struct  {
    uint32_t signature;


    // last member
    uint32_t crc;
} SETTING_t;

uint8_t setting_init();
uint8_t setting_load();
uint8_t setting_save();
uint8_t setting_reset();
