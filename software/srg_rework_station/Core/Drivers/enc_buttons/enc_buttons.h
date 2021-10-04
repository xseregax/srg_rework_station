
/**
 * @file enc_buttons.h
 *
 */

#ifndef ENC_BUTTONS_H
#define ENC_BUTTONS_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#include "lvgl.h"
#include "tim.h"

/*********************
 *      DEFINES
 *********************/
#define ENCODER_PORT &htim8

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/
void lv_port_indev_init(void);
void lv_port_encoder_handler(void);


/**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*ENC_BUTTONS_H*/
