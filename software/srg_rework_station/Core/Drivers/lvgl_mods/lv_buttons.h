
#pragma once

#include "lvgl.h"
#include "stm32f7xx_hal.h"
#include <cmsis_os.h>

typedef struct {
    // GPIO
    TIM_HandleTypeDef *timEncoderHandle;

    lv_indev_t *indev_encoder;
    lv_indev_t *indev_button;

    int32_t encoder_diff;
    lv_indev_state_t encoder_state;

    int32_t encoder_value_curr;
    int32_t encoder_value_prev;

} LV_BUTTONS;


void lv_buttons_init(TIM_HandleTypeDef *timEncoderHandle);

void lv_buttons_encoder_handler(void);
