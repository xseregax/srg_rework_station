//
// Created by srg on 10/5/21.
//

#pragma once

#include <lvgl.h>
#include "ST7789VW.h"

//pixel count, buffer split to 3 parts or TFT_HOR_RES*10 rows
#define LV_DISPLAY_BUFFER TFT_HOR_RES*TFT_VER_RES/3
// 1 or 2 buffers
#define LV_DISPLAY_BUFFER_COUNT 1


void lv_display_init(void);

void lv_display_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_p);


