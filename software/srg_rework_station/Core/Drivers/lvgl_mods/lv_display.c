
#include "lv_display.h"


void lv_display_init(void)
{
    static lv_disp_draw_buf_t draw_buf_dsc; //Create a buffer for drawing
    static lv_color_t buf_1[LV_DISPLAY_BUFFER];

#if LV_DISPLAY_BUFFER_COUNT == 1
    lv_disp_draw_buf_init(&draw_buf_dsc, buf_1, NULL, LV_DISPLAY_BUFFER);   /*Initialize display buffer*/
#else
    static lv_color_t buf_2[LV_DISPLAY_BUFFER];
    lv_disp_draw_buf_init(&draw_buf_dsc, buf_1, buf_2,  LV_DISPLAY_BUFFER);   /*Initialize two display buffers*/
#endif

    static lv_disp_drv_t disp_drv; //Register the display in LVGL
    lv_disp_drv_init(&disp_drv);

    //Set the resolution of the display
    disp_drv.hor_res = TFT_HOR_RES;
    disp_drv.ver_res = TFT_VER_RES;

    disp_drv.draw_buf = &draw_buf_dsc;
    disp_drv.flush_cb = lv_display_flush;

    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);
    lv_disp_set_default(disp);
}


//transmit display buffer to st7789vw
void lv_display_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p) {
    if (area->x2 < 0) return;
    if (area->y2 < 0) return;
    if (area->x1 > TFT_HOR_RES - 1) return;
    if (area->y1 > TFT_VER_RES - 1) return;

    ST7789VW_ShowBuffer(area->x1, area->y1, area->x2, area->y2, (uint8_t *) color_p);

    /*IMPORTANT!!!
     *Inform the graphics library that you are ready with the flushing*/
    lv_disp_flush_ready(disp_drv);


}


