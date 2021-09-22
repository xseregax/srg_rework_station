/**
 * @file st7789.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "st7789.h"

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/


/**********************
 *  STATIC VARIABLES
 **********************/

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void lv_port_disp_init(void)
{
  /*-------------------------
   * Initialize your display
   * -----------------------*/
  ST7789_Init();

  /*-----------------------------
   * Create a buffer for drawing
   *----------------------------*/

  /**
   * LVGL requires a buffer where it internally draws the widgets.
   * Later this buffer will passed to your display driver's `flush_cb` to copy its content to your display.
   * The buffer has to be greater than 1 display row
   *
   * There are 3 buffering configurations:
   * 1. Create ONE buffer:
   *      LVGL will draw the display's content here and writes it to your display
   *
   * 2. Create TWO buffer:
   *      LVGL will draw the display's content to a buffer and writes it your display.
   *      You should use DMA to write the buffer's content to the display.
   *      It will enable LVGL to draw the next part of the screen to the other buffer while
   *      the data is being sent form the first buffer. It makes rendering and flushing parallel.
   */

  /* Example for 1) */
  static lv_disp_draw_buf_t draw_buf_dsc_1;
  static lv_color_t buf_1[TFT_HOR_RES * 10];                          /*A buffer for 10 rows*/
  lv_disp_draw_buf_init(&draw_buf_dsc_1, buf_1, NULL, TFT_HOR_RES * 10);   /*Initialize the display buffer*/

  //    /* Example for 2) */
  //    static lv_disp_draw_buf_t draw_buf_dsc_2;
  //    static lv_color_t buf_2_1[MY_DISP_HOR_RES * 10];                        /*A buffer for 10 rows*/
  //    static lv_color_t buf_2_1[MY_DISP_HOR_RES * 10];                        /*An other buffer for 10 rows*/
  //    lv_disp_draw_buf_init(&draw_buf_dsc_2, buf_2_1, buf_2_1, MY_DISP_HOR_RES * 10);   /*Initialize the display buffer*/

  /*-----------------------------------
   * Register the display in LVGL
   *----------------------------------*/

  static lv_disp_drv_t disp_drv;                         /*Descriptor of a display driver*/
  lv_disp_drv_init(&disp_drv);                    /*Basic initialization*/

  /*Set up the functions to access to your display*/

  /*Set the resolution of the display*/
  disp_drv.hor_res = TFT_HOR_RES;
  disp_drv.ver_res = TFT_VER_RES;

  /*Used to copy the buffer's content to the display*/
  disp_drv.flush_cb = lv_port_disp_flush;

  /*Set a display buffer*/
  disp_drv.draw_buf = &draw_buf_dsc_1;

  /*Finally register the driver*/
  lv_disp_drv_register(&disp_drv);
}

/*Flush the content of the internal buffer the specific area on the display
 *You can use DMA or any hardware acceleration to do this operation in the background but
 *'lv_disp_flush_ready()' has to be called when finished.*/
void lv_port_disp_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

  ST7789_SetAddressWindow(area->x1, area->y1, area->x2, area->y2);

  ST7789_Select();
  ST7789_DC_Set();

  HAL_SPI_Transmit_DMA(&ST7789_SPI_PORT, (unsigned char*)color_p, w*h*2);
  ST7789_UnSelect();


  /*IMPORTANT!!!
   *Inform the graphics library that you are ready with the flushing*/
  lv_disp_flush_ready(disp_drv);
}


/*Initialize your display and the required peripherals.*/
void ST7789_Init(void)
{
  HAL_Delay(25);
  ST7789_RST_Clr();
  HAL_Delay(25);
  ST7789_RST_Set();
  HAL_Delay(50);

  ST7789_WriteCommand(ST7789_COLMOD);     //  Set color mode
  ST7789_WriteSmallData(ST7789_COLOR_MODE_16bit);
  ST7789_WriteCommand(0xB2);              //  Porch control
  {
      uint8_t data[] = {0x0C, 0x0C, 0x00, 0x33, 0x33};
      ST7789_WriteData(data, sizeof(data));
  }
  ST7789_SetRotation(ST7789_ROTATION);    //  MADCTL (Display Rotation)

  /* Internal LCD Voltage generator settings */
  ST7789_WriteCommand(0XB7);              //  Gate Control
  ST7789_WriteSmallData(0x35);            //  Default value
  ST7789_WriteCommand(0xBB);              //  VCOM setting
  ST7789_WriteSmallData(0x19);            //  0.725v (default 0.75v for 0x20)
  ST7789_WriteCommand(0xC0);              //  LCMCTRL
  ST7789_WriteSmallData (0x2C);           //  Default value
  ST7789_WriteCommand (0xC2);             //  VDV and VRH command Enable
  ST7789_WriteSmallData (0x01);           //  Default value
  ST7789_WriteCommand (0xC3);             //  VRH set
  ST7789_WriteSmallData (0x12);           //  +-4.45v (defalut +-4.1v for 0x0B)
  ST7789_WriteCommand (0xC4);             //  VDV set
  ST7789_WriteSmallData (0x20);           //  Default value
  ST7789_WriteCommand (0xC6);             //  Frame rate control in normal mode
  ST7789_WriteSmallData (0x0F);           //  Default value (60HZ)
  ST7789_WriteCommand (0xD0);             //  Power control
  ST7789_WriteSmallData (0xA4);           //  Default value
  ST7789_WriteSmallData (0xA1);           //  Default value
  /**************** Division line ****************/

  ST7789_WriteCommand(0xE0);
  {
      uint8_t data[] = {0xD0, 0x04, 0x0D, 0x11, 0x13, 0x2B, 0x3F, 0x54, 0x4C, 0x18, 0x0D, 0x0B, 0x1F, 0x23};
      ST7789_WriteData(data, sizeof(data));
  }

  ST7789_WriteCommand(0xE1);
  {
      uint8_t data[] = {0xD0, 0x04, 0x0C, 0x11, 0x13, 0x2C, 0x3F, 0x44, 0x51, 0x2F, 0x1F, 0x1F, 0x20, 0x23};
      ST7789_WriteData(data, sizeof(data));
  }
  ST7789_WriteCommand (ST7789_INVON);     //  Inversion ON
  ST7789_WriteCommand (ST7789_SLPOUT);    //  Out of sleep mode
  ST7789_WriteCommand (ST7789_NORON);     //  Normal Display on
  ST7789_WriteCommand (ST7789_DISPON);    //  Main screen turned on
}


/**
 * @brief Write command to ST7789 controller
 * @param cmd -> command to write
 * @return none
 */
void ST7789_WriteCommand(uint8_t cmd)
{
  ST7789_Select();
  ST7789_DC_Clr();
  HAL_SPI_Transmit(&ST7789_SPI_PORT, &cmd, sizeof(cmd), HAL_MAX_DELAY);
  ST7789_UnSelect();
}

/**
 * @brief Write data to ST7789 controller
 * @param buff -> pointer of data buffer
 * @param buff_size -> size of the data buffer
 * @return none
 */
void ST7789_WriteData(uint8_t *buff, size_t buff_size)
{
  ST7789_Select();
  ST7789_DC_Set();

  // split data in small chunks because HAL can't send more than 64K at once

  while (buff_size > 0) {
      uint16_t chunk_size = buff_size > 65535 ? 65535 : buff_size;
      HAL_SPI_Transmit(&ST7789_SPI_PORT, buff, chunk_size, HAL_MAX_DELAY);
      buff += chunk_size;
      buff_size -= chunk_size;
  }

  ST7789_UnSelect();
}
/**
 * @brief Write data to ST7789 controller, simplify for 8bit data.
 * data -> data to write
 * @return none
 */
void ST7789_WriteSmallData(uint8_t data)
{
  ST7789_Select();
  ST7789_DC_Set();
  HAL_SPI_Transmit(&ST7789_SPI_PORT, &data, sizeof(data), HAL_MAX_DELAY);
  ST7789_UnSelect();
}

/**
 * @brief Set the rotation direction of the display
 * @param m -> rotation parameter(please refer it in st7789.h)
 * @return none
 */
void ST7789_SetRotation(uint8_t m)
{
  ST7789_WriteCommand(ST7789_MADCTL); // MADCTL
  switch (m) {
    case 0:
        ST7789_WriteSmallData(ST7789_MADCTL_MX | ST7789_MADCTL_MY | ST7789_MADCTL_RGB);
        break;
    case 1:
        ST7789_WriteSmallData(ST7789_MADCTL_MY | ST7789_MADCTL_MV | ST7789_MADCTL_RGB);
        break;
    case 2:
        ST7789_WriteSmallData(ST7789_MADCTL_RGB);
        break;
    case 3:
        ST7789_WriteSmallData(ST7789_MADCTL_MX | ST7789_MADCTL_MV | ST7789_MADCTL_RGB);
        break;
    default:
        break;
  }
}

/**
 * @brief Set address of DisplayWindow
 * @param xi&yi -> coordinates of window
 * @return none
 */
void ST7789_SetAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
  ST7789_Select();
  uint16_t x_start = x0 + X_SHIFT, x_end = x1 + X_SHIFT;
  uint16_t y_start = y0 + Y_SHIFT, y_end = y1 + Y_SHIFT;

  /* Column Address set */
  ST7789_WriteCommand(ST7789_CASET);
  {
      uint8_t data[] = {x_start >> 8, x_start & 0xFF, x_end >> 8, x_end & 0xFF};
      ST7789_WriteData(data, sizeof(data));
  }

  /* Row Address set */
  ST7789_WriteCommand(ST7789_RASET);
  {
      uint8_t data[] = {y_start >> 8, y_start & 0xFF, y_end >> 8, y_end & 0xFF};
      ST7789_WriteData(data, sizeof(data));
  }
  /* Write to RAM */
  ST7789_WriteCommand(ST7789_RAMWR);
  ST7789_UnSelect();
}

/**
 * @brief Invert Fullscreen color
 * @param invert -> Whether to invert
 * @return none
 */
void ST7789_InvertColors(uint8_t invert)
{
    ST7789_Select();
    ST7789_WriteCommand(invert ? 0x21 /* INVON */ : 0x20 /* INVOFF */);
    ST7789_UnSelect();
}


/**
 * @brief Open/Close tearing effect line
 * @param tear -> Whether to tear
 * @return none
 */
void ST7789_TearEffect(uint8_t tear)
{
    ST7789_Select();
    ST7789_WriteCommand(tear ? 0x35 /* TEON */ : 0x34 /* TEOFF */);
    ST7789_UnSelect();
}

