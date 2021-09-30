/**
 * @file  ST7789VW.c
 *
 */

#include "ST7789VW.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "semphr.h"

extern osMutexId_t myMutexLcdHandle;

// Basic functions
void ST7789VW_Init(void);
void ST7789VW_SetRotation(uint8_t m);
void ST7789VW_WriteCommand(uint8_t cmd);
void ST7789VW_WriteData(uint8_t *buff, size_t buff_size);
void ST7789VW_WriteByteData(uint8_t data);


//init display and register in lvgl
void lv_port_disp_init(void)
{
  ST7789VW_Init(); //Initialize ST7789VW display

  static lv_disp_draw_buf_t draw_buf_dsc; //Create a buffer for drawing
  static lv_color_t buf_1[ ST7789VW_BUFFER];

#if  ST7789VW_BUFFER_COUNT == 1
  lv_disp_draw_buf_init(&draw_buf_dsc, buf_1, NULL,  ST7789VW_BUFFER);   /*Initialize display buffer*/
#else
  static lv_color_t buf_2[ ST7789VW_BUFFER];
  lv_disp_draw_buf_init(&draw_buf_dsc, buf_1, buf_2,  ST7789VW_BUFFER);   /*Initialize two display buffers*/
#endif

  static lv_disp_drv_t disp_drv; //Register the display in LVGL
  lv_disp_drv_init(&disp_drv);

  //Set the resolution of the display
  disp_drv.hor_res = TFT_HOR_RES;
  disp_drv.ver_res = TFT_VER_RES;

  disp_drv.draw_buf = &draw_buf_dsc;
  disp_drv.flush_cb = lv_port_disp_flush;

  lv_disp_drv_register(&disp_drv);
}

//transmit display buffer to st7789vw
void lv_port_disp_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
  xSemaphoreTake(myMutexLcdHandle, portMAX_DELAY);

  ST7789VW_Select();

  ST7789VW_WriteCommand(ST7789VW_CMD_CASET); //Column Address set
  {
    uint8_t data[] = {area->x1 >> 8, area->x1 & 0xFF, area->x2 >> 8, area->x2 & 0xFF};
    ST7789VW_WriteData(data, sizeof(data));
  }

  ST7789VW_WriteCommand(ST7789VW_CMD_RASET); //Row Address set
  {
    uint8_t data[] = {area->y1 >> 8, area->y1 & 0xFF, area->y2 >> 8, area->y2 & 0xFF};
    ST7789VW_WriteData(data, sizeof(data));
  }

  ST7789VW_WriteCommand(ST7789VW_CMD_RAMWR); //Write to RAM

  uint16_t w = (area->x2 - area->x1 + 1);
  uint16_t h = (area->y2 - area->y1 + 1);

  HAL_SPI_Transmit_DMA(&ST7789VW_SPI_PORT, (uint8_t *)color_p, w * h * 2);

  xSemaphoreTake(myMutexLcdHandle, portMAX_DELAY);

  ST7789VW_UnSelect();

  /*IMPORTANT!!!
   *Inform the graphics library that you are ready with the flushing*/
  lv_disp_flush_ready(disp_drv);

  xSemaphoreGive(myMutexLcdHandle);
}

void lv_port_disp_give_ISR(void)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(myMutexLcdHandle, &xHigherPriorityTaskWoken);

  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/*Initialize your display and the required peripherals.*/
void ST7789VW_Init(void)
{
  xSemaphoreTake(myMutexLcdHandle, portMAX_DELAY);

  HAL_Delay(25);

  ST7789VW_RST_Clr(); //Reset chip
  HAL_Delay(25);
  ST7789VW_RST_Set(); //Reset chip run
  HAL_Delay(50);

  ST7789VW_Select(); //Select ST7789VW chip
  ST7789VW_DC_Set(); //Set data mode

  ST7789VW_WriteCommand(ST7789VW_CMD_COLMOD); //Set color mode
  ST7789VW_WriteByteData(ST7789VW_COLMOD_16bit);

  ST7789VW_SetRotation(ST7789VW_ROTATION); //MADCTL (Display Rotation)

  ST7789VW_WriteCommand(ST7789VW_CMD_INVON); //Inversion ON

  ST7789VW_WriteCommand(ST7789VW_CMD_PORCTRL); //Porch control
  {
    uint8_t data[] = {0x0C, 0x0C, 0x00, 0x33, 0x33};
    ST7789VW_WriteData(data, sizeof(data));
  }

  ST7789VW_WriteCommand(ST7789VW_CMD_GCTRL); //Gate Control
  ST7789VW_WriteByteData(0x35); //Default value

  ST7789VW_WriteCommand(ST7789VW_CMD_VCOMS); //VCOM setting
  ST7789VW_WriteByteData(0x1f); //0x19: 0.725v, 1Fh: 0.875v

  ST7789VW_WriteCommand(ST7789VW_CMD_LCMCTRL); //LCMCTRL
  ST7789VW_WriteByteData(0x2C); //Default value

  ST7789VW_WriteCommand(ST7789VW_CMD_VDVVRHEN); //VDV and VRH command Enable
  ST7789VW_WriteByteData(0x01); //Default value

  ST7789VW_WriteCommand(ST7789VW_CMD_VRHS); //VRH set
  ST7789VW_WriteByteData(0x12); //4.45+( vcom+vcom offset+vdv)

  ST7789VW_WriteCommand(ST7789VW_CMD_VDVSET); //VDV set
  ST7789VW_WriteByteData(0x20); //Default value

  ST7789VW_WriteCommand(ST7789VW_CMD_FRCTR2); //Frame rate control in normal mode
  ST7789VW_WriteByteData(0x0F); //Default value (60HZ)

  ST7789VW_WriteCommand (ST7789VW_CMD_PWCTRL1); //Power control
  ST7789VW_WriteByteData (0xA4); //Default value
  ST7789VW_WriteByteData (0xA1); //Default value

//  ST7789VW_WriteCommand(ST7789VW_CMD_PVGAMCTRL); //Positive Voltage Gamma Control
//  {
//    //uint8_t data[] = {0xD0, 0x04, 0x0D, 0x11, 0x13, 0x2B, 0x3F, 0x54, 0x4C, 0x18, 0x0D, 0x0B, 0x1F, 0x23};
//    uint8_t data[] = {0xD0, 0x08, 0x11, 0x08, 0x0C, 0x15, 0x39, 0x33, 0x50, 0x36, 0x13, 0x14, 0x29, 0x2D};
//    ST7789VW_WriteData(data, sizeof(data));
//  }
//
//  ST7789VW_WriteCommand(ST7789VW_CMD_NVGAMCTRL); //Negative Voltage Gamma Control
//  {
//    //uint8_t data[] = {0xD0, 0x04, 0x0C, 0x11, 0x13, 0x2C, 0x3F, 0x44, 0x51, 0x2F, 0x1F, 0x1F, 0x20, 0x23};
//    uint8_t data[] = {0xD0, 0x08, 0x10, 0x08, 0x06, 0x06, 0x39, 0x44, 0x51, 0x0B, 0x16, 0x14, 0x2F, 0x31};
//     ST7789VW_WriteData(data, sizeof(data));
//  }

  //ST7789VW_WriteCommand(1 ? ST7789VW_CMD_TEON : ST7789VW_CMD_TEOFF); //Tearing effect line

  ST7789VW_WriteCommand(ST7789VW_CMD_SLPOUT); //Out of sleep mode
  ST7789VW_WriteCommand(ST7789VW_CMD_NORON); //Normal Display on
  ST7789VW_WriteCommand(ST7789VW_CMD_DISPON); //Main screen turned on

  ST7789VW_UnSelect();

  xSemaphoreGive(myMutexLcdHandle);
}


/**
 * @brief Write command to  ST7789VW controller
 * @param cmd -> command to write
 * @return none
 */
void ST7789VW_WriteCommand(uint8_t cmd)
{
   ST7789VW_DC_Clr(); //Set command mode
   HAL_SPI_Transmit(&ST7789VW_SPI_PORT, &cmd, sizeof(cmd), HAL_MAX_DELAY);
   ST7789VW_DC_Set(); //Restore data mode
}

/**
 * @brief Write data to  ST7789VW controller
 * @param buff -> pointer of data buffer
 * @param buff_size -> size of the data buffer
 * @return none
 */
void ST7789VW_WriteData(uint8_t *buff, size_t buff_size)
{
  // split data in small chunks because HAL can't send more than 64K at once

  while (buff_size > 0) {
      uint16_t chunk_size = buff_size > 65535 ? 65535 : buff_size;
      HAL_SPI_Transmit(&ST7789VW_SPI_PORT, buff, chunk_size, HAL_MAX_DELAY);
      buff += chunk_size;
      buff_size -= chunk_size;
  }

}
/**
 * @brief Write data to  ST7789VW controller, simplify for 8bit data.
 * data -> data to write
 * @return none
 */
void ST7789VW_WriteByteData(uint8_t data)
{
  HAL_SPI_Transmit(&ST7789VW_SPI_PORT, &data, sizeof(data), HAL_MAX_DELAY);
}

/**
 * @brief Set the rotation direction of the display
 * @param m -> rotation parameter(please refer it in  ST7789VW.h)
 * @return none
 */
void ST7789VW_SetRotation(uint8_t m)
{
  ST7789VW_WriteCommand(ST7789VW_CMD_MADCTL); // MADCTL
  switch (m) {
    case 0: //X-Mirror Y-Mirror
      ST7789VW_WriteByteData( ST7789VW_MADCTL_MX |  ST7789VW_MADCTL_MY |  ST7789VW_MADCTL_RGB);
      break;
    case 1: //X-Y  Exchange Y-Mirror
      ST7789VW_WriteByteData( ST7789VW_MADCTL_MY |  ST7789VW_MADCTL_MV |  ST7789VW_MADCTL_RGB);
      break;
    case 2: //Normal
      ST7789VW_WriteByteData( ST7789VW_MADCTL_RGB);
      break;
    case 3: //X-Y Exchange X-Mirror
      ST7789VW_WriteByteData( ST7789VW_MADCTL_MX |  ST7789VW_MADCTL_MV |  ST7789VW_MADCTL_RGB);
      break;
    default:
      break;
  }
}


