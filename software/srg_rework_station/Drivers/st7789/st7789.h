/**
 * @file st7789.h
 *
 */

#ifndef ST7789_H
#define ST7789_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#include "main.h"

/*********************
 *      DEFINES
 *********************/
#define TFT_HOR_RES     320
#define TFT_VER_RES     240

#define ST7789_ROTATION 0
#define ST7789_WIDTH TFT_HOR_RES
#define ST7789_HEIGHT TFT_VER_RES
#define X_SHIFT 0
#define Y_SHIFT 0
/**********************
 *      TYPEDEFS
 **********************/

/* choose a Hardware SPI port to use. */
#define ST7789_SPI_PORT hspi1
extern SPI_HandleTypeDef ST7789_SPI_PORT;

/* Pin connection*/
#define ST7789_RST_PORT LCD_RESET_GPIO_Port
#define ST7789_RST_PIN  LCD_RESET_Pin
#define ST7789_DC_PORT  LCD_DC_GPIO_Port
#define ST7789_DC_PIN   LCD_DC_Pin
#define ST7789_CS_PORT  LCD_CS_GPIO_Port
#define ST7789_CS_PIN   LCD_CS_Pin

/***** Use if need backlight control *****/
#define BLK_PORT LCD_PWM_GPIO_Port
#define BLK_PIN  LCD_PWM_Pin


#define LCD_BUF_DMA_STREAM               DMA2_Stream3
#define LCD_BUF_DMA_CHANNEL              DMA_CHANNEL_0
#define LCD_BUF_DMA_STREAM_IRQ           DMA2_Stream3_IRQn
#define LCD_BUF_DMA_STREAM_IRQHANDLER    DMA2_Stream3_IRQHandler


/* Control Registers and constant codes */
#define ST7789_NOP     0x00
#define ST7789_SWRESET 0x01
#define ST7789_RDDID   0x04
#define ST7789_RDDST   0x09

#define ST7789_SLPIN   0x10
#define ST7789_SLPOUT  0x11
#define ST7789_PTLON   0x12
#define ST7789_NORON   0x13

#define ST7789_INVOFF  0x20
#define ST7789_INVON   0x21
#define ST7789_DISPOFF 0x28
#define ST7789_DISPON  0x29
#define ST7789_CASET   0x2A
#define ST7789_RASET   0x2B
#define ST7789_RAMWR   0x2C
#define ST7789_RAMRD   0x2E

#define ST7789_PTLAR   0x30
#define ST7789_COLMOD  0x3A
#define ST7789_MADCTL  0x36

/**
 * Memory Data Access Control Register (0x36H)
 * MAP:     D7  D6  D5  D4  D3  D2  D1  D0
 * param:   MY  MX  MV  ML  RGB MH  -   -
 *
 */

/* Page Address Order ('0': Top to Bottom, '1': the opposite) */
#define ST7789_MADCTL_MY  0x80
/* Column Address Order ('0': Left to Right, '1': the opposite) */
#define ST7789_MADCTL_MX  0x40
/* Page/Column Order ('0' = Normal Mode, '1' = Reverse Mode) */
#define ST7789_MADCTL_MV  0x20
/* Line Address Order ('0' = LCD Refresh Top to Bottom, '1' = the opposite) */
#define ST7789_MADCTL_ML  0x10
/* RGB/BGR Order ('0' = RGB, '1' = BGR) */
#define ST7789_MADCTL_RGB 0x00

#define ST7789_RDID1   0xDA
#define ST7789_RDID2   0xDB
#define ST7789_RDID3   0xDC
#define ST7789_RDID4   0xDD

/* Advanced options */
/**
 * Caution: Do not operate these settings
 * You know what you are doing
 */

#define ST7789_COLOR_MODE_16bit 0x55    //  RGB565 (16bit)
#define ST7789_COLOR_MODE_18bit 0x66    //  RGB666 (18bit)

/* Basic operations */
#define ST7789_RST_Clr() HAL_GPIO_WritePin(ST7789_RST_PORT, ST7789_RST_PIN, GPIO_PIN_RESET)
#define ST7789_RST_Set() HAL_GPIO_WritePin(ST7789_RST_PORT, ST7789_RST_PIN, GPIO_PIN_SET)

#define ST7789_DC_Clr() HAL_GPIO_WritePin(ST7789_DC_PORT, ST7789_DC_PIN, GPIO_PIN_RESET)
#define ST7789_DC_Set() HAL_GPIO_WritePin(ST7789_DC_PORT, ST7789_DC_PIN, GPIO_PIN_SET)

#define ST7789_Select() HAL_GPIO_WritePin(ST7789_CS_PORT, ST7789_CS_PIN, GPIO_PIN_RESET)
#define ST7789_UnSelect() HAL_GPIO_WritePin(ST7789_CS_PORT, ST7789_CS_PIN, GPIO_PIN_SET)


/******************************************/
/**********************
 * GLOBAL PROTOTYPES
 **********************/
void lv_port_disp_init(void);
void lv_port_disp_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t * color_p);

/* Basic functions. */
void ST7789_Init(void);
void ST7789_SetRotation(uint8_t m);
void ST7789_WriteCommand(uint8_t cmd);
void ST7789_WriteData(uint8_t *buff, size_t buff_size);
void ST7789_WriteSmallData(uint8_t data);
void ST7789_SetAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
void ST7789_InvertColors(uint8_t invert);
void ST7789_TearEffect(uint8_t tear);

/**********************
 *      MACROS
 **********************/
#ifdef __cplusplus
}
#endif


#endif /*ST7789_H*/
