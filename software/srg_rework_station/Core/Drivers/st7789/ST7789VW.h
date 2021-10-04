/**
 * @file  ST7789VW.h
 *
 */

#ifndef ST7789VW_H
#define ST7789VW_H

#ifdef __cplusplus
extern "C" {
#endif


#include "main.h"
#include "spi.h"

/*
ST7789VW 2.0"
4-line serial interface Ⅰ
16bit (RGB565)
CSX Chip selection signal
WRX
   Data is regarded as a command when WRX is low
   Data is regarded as a parameter or data when WRX is high
DCX Clock signal
SDA Serial input/output data

320*240*16bit = 153 600 bytes

*/

// SPI
#define ST7789VW_SPI_PORT hspi1

// Horizontal resolution
#define TFT_HOR_RES 320
// Vertical resolution
#define TFT_VER_RES 240
// Normal position
#define ST7789VW_ROTATION 2

//pixel count, buffer split to 3 parts or TFT_HOR_RES*10 rows
#define ST7789VW_BUFFER TFT_HOR_RES*TFT_VER_RES/3
// 1 or 2 buffers
#define ST7789VW_BUFFER_COUNT 1


/***********************/

// Pin connection
#define ST7789VW_RST_PORT LCD_RESET_GPIO_Port
#define ST7789VW_RST_PIN  LCD_RESET_Pin
#define ST7789VW_DC_PORT  LCD_DC_GPIO_Port
#define ST7789VW_DC_PIN   LCD_DC_Pin
#define ST7789VW_CS_PORT  LCD_CS_GPIO_Port
#define ST7789VW_CS_PIN   LCD_CS_Pin
#define ST7789VW_LED_PORT LCD_PWM_GPIO_Port
#define ST7789VW_LED_PIN  LCD_PWM_Pin


// ST7789VW System Function Command
#define ST7789VW_CMD_NOP        0x00 //No operation
#define ST7789VW_CMD_SWRESET    0x01 //Software reset
#define ST7789VW_CMD_RDDID      0x04 //Read display ID
#define ST7789VW_CMD_RDDST      0x09 //Read display status
#define ST7789VW_CMD_RDDPM      0x0A //Read display power
#define ST7789VW_CMD_RDDMADCTL  0x0B //Read display
#define ST7789VW_CMD_RDDCOLMOD  0x0C //Read display pixel
#define ST7789VW_CMD_RDDIM      0x0D //Read display image
#define ST7789VW_CMD_RDDSM      0x0E //Read display signal
#define ST7789VW_CMD_RDDSDR     0x0F //Read display self-diagnostic result
#define ST7789VW_CMD_SLPIN      0x10 //Sleep in
#define ST7789VW_CMD_SLPOUT     0x11 //Sleep out
#define ST7789VW_CMD_PTLON      0x12 //Partial mode on
#define ST7789VW_CMD_NORON      0x13 //Partial off (Normal)
#define ST7789VW_CMD_INVOFF     0x20 //Display inversion off
#define ST7789VW_CMD_INVON      0x21 //Display inversion on
#define ST7789VW_CMD_GAMSET     0x26 //Display inversion on ?
#define ST7789VW_CMD_DISPOFF    0x28 //Display off
#define ST7789VW_CMD_DISPON     0x29 //Display on
#define ST7789VW_CMD_CASET      0x2A //Column address set
#define ST7789VW_CMD_RASET      0x2B //Row address set
#define ST7789VW_CMD_RAMWR      0x2C //Memory write
#define ST7789VW_CMD_RAMRD      0x2E //Memory read
#define ST7789VW_CMD_PTLAR      0x30 //Partial sart/end address set
#define ST7789VW_CMD_VSCRDEF    0x33 //Vertical scrolling definition
#define ST7789VW_CMD_TEOFF      0x34 //Tearing effect line off
#define ST7789VW_CMD_TEON       0x35 //Tearing effect line on

#define ST7789VW_CMD_MADCTL     0x36 //Memory data access control
    /**
     * Memory Data Access Control Register (0x36H)
     * MAP:     D7  D6  D5  D4  D3  D2  D1  D0
     * param:   MY  MX  MV  ML  RGB MH  -   -
     */
    /* Page Address Order ('0': Top to Bottom, '1': the opposite) */
    #define ST7789VW_MADCTL_MY  0x80
    /* Column Address Order ('0': Left to Right, '1': the opposite) */
    #define ST7789VW_MADCTL_MX  0x40
    /* Page/Column Order ('0' = Normal Mode, '1' = Reverse Mode) */
    #define ST7789VW_MADCTL_MV  0x20
    /* Line Address Order ('0' = LCD Refresh Top to Bottom, '1' = the opposite) */
    #define ST7789VW_MADCTL_ML  0x10
    /* RGB/BGR Order ('0' = RGB, '1' = BGR) */
    #define ST7789VW_MADCTL_RGB 0x00

#define ST7789VW_CMD_VSCRSADD   0x37 //Vertical scrolling start address
#define ST7789VW_CMD_IDMOFF     0x38 //Idle mode off
#define ST7789VW_CMD_IDMON      0x39 //Idle mode on

#define ST7789VW_CMD_COLMOD     0x3A     //Interface pixel format
    #define ST7789VW_COLMOD_12bit 0x03    //  RGB444 (12bit)
    #define ST7789VW_COLMOD_16bit 0x05    //  RGB565 (16bit)
    #define ST7789VW_COLMOD_18bit 0x06    //  RGB666 (18bit)

#define ST7789VW_CMD_RAMWRC     0x3C     //Memory write continue
#define ST7789VW_CMD_RAMRDC     0x3E     //Memory read continue
#define ST7789VW_CMD_TESCAN     0x44     //Set tear scanline
#define ST7789VW_CMD_RDTESCAN   0x45     //Get scanline
#define ST7789VW_CMD_WRDISBV    0x51     //Write display brightness
#define ST7789VW_CMD_RDDISBV    0x52     //Read display brightness value
#define ST7789VW_CMD_WRCTRLD    0x53     //Write CTRL display
#define ST7789VW_CMD_RDCTRLD    0x54     //Read CTRL value display
#define ST7789VW_CMD_WRCACE     0x55     //Write content adaptive brightness control and Color enhancement
#define ST7789VW_CMD_RDCABC     0x56     //Read content adaptive brightness control
#define ST7789VW_CMD_WRCABCMB   0x57     //Write CABC minimum brightness
#define ST7789VW_CMD_RDCABCMB   0x5F     //Read CABC minimum brightness
#define ST7789VW_CMD_RDABCSDR   0x68     //Read Automatic Brightness Control Self-Diagnostic Result
#define ST7789VW_CMD_RDID1      0xDA     //Read ID1
#define ST7789VW_CMD_RDID2      0xDB     //Read ID2
#define ST7789VW_CMD_RDID3      0xDC     //Read ID3


#define ST7789VW_CMD_RAMCTRL    0xB0     //RAM Control
#define ST7789VW_CMD_RGBCTRL    0xB1     //RGB Control
#define ST7789VW_CMD_PORCTRL    0xB2     //Porch control
#define ST7789VW_CMD_FRCTRL1    0xB3     //Frame Rate Control 1
#define ST7789VW_CMD_PARCTRL    0xB5     //Partial control
#define ST7789VW_CMD_GCTRL      0xB7     //Gate control
#define ST7789VW_CMD_GTADJ      0xB8     //Gate on timing adjustment
#define ST7789VW_CMD_DGMEN      0xBA     //Digital Gamma Enable
#define ST7789VW_CMD_VCOMS      0xBB     //VCOM settings
#define ST7789VW_CMD_POWSAVE    0xBC     //Power saving mode
#define ST7789VW_CMD_DLPOFFSAVE 0xBD     //Display off power save
#define ST7789VW_CMD_LCMCTRL    0xC0     //LCM control
#define ST7789VW_CMD_IDSET      0xC1     //ID Setting
#define ST7789VW_CMD_VDVVRHEN   0xC2     //VDV and VRH Command Enable
#define ST7789VW_CMD_VRHS       0xC3     //VRH Set
#define ST7789VW_CMD_VDVSET     0xC4     //VDV setting
#define ST7789VW_CMD_VCMOFSET   0xC5     //VCOM offset set
#define ST7789VW_CMD_FRCTR2     0xC6     //FR Control 2
#define ST7789VW_CMD_CABCCTRL   0xC7     //CABC control
#define ST7789VW_CMD_REGSEL1    0xC8     //Register value selection1
#define ST7789VW_CMD_REGSEL2    0xCA     //Register value selection2
#define ST7789VW_CMD_PWMFRSEL   0xCC     //PWM Frequency Selection
#define ST7789VW_CMD_PWCTRL1    0xD0     //Power Control 1
#define ST7789VW_CMD_VAPVANEN   0xD2     //Enable VAP/VAN signal output
#define ST7789VW_CMD_CMD2EN     0xDF     //Command 2 Enable
#define ST7789VW_CMD_PVGAMCTRL  0xE0     //Positive Voltage Gamma Control
#define ST7789VW_CMD_NVGAMCTRL  0xE1     //Negative Voltage Gamma Control
#define ST7789VW_CMD_DGMLUTR    0xE2     //Digital Gamma Look-up Table for Red
#define ST7789VW_CMD_DGMLUTB    0xE3     //Digital Gamma Look-up Table for Blue
#define ST7789VW_CMD_GATECTRL   0xE4     //Gate control
#define ST7789VW_CMD_SPI2EN     0xE7     //SPI2 enable
#define ST7789VW_CMD_PWCTRL2    0xE8     //Power Control 2
#define ST7789VW_CMD_EQCTRL     0xE9     //Equalize Time Control
#define ST7789VW_CMD_PROMCTRL   0xEC     //Program Control
#define ST7789VW_CMD_PROMEN     0xFA     //Program Mode Enable
#define ST7789VW_CMD_NVMSET     0xFC     //NVM Setting
#define ST7789VW_CMD_PROMACT    0xFE     //Program Action



// Basic operations
#define ST7789VW_RST_Clr() HAL_GPIO_WritePin(ST7789VW_RST_PORT, ST7789VW_RST_PIN, GPIO_PIN_RESET)
#define ST7789VW_RST_Set() HAL_GPIO_WritePin(ST7789VW_RST_PORT, ST7789VW_RST_PIN, GPIO_PIN_SET)

#define ST7789VW_DC_Clr() HAL_GPIO_WritePin(ST7789VW_DC_PORT, ST7789VW_DC_PIN, GPIO_PIN_RESET)
#define ST7789VW_DC_Set() HAL_GPIO_WritePin(ST7789VW_DC_PORT, ST7789VW_DC_PIN, GPIO_PIN_SET)

#define ST7789VW_Select() HAL_GPIO_WritePin(ST7789VW_CS_PORT, ST7789VW_CS_PIN, GPIO_PIN_RESET)
#define ST7789VW_UnSelect() HAL_GPIO_WritePin(ST7789VW_CS_PORT, ST7789VW_CS_PIN, GPIO_PIN_SET)


/******************************************/

void lv_port_disp_init(void);
void lv_port_disp_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t * color_p);
void lv_port_disp_give_ISR(void);


#ifdef __cplusplus
}
#endif


#endif /* ST7789VW_H*/
