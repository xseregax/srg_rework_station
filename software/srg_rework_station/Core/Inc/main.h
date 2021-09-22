/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lvgl.h"
#include "st7789.h"
#include "enc_buttons.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define HSE_IN_Pin GPIO_PIN_0
#define HSE_IN_GPIO_Port GPIOH
#define HSE_OUT_Pin GPIO_PIN_1
#define HSE_OUT_GPIO_Port GPIOH
#define IRON1_TEMP_Pin GPIO_PIN_0
#define IRON1_TEMP_GPIO_Port GPIOC
#define IRON2_TEMP_Pin GPIO_PIN_1
#define IRON2_TEMP_GPIO_Port GPIOC
#define IRON1_CURR_Pin GPIO_PIN_2
#define IRON1_CURR_GPIO_Port GPIOC
#define IRON2_CURR_Pin GPIO_PIN_3
#define IRON2_CURR_GPIO_Port GPIOC
#define AIR_TEMP_Pin GPIO_PIN_0
#define AIR_TEMP_GPIO_Port GPIOA
#define ZCD_Pin GPIO_PIN_1
#define ZCD_GPIO_Port GPIOA
#define WIRE1_Pin GPIO_PIN_2
#define WIRE1_GPIO_Port GPIOA
#define AIR_PWM_Pin GPIO_PIN_3
#define AIR_PWM_GPIO_Port GPIOA
#define DC_VOLTAGE_Pin GPIO_PIN_4
#define DC_VOLTAGE_GPIO_Port GPIOA
#define AC_CURRENT_Pin GPIO_PIN_5
#define AC_CURRENT_GPIO_Port GPIOA
#define AC_VOLTAGE_Pin GPIO_PIN_6
#define AC_VOLTAGE_GPIO_Port GPIOA
#define ARCMID_Pin GPIO_PIN_7
#define ARCMID_GPIO_Port GPIOA
#define FAN_VOLTAGE_Pin GPIO_PIN_4
#define FAN_VOLTAGE_GPIO_Port GPIOC
#define FAN_TACHO_Pin GPIO_PIN_0
#define FAN_TACHO_GPIO_Port GPIOB
#define FAN_PWM_Pin GPIO_PIN_1
#define FAN_PWM_GPIO_Port GPIOB
#define FLASH_CLK_Pin GPIO_PIN_2
#define FLASH_CLK_GPIO_Port GPIOB
#define I2C_SCL_Pin GPIO_PIN_10
#define I2C_SCL_GPIO_Port GPIOB
#define I2C_SDA_Pin GPIO_PIN_11
#define I2C_SDA_GPIO_Port GPIOB
#define EXTPORTINTA_Pin GPIO_PIN_12
#define EXTPORTINTA_GPIO_Port GPIOB
#define USB_DETECT_Pin GPIO_PIN_13
#define USB_DETECT_GPIO_Port GPIOB
#define USB_DM_Pin GPIO_PIN_14
#define USB_DM_GPIO_Port GPIOB
#define USB_DP_Pin GPIO_PIN_15
#define USB_DP_GPIO_Port GPIOB
#define ENC_A_Pin GPIO_PIN_6
#define ENC_A_GPIO_Port GPIOC
#define ENC_B_Pin GPIO_PIN_7
#define ENC_B_GPIO_Port GPIOC
#define FLASH_IO0_Pin GPIO_PIN_9
#define FLASH_IO0_GPIO_Port GPIOC
#define BUZZER_Pin GPIO_PIN_8
#define BUZZER_GPIO_Port GPIOA
#define UART_TX_Pin GPIO_PIN_9
#define UART_TX_GPIO_Port GPIOA
#define UART_RX_Pin GPIO_PIN_10
#define UART_RX_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define FLASH_IO1_Pin GPIO_PIN_10
#define FLASH_IO1_GPIO_Port GPIOC
#define LCD_RESET_Pin GPIO_PIN_11
#define LCD_RESET_GPIO_Port GPIOC
#define LCD_DC_Pin GPIO_PIN_12
#define LCD_DC_GPIO_Port GPIOC
#define LCD_CS_Pin GPIO_PIN_2
#define LCD_CS_GPIO_Port GPIOD
#define LCD_CLK_Pin GPIO_PIN_3
#define LCD_CLK_GPIO_Port GPIOB
#define LCD_MOSI_Pin GPIO_PIN_5
#define LCD_MOSI_GPIO_Port GPIOB
#define FLASH_CS_Pin GPIO_PIN_6
#define FLASH_CS_GPIO_Port GPIOB
#define LCD_PWM_Pin GPIO_PIN_7
#define LCD_PWM_GPIO_Port GPIOB
#define IRON1_PWM_Pin GPIO_PIN_8
#define IRON1_PWM_GPIO_Port GPIOB
#define IRON2_PWM_Pin GPIO_PIN_9
#define IRON2_PWM_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
