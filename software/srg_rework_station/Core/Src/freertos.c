/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lvgl.h"
#include "mcp23017.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTaskLvgl */
osThreadId_t myTaskLvglHandle;
const osThreadAttr_t myTaskLvgl_attributes = {
  .name = "myTaskLvgl",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myTaskInputs */
osThreadId_t myTaskInputsHandle;
const osThreadAttr_t myTaskInputs_attributes = {
  .name = "myTaskInputs",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myTaskEncoder */
osThreadId_t myTaskEncoderHandle;
const osThreadAttr_t myTaskEncoder_attributes = {
  .name = "myTaskEncoder",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myTaskZcd */
osThreadId_t myTaskZcdHandle;
const osThreadAttr_t myTaskZcd_attributes = {
  .name = "myTaskZcd",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myTaskUart */
osThreadId_t myTaskUartHandle;
const osThreadAttr_t myTaskUart_attributes = {
  .name = "myTaskUart",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myMutexLcd */
osMutexId_t myMutexLcdHandle;
const osMutexAttr_t myMutexLcd_attributes = {
  .name = "myMutexLcd"
};
/* Definitions for myMutexI2cInputs */
osMutexId_t myMutexI2cInputsHandle;
const osMutexAttr_t myMutexI2cInputs_attributes = {
  .name = "myMutexI2cInputs"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTaskLvgl(void *argument);
void StartTaskInputs(void *argument);
void StartTaskEncoder(void *argument);
void StartTaskZcd(void *argument);
void StartTaskUart(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationTickHook(void);

/* USER CODE BEGIN 3 */
void vApplicationTickHook( void )
{
   /* This function will be called by each tick interrupt if
   configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h. User code can be
   added here, but the tick hook is called from an interrupt context, so
   code must not attempt to block, and only the interrupt safe FreeRTOS API
   functions can be used (those that end in FromISR()). */

   lv_tick_inc(portTICK_PERIOD_MS);
}
/* USER CODE END 3 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of myMutexLcd */
  myMutexLcdHandle = osMutexNew(&myMutexLcd_attributes);

  /* creation of myMutexI2cInputs */
  myMutexI2cInputsHandle = osMutexNew(&myMutexI2cInputs_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of myTaskLvgl */
  myTaskLvglHandle = osThreadNew(StartTaskLvgl, NULL, &myTaskLvgl_attributes);

  /* creation of myTaskInputs */
  myTaskInputsHandle = osThreadNew(StartTaskInputs, NULL, &myTaskInputs_attributes);

  /* creation of myTaskEncoder */
  myTaskEncoderHandle = osThreadNew(StartTaskEncoder, NULL, &myTaskEncoder_attributes);

  /* creation of myTaskZcd */
  myTaskZcdHandle = osThreadNew(StartTaskZcd, NULL, &myTaskZcd_attributes);

  /* creation of myTaskUart */
  myTaskUartHandle = osThreadNew(StartTaskUart, NULL, &myTaskUart_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTaskLvgl */
/**
* @brief Function implementing the myTaskLvgl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskLvgl */
void StartTaskLvgl(void *argument)
{
  /* USER CODE BEGIN StartTaskLvgl */

  /* Infinite loop */
  for(;;)
  {
	lv_timer_handler();
    osDelay(5);
  }
  /* USER CODE END StartTaskLvgl */
}

/* USER CODE BEGIN Header_StartTaskInputs */
/**
* @brief Function implementing the myTaskInputs thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskInputs */
void StartTaskInputs(void *argument)
{
  /* USER CODE BEGIN StartTaskInputs */

  // init mcp23017 extender ports
  mcp23017_init();

  /* Infinite loop */
  for(;;)
  {
    uint32_t ulNotifiedValue = ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
    osDelay(1);
  }
  /* USER CODE END StartTaskInputs */
}

/* USER CODE BEGIN Header_StartTaskEncoder */
/**
* @brief Function implementing the myTaskEncoder thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskEncoder */
void StartTaskEncoder(void *argument)
{
  /* USER CODE BEGIN StartTaskEncoder */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTaskEncoder */
}

/* USER CODE BEGIN Header_StartTaskZcd */
/**
* @brief Function implementing the myTaskZcd thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskZcd */
void StartTaskZcd(void *argument)
{
  /* USER CODE BEGIN StartTaskZcd */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTaskZcd */
}

/* USER CODE BEGIN Header_StartTaskUart */
/**
* @brief Function implementing the myTaskUart thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskUart */
void StartTaskUart(void *argument)
{
  /* USER CODE BEGIN StartTaskUart */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTaskUart */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
