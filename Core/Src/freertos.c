/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "motor_speed_control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOTOR_CONTROL_TASK_PERIOD_TICKS MOTOR_SPEED_CONTROL_PERIOD_MS
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
/* Definitions for Track_line */
osThreadId_t Track_lineHandle;
const osThreadAttr_t Track_line_attributes = {
  .name = "Track_line",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Gimbal_ctrl */
osThreadId_t Gimbal_ctrlHandle;
const osThreadAttr_t Gimbal_ctrl_attributes = {
  .name = "Gimbal_ctrl",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void Track_line_App(void *argument);
void Gimbal_ctrl_App(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

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

  /* creation of Track_line */
  Track_lineHandle = osThreadNew(Track_line_App, NULL, &Track_line_attributes);

  /* creation of Gimbal_ctrl */
  Gimbal_ctrlHandle = osThreadNew(Gimbal_ctrl_App, NULL, &Gimbal_ctrl_attributes);

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
  /* USER CODE BEGIN StartDefaultTask */
  uint32_t next_wake_tick = osKernelGetTickCount();

  (void)argument;
  motor_speed_control_init();

  /* Infinite loop */
  for(;;)
  {
    motor_speed_control_process();
    next_wake_tick += MOTOR_CONTROL_TASK_PERIOD_TICKS;
    osDelayUntil(next_wake_tick);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Track_line_App */
/**
* @brief Function implementing the Track_line thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Track_line_App */
__weak void Track_line_App(void *argument)
{
  /* USER CODE BEGIN Track_line_App */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Track_line_App */
}

/* USER CODE BEGIN Header_Gimbal_ctrl_App */
/**
* @brief Function implementing the Gimbal_ctrl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Gimbal_ctrl_App */
__weak void Gimbal_ctrl_App(void *argument)
{
  /* USER CODE BEGIN Gimbal_ctrl_App */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Gimbal_ctrl_App */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

