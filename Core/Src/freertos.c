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
#include "decision_task.h"
#include "mission.h"
#include "route_planning.h"
#include "vision_uart.h"
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
osThreadId_t Vision_uartHandle;
const osThreadAttr_t Vision_uart_attributes = {
  .name = "Vision_uart",
  .stack_size = 768 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
osThreadId_t MissionHandle;
const osThreadAttr_t Mission_attributes = {
  .name = "Mission",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Robot_arm_ctrl */
osThreadId_t Robot_arm_ctrlHandle;
const osThreadAttr_t Robot_arm_ctrl_attributes = {
  .name = "Robot_arm_ctrl",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Decision */
osThreadId_t DecisionHandle;
const osThreadAttr_t Decision_attributes = {
  .name = "Decision",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Route_planning */
osThreadId_t Route_planningHandle;
const osThreadAttr_t Route_planning_attributes = {
  .name = "Route_planning",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void VisionUart_App(void *argument);
void Mission_App(void *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void Robot_arm_ctrl_App(void *argument);
void Decision_App(void *argument);
void Route_planning_App(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  DecisionTask_Init();
  RoutePlanning_Init();
  VisionUart_Init();
  Mission_Init();
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

  /* creation of Robot_arm_ctrl */
  Robot_arm_ctrlHandle = osThreadNew(Robot_arm_ctrl_App, NULL, &Robot_arm_ctrl_attributes);

  /* creation of Decision */
  DecisionHandle = osThreadNew(Decision_App, NULL, &Decision_attributes);

  /* creation of Route_planning */
  Route_planningHandle = osThreadNew(Route_planning_App, NULL, &Route_planning_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  Vision_uartHandle = osThreadNew(VisionUart_App, NULL, &Vision_uart_attributes);
  MissionHandle = osThreadNew(Mission_App, NULL, &Mission_attributes);

  /* Creation failures are otherwise silent: the board boots, the axes home and
     the keys do nothing because Mission_App was never scheduled. */
  configASSERT(defaultTaskHandle != NULL);
  configASSERT(Robot_arm_ctrlHandle != NULL);
  configASSERT(DecisionHandle != NULL);
  configASSERT(Route_planningHandle != NULL);
  configASSERT(Vision_uartHandle != NULL);
  configASSERT(MissionHandle != NULL);
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
  (void)argument;

  /* Vision acquisition is armed by the mission task on a key press. */

  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Robot_arm_ctrl_App */
/**
* @brief Function implementing the Robot_arm_ctrl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Robot_arm_ctrl_App */
__weak void Robot_arm_ctrl_App(void *argument)
{
  /* USER CODE BEGIN Robot_arm_ctrl_App */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Robot_arm_ctrl_App */
}

/* USER CODE BEGIN Header_Decision_App */
/**
* @brief Function implementing the Decision thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Decision_App */
__weak void Decision_App(void *argument)
{
  /* USER CODE BEGIN Decision_App */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Decision_App */
}

/* USER CODE BEGIN Header_Route_planning_App */
/**
* @brief Function implementing the Route_planning thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Route_planning_App */
__weak void Route_planning_App(void *argument)
{
  /* USER CODE BEGIN Route_planning_App */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Route_planning_App */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
  * @brief  Called by the kernel when pvPortMalloc() fails.
  * @note   Without this hook an undersized configTOTAL_HEAP_SIZE makes
  *         osThreadNew() return NULL silently, so the threads created last
  *         (Vision_uart, Mission) never run and the device looks dead.
  */
void vApplicationMallocFailedHook(void)
{
  Error_Handler();
}

/**
  * @brief  Called by the kernel when a task overflows its stack.
  * @param  xTask Handle of the offending task.
  * @param  pcTaskName Name of the offending task.
  */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void)xTask;
  (void)pcTaskName;
  Error_Handler();
}

/* USER CODE END Application */

