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
#include "route_planning.h"
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
  (void)argument;

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

/* USER CODE END Application */

