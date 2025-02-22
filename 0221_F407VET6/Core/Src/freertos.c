/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
/* Definitions for Task_Main */
osThreadId_t Task_MainHandle;
const osThreadAttr_t Task_Main_attributes = {
  .name = "Task_Main",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task_Key */
osThreadId_t Task_KeyHandle;
const osThreadAttr_t Task_Key_attributes = {
  .name = "Task_Key",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task_MaixCAM */
osThreadId_t Task_MaixCAMHandle;
const osThreadAttr_t Task_MaixCAM_attributes = {
  .name = "Task_MaixCAM",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task_HandleMove */
osThreadId_t Task_HandleMoveHandle;
const osThreadAttr_t Task_HandleMove_attributes = {
  .name = "Task_HandleMove",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task_BlueTooth */
osThreadId_t Task_BlueToothHandle;
const osThreadAttr_t Task_BlueTooth_attributes = {
  .name = "Task_BlueTooth",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task_MotorWhale */
osThreadId_t Task_MotorWhaleHandle;
const osThreadAttr_t Task_MotorWhale_attributes = {
  .name = "Task_MotorWhale",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Queue_HandleCalculate */
osMessageQueueId_t Queue_HandleCalculateHandle;
const osMessageQueueAttr_t Queue_HandleCalculate_attributes = {
  .name = "Queue_HandleCalculate"
};
/* Definitions for Queue_Motor */
osMessageQueueId_t Queue_MotorHandle;
const osMessageQueueAttr_t Queue_Motor_attributes = {
  .name = "Queue_Motor"
};
/* Definitions for Queue_HandleStraight */
osMessageQueueId_t Queue_HandleStraightHandle;
const osMessageQueueAttr_t Queue_HandleStraight_attributes = {
  .name = "Queue_HandleStraight"
};
/* Definitions for Queue_Status */
osMessageQueueId_t Queue_StatusHandle;
const osMessageQueueAttr_t Queue_Status_attributes = {
  .name = "Queue_Status"
};
/* Definitions for Mutex_Print */
osMutexId_t Mutex_PrintHandle;
const osMutexAttr_t Mutex_Print_attributes = {
  .name = "Mutex_Print"
};
/* Definitions for EventGroups_Car */
osEventFlagsId_t EventGroups_CarHandle;
const osEventFlagsAttr_t EventGroups_Car_attributes = {
  .name = "EventGroups_Car"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void AppTask_Main(void *argument);
void AppTask_Key(void *argument);
void AppTask_MaixCAM(void *argument);
void AppTask_HandleMove(void *argument);
void AppTask_BlueTooth(void *argument);
void AppTask_MotorWhale(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of Mutex_Print */
  Mutex_PrintHandle = osMutexNew(&Mutex_Print_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of Queue_HandleCalculate */
  Queue_HandleCalculateHandle = osMessageQueueNew (1, sizeof(void *), &Queue_HandleCalculate_attributes);

  /* creation of Queue_Motor */
  Queue_MotorHandle = osMessageQueueNew (1, sizeof(void *), &Queue_Motor_attributes);

  /* creation of Queue_HandleStraight */
  Queue_HandleStraightHandle = osMessageQueueNew (1, sizeof(void *), &Queue_HandleStraight_attributes);

  /* creation of Queue_Status */
  Queue_StatusHandle = osMessageQueueNew (1, sizeof(uint8_t), &Queue_Status_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Task_Main */
  Task_MainHandle = osThreadNew(AppTask_Main, NULL, &Task_Main_attributes);

  /* creation of Task_Key */
  Task_KeyHandle = osThreadNew(AppTask_Key, NULL, &Task_Key_attributes);

  /* creation of Task_MaixCAM */
  Task_MaixCAMHandle = osThreadNew(AppTask_MaixCAM, NULL, &Task_MaixCAM_attributes);

  /* creation of Task_HandleMove */
  Task_HandleMoveHandle = osThreadNew(AppTask_HandleMove, NULL, &Task_HandleMove_attributes);

  /* creation of Task_BlueTooth */
  Task_BlueToothHandle = osThreadNew(AppTask_BlueTooth, NULL, &Task_BlueTooth_attributes);

  /* creation of Task_MotorWhale */
  Task_MotorWhaleHandle = osThreadNew(AppTask_MotorWhale, NULL, &Task_MotorWhale_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* creation of EventGroups_Car */
  EventGroups_CarHandle = osEventFlagsNew(&EventGroups_Car_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_AppTask_Main */
/**
  * @brief  Function implementing the Task_Main thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_AppTask_Main */
__weak void AppTask_Main(void *argument)
{
  /* USER CODE BEGIN AppTask_Main */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END AppTask_Main */
}

/* USER CODE BEGIN Header_AppTask_Key */
/**
* @brief Function implementing the Task_Key thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_AppTask_Key */
__weak void AppTask_Key(void *argument)
{
  /* USER CODE BEGIN AppTask_Key */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END AppTask_Key */
}

/* USER CODE BEGIN Header_AppTask_MaixCAM */
/**
* @brief Function implementing the Task_MaixCAM thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_AppTask_MaixCAM */
__weak void AppTask_MaixCAM(void *argument)
{
  /* USER CODE BEGIN AppTask_MaixCAM */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END AppTask_MaixCAM */
}

/* USER CODE BEGIN Header_AppTask_HandleMove */
/**
* @brief Function implementing the Task_HandleMove thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_AppTask_HandleMove */
__weak void AppTask_HandleMove(void *argument)
{
  /* USER CODE BEGIN AppTask_HandleMove */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END AppTask_HandleMove */
}

/* USER CODE BEGIN Header_AppTask_BlueTooth */
/**
* @brief Function implementing the Task_BlueTooth thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_AppTask_BlueTooth */
__weak void AppTask_BlueTooth(void *argument)
{
  /* USER CODE BEGIN AppTask_BlueTooth */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END AppTask_BlueTooth */
}

/* USER CODE BEGIN Header_AppTask_MotorWhale */
/**
* @brief Function implementing the Task_MotorWhale thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_AppTask_MotorWhale */
__weak void AppTask_MotorWhale(void *argument)
{
  /* USER CODE BEGIN AppTask_MotorWhale */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END AppTask_MotorWhale */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

