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
/* Definitions for Task_MaixCAM */
osThreadId_t Task_MaixCAMHandle;
const osThreadAttr_t Task_MaixCAM_attributes = {
  .name = "Task_MaixCAM",
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
/* Definitions for Task_PID */
osThreadId_t Task_PIDHandle;
const osThreadAttr_t Task_PID_attributes = {
  .name = "Task_PID",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task_HWT101 */
osThreadId_t Task_HWT101Handle;
const osThreadAttr_t Task_HWT101_attributes = {
  .name = "Task_HWT101",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task_Show */
osThreadId_t Task_ShowHandle;
const osThreadAttr_t Task_Show_attributes = {
  .name = "Task_Show",
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
void AppTask_MaixCAM(void *argument);
void AppTask_BlueTooth(void *argument);
void AppTask_MotorWhale(void *argument);
void AppTask_PID(void *argument);
void AppTask_HWT101(void *argument);
void AppTask_Show(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

unsigned long getRunTimeCounterValue(void) {
    return get_system_ms();
}
/* USER CODE END 1 */

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

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

  /* creation of Task_MaixCAM */
  Task_MaixCAMHandle = osThreadNew(AppTask_MaixCAM, NULL, &Task_MaixCAM_attributes);

  /* creation of Task_BlueTooth */
  Task_BlueToothHandle = osThreadNew(AppTask_BlueTooth, NULL, &Task_BlueTooth_attributes);

  /* creation of Task_MotorWhale */
  Task_MotorWhaleHandle = osThreadNew(AppTask_MotorWhale, NULL, &Task_MotorWhale_attributes);

  /* creation of Task_PID */
  Task_PIDHandle = osThreadNew(AppTask_PID, NULL, &Task_PID_attributes);

  /* creation of Task_HWT101 */
  Task_HWT101Handle = osThreadNew(AppTask_HWT101, NULL, &Task_HWT101_attributes);

  /* creation of Task_Show */
  Task_ShowHandle = osThreadNew(AppTask_Show, NULL, &Task_Show_attributes);

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

/* USER CODE BEGIN Header_AppTask_PID */
/**
* @brief Function implementing the Task_PID thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_AppTask_PID */
__weak void AppTask_PID(void *argument)
{
  /* USER CODE BEGIN AppTask_PID */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END AppTask_PID */
}

/* USER CODE BEGIN Header_AppTask_HWT101 */
/**
* @brief Function implementing the Task_HWT101 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_AppTask_HWT101 */
__weak void AppTask_HWT101(void *argument)
{
  /* USER CODE BEGIN AppTask_HWT101 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END AppTask_HWT101 */
}

/* USER CODE BEGIN Header_AppTask_Show */
/**
* @brief Function implementing the Task_Show thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_AppTask_Show */
__weak void AppTask_Show(void *argument)
{
  /* USER CODE BEGIN AppTask_Show */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END AppTask_Show */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

