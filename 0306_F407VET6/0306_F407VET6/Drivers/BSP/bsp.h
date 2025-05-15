/*
*********************************************************************************************************
*
*	模块名称 : BSP模块
*	文件名称 : bsp.h
*	说    明 : 这是底层驱动模块所有的h文件的汇总文件。 应用程序只需 #include bsp.h 即可，
*			  不需要#include 每个模块的 h 文件
*
*	Copyright (C), 2013-2014, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/

#ifndef __BSP_H
#define __BSP_H
 
#define DEBUG_TO_COM1	/* 打印数据到串口1 */


/*
*********************************************************************************************************
*                                         CubeMX
*********************************************************************************************************
*/
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/*
*********************************************************************************************************
*                                         标准库
*********************************************************************************************************
*/
#include <math.h>
#include <stdlib.h> 
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "perf_counter.h"

/*
*********************************************************************************************************
*                                           OS
*********************************************************************************************************
*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#include "stream_buffer.h"
#include "message_buffer.h"

/*
*********************************************************************************************************
*                                           宏定义
*********************************************************************************************************
*/

/* 小车用到的事件标志 */
#define EventGroupsCarBlueTooth_EN_0  (1 << 0)        //蓝牙调试开关标志位
#define EventGroupsCarMaixCAM_EN_1    (1 << 1)        //摄像头的开关标志位
#define EventGroupsCarReserved_2      (1 << 2)        //摄像头的保留位
#define EventGroupsCarReserved_3      (1 << 3)        //摄像头的保留位
#define EventGroupsCarHWT101_EN_4     (1 << 4)        //陀螺仪的开关标志位
#define EventGroupsCarReserved_5      (1 << 5)        //陀螺仪的保留位
#define EventGroupsCarReserved_6      (1 << 6)        //陀螺仪的保留位 
#define EventGroupsCarReserved_7      (1 << 7)
#define EventGroupsCarReserved_8      (1 << 8)
#define EventGroupsCarReserved_9      (1 << 9)
#define EventGroupsCarReserved_10     (1 << 10)
#define EventGroupsCarReserved_11     (1 << 11)
#define EventGroupsCarReserved_12     (1 << 12)
#define EventGroupsCarReserved_13     (1 << 13)
#define EventGroupsCarReserved_14     (1 << 14)
#define EventGroupsCarReserved_15     (1 << 15)
    
//#define MusicTaskWaitFlag   		    (MusicTaskAudioFillBuffer0_10 | MusicTaskAudioFillBuffer1_9 | MusicTaskAudioReturn_6 | MusicTaskAudioGetTime_7)
//#define MusicTaskWaitBuffer  		    (MusicTaskAudioFillBuffer0_10 | MusicTaskAudioFillBuffer1_9)

/*
*********************************************************************************************************
*                                         BSP / APP
*********************************************************************************************************
*/
#include "bsp_keyled.h"
#include "bsp_uart_dma.h"
#include "bsp_bluetooth.h"
#include "bsp_maixcam.h"
#include "bsp_hwt101.h"
#include "bsp_qr.h"
#include "bsp_screen.h"
#include "bsp_servo.h"
#include "bsp_can.h"
#include "bsp_can_zdt.h"
#include "bsp_can_2006.h"
#include "bsp_pid.h"

#include "task_car.h"
#include "app_main.h"
#include "app_bluetooth.h"
#include "app_maixcam.h"
#include "app_hwt101.h"
#include "app_servo.h"
#include "app_handle.h"
#include "app_motor_2006.h"
#include "app_motor_zdt.h"

/*
*********************************************************************************************************
*                                          变量和函数
*********************************************************************************************************
*/
//任务句柄
extern osThreadId_t Task_MainHandle;
extern osThreadId_t Task_KeyHandle;
extern osThreadId_t Task_MaixCAMHandle;
extern osThreadId_t Task_HandleCalculateHandle;
extern osThreadId_t Task_BlueToothHandle;
extern osThreadId_t Task_PIDHandle;
//队列句柄
extern osMessageQueueId_t Queue_HandleCalculateHandle;
extern osMessageQueueId_t Queue_HandleStraightHandle;
extern osMessageQueueId_t Queue_MotorHandle;
extern osMessageQueueId_t Queue_StatusHandle;
//互斥量句柄
extern osMutexId_t Mutex_PrintHandle;
//事件组句柄
extern osEventFlagsId_t EventGroups_CarHandle;
//流缓冲区句柄
//extern StreamBufferHandle_t StreamBuffer_MaixCAM_AND_HC;


void bsp_Init(void);



#endif /*BSP_H*/
