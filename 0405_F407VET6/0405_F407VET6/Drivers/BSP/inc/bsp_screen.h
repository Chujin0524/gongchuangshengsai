/*
*********************************************************************************************************
*
*	模块名称 : SCREEN 串口WIFI模块驱动程序
*	文件名称 : bsp_SCREEN.h
*	版    本 : V1.0
*	说    明 : 头文件
*
*	Copyright (C), 2014-2015, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/

#ifndef __BSP_SCREEN_H
#define __BSP_SCREEN_H

#define COM_SCREEN	COM4		/* 选择串口 */

/* 定义下面这句话, 将把收到的字符发送到调试串口1 */
//#define SCREEN_TO_COM1_EN

/* 本模块部分函数用到了软件定时器最后1个ID。 因此主程序调用本模块的函数时，请注意回避定时器 TMR_COUNT - 1。
  bsp_StartTimer(3, _usTimeOut);

  TMR_COUNT 在 bsp_timer.h 文件定义
*/
#define SCREEN_TMR_ID	(TMR_COUNT - 1)



/* 供外部调用的函数声明 */
void bsp_InitSCREEN(void);
void SCREEN_SendData(uint8_t *_databuf, uint16_t _len);
void SCREEN_Dispaly(uint8_t *_ucaQRScan_buff);
#endif

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
