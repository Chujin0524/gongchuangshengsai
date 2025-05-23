/*
*********************************************************************************************************
*
*    模块名称 : BSP模块(For STM32F407)
*    文件名称 : bsp.c
*    版    本 : V1.0
*    说    明 : 这是硬件底层驱动程序的主文件。每个c文件可以 #include "bsp.h" 来包含所有的外设驱动模块。
*               bsp = Borad surport packet 板级支持包
*    修改记录 :
*        版本号  日期         作者       说明
*        V1.0    2018-07-29  Eric2013   正式发布
*
*    Copyright (C), 2018-2030, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/
#include "main.h"



/*
*********************************************************************************************************
*                                       函数声明
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*    函 数 名: bsp_Init
*    功能说明: 初始化所有的硬件设备。该函数配置CPU寄存器和外设的寄存器并初始化一些全局变量。只需要调用一次
*    形    参：无
*    返 回 值: 无
*********************************************************************************************************
*/
void bsp_Init(void)
{
    init_cycle_counter(true);
    
    bsp_InitUart();        /* 初始化串口 */
    bsp_InitCAN();
    wheel_init();
//    bsp_InitBLUE_TOOTH();   //初始化蓝牙串口1
//    bsp_InitMOTOR_RAIL();   //初始化导轨电机串口2
//    bsp_InitBOTTOM_F103();  //初始化下板103串口3
//    bsp_InitMAIXCAM();      //初始化摄像头串口4
//    bsp_InitQR();           //初始化二维码模块
//    bsp_InitSCREEN();       //初始化串口屏
//    
    bsp_InitServo();        //初始化舵机
		hwt_init();
}

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
