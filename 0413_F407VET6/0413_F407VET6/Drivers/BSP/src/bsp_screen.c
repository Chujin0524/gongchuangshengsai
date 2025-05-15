/*
*********************************************************************************************************
*
*	模块名称 : SCREEN 串口WIFI模块驱动程序
*	文件名称 : bsp_SCREEN.c
*	版    本 : V1.1
*	说    明 : 封装 SCREEN 模块相关的命令
*
*********************************************************************************************************
*/

#include "bsp.h"

/* SCREEN 模块接线图
	SCREEN模块    STM32-V5开发板


		UTXD   ---  PC11/UART4_RX
		GND    ---  GND
		VCC    ---  5V  (供电)
		URXD   ---  PC10/UART4_TX


	模块缺省波特率 115200

*/

/*
*********************************************************************************************************
*	函 数 名: bsp_InitSCREEN
*	功能说明: 初始化二维码扫描模块的串口,  该函数被 bsp_Init() 调用。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitSCREEN(void)
{
    //初始化串口4，已在bsp_uart_fifo.c里面初始化，此处不必再进行初始化
}

/*
*********************************************************************************************************
*	函 数 名: SCREEN_PrintRxData
*	功能说明: 打印STM32从SCREEN收到的数据到COM1串口，主要用于跟踪调试
*	形    参: _ch : 收到的数据
*	返 回 值: 无
*********************************************************************************************************
*/
void SCREEN_PrintRxData(uint8_t _ch)
{
#ifdef SCREEN_TO_COM1_EN
    comSendChar(COM1, _ch);		/* 将接收到数据打印到调试串口1 */
#endif
}

/*
*********************************************************************************************************
*	函 数 名: SCREEN_SendData
*	功能说明: 发送数据包
*	形    参: _databuf 数据
*			 _len 数据长度
*	返 回 值: 无
*********************************************************************************************************
*/
void SCREEN_SendData(uint8_t *_databuf, uint16_t _len)
{
    if (_len > 2048)
    {
        _len = 2048;
    }

    comSendBuf(COM_SCREEN, _databuf, _len);
}

/*
*********************************************************************************************************
*	函 数 名: SCREEN_Dispaly
*	功能说明: 发送指令使屏幕显示
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void SCREEN_Dispaly(uint8_t *_ucaQRScan_buff)
{
//    static char screen_tx_buf[256] = {0};                            //屏幕发送缓存
//    static uint8_t screen_tx_end_buf[3] = {0xFF, 0xFF, 0xFF};        //屏幕发送结束符缓存

////    sprintf( screen_tx_buf, "t0.txt=\"%s\"\xff\xff\xff", _ucaQRScan_buff );
//    sprintf( screen_tx_buf, "t0.txt=\"%s\"", _ucaQRScan_buff );
//    SCREEN_SendData((uint8_t *)screen_tx_buf, strlen(screen_tx_buf));
//    HAL_Delay(200);   //SCREEN_SendData是以串口DMA传输的，加延迟使DMA传输完数据。此处为简单处理
//    SCREEN_SendData(screen_tx_end_buf, 3);
//    
//    __NOP();
    
    static char screen_tx_buf[256] = {0x74, 0x30, 0x2E, 0x74, 0x78, 0x74, 0x3D, 0x22, 0x31, 0x32, 0x33, 0x2B, 0x33, 0x32, 0x31, 0x22, 0xff, 0xff, 0xff};
//    int pos = sprintf(screen_tx_buf, "t0.txt=\"%s\"", _ucaQRScan_buff);

//    // 直接追加 0xFF 0xFF 0xFF
//    memcpy(screen_tx_buf + pos, "\xFF\xFF\xFF", 3);

    // 发送数据
//    SCREEN_SendData((uint8_t *)screen_tx_buf, pos + 3);
    HAL_UART_Transmit( &huart4, (uint8_t *)screen_tx_buf, 19, 0xFFFF );
    HAL_Delay(200);
    HAL_UART_Transmit( &huart4, (uint8_t *)screen_tx_buf, 19, 0xFFFF );
    HAL_Delay(200);
    
    __NOP();
}

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
