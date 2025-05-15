/*
*********************************************************************************************************
*
*   模块名称 : QR 串口二维码模块驱动程序
*   文件名称 : bsp_QR.c
*   版    本 : V1.1
*   说    明 : 封装 QR 模块相关的命令
*
*********************************************************************************************************
*/

#include "main.h"

/* QR 模块接线图


        UTXD   ---  PD2/UART5_RX
        URXD   ---  PC12/UART5_TX


    模块缺省波特率 9600

*/

static uint8_t QR_tx1_buf[256] = {0x5A, 0x00, 0x00, 0x0a, 0x53, 0x5f, 0x43, 0x4d, 0x44, 0x5f, 0x30, 0x32, 0x30, 0x44, 0x65, 0xA5};
static uint8_t QR_tx2_buf[256] = {0x5A, 0x00, 0x00, 0x08, 0x53, 0x52, 0x30, 0x33, 0x30, 0x33, 0x30, 0x31, 0x08, 0xA5};

static uint8_t s_ucQRFrameData[64] = {0};
static FRAME_T s_tQRFrame =
{
    .ucFrameIndex = 0,
    .pucFrameData = s_ucQRFrameData,
};

/*
*********************************************************************************************************
*   函 数 名: bsp_InitQR
*   功能说明: 初始化二维码扫描模块的串口，注册数据解析回调函数。该函数被 bsp_Init() 调用。
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitQR( void )
{
    //使用串口5，已在bsp_uart_fifo.c里面初始化，此处不必再进行初始化
}

/*
*********************************************************************************************************
*   函 数 名: QR_PrintRxData
*   功能说明: 打印STM32从QR收到的数据到COM1串口，主要用于跟踪调试
*   形    参: _ch : 收到的数据
*   返 回 值: 无
*********************************************************************************************************
*/
static void QR_PrintRxData( uint8_t _ch )
{
#ifdef QR_TO_COM1_EN
    HAL_UART_Transmit(&huart1, &_ch, 1, 0xffff ); /* 将接收到数据打印到调试串口1 */
#endif
}

/*
*********************************************************************************************************
*   函 数 名: QR_SendData
*   功能说明: 发送数据包
*   形    参: _databuf 数据
*             _len 数据长度
*   返 回 值: 无
*********************************************************************************************************
*/
void QR_SendData( uint8_t* _databuf, uint16_t _len )
{
    if( _len > 2048 )
    {
        _len = 2048;
    }

    comSendBuf( COM_QR, _databuf, _len );
}

/*
*********************************************************************************************************
*   函 数 名: QR_SendAndRead
*   功能说明: 读取串口数据，帧解析成功后调用回调函数进行数据解析
*   形    参:  无
*   返 回 值: 无
*********************************************************************************************************
*/
uint8_t QR_SendAndRead( uint8_t* _pucBuf )
{
    uint8_t ucData = 0;
    uint8_t ret = 0;
    static uint8_t ucSendFlag = 0;    //静态变量，再次进入函数时不会重新赋初值

    if( ucSendFlag == 0 )   //只发送一次
    {
        QR_SendData( QR_tx1_buf, 16 );
        HAL_Delay( 200 );   //此处为HAL库延迟，需注意任务优先级
        QR_SendData( QR_tx2_buf, 14 );

        ucSendFlag = 1;
    }

    /* 从 串口读取1个字节 comGetChar() 函数由 bsp_uart_fifo.c 实现 */
    while( 1 )
    {
        if( comGetChar( COM_QR, &ucData ) )
        {
            QR_PrintRxData( ucData );

            if(( ucData >= '1' && ucData <= '3' ) || ucData == '+' )
            {
                s_tQRFrame.pucFrameData[s_tQRFrame.ucFrameIndex++] = ucData;        /* 保存接收到的数据 */

                if( s_tQRFrame.ucFrameIndex >= 7 )
                {
                    memcpy( _pucBuf, s_tQRFrame.pucFrameData, s_tQRFrame.ucFrameIndex ); // 复制数据
                    memset( s_tQRFrame.pucFrameData, 0, s_tQRFrame.ucFrameIndex );
                    ret = 1;
                }
            }
            continue;   /* 可能还有数据，继续分析 */
        }

        break;  /* 分析完毕，退出函数 */
    }

    return ret;
}
