/*
*********************************************************************************************************
*
*   模块名称 : BLUETOOTH 串口WIFI模块驱动程序
*   文件名称 : bsp_BLUETOOTH.c
*   版    本 : V1.1
*   说    明 : 封装 BLUETOOTH 模块相关的命令
*
*********************************************************************************************************
*/

#include "main.h"

/* BLUETOOTH 模块接线图


        UTXD   ---  PA10/USART1_RX
        URXD   ---  PA9 /USART1_TX

    模块缺省波特率 115200

*/

static BLUETOOTH_HandleFrameCallback s_BLUETOOTH_HandleFrame = NULL;

#define BLUETOOTH_FRAME_HEADER '<'
#define BLUETOOTH_FRAME_TAIL   '>'
static uint8_t s_ucBLUETOOTHFrameData[64] = {0};
static UART_PARSE_STATE_E s_ucState = STATE_IDLE;
static FRAME_T s_tBLUETOOTHFrame =
{
    .ucFrameHeader[0] = BLUETOOTH_FRAME_HEADER,
    .ucFrameTail[0] = BLUETOOTH_FRAME_TAIL,

    .ucFrameIndex = 0,
    .pucFrameData = s_ucBLUETOOTHFrameData,
};

/*
*********************************************************************************************************
*   函 数 名: bsp_InitBLUETOOTH
*   功能说明: 初始化二维码扫描模块的串口，注册数据解析回调函数。该函数被 bsp_Init() 调用。
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitBLUETOOTH( BLUETOOTH_HandleFrameCallback _BLUETOOTH_HandleFrame )
{
    //使用串口4，已在bsp_uart_fifo.c里面初始化，此处不必再进行初始化

    if( _BLUETOOTH_HandleFrame != NULL )
    {
        s_BLUETOOTH_HandleFrame = _BLUETOOTH_HandleFrame;
    }
}

/*
*********************************************************************************************************
*   函 数 名: BLUETOOTH_PrintRxData
*   功能说明: 打印STM32从BLUETOOTH收到的数据到COM1串口，主要用于跟踪调试
*   形    参: _ch : 收到的数据
*   返 回 值: 无
*********************************************************************************************************
*/
static void BLUETOOTH_PrintRxData( uint8_t _ch )
{
#ifdef BLUETOOTH_TO_COM1_EN
    HAL_UART_Transmit(&huart1, &_ch, 1, 0xffff ); /* 将接收到数据打印到调试串口1 */
#endif
}

/*
*********************************************************************************************************
*   函 数 名: BLUETOOTH_SendData
*   功能说明: 发送数据包
*   形    参: _databuf 数据
*            _len 数据长度
*   返 回 值: 无
*********************************************************************************************************
*/
void BLUETOOTH_SendData( uint8_t* _databuf, uint16_t _len )
{
    if( _len > 2048 )
    {
        _len = 2048;
    }

    comSendBuf( COM_BLUETOOTH, _databuf, _len );
}

/*
*********************************************************************************************************
*   函 数 名: BLUETOOTH_ReadData
*   功能说明: 读取串口数据，帧解析成功后调用回调函数进行数据解析，该函数在app_BLUETOOTH中被调用
*   形    参:  无
*   返 回 值: 无
*********************************************************************************************************
*/
uint8_t BLUETOOTH_ReadData( void )
{
    uint8_t ucData = 0;
    uint8_t ret = 0;

    /* 从 串口读取1个字节 comGetChar() 函数由 bsp_uart_fifo.c 实现 */
    while( 1 )
    {
        if( comGetChar( COM_BLUETOOTH, &ucData ) )
        {
            BLUETOOTH_PrintRxData( ucData );

            switch( s_ucState )
            {
            case STATE_IDLE:
                if( ucData == s_tBLUETOOTHFrame.ucFrameHeader[0] ) //收到帧头
                {
                    s_ucState = STATE_RECEIVING;
                    s_tBLUETOOTHFrame.ucFrameIndex = 0;               //更新索引，准备接收数据域（包括功能码和数据）
                }
                else
                {
//                         LED_R_ON();
                }
                break;

            case STATE_RECEIVING:
                if( s_tBLUETOOTHFrame.ucFrameIndex < ( sizeof( s_ucBLUETOOTHFrameData ) -1 ) ) //防止缓冲区溢出
                {
                    if( ucData == s_tBLUETOOTHFrame.ucFrameTail[0] )           //收到帧尾
                    {
                        s_tBLUETOOTHFrame.pucFrameData[s_tBLUETOOTHFrame.ucFrameIndex] = '\0'; //添加字符串结束符

                        // 帧解析成功，处理数据
                        if( s_BLUETOOTH_HandleFrame != NULL )
                        {
                            s_BLUETOOTH_HandleFrame( s_tBLUETOOTHFrame.pucFrameData, s_tBLUETOOTHFrame.ucFrameIndex );

                            //接收成功，复位
                            ret = 1;
                            s_ucState = STATE_IDLE;
                        }
                    }
                    else
                    {
                        s_tBLUETOOTHFrame.pucFrameData[s_tBLUETOOTHFrame.ucFrameIndex++] = ucData; //保存接收到的数据
                    }
                }
                else
                {
                    //缓冲区溢出，状态复位
                    s_ucState = STATE_IDLE;
                }
                break;

            default:
                s_ucState = STATE_IDLE;
                break;
            }

            continue;   /* 可能还有数据，继续分析 */
        }

        break;  /* 分析完毕，退出函数 */
    }

    return ret;
}
