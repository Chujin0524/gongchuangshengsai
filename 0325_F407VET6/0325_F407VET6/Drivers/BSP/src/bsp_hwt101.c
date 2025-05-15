/*
*********************************************************************************************************
*
*   模块名称 : HWT101 串口WIFI模块驱动程序
*   文件名称 : bsp_HWT101.c
*   版    本 : V1.1
*   说    明 : 封装 HWT101 模块相关的命令
*
*********************************************************************************************************
*/

#include "main.h"

/* HWT101 模块接线图


        UTXD   ---  PB11/USART2_RX
        URXD   ---  PB10/USART2_TX

    模块缺省波特率 115200

*/

static HWT101_HandleFrameCallback s_HWT101_HandleFrame = NULL;

#define HWT101_FRAMEHEADER0 0x55
#define HWT101_FRAMEHEADER1 0x53
#define HWT101_FRAMETAIL0   0x53
#define HWT101_FRAMETAIL1   0x53
static uint8_t s_ucHWT101FrameData[32] = {0};
static FRAME_T s_tHWT101Frame =
{
    .ucFrameHeader[0] = HWT101_FRAMEHEADER0,
    .ucFrameHeader[1] = HWT101_FRAMEHEADER1,
    .ucFrameTail[0] = HWT101_FRAMETAIL0,
    .ucFrameTail[1] = HWT101_FRAMETAIL1,

    .ucFrameIndex = 0,
    .ucFrameLength = 0,
    .pucFrameData = s_ucHWT101FrameData,
    .ucCheckSum = 0
};

int hwt_flag = 1;

/*
*********************************************************************************************************
*   函 数 名: bsp_InitHWT101
*   功能说明: 初始化二维码扫描模块的串口，注册数据解析回调函数。该函数被 bsp_Init() 调用。
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitHWT101( HWT101_HandleFrameCallback _HWT101_HandleFrame )
{
    //使用串口2，已在bsp_uart_fifo.c里面初始化，此处不必再进行初始化

    if( _HWT101_HandleFrame != NULL )
    {
        s_HWT101_HandleFrame = _HWT101_HandleFrame;
    }
}
/*
*********************************************************************************************************
*   函 数 名: hwt_init
*   功能说明: 打印STM32从HWT101收到的数据到COM1串口，主要用于跟踪调试
*   形    参: _ch : 收到的数据
*   返 回 值: 无
*********************************************************************************************************
*/
void hwt_init(void)
{
    uint8_t tempData_head[5] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
    uint8_t tempData[5] = {0xFF, 0xAA, 0x76, 0x00, 0x00};
    uint8_t tempData_tail[5] = {0xFF, 0xAA, 0x00, 0x00, 0x00};
    HAL_UART_Transmit_DMA(&huart3, tempData_head,sizeof(tempData_head));
    HAL_Delay(1000);
    HAL_UART_Transmit_DMA(&huart3, tempData,sizeof(tempData));
    HAL_Delay(1000);
    HAL_UART_Transmit_DMA(&huart3, tempData_tail,sizeof(tempData_tail));
    HAL_Delay(1000);
}

/*
*********************************************************************************************************
*   函 数 名: HWT101_PrintRxData
*   功能说明: 打印STM32从HWT101收到的数据到COM1串口，主要用于跟踪调试
*   形    参: _ch : 收到的数据
*   返 回 值: 无
*********************************************************************************************************
*/
static void HWT101_PrintRxData( uint8_t _ch )
{
#ifdef HWT101_TO_COM1_EN
    HAL_UART_Transmit(&huart1, &_ch, 1, 0xffff ); /* 将接收到数据打印到调试串口1 */
#endif
}

/*
*********************************************************************************************************
*   函 数 名: HWT101_SendData
*   功能说明: 发送数据包
*   形    参: _databuf 数据
*            _len 数据长度
*   返 回 值: 无
*********************************************************************************************************
*/
void HWT101_SendData( uint8_t* _databuf, uint16_t _len )
{
    if( _len > 2048 )
    {
        _len = 2048;
    }

    comSendBuf( COM_HWT101, _databuf, _len );
}

/*
*********************************************************************************************************
*   函 数 名: HWT101_ReadData
*   功能说明: 读取串口数据，帧解析成功后调用回调函数进行数据解析，该函数在app_HWT101中被调用
*   形    参:  无
*   返 回 值: 无
*********************************************************************************************************
*/
uint8_t HWT101_ReadData( void )
{
    static UART_PARSE_STATE_E s_ucState = STATE_IDLE;
    uint8_t ucData = 0;
    uint8_t ret = 0;

    /* 从 串口读取1个字节 comGetChar() 函数由 bsp_uart_fifo.c 实现 */
    while( 1 )
    {
        if( comGetChar( COM_HWT101, &ucData ) )
        {
            HWT101_PrintRxData( ucData );

            switch( s_ucState )
            {
            case STATE_IDLE:
                if( ucData == s_tHWT101Frame.ucFrameHeader[0] ) //收到帧头0
                {
                    s_ucState = STATE_HEADER_0;
                }
                else
                {
//                         LED_R_ON();
                }
                break;

            case STATE_HEADER_0:
                if( ucData == s_tHWT101Frame.ucFrameHeader[1] ) //收到帧头1
                {
                    s_tHWT101Frame.ucFrameIndex = 0;      //更新索引，准备接收数据域
                    s_tHWT101Frame.ucFrameLength = 8;     //帧长（数据域长度，包括数据和版本号）
                    s_tHWT101Frame.ucCheckSum = s_tHWT101Frame.ucFrameHeader[0] + s_tHWT101Frame.ucFrameHeader[1];
                    s_ucState = STATE_DATA;
                }
                else
                {
//                         LED_R_ON();
                    s_ucState = STATE_IDLE;
                }
                break;

            case STATE_DATA:
                s_tHWT101Frame.pucFrameData[s_tHWT101Frame.ucFrameIndex++] = ucData;
                if( s_tHWT101Frame.ucFrameIndex <= s_tHWT101Frame.ucFrameLength )    //根据帧长截断
                {
                    s_tHWT101Frame.ucCheckSum += ucData;                                  //收到数据
                }
                else
                {
                    if( ucData == ( s_tHWT101Frame.ucCheckSum & 0xFF ) )     //校验和
                    {
                        // 帧解析成功，处理数据
                        if( s_HWT101_HandleFrame != NULL )
                        {
                            ret = 1;
                            s_HWT101_HandleFrame( s_tHWT101Frame.pucFrameData, s_tHWT101Frame.ucFrameLength );
                            comClearRxFifo(COM_HWT101);
                        }
                    }
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











//void hwt_init(void)
//{
//    uint8_t tempData[5] = {0xFF, 0xAA, 0x76, 0x00, 0x00};
//    uint8_t unlock[5] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};//解锁寄存器
//    uint8_t get_zero[5] = {0xFF, 0xAA, 0x48, 0x01, 0x00};//自动获取零偏
//    uint8_t save_zero[5] = {0xFF, 0xAA, 0x00, 0x00, 0x00};//保存在寄存器中
//    int i;
//    for (i = 0; i < 5; i++) //解锁寄存器
//    {
//        HAL_UART_Transmit(&huart2, &unlock[i], 1, 100);
//    }

//    for(int test1=0;test1<5000;test1++){
//          //1s延时
//    }

//    /*for (i = 0; i < 5; i++)//自动零偏
//    {
//        HAL_UART_Transmit(&huart2, &get_zero[i], 1, 100);
//    }

//    for(int test1=0;test1<20000;test1++){
//      for(int test2=0;test2<30000;test2++){
//          //31s延时
//      }
//    }*/

//    for (i = 0; i < 5; i++) // z轴清零
//    {
//        HAL_UART_Transmit(&huart2, &tempData[i], 1, 100);
//    }

//    for(int test1=0;test1<5000;test1++){
//          //1s延时
//    }

//    for (i = 0; i < 5; i++)//解锁寄存器
//    {
//        HAL_UART_Transmit(&huart2, &unlock[i], 1, 100);
//    }

//    for(int test1=0;test1<5000;test1++){
//          //1s延时
//    }

//    for (i = 0; i < 5; i++)// 保存
//    {
//        HAL_UART_Transmit(&huart2, &save_zero[i], 1, 100);
//    }

//  //osDelay(500);
//  //set_angle(4, 60);


//}
