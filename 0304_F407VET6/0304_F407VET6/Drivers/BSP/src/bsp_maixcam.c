/*
*********************************************************************************************************
*
*   模块名称 : MAIXCAM 串口WIFI模块驱动程序
*   文件名称 : bsp_MAIXCAM.c
*   版    本 : V1.1
*   说    明 : 封装 MAIXCAM 模块相关的命令
*
*********************************************************************************************************
*/

#include "main.h"

/* MAIXCAM 模块接线图


        UTXD   ---  PA3/USART2_RX
        URXD   ---  PA2/USART2_TX

    模块缺省波特率 115200

*/

static MAIXCAM_HandleFrameCallback s_MAIXCAM_HandleFrame = NULL;

#define MAIXCAM_FRAMEHEADER 0xFF
#define MAIXCAM_FRAMETAIL   0xFE
static uint8_t s_ucMaixCamFrameData[32] = {0};
static FRAME_T s_tMaixCamFrame =
{
    .ucFrameHeader[0] = MAIXCAM_FRAMEHEADER,
    .ucFrameTail[0] = MAIXCAM_FRAMETAIL,

    .ucFrameIndex = 0,
    .ucFrameLength = 0,
    .pucFrameData = s_ucMaixCamFrameData,
    .ucCheckSum = 0
};

/*
*********************************************************************************************************
*   函 数 名: bsp_InitMAIXCAM
*   功能说明: 初始化二维码扫描模块的串口，注册数据解析回调函数。该函数被 bsp_Init() 调用。
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitMAIXCAM( MAIXCAM_HandleFrameCallback _MAIXCAM_HandleFrame )
{
    //使用串口2，已在bsp_uart_fifo.c里面初始化，此处不必再进行初始化

    if( _MAIXCAM_HandleFrame != NULL )
    {
        s_MAIXCAM_HandleFrame = _MAIXCAM_HandleFrame;
    }
}

/*
*********************************************************************************************************
*   函 数 名: MAIXCAM_PrintRxData
*   功能说明: 打印STM32从MAIXCAM收到的数据到COM1串口，主要用于跟踪调试
*   形    参: _ch : 收到的数据
*   返 回 值: 无
*********************************************************************************************************
*/
static void MAIXCAM_PrintRxData( uint8_t _ch )
{
#ifdef MAIXCAM_TO_COM1_EN
    HAL_UART_Transmit(&huart1, &_ch, 1, 0xffff ); /* 将接收到数据打印到调试串口1 */
#endif
}

/*
*********************************************************************************************************
*   函 数 名: MAIXCAM_SendData
*   功能说明: 发送数据包
*   形    参: _databuf 数据
*            _len 数据长度
*   返 回 值: 无
*********************************************************************************************************
*/
static void MAIXCAM_SendData( uint8_t* _databuf, uint16_t _len )
{
    if( _len > 2048 )
    {
        _len = 2048;
    }

    comSendBuf( COM_MAIXCAM, _databuf, _len );
}

/*
*********************************************************************************************************
*   函 数 名: MAIXCAM_SendCmd
*   功能说明: 根据特定的颜色和元素，给MaixCAM发送指令，该函数在app_maixcam中被调用
*   形    参: _ucElement : 元素
*             _ucColor : 颜色
*   返 回 值: 无
*********************************************************************************************************
*/
void MAIXCAM_SendCmd( MAIXCAM_ELEMENT _ucElement, MAIXCAM_COLOR _ucColor )
{
    static char maixcam_tx_buf[256] = {0};       //相机发送缓存

    switch( _ucElement ) //根据_element选择识别什么东西
    {
    case BLOB: //圆环
        _ucElement = 1;
        break;

    case CIRCLE:        //物块
        _ucElement = 2;
        break;

    case ELEMENT_ALL:       //纯圆
        _ucElement = 3;
        break;

    default:
        break;
    }
    switch( _ucColor )  //根据_color选择识别什么颜色
    {
    case RED:
        _ucColor = 'R';
        break;

    case GREEN:
        _ucColor = 'G';
        break;

    case BLUE:
        _ucColor = 'B';
        break;

    case COLOR_ALL:     //纯圆
        _ucColor = 'A';
        break;

    default:
        break;
    }

    sprintf( maixcam_tx_buf, "mode=%d%c", _ucElement, _ucColor );
    MAIXCAM_SendData(( uint8_t* )maixcam_tx_buf, strlen( maixcam_tx_buf ) );
}

/*
*********************************************************************************************************
*   函 数 名: MAIXCAM_ReadData
*   功能说明: 读取串口数据，帧解析成功后调用回调函数进行数据解析，该函数在app_maixcam中被调用
*   形    参:  无
*   返 回 值: 无
*********************************************************************************************************
*/
uint8_t MAIXCAM_ReadData( void )
{
    static UART_PARSE_STATE_E s_ucState = STATE_IDLE;
    uint8_t ucData = 0;
    uint8_t ret = 0;

    /* 从 串口读取1个字节 comGetChar() 函数由 bsp_uart_fifo.c 实现 */
    while( 1 )
    {
        if( comGetChar( COM_MAIXCAM, &ucData ) )
        {
            MAIXCAM_PrintRxData( ucData );

            switch( s_ucState )
            {
            case STATE_IDLE:
                if( ucData == s_tMaixCamFrame.ucFrameHeader[0] ) //收到帧头
                {
                    s_ucState = STATE_HEADER_0;
                }
                else
                {
//                         LED_R_ON();
                }
                break;

            case STATE_HEADER_0:
                s_tMaixCamFrame.ucFrameLength = ucData;         //收到帧长（数据域长度，包括功能码和数据）
                s_ucState = STATE_LENGTH;
                break;

            case STATE_LENGTH:
                s_tMaixCamFrame.ucFrameIndex = 0;               //更新索引，准备接收数据域（包括功能码和数据）
                s_tMaixCamFrame.ucCheckSum = s_tMaixCamFrame.ucFrameHeader[0] + s_tMaixCamFrame.ucFrameLength + ucData;
                s_tMaixCamFrame.pucFrameData[s_tMaixCamFrame.ucFrameIndex++] = ucData; //收到功能码（如果有）
                s_ucState = STATE_DATA;
                break;

            case STATE_DATA:
                s_tMaixCamFrame.pucFrameData[s_tMaixCamFrame.ucFrameIndex++] = ucData;
                s_tMaixCamFrame.ucCheckSum += ucData;                                  //收到数据
                if( s_tMaixCamFrame.ucFrameIndex >= s_tMaixCamFrame.ucFrameLength )    //根据帧长截断
                {
//                    s_ucState = STATE_CHECKSUM;          //不加CheckSum了，不方便调
                    s_ucState = STATE_TAIL_0;
                }
                break;

            case STATE_CHECKSUM:
                if( ucData == ( s_tMaixCamFrame.ucCheckSum & 0xFF ) )     //校验和
                {
                    s_ucState = STATE_TAIL_0;
                }
                else
                {
//                        LED_R_ON();
                    s_ucState = STATE_IDLE; // 校验失败
                }
                break;

            case STATE_TAIL_0:
                if( ucData == s_tMaixCamFrame.ucFrameTail[0] )           //收到帧尾
                {
                    // 帧解析成功，处理数据
                    if( s_MAIXCAM_HandleFrame != NULL )
                    {
                        ret = 1;
                        s_MAIXCAM_HandleFrame( s_tMaixCamFrame.pucFrameData, s_tMaixCamFrame.ucFrameLength );
                    }
                }
                s_ucState = STATE_IDLE;
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

/*
*********************************************************************************************************
*   函 数 名: num2color
*   功能说明: 将字符转换为颜色枚举类型
*   形    参: _char : 字符
*             _ucColor : 颜色
*   返 回 值: 无
*********************************************************************************************************
*/
MAIXCAM_COLOR num2color( uint8_t _num )
{
    switch( _num )
    {
    case 1:
        return BLUE;
    case 2:
        return GREEN;
    case 3:
        return RED;
    default:
        return COLOR_NONE;
    }
}
