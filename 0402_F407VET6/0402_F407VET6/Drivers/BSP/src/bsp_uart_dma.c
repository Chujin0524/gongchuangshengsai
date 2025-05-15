#include "main.h"
#include "bsp_hwt101.h"

/* 定义每个串口结构体变量 */
#if UART1_DMA_EN == 1
static UART_T s_tUart1;
static uint8_t s_RxBuf1[UART1_RX_BUF_SIZE];        /* 接收缓冲区 */
#endif

#if UART2_DMA_EN == 1
static UART_T s_tUart2;
static uint8_t s_RxBuf2[UART2_RX_BUF_SIZE];        /* 接收缓冲区 */
#endif

#if UART3_DMA_EN == 1
static UART_T s_tUart3;
static uint8_t s_RxBuf3[UART3_RX_BUF_SIZE];        /* 接收缓冲区 */
#endif

#if UART4_DMA_EN == 1
static UART_T s_tUart4;
static uint8_t s_RxBuf4[UART4_RX_BUF_SIZE];        /* 接收缓冲区 */
#endif

#if UART5_DMA_EN == 1
static UART_T s_tUart5;
static uint8_t s_RxBuf5[UART5_RX_BUF_SIZE];        /* 接收缓冲区 */
#endif

static void UartVarInit( void );
static void InitHardUart( void );

static UART_T* ComToUart( COM_PORT_E _ucPort );
static UART_T* HuartToUart( UART_HandleTypeDef *huart );

static void UartSend( UART_T *_pUart, uint8_t* _ucaBuf, uint16_t _usLen );
static uint8_t UartGetChar( UART_T *_pUart, uint8_t* _pByte );

void bsp_InitUart( void )
{
    UartVarInit();
    InitHardUart();
}

static void UartVarInit( void )
{
#if UART1_DMA_EN == 1
    s_tUart1.huart = &huart1;                    /* STM32 串口设备 */
    s_tUart1.pRxBuf = s_RxBuf1;                    /* 接收缓冲区指针 */
    s_tUart1.usRxBufSize = UART1_RX_BUF_SIZE;    /* 接收缓冲区大小 */
    s_tUart1.usRxWrite = 0;                        /* 接收FIFO写索引 */
    s_tUart1.usRxRead = 0;                        /* 接收FIFO读索引 */
    s_tUart1.usRxBuffLeft = 0;                    /* 接收缓冲区剩余空间 */
#endif
#if UART2_DMA_EN == 1
    s_tUart2.huart = &huart2;                    /* STM32 串口设备 */
    s_tUart2.pRxBuf = s_RxBuf2;                    /* 接收缓冲区指针 */
    s_tUart2.usRxBufSize = UART2_RX_BUF_SIZE;    /* 接收缓冲区大小 */
    s_tUart2.usRxWrite = 0;                        /* 接收FIFO写索引 */
    s_tUart2.usRxRead = 0;                        /* 接收FIFO读索引 */
    s_tUart2.usRxBuffLeft = 0;                    /* 接收缓冲区剩余空间 */
#endif
#if UART3_DMA_EN == 1
    s_tUart3.huart = &huart3;                    /* STM32 串口设备 */
    s_tUart3.pRxBuf = s_RxBuf3;                    /* 接收缓冲区指针 */
    s_tUart3.usRxBufSize = UART3_RX_BUF_SIZE;    /* 接收缓冲区大小 */
    s_tUart3.usRxWrite = 0;                        /* 接收FIFO写索引 */
    s_tUart3.usRxRead = 0;                        /* 接收FIFO读索引 */
    s_tUart3.usRxBuffLeft = 0;                    /* 接收缓冲区剩余空间 */
#endif
#if UART4_DMA_EN == 1
    s_tUart4.huart = &huart4;                    /* STM32 串口设备 */
    s_tUart4.pRxBuf = s_RxBuf4;                    /* 接收缓冲区指针 */
    s_tUart4.usRxBufSize = UART4_RX_BUF_SIZE;    /* 接收缓冲区大小 */
    s_tUart4.usRxWrite = 0;                        /* 接收FIFO写索引 */
    s_tUart4.usRxRead = 0;                        /* 接收FIFO读索引 */
    s_tUart4.usRxBuffLeft = 0;                    /* 接收缓冲区剩余空间 */
#endif
#if UART5_DMA_EN == 1
    s_tUart5.huart = &huart5;                    /* STM32 串口设备 */
    s_tUart5.pRxBuf = s_RxBuf5;                    /* 接收缓冲区指针 */
    s_tUart5.usRxBufSize = UART5_RX_BUF_SIZE;    /* 接收缓冲区大小 */
    s_tUart5.usRxWrite = 0;                        /* 接收FIFO写索引 */
    s_tUart5.usRxRead = 0;                        /* 接收FIFO读索引 */
    s_tUart5.usRxBuffLeft = 0;                    /* 接收缓冲区剩余空间 */
#endif
}

static void InitHardUart( void )
{
#if UART1_DMA_EN == 1
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, s_tUart1.pRxBuf, s_tUart1.usRxBufSize ); //开启DMA接收
    __HAL_DMA_DISABLE_IT( huart1.hdmarx, DMA_IT_HT );                             //关闭半满中断
#endif
#if UART2_DMA_EN == 1
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, s_tUart2.pRxBuf, s_tUart2.usRxBufSize ); //开启DMA接收
    __HAL_DMA_DISABLE_IT( huart2.hdmarx, DMA_IT_HT );                             //关闭半满中断
#endif
#if UART3_DMA_EN == 1
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, s_tUart3.pRxBuf, s_tUart3.usRxBufSize ); //开启DMA接收
    __HAL_DMA_DISABLE_IT( huart3.hdmarx, DMA_IT_HT );                             //关闭半满中断
#endif
#if UART4_DMA_EN == 1
    HAL_UARTEx_ReceiveToIdle_DMA(&huart4, s_tUart4.pRxBuf, s_tUart4.usRxBufSize ); //开启DMA接收
    __HAL_DMA_DISABLE_IT( huart4.hdmarx, DMA_IT_HT );                             //关闭半满中断
#endif
#if UART5_DMA_EN == 1
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, s_tUart5.pRxBuf, s_tUart5.usRxBufSize ); //开启DMA接收
    __HAL_DMA_DISABLE_IT( huart5.hdmarx, DMA_IT_HT );                             //关闭半满中断
#endif
}

/*
*********************************************************************************************************
*    函 数 名: ComToUart
*    功能说明: 将COM端口号转换为UART指针
*    形    参: _ucPort: 端口号(COM1 - COM8)
*    返 回 值: uart指针
*********************************************************************************************************
*/
static UART_T* ComToUart( COM_PORT_E _ucPort )
{
    if( _ucPort == COM1 )
    {
#if UART1_DMA_EN == 1
        return &s_tUart1;
#else
        return 0;
#endif
    }
    else if( _ucPort == COM2 )
    {
#if UART2_DMA_EN == 1
        return &s_tUart2;
#else
        return 0;
#endif
    }
    else if( _ucPort == COM3 )
    {
#if UART3_DMA_EN == 1
        return &s_tUart3;
#else
        return 0;
#endif
    }
    else if( _ucPort == COM4 )
    {
#if UART4_DMA_EN == 1
        return &s_tUart4;
#else
        return 0;
#endif
    }
    else if( _ucPort == COM5 )
    {
#if UART5_DMA_EN == 1
        return &s_tUart5;
#else
        return 0;
#endif
    }
    else
    {
        Error_Handler();
        return 0;
    }
}

static UART_T* HuartToUart( UART_HandleTypeDef *huart )
{
#if UART1_DMA_EN == 1
    if( huart == &huart1 )
    {
        return &s_tUart1;
    }
#endif
#if UART2_DMA_EN == 1
    else if( huart == &huart2 )
    {
        return &s_tUart2;
    }
#endif
#if UART3_DMA_EN == 1
    else if( huart == &huart3 )
    {
        return &s_tUart3;
    }
#endif
#if UART4_DMA_EN == 1
    else if( huart == &huart4 )
    {
        return &s_tUart4;
    }
#endif
#if UART5_DMA_EN == 1
    else if( huart == &huart5 )
    {
        return &s_tUart5;
    }
#endif
    Error_Handler();
    return 0;
}

/*
*********************************************************************************************************
*    函 数 名: UartSend
*    功能说明: 填写数据到UART发送缓冲区,并启动发送中断。中断处理函数发送完毕后，自动关闭发送中断
*    形    参: 无
*    返 回 值: 无
*********************************************************************************************************
*/
static void UartSend( UART_T *_pUart, uint8_t* _ucaBuf, uint16_t _usLen )
{
    HAL_UART_Transmit_DMA( _pUart->huart, _ucaBuf, _usLen );
}

/*
*********************************************************************************************************
*    函 数 名: comSendBuf
*    功能说明: 向串口发送一组数据。数据放到发送缓冲区后立即返回，由中断服务程序在后台完成发送
*    形    参: _ucPort: 端口号(COM1 - COM8)
*              _ucaBuf: 待发送的数据缓冲区
*              _usLen : 数据长度
*    返 回 值: 无
*********************************************************************************************************
*/
void comSendBuf( COM_PORT_E _ucPort, uint8_t* _ucaBuf, uint16_t _usLen )
{
    UART_T *pUart;

    pUart = ComToUart( _ucPort );
    if( pUart == 0 )
    {
        return;
    }

    UartSend( pUart, _ucaBuf, _usLen );
}

/*
*********************************************************************************************************
*    函 数 名: comGetChar
*    功能说明: 从接收缓冲区读取1字节，非阻塞。无论有无数据均立即返回。
*    形    参: _ucPort: 端口号(COM1 - COM8)
*              _pByte: 接收到的数据存放在这个地址
*    返 回 值: 0 表示无数据, 1 表示读取到有效字节
*********************************************************************************************************
*/
uint8_t comGetChar( COM_PORT_E _ucPort, uint8_t* _pByte )
{
    UART_T *pUart;

    pUart = ComToUart( _ucPort );
    if( pUart == 0 )
    {
        return 0;
    }

    return UartGetChar( pUart, _pByte );
}

/*
*********************************************************************************************************
*    函 数 名: comClearRxFifo
*    功能说明: 清零串口接收缓冲区
*    形    参: _ucPort: 端口号(COM1 - COM6)
*    返 回 值: 无
*********************************************************************************************************
*/
void comClearRxFifo( COM_PORT_E _ucPort )
{
    UART_T *pUart;

    pUart = ComToUart( _ucPort );
    if( pUart == 0 )
    {
        return;
    }

    pUart->usRxRead = pUart->usRxWrite;
}

/*
*********************************************************************************************************
*    函 数 名: UartGetChar
*    功能说明: 从串口接收缓冲区读取1字节数据 （用于主程序调用）
*    形    参: _pUart : 串口设备
*              _pByte : 存放读取数据的指针
*    返 回 值: 0 表示无数据  1表示读取到数据
*********************************************************************************************************
*/
static uint8_t UartGetChar( UART_T *_pUart, uint8_t* _pByte )
{
    /* 如果读和写索引相同，则返回0 */
    if( _pUart->usRxRead == _pUart->usRxWrite )
    {
        return 0;
    }
    else
    {
        *_pByte = _pUart->pRxBuf[_pUart->usRxRead];        /* 从串口接收FIFO取1个数据 */

        /* 改写FIFO读索引 */
        if(++_pUart->usRxRead >= _pUart->usRxBufSize )
        {
            _pUart->usRxRead = 0;
        }
        return 1;
    }
}

void HAL_UARTEx_RxEventCallback( UART_HandleTypeDef *huart, uint16_t Size )
{
    UNUSED( Size );
    UART_T *pUart;

    pUart = HuartToUart( huart );
    // 更新接收缓冲区的写入索引，等于缓冲区总长度减去huart->hdmarx->Instance->CNDTR
    pUart->usRxBuffLeft = huart->hdmarx->Instance->NDTR;
    pUart->usRxWrite = pUart->usRxBufSize - pUart->usRxBuffLeft;
}

int fputc( int ch, FILE *f )
{
    UNUSED( f );
    HAL_UART_Transmit(&huart1, ( uint8_t* )&ch, 1, 0xffff );
    return( ch );
}
