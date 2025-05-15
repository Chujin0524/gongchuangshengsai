#include "main.h"
#include "bsp_hwt101.h"

/* ����ÿ�����ڽṹ����� */
#if UART1_DMA_EN == 1
static UART_T s_tUart1;
static uint8_t s_RxBuf1[UART1_RX_BUF_SIZE];        /* ���ջ����� */
#endif

#if UART2_DMA_EN == 1
static UART_T s_tUart2;
static uint8_t s_RxBuf2[UART2_RX_BUF_SIZE];        /* ���ջ����� */
#endif

#if UART3_DMA_EN == 1
static UART_T s_tUart3;
static uint8_t s_RxBuf3[UART3_RX_BUF_SIZE];        /* ���ջ����� */
#endif

#if UART4_DMA_EN == 1
static UART_T s_tUart4;
static uint8_t s_RxBuf4[UART4_RX_BUF_SIZE];        /* ���ջ����� */
#endif

#if UART5_DMA_EN == 1
static UART_T s_tUart5;
static uint8_t s_RxBuf5[UART5_RX_BUF_SIZE];        /* ���ջ����� */
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
    s_tUart1.huart = &huart1;                    /* STM32 �����豸 */
    s_tUart1.pRxBuf = s_RxBuf1;                    /* ���ջ�����ָ�� */
    s_tUart1.usRxBufSize = UART1_RX_BUF_SIZE;    /* ���ջ�������С */
    s_tUart1.usRxWrite = 0;                        /* ����FIFOд���� */
    s_tUart1.usRxRead = 0;                        /* ����FIFO������ */
    s_tUart1.usRxBuffLeft = 0;                    /* ���ջ�����ʣ��ռ� */
#endif
#if UART2_DMA_EN == 1
    s_tUart2.huart = &huart2;                    /* STM32 �����豸 */
    s_tUart2.pRxBuf = s_RxBuf2;                    /* ���ջ�����ָ�� */
    s_tUart2.usRxBufSize = UART2_RX_BUF_SIZE;    /* ���ջ�������С */
    s_tUart2.usRxWrite = 0;                        /* ����FIFOд���� */
    s_tUart2.usRxRead = 0;                        /* ����FIFO������ */
    s_tUart2.usRxBuffLeft = 0;                    /* ���ջ�����ʣ��ռ� */
#endif
#if UART3_DMA_EN == 1
    s_tUart3.huart = &huart3;                    /* STM32 �����豸 */
    s_tUart3.pRxBuf = s_RxBuf3;                    /* ���ջ�����ָ�� */
    s_tUart3.usRxBufSize = UART3_RX_BUF_SIZE;    /* ���ջ�������С */
    s_tUart3.usRxWrite = 0;                        /* ����FIFOд���� */
    s_tUart3.usRxRead = 0;                        /* ����FIFO������ */
    s_tUart3.usRxBuffLeft = 0;                    /* ���ջ�����ʣ��ռ� */
#endif
#if UART4_DMA_EN == 1
    s_tUart4.huart = &huart4;                    /* STM32 �����豸 */
    s_tUart4.pRxBuf = s_RxBuf4;                    /* ���ջ�����ָ�� */
    s_tUart4.usRxBufSize = UART4_RX_BUF_SIZE;    /* ���ջ�������С */
    s_tUart4.usRxWrite = 0;                        /* ����FIFOд���� */
    s_tUart4.usRxRead = 0;                        /* ����FIFO������ */
    s_tUart4.usRxBuffLeft = 0;                    /* ���ջ�����ʣ��ռ� */
#endif
#if UART5_DMA_EN == 1
    s_tUart5.huart = &huart5;                    /* STM32 �����豸 */
    s_tUart5.pRxBuf = s_RxBuf5;                    /* ���ջ�����ָ�� */
    s_tUart5.usRxBufSize = UART5_RX_BUF_SIZE;    /* ���ջ�������С */
    s_tUart5.usRxWrite = 0;                        /* ����FIFOд���� */
    s_tUart5.usRxRead = 0;                        /* ����FIFO������ */
    s_tUart5.usRxBuffLeft = 0;                    /* ���ջ�����ʣ��ռ� */
#endif
}

static void InitHardUart( void )
{
#if UART1_DMA_EN == 1
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, s_tUart1.pRxBuf, s_tUart1.usRxBufSize ); //����DMA����
    __HAL_DMA_DISABLE_IT( huart1.hdmarx, DMA_IT_HT );                             //�رհ����ж�
#endif
#if UART2_DMA_EN == 1
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, s_tUart2.pRxBuf, s_tUart2.usRxBufSize ); //����DMA����
    __HAL_DMA_DISABLE_IT( huart2.hdmarx, DMA_IT_HT );                             //�رհ����ж�
#endif
#if UART3_DMA_EN == 1
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, s_tUart3.pRxBuf, s_tUart3.usRxBufSize ); //����DMA����
    __HAL_DMA_DISABLE_IT( huart3.hdmarx, DMA_IT_HT );                             //�رհ����ж�
#endif
#if UART4_DMA_EN == 1
    HAL_UARTEx_ReceiveToIdle_DMA(&huart4, s_tUart4.pRxBuf, s_tUart4.usRxBufSize ); //����DMA����
    __HAL_DMA_DISABLE_IT( huart4.hdmarx, DMA_IT_HT );                             //�رհ����ж�
#endif
#if UART5_DMA_EN == 1
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, s_tUart5.pRxBuf, s_tUart5.usRxBufSize ); //����DMA����
    __HAL_DMA_DISABLE_IT( huart5.hdmarx, DMA_IT_HT );                             //�رհ����ж�
#endif
}

/*
*********************************************************************************************************
*    �� �� ��: ComToUart
*    ����˵��: ��COM�˿ں�ת��ΪUARTָ��
*    ��    ��: _ucPort: �˿ں�(COM1 - COM8)
*    �� �� ֵ: uartָ��
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
*    �� �� ��: UartSend
*    ����˵��: ��д���ݵ�UART���ͻ�����,�����������жϡ��жϴ�����������Ϻ��Զ��رշ����ж�
*    ��    ��: ��
*    �� �� ֵ: ��
*********************************************************************************************************
*/
static void UartSend( UART_T *_pUart, uint8_t* _ucaBuf, uint16_t _usLen )
{
    HAL_UART_Transmit_DMA( _pUart->huart, _ucaBuf, _usLen );
}

/*
*********************************************************************************************************
*    �� �� ��: comSendBuf
*    ����˵��: �򴮿ڷ���һ�����ݡ����ݷŵ����ͻ��������������أ����жϷ�������ں�̨��ɷ���
*    ��    ��: _ucPort: �˿ں�(COM1 - COM8)
*              _ucaBuf: �����͵����ݻ�����
*              _usLen : ���ݳ���
*    �� �� ֵ: ��
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
*    �� �� ��: comGetChar
*    ����˵��: �ӽ��ջ�������ȡ1�ֽڣ��������������������ݾ��������ء�
*    ��    ��: _ucPort: �˿ں�(COM1 - COM8)
*              _pByte: ���յ������ݴ���������ַ
*    �� �� ֵ: 0 ��ʾ������, 1 ��ʾ��ȡ����Ч�ֽ�
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
*    �� �� ��: comClearRxFifo
*    ����˵��: ���㴮�ڽ��ջ�����
*    ��    ��: _ucPort: �˿ں�(COM1 - COM6)
*    �� �� ֵ: ��
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
*    �� �� ��: UartGetChar
*    ����˵��: �Ӵ��ڽ��ջ�������ȡ1�ֽ����� ��������������ã�
*    ��    ��: _pUart : �����豸
*              _pByte : ��Ŷ�ȡ���ݵ�ָ��
*    �� �� ֵ: 0 ��ʾ������  1��ʾ��ȡ������
*********************************************************************************************************
*/
static uint8_t UartGetChar( UART_T *_pUart, uint8_t* _pByte )
{
    /* �������д������ͬ���򷵻�0 */
    if( _pUart->usRxRead == _pUart->usRxWrite )
    {
        return 0;
    }
    else
    {
        *_pByte = _pUart->pRxBuf[_pUart->usRxRead];        /* �Ӵ��ڽ���FIFOȡ1������ */

        /* ��дFIFO������ */
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
    // ���½��ջ�������д�����������ڻ������ܳ��ȼ�ȥhuart->hdmarx->Instance->CNDTR
    pUart->usRxBuffLeft = huart->hdmarx->Instance->NDTR;
    pUart->usRxWrite = pUart->usRxBufSize - pUart->usRxBuffLeft;
}

int fputc( int ch, FILE *f )
{
    UNUSED( f );
    HAL_UART_Transmit(&huart1, ( uint8_t* )&ch, 1, 0xffff );
    return( ch );
}
