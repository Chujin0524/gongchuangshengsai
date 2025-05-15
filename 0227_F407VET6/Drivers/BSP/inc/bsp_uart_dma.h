#ifndef __BSP_UART_DMA_H__
#define __BSP_UART_DMA_H__

#include "main.h"

#define	UART1_DMA_EN	1
#define	UART2_DMA_EN	1
#define	UART3_DMA_EN	1
#define	UART4_DMA_EN	1
#define	UART5_DMA_EN	1
#define	UART6_DMA_EN	0

/* ���崮��FIFO��������С */
#if UART1_DMA_EN == 1
	#define UART1_RX_BUF_SIZE	1*1024
#endif

#if UART2_DMA_EN == 1
	#define UART2_RX_BUF_SIZE	1
#endif

#if UART3_DMA_EN == 1
	#define UART3_RX_BUF_SIZE	1*1024
#endif

#if UART4_DMA_EN == 1
	#define UART4_RX_BUF_SIZE	1*1024
#endif

#if UART5_DMA_EN == 1
	#define UART5_RX_BUF_SIZE	1*1024
#endif

/* ���崮��֡����״̬�� */
typedef enum 
{
    STATE_IDLE = 0,
    
    /* �ֽ���״̬ */
    STATE_HEADER_1,
    STATE_HEADER_2,
    STATE_LENGTH,
    STATE_CMD,
    STATE_DATA,
    STATE_CHECKSUM,
    STATE_TAIL_1,
    STATE_TAIL_2,
    
    /* �ַ���״̬ */
    STATE_RECEIVING,
    STATE_END,
}UART_PARSE_STATE_E;

/* ֡�ṹ�� */
typedef struct
{
    __IO uint8_t ucFrameIndex;       //����
	__IO uint8_t ucFrameHeader[2];   //֡ͷ1 
	__IO uint8_t ucFrameLength;      //֡���ȣ�����֡ͷ֡βУ��λ��
    uint8_t *pucFrameData;           //������
    __IO uint8_t ucCheckSum;         //У���  У���֮ǰ��������У���)������֮��
	__IO uint8_t ucFrameTail[2];     //֡β 
}FRAME_T;

/* ����˿ں� */
typedef enum
{
	COM1 = 0,	/* USART1 */
	COM2 = 1,	/* USART2 */
	COM3 = 2,	/* USART3 */
	COM4 = 3,	/* UART4 */
	COM5 = 4,	/* UART5 */
	COM6 = 5,	/* USART6 */
}COM_PORT_E;

/* �����豸�ṹ�� */
typedef struct
{
	UART_HandleTypeDef *huart;		/* STM32�ڲ������豸ָ�� */
	uint8_t *pRxBuf;			/* ���ջ����� */
	uint16_t usRxBufSize;		/* ���ջ�������С */

	__IO uint16_t usRxWrite;	/* ���ջ�����дָ�� */
	__IO uint16_t usRxRead;		/* ���ջ�������ָ�� */
	__IO uint16_t usRxBuffLeft;	/* ���ջ�����ʣ��ռ� */
}UART_T;

//���ⲿ���õĺ���
void bsp_InitUart(void);
void comClearRxFifo(COM_PORT_E _ucPort);
void comSendBuf(COM_PORT_E _ucPort, uint8_t *_ucaBuf, uint16_t _usLen);
uint8_t comGetChar(COM_PORT_E _ucPort, uint8_t *_pByte);

#endif /* __BSP_UART_DMA_H__ */

