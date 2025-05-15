#ifndef __BSP_UART_DMA_H__
#define __BSP_UART_DMA_H__

#include "main.h"

#define	UART1_DMA_EN	1
#define	UART2_DMA_EN	1
#define	UART3_DMA_EN	1
#define	UART4_DMA_EN	1
#define	UART5_DMA_EN	1
#define	UART6_DMA_EN	0

/* 定义串口FIFO缓冲区大小 */
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

/* 定义串口帧解析状态机 */
typedef enum 
{
    STATE_IDLE = 0,
    
    /* 字节流状态 */
    STATE_HEADER_1,
    STATE_HEADER_2,
    STATE_LENGTH,
    STATE_CMD,
    STATE_DATA,
    STATE_CHECKSUM,
    STATE_TAIL_1,
    STATE_TAIL_2,
    
    /* 字符串状态 */
    STATE_RECEIVING,
    STATE_END,
}UART_PARSE_STATE_E;

/* 帧结构体 */
typedef struct
{
    __IO uint8_t ucFrameIndex;       //索引
	__IO uint8_t ucFrameHeader[2];   //帧头1 
	__IO uint8_t ucFrameLength;      //帧长度（不含帧头帧尾校验位）
    uint8_t *pucFrameData;           //数据域
    __IO uint8_t ucCheckSum;         //校验和  校验和之前（不包括校验和)的数据之和
	__IO uint8_t ucFrameTail[2];     //帧尾 
}FRAME_T;

/* 定义端口号 */
typedef enum
{
	COM1 = 0,	/* USART1 */
	COM2 = 1,	/* USART2 */
	COM3 = 2,	/* USART3 */
	COM4 = 3,	/* UART4 */
	COM5 = 4,	/* UART5 */
	COM6 = 5,	/* USART6 */
}COM_PORT_E;

/* 串口设备结构体 */
typedef struct
{
	UART_HandleTypeDef *huart;		/* STM32内部串口设备指针 */
	uint8_t *pRxBuf;			/* 接收缓冲区 */
	uint16_t usRxBufSize;		/* 接收缓冲区大小 */

	__IO uint16_t usRxWrite;	/* 接收缓冲区写指针 */
	__IO uint16_t usRxRead;		/* 接收缓冲区读指针 */
	__IO uint16_t usRxBuffLeft;	/* 接收缓冲区剩余空间 */
}UART_T;

//供外部调用的函数
void bsp_InitUart(void);
void comClearRxFifo(COM_PORT_E _ucPort);
void comSendBuf(COM_PORT_E _ucPort, uint8_t *_ucaBuf, uint16_t _usLen);
uint8_t comGetChar(COM_PORT_E _ucPort, uint8_t *_pByte);

#endif /* __BSP_UART_DMA_H__ */

