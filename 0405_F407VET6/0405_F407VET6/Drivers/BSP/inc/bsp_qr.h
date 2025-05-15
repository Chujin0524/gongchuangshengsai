#ifndef __QR_H
#define __QR_H

#define COM_QR	COM5		/* 选择串口 */

//#define QR_TO_COM1_EN

/* 供外部调用的函数声明 */
void bsp_InitQR( void );
void QR_SendData( uint8_t* _databuf, uint16_t _len );
uint8_t QR_SendAndRead( uint8_t *_pucBuf );

#endif
