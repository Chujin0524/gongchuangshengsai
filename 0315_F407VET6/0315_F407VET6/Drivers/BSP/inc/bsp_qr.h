#ifndef __QR_H
#define __QR_H

#define COM_QR	COM5		/* ѡ�񴮿� */

//#define QR_TO_COM1_EN

typedef void (*QR_HandleFrameCallback) (uint8_t *pucFrameData, uint8_t ucFrameLength);

/* ���ⲿ���õĺ������� */
void bsp_InitQR( QR_HandleFrameCallback _QR_HandleFrame );
void QR_SendData( uint8_t* _databuf, uint16_t _len );
uint8_t QR_SendAndRead( uint8_t *_pucBuf );

#endif
