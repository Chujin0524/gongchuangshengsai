#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H

#define COM_BLUETOOTH	COM1		/* 选择串口 */

//#define BLUETOOTH_TO_COM1_EN

typedef void (*BLUETOOTH_HandleFrameCallback) (uint8_t *pucFrameData, uint8_t ucFrameLength);

/* 供外部调用的函数声明 */
void bsp_InitBLUETOOTH( BLUETOOTH_HandleFrameCallback _BLUETOOTH_HandleFrame );
uint8_t BLUETOOTH_ReadData( void );
void BLUETOOTH_SendData( uint8_t* _databuf, uint16_t _len );

#endif
