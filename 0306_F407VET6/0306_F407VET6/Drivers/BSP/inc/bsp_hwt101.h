#ifndef _BSP_hwt101_h
#define _BSP_hwt101_h

#define COM_HWT101    COM3        /* Ñ¡Ôñ´®¿Ú */

//#define HWT101_TO_COM1_EN

typedef void (*HWT101_HandleFrameCallback) (uint8_t *pucFrameData, uint8_t ucFrameLength);

void bsp_InitHWT101( HWT101_HandleFrameCallback _HWT101_HandleFrame );
void HWT101_SendData( uint8_t* _databuf, uint16_t _len );
uint8_t HWT101_ReadData( void );
extern int yaw0;
void hwt_init(void);

#endif
