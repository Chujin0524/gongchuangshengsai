#ifndef __APP_HWT101_H
#define __APP_HWT101_H

#define IS_NEGATIVE_HEX(hexValue) ((hexValue & 0x80) != 0)

void APP_HWT101_Init( void );
uint8_t APP_HWT101_Read( void );
float *APP_HWT101_GetAnglePoint(void);

extern __IO float g_fGyro_z;

#endif