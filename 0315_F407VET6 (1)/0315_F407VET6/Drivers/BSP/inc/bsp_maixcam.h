#ifndef __BSP_MAIXCAM_H
#define __BSP_MAIXCAM_H

#define COM_MAIXCAM    COM2        /* 选择串口 */

//#define MAIXCAM_TO_COM1_EN

typedef enum
{
    ELEMENT_NONE = 0,
    BLOB,
    CIRCLE,
    ELEMENT_ALL
}MAIXCAM_ELEMENT;

typedef struct
{
    MAIXCAM_ELEMENT ucElement;        //元素
    uint8_t ucColor;            //颜色
}MAIXCAM_ID_T; 

typedef void (*MAIXCAM_HandleFrameCallback) (uint8_t *pucFrameData, uint8_t ucFrameLength);

/* 供外部调用的函数声明 */
void bsp_InitMAIXCAM( MAIXCAM_HandleFrameCallback _MAIXCAM_HandleFrame );
void MAIXCAM_SendCmd( MAIXCAM_ELEMENT _ucElement, uint8_t _ucColor );
uint8_t MAIXCAM_ReadData( void );

#endif

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
