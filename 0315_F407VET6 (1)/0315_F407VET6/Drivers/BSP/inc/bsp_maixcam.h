#ifndef __BSP_MAIXCAM_H
#define __BSP_MAIXCAM_H

#define COM_MAIXCAM    COM2        /* ѡ�񴮿� */

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
    MAIXCAM_ELEMENT ucElement;        //Ԫ��
    uint8_t ucColor;            //��ɫ
}MAIXCAM_ID_T; 

typedef void (*MAIXCAM_HandleFrameCallback) (uint8_t *pucFrameData, uint8_t ucFrameLength);

/* ���ⲿ���õĺ������� */
void bsp_InitMAIXCAM( MAIXCAM_HandleFrameCallback _MAIXCAM_HandleFrame );
void MAIXCAM_SendCmd( MAIXCAM_ELEMENT _ucElement, uint8_t _ucColor );
uint8_t MAIXCAM_ReadData( void );

#endif

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
