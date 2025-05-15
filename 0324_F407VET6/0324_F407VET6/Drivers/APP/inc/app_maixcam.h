#ifndef __APP_MAIXCAM_H
#define __APP_MAIXCAM_H

#define CAL   0
#define GRAB  1

#define MoveCar 0
#define MoveXR  1

typedef struct
{
    uint8_t ucMoveFlag;              //�ƶ���־λ   0-�ƶ���; 1-�ƶ�X�����R���򲽽����
    uint8_t ucSendFlag;              //�Ƿ��ͱ�־

    uint8_t ucCalFlag;               //У׼��־λ
    uint8_t ucStableCount;           //�ȶ�����

    uint16_t usCoord_X;              //Բ��X
    uint16_t usCoord_Y;              //Բ��Y
    int16_t dCoord_X;                //Բ��X�����ĵ�֮��
    int16_t dCoord_Y;                //Բ��Y�����ĵ�֮��
    int16_t dCoord_R;                //Բ�������ĵ�֮�ǶȲ�

    uint16_t usBlobCenter_X;         //ת�������ĵ�X
    uint16_t usBlobCenter_Y;         //ת�������ĵ�Y

//    uint16_t usBlobCenter_R_X;         //ת�������ĵ�X��ɫ
//    uint16_t usBlobCenter_R_Y;         //ת�������ĵ�Y
//    uint16_t usBlobCenter_B_X;         //ת�������ĵ�X��ɫ
//    uint16_t usBlobCenter_B_Y;         //ת�������ĵ�Y
//    uint16_t usBlobCenter_G_X;         //ת�������ĵ�X��ɫ
//    uint16_t usBlobCenter_G_Y;         //ת�������ĵ�Y
	
    uint8_t ucBlobCenterErrCal;      //ת����У׼���������ΧС���Ա㾫׼У׼λ�ã�
    uint8_t ucBlobCenterErrGrab;     //ת����ץȡ�������
    uint8_t ucBlobStableThreshold;   //ת�����ȶ�������ֵ�����ڸò�����ʹС������������ƶ�

    uint16_t usCircleCenter_X;       //ɫ�������ĵ�X
    uint16_t usCircleCenter_Y;       //ɫ�������ĵ�Y
    uint8_t ucCircleCenterErrCal;    //ɫ����У׼�������
    uint8_t ucCircleStableThreshold;  //ɫ����У׼��׼���������ڸò�����ʹС����ȷ��׼����
} MAIXCAM_DATA_T;

void APP_MAIXCAM_Init( void );
MAIXCAM_DATA_T *APP_MAIXCAM_GetPoint(void);
void APP_MaixCAMControl( void );
void APP_MAIXCAM_SendAndRead( uint8_t _ucElement, uint8_t _ucColor );
void APP_MAIXCAM_Cal( uint8_t _ucElement, uint8_t _ucColor, uint8_t _ucIsCalFlag, uint8_t _ucMoveFlag );

extern __IO int16_t g_dX;
extern __IO int16_t g_dY;
extern __IO int16_t g_dR;

#endif