#ifndef __APP_MAIXCAM_H
#define __APP_MAIXCAM_H

#define CAL   0
#define GRAB  1

#define MoveCar 0
#define MoveXR  1

typedef struct
{
    uint8_t ucMoveFlag;              //移动标志位   0-移动车; 1-移动X方向和R方向步进电机
    uint8_t ucSendFlag;              //是否发送标志

    uint8_t ucCalFlag;               //校准标志位
    uint8_t ucStableCount;           //稳定计数

    uint16_t usCoord_X;              //圆心X
    uint16_t usCoord_Y;              //圆心Y
    int16_t dCoord_X;                //圆心X与中心点之差
    int16_t dCoord_Y;                //圆心Y与中心点之差
    int16_t dCoord_R;                //圆心与中心点之角度差

    uint16_t usBlobCenter_X;         //转盘区中心点X
    uint16_t usBlobCenter_Y;         //转盘区中心点Y

//    uint16_t usBlobCenter_R_X;         //转盘区中心点X红色
//    uint16_t usBlobCenter_R_Y;         //转盘区中心点Y
//    uint16_t usBlobCenter_B_X;         //转盘区中心点X蓝色
//    uint16_t usBlobCenter_B_Y;         //转盘区中心点Y
//    uint16_t usBlobCenter_G_X;         //转盘区中心点X绿色
//    uint16_t usBlobCenter_G_Y;         //转盘区中心点Y
	
    uint8_t ucBlobCenterErrCal;      //转盘区校准误差允许（范围小，以便精准校准位置）
    uint8_t ucBlobCenterErrGrab;     //转盘区抓取误差允许
    uint8_t ucBlobStableThreshold;   //转盘区稳定次数阈值，调节该参数以使小车不会随物块移动

    uint16_t usCircleCenter_X;       //色环区中心点X
    uint16_t usCircleCenter_Y;       //色环区中心点Y
    uint8_t ucCircleCenterErrCal;    //色环区校准误差允许
    uint8_t ucCircleStableThreshold;  //色环区校准对准次数，调节该参数以使小车精确对准中心
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