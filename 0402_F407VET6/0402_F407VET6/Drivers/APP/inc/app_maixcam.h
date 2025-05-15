#ifndef __APP_MAIXCAM_H
#define __APP_MAIXCAM_H

#define CAL   0
#define GRAB  1

#define MoveCar 0
#define MoveXR  1

#define BlobCalFlag 1
#define CircleCalFlag 2

typedef enum
{
    MAIXCAM_STATUS_Idle = 0, //0
    MAIXCAM_STATUS_BLOB_ArriveAndWaitCal,   //1
    MAIXCAM_STATUS_BLOB_Cal,                //2
    MAIXCAM_STATUS_Blob_WaitGrab1,          //3
    MAIXCAM_STATUS_Blob_Grab1,              //4
} MAIXCAM_STATUS_E;

typedef struct
{
    MAIXCAM_STATUS_E ucStatus;                //相机坐标解析状态
    uint8_t BlobDirMappingColor[4];   //颜色映射方向   0-空 1-左 2-中 3-右
    
    uint8_t BlobColorFir;            //到达转盘时物块颜色
    uint16_t usBlobCoord_X_Fir;              //圆心X
    uint16_t usBlobCoord_Y_Fir;              //圆心Y

    uint8_t BlobColorSec;            //校准完成物块颜色

    uint8_t BlobColorLeft;           //左间物块颜色
    uint8_t BlobColorMiddle;         //中间物块颜色
    uint8_t BlobColorRight;          //右边物块颜色

    uint8_t RotateSeq;               //旋转顺序

    uint8_t ucMoveFlag;              //移动标志位   0-移动车; 1-移动X方向和R方向步进电机
    uint8_t ucSendFlag;              //是否发送标志

    uint8_t ucCalFlag;               //校准标志位
    uint8_t ucStableCount;           //稳定计数

    uint8_t BlobColor;             //物块颜色
    uint16_t usCoord_X;              //圆心X
    uint16_t usCoord_Y;              //圆心Y
    int16_t dCoord_X;                //圆心X与中心点之差
    int16_t dCoord_Y;                //圆心Y与中心点之差
    int16_t dCoord_R;                //圆心与中心点之角度差

    uint16_t usBlobCenter_X;         //转盘区中心点X
    uint16_t usBlobCenter_Y;         //转盘区中心点Y
    uint8_t ucBlobCenterErrCal;      //转盘区校准误差允许（范围小，以便精准校准位置）
    uint8_t ucBlobCenterErrGrab;     //转盘区抓取误差允许
    uint8_t ucBlobStableThreshold;   //转盘区稳定次数阈值，调节该参数以使小车不会随物块移动

    uint16_t usCircleCenter_X;       //粗校准时中间色环区中心点X
    uint16_t usCircleCenter_Y;       //粗校准时中间色环区中心点Y
    uint16_t usRedCircleCenter_X;    //细校准时红色环区中心点X
    uint16_t usRedCircleCenter_Y;    //细校准时红色环区中心点Y
    uint16_t usGreenCircleCenter_X;  //细校准时绿色环区中心点X
    uint16_t usGreenCircleCenter_Y;  //细校准时绿色环区中心点Y
    uint16_t usBlueCircleCenter_X;   //细校准时蓝色环区中心点X
    uint16_t usBlueCircleCenter_Y;   //细校准时蓝色环区中心点Y
    uint8_t ucCircleCenterErrCal;    //色环区校准误差允许
    uint8_t ucCircleStableThreshold; //色环区校准对准次数，调节该参数以使小车精确对准中心
} MAIXCAM_DATA_T;

void APP_MAIXCAM_Init(void);
MAIXCAM_DATA_T *APP_MAIXCAM_GetPoint(void);
uint8_t *APP_MAIXCAM_GetDirMappingColor(void);
void APP_MaixCAMControl(void);
void APP_MAIXCAM_SendAndRead(uint8_t _ucElement, uint8_t _ucColor);
void APP_MAIXCAM_Cal(uint8_t _ucElement, uint8_t _ucColor, uint8_t _ucIsCalFlag, uint8_t _ucMoveFlag);

extern __IO int16_t g_dX;
extern __IO int16_t g_dY;
extern __IO int16_t g_dR;

extern __IO uint16_t g_X;
extern __IO uint16_t g_Y;

#endif