#include "main.h"

#define MAIXCAM_CMD_BLOB        (0x01 << 0)

#define MAIXCAM_CMD_CIRCLE      (0x01 << 4)

static uint16_t s_usPresentX = 0, s_usPresentY = 0, s_usPreviousX = 0, s_usPreviousY = 0;
static uint32_t *MOTOR_X_Pos = NULL;
static uint32_t *MOTOR_R_Pos = NULL;
static uint8_t (*DirMapping)[4];  // 指向包含 4 个 uint8_t 元素的一维数组的指针

static void APP_MAIXCAM_Parse(uint8_t* pucFrameData, uint8_t ucFrameLength);
static uint8_t APP_MAIXCAM_Parse_BLOB_ArriveAndWaitCal(void);
static uint8_t APP_MAIXCAM_Parse_BLOB_Cal(void);
static uint8_t APP_MAIXCAM_Parse_BLOB_WaitGrab1(void);


static void APP_MAIXCAM_Parse_BLOB_Grab(void);
static void APP_MAIXCAM_Parse_CIRCLE_CalCar(void);
static void APP_MAIXCAM_Parse_CIRCLE_CalXR(void);
static void SendToAPP_MotorWhale(void);
float calculate_hypotenuse(float a, float b);
int32_t int32_constrain(int32_t Value, int32_t minValue, int32_t maxValue);

__IO int16_t g_dX = 0;
__IO int16_t g_dY = 0;
__IO int16_t g_dR = 0;

__IO uint16_t g_X = 0;
__IO uint16_t g_Y = 0;

static MAIXCAM_DATA_T s_tMaixCAM_Data =
{



    .ucCalFlag = CAL,              //是否校准标志位
    .ucMoveFlag = MoveCar,         // 0-移动车; 1-移动X方向和R方向步进电机

    .ucSendFlag = 0,             // 数据设置为无效

    .ucStableCount = 0,          //中心标志位计数

    .usBlobCenter_X = 160,       //转盘区中心X
    .usBlobCenter_Y = 120,       //转盘区中心Y
    //    .ucBlobCenterErrCal = 10,    //转盘区校准误差允许（范围小，以便精准校准位置）
    .ucBlobCenterErrCal = 10,    //转盘区校准误差允许（范围小，以便精准校准位置）
    .ucBlobCenterErrGrab = 30,   //转盘区抓取误差允许（范围大，以便及时响应抓取）
    //    .ucBlobStableThreshold = 3,  //转盘区稳定次数阈值，调节该参数以使小车不会随物块移动
    .ucBlobStableThreshold = 1,  //转盘区稳定次数阈值，调节该参数以使小车不会随物块移动

    .usCircleCenter_X = 147,     //中间色环区中心点X
    .usCircleCenter_Y = 117,     //中间色环区中心点Y

    .usRedCircleCenter_X = 147,     //红色环区中心点X
    .usRedCircleCenter_Y = 117,     //红色环区中心点Y
    .usGreenCircleCenter_X = 147,     //绿色环区中心点X
    .usGreenCircleCenter_Y = 117,     //绿色环区中心点Y
    .usBlueCircleCenter_X = 147,     //蓝色环区中心点X
    .usBlueCircleCenter_Y = 117,     //蓝色环区中心点Y
    .ucCircleCenterErrCal = 2,   //色环区校准误差允许
    .ucCircleStableThreshold = 1, //色环区校准对准次数，调节该参数以使小车精确对准中心

    .BlobColor = WHITE,
    .dCoord_X = 0,               //X坐标差
    .dCoord_Y = 0,               //Y坐标差
    .dCoord_R = 0,               //R角度差
};

static MAIXCAM_ID_T s_MaixCAM_ID =
{
    .ucElement = ELEMENT_NONE,
    .ucColor = WHITE,
    //    .ucElement = CIRCLE,
    //    .ucColor = GREEN,
};

PID_TypeDef Maxi_Cam_Blob_X;
PID_TypeDef Maxi_Cam_Blob_Y;
PID_TypeDef Maxi_Cam_Circle_X;
PID_TypeDef Maxi_Cam_Circle_Y;
PID_TypeDef Maxi_Cam_Circlexy_X;
PID_TypeDef Maxi_Cam_Circlexy_R;


/*
*********************************************************************************************************
*    函 数 名: APP_MAIXCAM_Init
*    功能说明: 初始化 MaixCAM 模块，并注册解析回调函数。
*    形    参: 无
*    返 回 值: 无
*********************************************************************************************************
*/
void APP_MAIXCAM_Init(void)
{
    bsp_InitMAIXCAM(APP_MAIXCAM_Parse);      //注册解析回调函数

    MOTOR_R_Pos = APP_Handle_GetPosPoint(MOTOR_R);
    MOTOR_X_Pos = APP_Handle_GetPosPoint(MOTOR_X);
    DirMapping = APP_Handle_GetMapPoint();

    float pid_1[3] = {1.5, 0, 0};
    PID_init(&Maxi_Cam_Blob_X, PID_POSITION, pid_1, 6000, 0.1);
    PID_init(&Maxi_Cam_Blob_Y, PID_POSITION, pid_1, 6000, 0.1);
    float pid_2[3] = {1.6, 0, 0};
    PID_init(&Maxi_Cam_Circle_X, PID_POSITION, pid_2, 6000, 0.1);
    PID_init(&Maxi_Cam_Circle_Y, PID_POSITION, pid_2, 6000, 0.1);
		float pid_3[3] = {15, 0, 0};
    PID_init(&Maxi_Cam_Circlexy_X, PID_POSITION, pid_3, 6000, 0.1);
    PID_init(&Maxi_Cam_Circlexy_R, PID_POSITION, pid_3, 6000, 0.1);
}

MAIXCAM_DATA_T *APP_MAIXCAM_GetPoint(void)
{
    return &s_tMaixCAM_Data;
}

uint8_t *APP_MAIXCAM_GetDirMappingColor(void)
{
    return s_tMaixCAM_Data.BlobDirMappingColor;
}

/*
*********************************************************************************************************
*    函 数 名: APP_MaixCAMControl
*    功能说明: 任务函数
*    形    参: 无
*    返 回 值: 无
*********************************************************************************************************
*/
void APP_MaixCAMControl(void)
{
    for (;;)
    {
        APP_MAIXCAM_SendAndRead(s_MaixCAM_ID.ucElement, s_MaixCAM_ID.ucColor);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/*
*********************************************************************************************************
*    函 数 名: APP_MAIXCAM_SendAndRead
*    功能说明: 发送 MaixCAM 命令并读取数据。
*    形    参: _ucElement    要检测的元素类型（例如 Blob 或 Circle）
*              _ucColor      要检测的颜色(RED GREEN BLUE)
*    返 回 值: 无
*********************************************************************************************************
*/
void APP_MAIXCAM_SendAndRead(uint8_t _ucElement, uint8_t _ucColor)
{
    if (_ucElement == ELEMENT_NONE || _ucColor == WHITE)
    {
        return;
    }

    if (s_tMaixCAM_Data.ucSendFlag == 0)        //如果还未发送指令，则进行一次发送。在APP_MOTOR_2006_WHALE_MovePID之后清零该标志位
    {
        MAIXCAM_SendCmd(_ucElement, _ucColor);

        s_tMaixCAM_Data.ucSendFlag = 1;         //发送标志位置1
    }

    if (MAIXCAM_ReadData())   //成功接收一帧数据流，并调用解析回调函数APP_MAIXCAM_Parse
    {
        s_tMaixCAM_Data.ucSendFlag = 0;
        __NOP();
    }
}

/*
*********************************************************************************************************
*    函 数 名: APP_MAIXCAM_Parse
*    功能说明: 解析 MaixCAM 接收到的数据，并根据功能码_pucFrameData[0]和ucCalFlag来决定调用哪个解析函数。
*    形    参: pucFrameData    接收到的数据帧。
*              _ucFrameLength  数据帧的长度
*    返 回 值: 无
*********************************************************************************************************
*/
static void APP_MAIXCAM_Parse(uint8_t* _pucFrameData, uint8_t _ucFrameLength)
{
    UNUSED(_ucFrameLength);
    //功能码 颜色 x[0] x[1] y[0] y[1]
    //功能码 左色 右色 0 0 0
    s_tMaixCAM_Data.BlobColor = _pucFrameData[1];
    s_tMaixCAM_Data.usCoord_X = ((uint16_t)_pucFrameData[2]) | ((uint16_t)_pucFrameData[3] << 8);
    s_tMaixCAM_Data.usCoord_Y = ((uint16_t)_pucFrameData[4]) | ((uint16_t)_pucFrameData[5] << 8);

    g_X = s_tMaixCAM_Data.usCoord_X;
    g_Y = s_tMaixCAM_Data.usCoord_Y;

    switch (s_tMaixCAM_Data.ucStatus)
    {
        case MAIXCAM_STATUS_BLOB_ArriveAndWaitCal:  //到达转盘等待校准
            if (APP_MAIXCAM_Parse_BLOB_ArriveAndWaitCal() == 1)
            {
                App_Printf("EXIT MAIXCAM_STATUS_BLOB_ArriveAndWaitCal\r\n");
                s_tMaixCAM_Data.ucStatus = MAIXCAM_STATUS_BLOB_Cal;
                App_Printf("ENTER MAIXCAM_STATUS_BLOB_Cal\r\n");
            }

            break;

        case MAIXCAM_STATUS_BLOB_Cal:
            if (APP_MAIXCAM_Parse_BLOB_Cal() == 1)
            {
                App_Printf("EXIT MAIXCAM_STATUS_BLOB_Cal\r\n");
                s_tMaixCAM_Data.ucStatus = MAIXCAM_STATUS_Blob_WaitGrab1;
                s_MaixCAM_ID.ucElement = BLOB;
                s_MaixCAM_ID.ucColor = COLOR_TWO;
                App_Printf("ENTER MAIXCAM_STATUS_Blob_WaitGrab1\r\n");
            }

            break;

        case MAIXCAM_STATUS_Blob_WaitGrab1:
            s_tMaixCAM_Data.BlobColorLeft = _pucFrameData[1];
            s_tMaixCAM_Data.BlobColorRight = _pucFrameData[2];

            if (APP_MAIXCAM_Parse_BLOB_WaitGrab1() == 1)
            {
                App_Printf("EXIT MAIXCAM_STATUS_Blob_WaitGrab1\r\n");
                s_tMaixCAM_Data.ucStatus = MAIXCAM_STATUS_Idle;
                App_Printf("BlobDirMappingColor : %d%d%d\r\n", s_tMaixCAM_Data.BlobDirMappingColor[LEFT_BLOB], s_tMaixCAM_Data.BlobDirMappingColor[MIDDLE_BLOB], s_tMaixCAM_Data.BlobDirMappingColor[RIGHT_BLOB] );
                App_Printf("ENTER MAIXCAM_STATUS_Idle\r\n");
                xTaskNotifyGive(Task_MainHandle);
            }

            break;

        case MAIXCAM_STATUS_Blob_Grab1:
            //            if (APP_MAIXCAM_Parse_BLOB_WaitGrab1() == 1)
            //            {
            //                s_tMaixCAM_Data.ucStatus = Blob_Grab1;
            //            }
            break;

        default:
            break;
    }




//    if (s_tMaixCAM_Data.ucMoveFlag == MoveCar)     //移动车
//    {
//        if (_pucFrameData[0] == MAIXCAM_CMD_CIRCLE && s_MaixCAM_ID.ucElement == CIRCLE)
//        {
//            APP_MAIXCAM_Parse_CIRCLE_CalCar();
//            //            App_Printf("APP_MAIXCAM_Parse_CIRCLE_CalCar\r\n");
//        }
//    }
//    else if (s_tMaixCAM_Data.ucMoveFlag == MoveXR)     //移动X R电机
//    {
//        APP_MAIXCAM_Parse_CIRCLE_CalXR();
//        s_tMaixCAM_Data.ucSendFlag = 0;
//    }
}


/*
*********************************************************************************************************
*    函 数 名: APP_MAIXCAM_Parse_BLOB_ArriveAndWaitCal
*    功能说明: 到达转盘等待校准状态
*    形    参: 无
*    返 回 值: ret  0-未检测到目标/不进行操作；1-状态成功结束；
*********************************************************************************************************
*/
static uint8_t APP_MAIXCAM_Parse_BLOB_ArriveAndWaitCal(void)
{
    s_tMaixCAM_Data.BlobColorFir = s_tMaixCAM_Data.BlobColorFir ? : s_tMaixCAM_Data.BlobColor;
    s_tMaixCAM_Data.usBlobCoord_X_Fir = s_tMaixCAM_Data.usBlobCoord_X_Fir ? : s_tMaixCAM_Data.usCoord_X;
    s_tMaixCAM_Data.usBlobCoord_Y_Fir = s_tMaixCAM_Data.usBlobCoord_Y_Fir ? : s_tMaixCAM_Data.usCoord_Y;

    if (s_tMaixCAM_Data.BlobColor != s_tMaixCAM_Data.BlobColorFir)
    {
        if (abs(s_tMaixCAM_Data.usBlobCoord_X_Fir - s_tMaixCAM_Data.usCoord_X) < 5 && abs(s_tMaixCAM_Data.usBlobCoord_Y_Fir - s_tMaixCAM_Data.usCoord_Y) < 5)
        {
            return 1;
        }
    }

    return 0;
}

/*
*********************************************************************************************************
*    函 数 名: APP_MAIXCAM_Parse_BLOB_Cal
*    功能说明: 物块校准状态
*    形    参: 无
*    返 回 值: ret  0-未检测到目标/不进行操作；1-状态成功结束；2-将坐标转为车身移动距离并发送
*********************************************************************************************************
*/
static uint8_t APP_MAIXCAM_Parse_BLOB_Cal(void)
{
    s_usPresentX = s_tMaixCAM_Data.usCoord_X;
    s_usPresentY = s_tMaixCAM_Data.usCoord_Y;

    if (!(s_usPresentX && s_usPresentY))   //如果收到的X Y数据有一个为0，则说明相机没有识别到，让摄像头重新发送
    {
        s_tMaixCAM_Data.ucSendFlag = 0;
        return 0;
    }

    s_tMaixCAM_Data.dCoord_X = s_usPresentX - s_tMaixCAM_Data.usBlobCenter_X;
    s_tMaixCAM_Data.dCoord_Y = s_usPresentY - s_tMaixCAM_Data.usBlobCenter_Y;
    g_dX = s_tMaixCAM_Data.dCoord_X;
    g_dY = s_tMaixCAM_Data.dCoord_Y;

    // 判断是否位于小范围中心
    if (abs(s_tMaixCAM_Data.dCoord_X) <= s_tMaixCAM_Data.ucBlobCenterErrCal &&
            abs(s_tMaixCAM_Data.dCoord_Y) <= s_tMaixCAM_Data.ucBlobCenterErrCal)
    {
        // 位于小范围中心
        s_MaixCAM_ID.ucElement = ELEMENT_NONE;
        s_MaixCAM_ID.ucColor = WHITE;
        s_tMaixCAM_Data.BlobColorSec = s_tMaixCAM_Data.BlobColorSec ? : s_tMaixCAM_Data.BlobColor;
        s_usPresentX = 0;
        s_usPresentY = 0;
        return 1;
    }

    //移动车身校准
    {
//        double err_x = s_tMaixCAM_Data.dCoord_X;
//        double err_y = s_tMaixCAM_Data.dCoord_Y;
//        //int speed_temp=(moto_chassis[0].speed_rpm+moto_chassis[1].speed_rpm+moto_chassis[2].speed_rpm+moto_chassis[3].speed_rpm)/4.0;
//        Maxi_Cam_Blob_X.set = 0;
//        Maxi_Cam_Blob_X.Ki = 0.75 + (fabs(err_x) / 4);
//        Maxi_Cam_Blob_Y.Ki = 0.75 + (fabs(err_y) / 4);
//        PID_calc(&Maxi_Cam_Blob_X, -err_x, 0);
//        PID_calc(&Maxi_Cam_Blob_Y, -err_y, 0);
//        float w = Maxi_Cam_Blob_X.out;

//        s_tMaixCAM_Data.dCoord_X = Maxi_Cam_Blob_X.out;
//        s_tMaixCAM_Data.dCoord_Y = Maxi_Cam_Blob_Y.out;

        // 发送移动距离量到APP_MotorWhale的消息队列
        SendToAPP_MotorWhale();
        return 2;
    }
}

/*
*********************************************************************************************************
*    函 数 名: APP_MAIXCAM_Parse_BLOB_WaitGrab1
*    功能说明: 物块校准状态
*    形    参: 无
*    返 回 值: ret  0-未检测到目标/不进行操作；1-状态成功结束；
*********************************************************************************************************
*/
static uint8_t APP_MAIXCAM_Parse_BLOB_WaitGrab1(void)
{
    if (s_tMaixCAM_Data.BlobColorLeft==s_tMaixCAM_Data.BlobColorSec || s_tMaixCAM_Data.BlobColorRight==s_tMaixCAM_Data.BlobColorSec)
    {
        s_tMaixCAM_Data.BlobDirMappingColor[LEFT_BLOB] = s_tMaixCAM_Data.BlobColorLeft;
        s_tMaixCAM_Data.BlobDirMappingColor[RIGHT_BLOB] = s_tMaixCAM_Data.BlobColorRight;
        //数学法，使用三个颜色的数值总和减去其中两个，可得第三个颜色的数值
        s_tMaixCAM_Data.BlobDirMappingColor[MIDDLE_BLOB] = 6 - s_tMaixCAM_Data.BlobDirMappingColor[LEFT_BLOB] - s_tMaixCAM_Data.BlobDirMappingColor[RIGHT_BLOB];

        if (s_tMaixCAM_Data.BlobColorLeft==s_tMaixCAM_Data.BlobColorSec)
        {
            s_tMaixCAM_Data.RotateSeq = 1;
        }
        if (s_tMaixCAM_Data.BlobColorRight==s_tMaixCAM_Data.BlobColorSec)
        {
            s_tMaixCAM_Data.RotateSeq = 2;
        }
        return 1;
    }
    return 0;
}

/*
*********************************************************************************************************
*    函 数 名: APP_MAIXCAM_Parse_CIRCLE_CalCar
*    功能说明: 色环校准小车
*    形    参: 无
*    返 回 值: 无
*********************************************************************************************************
*/
static void APP_MAIXCAM_Parse_CIRCLE_CalCar(void)
{
    uint8_t isInCenter = 0;

    s_usPresentX = s_tMaixCAM_Data.usCoord_X;
    s_usPresentY = s_tMaixCAM_Data.usCoord_Y;

    if (!(s_usPresentX && s_usPresentY))   //如果收到的X Y数据有一个为0，则说明相机没有识别到，让摄像头重新发送
    {
        s_tMaixCAM_Data.ucSendFlag = 0;
        return;
    }

    s_tMaixCAM_Data.dCoord_X = s_tMaixCAM_Data.usCoord_X - s_tMaixCAM_Data.usGreenCircleCenter_X;
    s_tMaixCAM_Data.dCoord_Y = s_tMaixCAM_Data.usCoord_Y - s_tMaixCAM_Data.usGreenCircleCenter_Y;

    g_dX = s_tMaixCAM_Data.dCoord_X;
    g_dY = s_tMaixCAM_Data.dCoord_Y;

    //坐标是否位于中心
    if (abs(s_tMaixCAM_Data.dCoord_X) <= s_tMaixCAM_Data.ucCircleCenterErrCal &&
            abs(s_tMaixCAM_Data.dCoord_Y) <= s_tMaixCAM_Data.ucCircleCenterErrCal)
    {
        // 坐标连续n次位于中心
        if (s_tMaixCAM_Data.ucStableCount < s_tMaixCAM_Data.ucCircleStableThreshold)
        {
            s_tMaixCAM_Data.ucStableCount++;
        }
        else   // 计数达到阈值，认为真正在中心
        {
            // 通知APP_Main并关闭摄像头
            xTaskNotifyGive(Task_MainHandle);
            xEventGroupClearBits(EventGroups_CarHandle, EventGroupsCarMaixCAM_EN_1);

            MAIXCAM_Send0();
            s_MaixCAM_ID.ucElement = ELEMENT_NONE;
            s_MaixCAM_ID.ucColor = WHITE;
            s_tMaixCAM_Data.ucStableCount = 0;   //重置计数
            s_tMaixCAM_Data.dCoord_R = 0;
            s_tMaixCAM_Data.dCoord_X = 0;
        }
    }
    else
    {
        // 如果不在中心区域，重置计数
        s_tMaixCAM_Data.ucStableCount = 0;
        double err_x = s_tMaixCAM_Data.dCoord_X;
        double err_y = s_tMaixCAM_Data.dCoord_Y;
        //int speed_temp=(moto_chassis[0].speed_rpm+moto_chassis[1].speed_rpm+moto_chassis[2].speed_rpm+moto_chassis[3].speed_rpm)/4.0;
        Maxi_Cam_Circle_X.set = 0;
        Maxi_Cam_Circle_X.Ki = 0.75 + (fabs(err_x) / 5);
        Maxi_Cam_Circle_Y.Ki = 0.75 + (fabs(err_y) / 5);
        PID_calc(&Maxi_Cam_Circle_X, -err_x, 0);
        PID_calc(&Maxi_Cam_Circle_Y, -err_y, 0);
        // 发送移动距离量到APP_MotorWhale的消息队列
        //        s_tMaixCAM_Data.dCoord_X = Maxi_Cam_Circle_X.out;
        //        s_tMaixCAM_Data.dCoord_Y = Maxi_Cam_Circle_Y.out;
        // 发送移动距离量到APP_MotorWhale的消息队列
        //        s_tMaixCAM_Data.dCoord_X = (s_usPresentX + s_usPreviousX) / 2.0 - s_tMaixCAM_Data.usCircleCenter_X;
        //        s_tMaixCAM_Data.dCoord_Y = (s_usPresentY + s_usPreviousY) / 2.0 - s_tMaixCAM_Data.usCircleCenter_Y;
        SendToAPP_MotorWhale();
    }

    g_dX = s_tMaixCAM_Data.dCoord_X;
    g_dY = s_tMaixCAM_Data.dCoord_Y;
}

/*
*********************************************************************************************************
*    函 数 名: APP_MAIXCAM_Parse_CIRCLE_CalXR
*    功能说明: 色环校准X和R方向步进电机
*    形    参: 无
*    返 回 值: 无
*********************************************************************************************************
*/
static void APP_MAIXCAM_Parse_CIRCLE_CalXR(void)
{
    uint8_t isInCenter = 0;

    float dR = 0, dX = 0;

    if ((s_tMaixCAM_Data.usCoord_X == 0) || (s_tMaixCAM_Data.usCoord_Y == 0))
    {
        return;
    }

    uint16_t usCircleCenter_X;
    uint16_t usCircleCenter_Y;

    switch (s_MaixCAM_ID.ucColor)
    {
        case RED:
            usCircleCenter_X = s_tMaixCAM_Data.usRedCircleCenter_X;       //色环区中心点X
            usCircleCenter_Y = s_tMaixCAM_Data.usRedCircleCenter_Y;       //色环区中心点Y
            break;

        case GREEN:
            usCircleCenter_X = s_tMaixCAM_Data.usGreenCircleCenter_X;       //色环区中心点X
            usCircleCenter_Y = s_tMaixCAM_Data.usGreenCircleCenter_Y;       //色环区中心点Y
            break;

        case BLUE:
            usCircleCenter_X = s_tMaixCAM_Data.usBlueCircleCenter_X;       //色环区中心点X
            usCircleCenter_Y = s_tMaixCAM_Data.usBlueCircleCenter_Y;       //色环区中心点Y
            break;

        default:
            break;
    }

    dR = atan2f(240 - s_tMaixCAM_Data.usCoord_Y, s_tMaixCAM_Data.usCoord_X - usCircleCenter_X) * 180.0f / PI - 90;
    dX = calculate_hypotenuse(240 - s_tMaixCAM_Data.usCoord_Y, s_tMaixCAM_Data.usCoord_X - usCircleCenter_X) - (240 - usCircleCenter_Y);

    s_tMaixCAM_Data.dCoord_R = dR;
    s_tMaixCAM_Data.dCoord_X = dX;

    //坐标是否位于中心
    //    if( abs( s_tMaixCAM_Data.dCoord_X ) <= s_tMaixCAM_Data.ucCircleCenterErrCal &&
    //        abs( s_tMaixCAM_Data.dCoord_Y ) <= s_tMaixCAM_Data.ucCircleCenterErrCal )
    if (abs(s_tMaixCAM_Data.dCoord_R) <= 1 &&
            abs(s_tMaixCAM_Data.dCoord_X) <= 1)
    {
        // 坐标连续n次位于中心
        if (s_tMaixCAM_Data.ucStableCount < s_tMaixCAM_Data.ucCircleStableThreshold)
        {
            s_tMaixCAM_Data.ucStableCount++;
        }
        else   // 计数达到阈值，认为真正在中心
        {
            // 通知APP_Main并关闭摄像头
            App_Printf("******** %d ********* CalOk\r\n", s_MaixCAM_ID.ucColor);
            xTaskNotifyGive(Task_MainHandle);
            xEventGroupClearBits(EventGroups_CarHandle, EventGroupsCarMaixCAM_EN_1);

            s_tMaixCAM_Data.ucStableCount = 0;   //重置计数

            MAIXCAM_Send0();
            s_MaixCAM_ID.ucElement = ELEMENT_NONE;
            s_MaixCAM_ID.ucColor = WHITE;
        }
    }
    else
    {
        // 如果不在中心区域，重置计数
        s_tMaixCAM_Data.ucStableCount = 0;
			
        Maxi_Cam_Circlexy_X.set = 0;
				Maxi_Cam_Circlexy_R.set = 0;
        Maxi_Cam_Circlexy_X.Ki = 0.75 + (fabs(dR) / 5);
        Maxi_Cam_Circlexy_R.Ki = 0.75 + (fabs(dX) / 5);
        PID_calc(&Maxi_Cam_Circlexy_X, dX, 0);
        PID_calc(&Maxi_Cam_Circlexy_R, dR, 0);
        // 发送移动距离量到APP_MotorWhale的消息队列
        //        s_tMaixCAM_Data.dCoord_X = Maxi_Cam_Circle_X.out;
        //        s_tMaixCAM_Data.dCoord_Y = Maxi_Cam_Circle_Y.out;
			
			
        s_tMaixCAM_Data.dCoord_R = Maxi_Cam_Circlexy_X.out;
        s_tMaixCAM_Data.dCoord_X = Maxi_Cam_Circlexy_R.out;
			
        // 改变X R电机的绝对距离
        MOTOR_X_Pos[DirMapping[Fir][s_MaixCAM_ID.ucColor]] += s_tMaixCAM_Data.dCoord_X;
        MOTOR_R_Pos[DirMapping[Fir][s_MaixCAM_ID.ucColor]] += s_tMaixCAM_Data.dCoord_R;
        MOTOR_X_Pos[DirMapping[Fir][s_MaixCAM_ID.ucColor]] = int32_constrain(MOTOR_X_Pos[DirMapping[Fir][s_MaixCAM_ID.ucColor]], MOTOR_X_Pos_Min, MOTOR_X_Pos_Max);    //限幅
        MOTOR_R_Pos[DirMapping[Fir][s_MaixCAM_ID.ucColor]] = int32_constrain(MOTOR_R_Pos[DirMapping[Fir][s_MaixCAM_ID.ucColor]], MOTOR_R_Pos_Min, MOTOR_R_Pos_Max);    //限幅

        APP_MOTOR_ZDT_Move_P(MOTOR_X, MOTOR_X_Direction_FORWARD,  20, 100, MOTOR_X_Pos[DirMapping[Fir][s_MaixCAM_ID.ucColor]], 1, 0);
        APP_MOTOR_ZDT_Move_P(MOTOR_R, MOTOR_R_Direction_CCR,      20, 100, MOTOR_R_Pos[DirMapping[Fir][s_MaixCAM_ID.ucColor]], 1, 0);
    }

    g_dX = s_tMaixCAM_Data.dCoord_X;
    g_dR = s_tMaixCAM_Data.dCoord_R;
}

/*
*********************************************************************************************************
*    函 数 名: SendToAPP_MotorWhale
*    功能说明: 发送消息给轮子控制任务
*    形    参: 无
*    返 回 值: 无
*********************************************************************************************************
*/
static void SendToAPP_MotorWhale(void)
{
    EventBits_t uxBits;

    uxBits = xEventGroupWaitBits(
                 EventGroups_CarHandle,
                 EventGroupsCarMaixCAM_EN_1,
                 pdFALSE,        // 不清除标志位，即开启摄像头开关之后，就不会因为该函数关闭
                 pdFALSE,        // pdTRUE与运算  pdFALSE或运算
                 0);

    if ((uxBits & (EventGroupsCarMaixCAM_EN_1)) == (EventGroupsCarMaixCAM_EN_1))
    {
        float k = 0.6;
        APP_CHASSIS_MovePID(CHASSIS_CAL, CHASSIS_CALL_BLOCKING, CARDIRECTION_X(s_tMaixCAM_Data.dCoord_X), abs(s_tMaixCAM_Data.dCoord_X) * k);
        APP_CHASSIS_MovePID(CHASSIS_CAL, CHASSIS_CALL_BLOCKING, CARDIRECTION_Y(s_tMaixCAM_Data.dCoord_Y), abs(s_tMaixCAM_Data.dCoord_Y) * k);
    }
}

void APP_MAIXCAM_Cal(uint8_t _ucElement, uint8_t _ucColor, uint8_t _ucIsCalFlag, uint8_t _ucMoveFlag)
{
    s_MaixCAM_ID.ucElement = _ucElement;
    s_MaixCAM_ID.ucColor = _ucColor;
    s_tMaixCAM_Data.ucCalFlag = _ucIsCalFlag;
    s_tMaixCAM_Data.ucMoveFlag = _ucMoveFlag;

    if (s_tMaixCAM_Data.ucCalFlag == BlobCalFlag) s_tMaixCAM_Data.ucStatus = MAIXCAM_STATUS_BLOB_ArriveAndWaitCal;
//    else if (s_tMaixCAM_Data.ucCalFlag == CircleCalFlag) s_tMaixCAM_Data.ucStatus = BLOB_ArriveAndWaitCal;

    //    if (s_tMaixCAM_Data.ucMoveFlag == MoveCar)
    //    {
    //        APP_MOTOR_ZDT_Move_P(MOTOR_R, MOTOR_R_Direction_CCR,     100, 250, MOTOR_R_Pos[MIDDLE], 1, 0);
    //        APP_MOTOR_ZDT_Move_P(MOTOR_X, MOTOR_X_Direction_FORWARD, 100, 250, MOTOR_X_Pos[MIDDLE], 1, 0);
    //        vTaskDelay(500);
    //    }
    if (s_tMaixCAM_Data.ucMoveFlag == MoveXR)
    {
        APP_MOTOR_ZDT_Move_P(MOTOR_R, MOTOR_R_Direction_CCR,     200, 250, MOTOR_R_Pos[DirMapping[Fir][s_MaixCAM_ID.ucColor]], 1, 0);
        APP_MOTOR_ZDT_Move_P(MOTOR_X, MOTOR_X_Direction_FORWARD, 200, 250, MOTOR_X_Pos[DirMapping[Fir][s_MaixCAM_ID.ucColor]], 1, 0);
        vTaskDelay(500);
    }

    xEventGroupSetBits(EventGroups_CarHandle, EventGroupsCarMaixCAM_EN_1);      //开启摄像头
}

float calculate_hypotenuse(float a, float b)
{
    float squared_sum = a * a + b * b;
    float result;
    arm_sqrt_f32(squared_sum, &result);  // 使用 CMSIS DSP 库的平方根函数
    return result;
}

//限幅函数
int32_t int32_constrain(int32_t Value, int32_t minValue, int32_t maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}
