#include "main.h"

#define MAIXCAM_CMD_BLOB        (0x01 << 0)

#define MAIXCAM_CMD_CIRCLE      (0x01 << 4)

static uint16_t s_usPresentX=0, s_usPresentY=0, s_usPreviousX=0, s_usPreviousY=0;
static uint32_t *MOTOR_X_Pos = NULL;
static uint32_t *MOTOR_R_Pos = NULL;
static uint8_t (*DirMapping)[4];  // 指向包含 4 个 uint8_t 元素的一维数组的指针

static void APP_MAIXCAM_Parse( uint8_t* pucFrameData, uint8_t ucFrameLength );
static void APP_MAIXCAM_Parse_BLOB_Cal(void);
static void APP_MAIXCAM_Parse_BLOB_Grab(void);
static void APP_MAIXCAM_Parse_CIRCLE_CalCar(void);
static void APP_MAIXCAM_Parse_CIRCLE_CalXR(void);
static void SendToAPP_MotorWhale(void);
float calculate_hypotenuse(float a, float b);
int32_t int32_constrain(int32_t Value, int32_t minValue, int32_t maxValue);

__IO int16_t g_dX = 0;
__IO int16_t g_dY = 0;
__IO int16_t g_dR = 0;

static MAIXCAM_DATA_T s_tMaixCAM_Data =
{
    .ucMoveFlag = 0,             // 0-移动车; 1-移动X方向和R方向步进电机
    .ucSendFlag = 0,             // 数据设置为无效

    .ucCalFlag = 0,              //是否校准标志位
    .ucStableCount = 0,          //中心标志位计数

    .usBlobCenter_X = 160,       //转盘区中心X
    .usBlobCenter_Y = 120,       //转盘区中心Y
//    .ucBlobCenterErrCal = 10,    //转盘区校准误差允许（范围小，以便精准校准位置）
    .ucBlobCenterErrCal = 10,    //转盘区校准误差允许（范围小，以便精准校准位置）
    .ucBlobCenterErrGrab = 30,   //转盘区抓取误差允许（范围大，以便及时响应抓取）
//    .ucBlobStableThreshold = 3,  //转盘区稳定次数阈值，调节该参数以使小车不会随物块移动
    .ucBlobStableThreshold = 1,  //转盘区稳定次数阈值，调节该参数以使小车不会随物块移动

    .usRedCircleCenter_X = 139,     //红色环区中心点X
    .usRedCircleCenter_Y = 108,     //红色环区中心点Y
    .usGreenCircleCenter_X = 140,     //绿色环区中心点X
    .usGreenCircleCenter_Y = 106,     //绿色环区中心点Y
    .usBlueCircleCenter_X = 140,     //蓝色环区中心点X
    .usBlueCircleCenter_Y = 106,     //蓝色环区中心点Y
    .ucCircleCenterErrCal = 1,   //色环区校准误差允许
    .ucCircleStableThreshold = 1, //色环区校准对准次数，调节该参数以使小车精确对准中心

    .dCoord_X = 0,               //X坐标差
    .dCoord_Y = 0,               //Y坐标差
    .dCoord_R = 0,               //R角度差
};

static MAIXCAM_ID_T s_MaixCAM_ID =
{
    .ucElement = ELEMENT_NONE,
    .ucColor = WHITE,
};

/*
*********************************************************************************************************
*    函 数 名: APP_MAIXCAM_Init
*    功能说明: 初始化 MaixCAM 模块，并注册解析回调函数。
*    形    参: 无
*    返 回 值: 无
*********************************************************************************************************
*/
void APP_MAIXCAM_Init( void )
{
    bsp_InitMAIXCAM( APP_MAIXCAM_Parse );    //注册解析回调函数

    MOTOR_R_Pos = APP_Handle_GetPosPoint(MOTOR_R);
    MOTOR_X_Pos = APP_Handle_GetPosPoint(MOTOR_X);
    DirMapping = APP_Handle_GetMapPoint();

}

MAIXCAM_DATA_T *APP_MAIXCAM_GetPoint(void)
{
    return &s_tMaixCAM_Data;
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
    EventBits_t uxBits;

//    xEventGroupSetBits(EventGroups_CarHandle, EventGroupsCarMaixCAM_EN_1);
    for(;; )
    {
        uxBits = xEventGroupWaitBits(
                     EventGroups_CarHandle,
                     EventGroupsCarMaixCAM_EN_1,
                     pdFALSE,        // 不清除标志位，即开启摄像头开关之后，就不会因为该函数关闭
                     pdFALSE,        // pdTRUE与运算  pdFALSE或运算
                     portMAX_DELAY );

        if(( uxBits & ( EventGroupsCarMaixCAM_EN_1 ) ) == ( EventGroupsCarMaixCAM_EN_1 ) )
        {
            APP_MAIXCAM_SendAndRead( s_MaixCAM_ID.ucElement, s_MaixCAM_ID.ucColor );
        }
        else
        {
//            LED_R_ON(); //应该不会进入此判断
        }
        vTaskDelay( pdMS_TO_TICKS(100) );
    }
}

/*
*********************************************************************************************************
*    函 数 名: APP_MAIXCAM_SendAndRead
*    功能说明: 发送 MaixCAM 命令并读取数据。
*    形    参: _ucElement    要检测的元素类型（例如 Blob 或 Circle）
*              _ucColor      要检测的颜色
*              _ucIsCalFlag  是否处于校准状态
*    返 回 值: 无
*********************************************************************************************************
*/
void APP_MAIXCAM_SendAndRead( uint8_t _ucElement, uint8_t _ucColor )
{
    if( s_tMaixCAM_Data.ucSendFlag == 0 )       //如果还未发送指令，则进行一次发送。在APP_MOTOR_2006_WHALE_MovePID之后清零该标志位
    {
        MAIXCAM_SendCmd( _ucElement, _ucColor );

        s_tMaixCAM_Data.ucSendFlag = 1;         //发送标志位置1
    }
    if (MAIXCAM_ReadData())   //成功接收一帧数据流，并调用解析回调函数APP_MAIXCAM_Parse
    {
        s_tMaixCAM_Data.ucSendFlag = 0;
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
static void APP_MAIXCAM_Parse( uint8_t* _pucFrameData, uint8_t _ucFrameLength )
{
    UNUSED( _ucFrameLength );

    s_tMaixCAM_Data.usCoord_X = (( uint16_t )_pucFrameData[1] ) | (( uint16_t )_pucFrameData[2] << 8 );
    s_tMaixCAM_Data.usCoord_Y = (( uint16_t )_pucFrameData[3] ) | (( uint16_t )_pucFrameData[4] << 8 );

    if (s_tMaixCAM_Data.ucMoveFlag == MoveCar)      //移动车
    {
        if (_pucFrameData[0]==MAIXCAM_CMD_BLOB && s_MaixCAM_ID.ucElement==BLOB)
        {
            if (s_tMaixCAM_Data.ucCalFlag == CAL)
            {
                APP_MAIXCAM_Parse_BLOB_Cal();
                App_Printf( "APP_MAIXCAM_Parse_BLOB_Cal\r\n" );
            }
            else if (s_tMaixCAM_Data.ucCalFlag == GRAB)
            {
                APP_MAIXCAM_Parse_BLOB_Grab();
                App_Printf( "APP_MAIXCAM_Parse_BLOB_Grab\r\n" );
            }
        }
        else if (_pucFrameData[0]==MAIXCAM_CMD_CIRCLE && s_MaixCAM_ID.ucElement==CIRCLE)
        {
            APP_MAIXCAM_Parse_CIRCLE_CalCar();
            App_Printf( "APP_MAIXCAM_Parse_CIRCLE_CalCar\r\n" );
        }
    }
    else if (s_tMaixCAM_Data.ucMoveFlag == MoveXR)     //移动X R电机
    {
        APP_MAIXCAM_Parse_CIRCLE_CalXR();
        s_tMaixCAM_Data.ucSendFlag = 0;
    }
}

/*
*********************************************************************************************************
*    函 数 名: APP_MAIXCAM_Parse_BLOB_Cal
*    功能说明: 物块校准状态
*    形    参: 无
*    返 回 值: 无
*********************************************************************************************************
*/
static void APP_MAIXCAM_Parse_BLOB_Cal(void)
{
    uint8_t isInCenter = 0;

    s_usPresentX = s_tMaixCAM_Data.usCoord_X;
    s_usPresentY = s_tMaixCAM_Data.usCoord_Y;
    if (!(s_usPresentX && s_usPresentY))   //如果收到的X Y数据有一个为0，则说明相机没有识别到，让摄像头重新发送
    {
        s_tMaixCAM_Data.ucSendFlag = 0;
        return;
    }

    s_tMaixCAM_Data.dCoord_X = s_tMaixCAM_Data.usCoord_X - s_tMaixCAM_Data.usBlobCenter_X;
    s_tMaixCAM_Data.dCoord_Y = s_tMaixCAM_Data.usCoord_Y - s_tMaixCAM_Data.usBlobCenter_Y;

    // 判断坐标是否连续n次不动
    if (abs(s_usPresentX - s_usPreviousX) < 5 && abs(s_usPresentY - s_usPreviousY) < 5)
    {
        // 坐标连续n次不动
        if (s_tMaixCAM_Data.ucStableCount < s_tMaixCAM_Data.ucBlobStableThreshold)
        {
            s_tMaixCAM_Data.ucStableCount++;
        }
    }
    else
    {
        // 坐标有变化，重置计数
        s_tMaixCAM_Data.ucStableCount = 0;
    }

    if (s_tMaixCAM_Data.ucStableCount >= s_tMaixCAM_Data.ucBlobStableThreshold)
    {
        // 判断是否位于小范围中心
        if( abs( s_tMaixCAM_Data.dCoord_X ) <= s_tMaixCAM_Data.ucBlobCenterErrCal &&
                abs( s_tMaixCAM_Data.dCoord_Y ) <= s_tMaixCAM_Data.ucBlobCenterErrCal )
        {
            // 位于小范围中心
            isInCenter = 1;
        }
    }

    if (isInCenter)
    {
        // 通知APP_Main并关闭摄像头
        xTaskNotifyGive(Task_MainHandle);
        xEventGroupClearBits( EventGroups_CarHandle, EventGroupsCarMaixCAM_EN_1 );

        MAIXCAM_Send0();
        s_tMaixCAM_Data.ucStableCount = 0;
        s_usPresentX = 0;
        s_usPresentY = 0;
        s_usPreviousX = 0;
        s_usPreviousY = 0;
    }
    else if (s_tMaixCAM_Data.ucStableCount >= s_tMaixCAM_Data.ucBlobStableThreshold)
    {
        // 发送移动距离量到APP_MotorWhale的消息队列
        s_tMaixCAM_Data.dCoord_X = (s_usPresentX + s_usPreviousX) / 2.0 - s_tMaixCAM_Data.usBlobCenter_X;
        s_tMaixCAM_Data.dCoord_Y = (s_usPresentY + s_usPreviousY) / 2.0 - s_tMaixCAM_Data.usBlobCenter_Y;
        SendToAPP_MotorWhale();

        s_tMaixCAM_Data.ucStableCount = 0;
    }

    s_usPreviousX = s_usPresentX;
    s_usPreviousY = s_usPresentY;

    g_dX = s_tMaixCAM_Data.dCoord_X;
    g_dY = s_tMaixCAM_Data.dCoord_Y;
}

/*
*********************************************************************************************************
*    函 数 名: APP_MAIXCAM_Parse_BLOB_Grab
*    功能说明: 物块抓取状态
*    形    参: 无
*    返 回 值: 无
*********************************************************************************************************
*/
static void APP_MAIXCAM_Parse_BLOB_Grab(void)
{
    uint8_t isInCenter = 0;

    s_usPresentX = s_tMaixCAM_Data.usCoord_X;
    s_usPresentY = s_tMaixCAM_Data.usCoord_Y;
    if (!(s_usPresentX && s_usPresentY))   //如果收到的X Y数据有一个为0，则说明相机没有识别到，让摄像头重新发送
    {
        s_tMaixCAM_Data.ucSendFlag = 0;
        return;
    }

    s_tMaixCAM_Data.dCoord_X = s_tMaixCAM_Data.usCoord_X - s_tMaixCAM_Data.usBlobCenter_X;
    s_tMaixCAM_Data.dCoord_Y = s_tMaixCAM_Data.usCoord_Y - s_tMaixCAM_Data.usBlobCenter_Y;

    // 判断是否位于大范围中心
    if( abs( s_tMaixCAM_Data.dCoord_X ) <= s_tMaixCAM_Data.ucBlobCenterErrGrab &&
            abs( s_tMaixCAM_Data.dCoord_Y ) <= s_tMaixCAM_Data.ucBlobCenterErrGrab )
    {
        // 位于小范围中心
        isInCenter = 1;
    }

    if (isInCenter)
    {
        // 通知APP_Main并关闭摄像头
        xTaskNotifyGive(Task_MainHandle);
        xEventGroupClearBits( EventGroups_CarHandle, EventGroupsCarMaixCAM_EN_1 );

        MAIXCAM_Send0();
    }

    g_dX = s_tMaixCAM_Data.dCoord_X;
    g_dY = s_tMaixCAM_Data.dCoord_Y;
    App_Printf("dX=%d dY=%d\r\n", s_tMaixCAM_Data.dCoord_X, s_tMaixCAM_Data.dCoord_Y);
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

    //坐标是否位于中心
    if( abs( s_tMaixCAM_Data.dCoord_X ) <= s_tMaixCAM_Data.ucCircleCenterErrCal &&
            abs( s_tMaixCAM_Data.dCoord_Y ) <= s_tMaixCAM_Data.ucCircleCenterErrCal )
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
            xEventGroupClearBits( EventGroups_CarHandle, EventGroupsCarMaixCAM_EN_1 );

//            MAIXCAM_Send0();
            s_tMaixCAM_Data.ucStableCount = 0;   //重置计数
            s_tMaixCAM_Data.dCoord_R = 0;
            s_tMaixCAM_Data.dCoord_X = 0;
        }
    }
    else
    {
        // 如果不在中心区域，重置计数
        s_tMaixCAM_Data.ucStableCount = 0;

        // 发送移动距离量到APP_MotorWhale的消息队列
//        s_tMaixCAM_Data.dCoord_X = (s_usPresentX + s_usPreviousX) / 2.0 - s_tMaixCAM_Data.usCircleCenter_X;
//        s_tMaixCAM_Data.dCoord_Y = (s_usPresentY + s_usPreviousY) / 2.0 - s_tMaixCAM_Data.usCircleCenter_Y;
        SendToAPP_MotorWhale();
    }

    g_dX = s_tMaixCAM_Data.dCoord_X;
    g_dY = s_tMaixCAM_Data.dCoord_Y;
    App_Printf("dX=%d dY=%d\r\n", s_tMaixCAM_Data.dCoord_X, s_tMaixCAM_Data.dCoord_Y);
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

    float dR=0, dX=0;
    if ((s_tMaixCAM_Data.usCoord_X == 0) || (s_tMaixCAM_Data.usCoord_Y == 0))
    {
        return;
    }

    uint16_t usCircleCenter_X;
    uint16_t usCircleCenter_Y;

    switch(s_MaixCAM_ID.ucColor)
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

    dR = atan2f(240-s_tMaixCAM_Data.usCoord_Y, s_tMaixCAM_Data.usCoord_X-usCircleCenter_X) * 180.0f / PI - 90;
    dX = calculate_hypotenuse(240-s_tMaixCAM_Data.usCoord_Y, s_tMaixCAM_Data.usCoord_X-usCircleCenter_X) - (240-usCircleCenter_Y);

    s_tMaixCAM_Data.dCoord_R = dR;
    s_tMaixCAM_Data.dCoord_X = dX;

    //坐标是否位于中心
//    if( abs( s_tMaixCAM_Data.dCoord_X ) <= s_tMaixCAM_Data.ucCircleCenterErrCal &&
//        abs( s_tMaixCAM_Data.dCoord_Y ) <= s_tMaixCAM_Data.ucCircleCenterErrCal )
    if( abs( s_tMaixCAM_Data.dCoord_R ) <= 1 &&
        abs( s_tMaixCAM_Data.dCoord_X ) <= 1 )
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
            xEventGroupClearBits( EventGroups_CarHandle, EventGroupsCarMaixCAM_EN_1 );

            s_tMaixCAM_Data.ucStableCount = 0;   //重置计数

            MAIXCAM_Send0();
        }
    }
    else
    {
        // 如果不在中心区域，重置计数
        s_tMaixCAM_Data.ucStableCount = 0;

        s_tMaixCAM_Data.dCoord_R = dR * 10;
        s_tMaixCAM_Data.dCoord_X = dX * 10;
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
//    App_Printf("dX=%.5f               dR=%.5f\r\n", dX, dR);
//    App_Printf("dCoord_X=%5d          dCoord_R=%5d\r\n", s_tMaixCAM_Data.dCoord_X, s_tMaixCAM_Data.dCoord_R);
    App_Printf("MOTOR_X_Pos[%d]=%5d   MOTOR_R_Pos[%d]=%5d\r\n", s_MaixCAM_ID.ucColor, MOTOR_X_Pos[DirMapping[Fir][s_MaixCAM_ID.ucColor]], s_MaixCAM_ID.ucColor, MOTOR_R_Pos[DirMapping[Fir][s_MaixCAM_ID.ucColor]]);
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
    float k = 1;
    APP_CHASSIS_MovePID( CHASSIS_CALL_BLOCKING, CARDIRECTION_X(s_tMaixCAM_Data.dCoord_X), abs( s_tMaixCAM_Data.dCoord_X ) * k );
    APP_CHASSIS_MovePID( CHASSIS_CALL_BLOCKING, CARDIRECTION_Y(s_tMaixCAM_Data.dCoord_Y), abs( s_tMaixCAM_Data.dCoord_Y ) * k );
}

void APP_MAIXCAM_Cal( uint8_t _ucElement, uint8_t _ucColor, uint8_t _ucIsCalFlag, uint8_t _ucMoveFlag )
{
    s_MaixCAM_ID.ucElement = _ucElement;
    s_MaixCAM_ID.ucColor = _ucColor;
    s_tMaixCAM_Data.ucCalFlag = _ucIsCalFlag;
    s_tMaixCAM_Data.ucMoveFlag = _ucMoveFlag;

//    if (s_tMaixCAM_Data.ucMoveFlag == MoveCar)
//    {
//        APP_MOTOR_ZDT_Move_P(MOTOR_R, MOTOR_R_Direction_CCR,     100, 250, MOTOR_R_Pos[MIDDLE], 1, 0);
//        APP_MOTOR_ZDT_Move_P(MOTOR_X, MOTOR_X_Direction_FORWARD, 100, 250, MOTOR_X_Pos[MIDDLE], 1, 0);
//        vTaskDelay(500);
//    }
    if (s_tMaixCAM_Data.ucMoveFlag == MoveXR)
    {
        APP_MOTOR_ZDT_Move_P(MOTOR_R, MOTOR_R_Direction_CCR,     100, 250, MOTOR_R_Pos[DirMapping[Fir][s_MaixCAM_ID.ucColor]], 1, 0);
        APP_MOTOR_ZDT_Move_P(MOTOR_X, MOTOR_X_Direction_FORWARD, 100, 250, MOTOR_X_Pos[DirMapping[Fir][s_MaixCAM_ID.ucColor]], 1, 0);
        vTaskDelay(500);
    }
    xEventGroupSetBits( EventGroups_CarHandle, EventGroupsCarMaixCAM_EN_1 );    //开启摄像头
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
