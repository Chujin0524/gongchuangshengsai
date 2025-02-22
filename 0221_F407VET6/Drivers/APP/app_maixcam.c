#include "main.h"

#define MAIXCAM_CMD_BLOB        (0x01 << 0)

#define MAIXCAM_CMD_CIRCLE      (0x01 << 4)

static uint16_t s_usPresentX=0, s_usPresentY=0, s_usPreviousX=0, s_usPreviousY=0;

static void APP_MAIXCAM_Parse( uint8_t* pucFrameData, uint8_t ucFrameLength );
static uint8_t APP_MAIXCAM_IsInCenter( uint8_t _ucCalFlag, uint8_t _ucCmd );
static void APP_MAIXCAM_Parse_BLOB_Cal(void);
static void APP_MAIXCAM_Parse_BLOB_Grab(void);
static void APP_MAIXCAM_Parse_CIRCLE_CAL(void);
static void SendToAPP_MotorWhale(void);

static MAIXCAM_DATA_T s_tMaixCAM_Data =
{
    .ucSendFlag = 0,             // 数据设置为无效

    .ucCalFlag = 0,              //是否校准标志位
    .ucStableCount = 0,          //中心标志位计数

    .usBlobCenter_X = 175,       //转盘区中心X
    .usBlobCenter_Y = 110,       //转盘区中心Y
    .ucBlobCenterErrCal = 10,    //转盘区校准误差允许（范围小，以便精准校准位置）
    .ucBlobCenterErrGrab = 30,   //转盘区抓取误差允许（范围大，以便及时响应抓取）
    .ucBlobStableThreshold = 3,  //转盘区稳定次数阈值，调节该参数以使小车不会随物块移动

    .usCircleCenter_X = 138,     //色环区中心X
    .usCircleCenter_Y = 108,     //色环区中心Y
    .ucCircleCenterErrCal = 1,   //色环区校准误差允许
    .ucCircleStableThreshold = 3, //色环区校准对准次数，调节该参数以使小车精确对准中心

    .dCoord_X = 0,               //X坐标差
    .dCoord_Y = 0,               //Y坐标差
};

static MAIXCAM_ID_T s_MaixCAM_ID =
{
    .ucElement = ELEMENT_NONE,
    .ucColor = COLOR_NONE,
    .ucIsCalFlag = 0
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
            APP_MAIXCAM_SendAndRead( s_MaixCAM_ID.ucElement, s_MaixCAM_ID.ucColor, s_MaixCAM_ID.ucIsCalFlag );
        }
        else
        {
//            LED_R_ON(); //应该不会进入此判断
        }
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
void APP_MAIXCAM_SendAndRead( uint8_t _ucElement, uint8_t _ucColor, uint8_t _ucIsCalFlag )
{
    const TickType_t xFrequency = 100;
    
    if( s_tMaixCAM_Data.ucSendFlag == 0 )       //如果还未发送指令，则进行一次发送
    {
        MAIXCAM_SendCmd( _ucElement, _ucColor );
        s_tMaixCAM_Data.ucSendFlag = 1;         //发送标志位置1
        s_tMaixCAM_Data.ucCalFlag = _ucIsCalFlag;
    }
    if (MAIXCAM_ReadData())   //成功接收一帧数据流，并调用解析回调函数APP_MAIXCAM_Parse
    {
        s_tMaixCAM_Data.ucSendFlag = 0;
//        App_Printf("RECEIVE\r\n");
         vTaskDelay( pdMS_TO_TICKS( xFrequency ) );
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

    if (_pucFrameData[0]==MAIXCAM_CMD_BLOB && s_MaixCAM_ID.ucElement==BLOB)
    {
        if (s_tMaixCAM_Data.ucCalFlag == 1)
        {
            APP_MAIXCAM_Parse_BLOB_Cal();
            App_Printf( "APP_MAIXCAM_Parse_BLOB_Cal\r\n" );
        }
        else if (s_tMaixCAM_Data.ucCalFlag == 0)
        {
            APP_MAIXCAM_Parse_BLOB_Grab();
            App_Printf( "APP_MAIXCAM_Parse_BLOB_Grab\r\n" );
        }
    }
    else if (_pucFrameData[0]==MAIXCAM_CMD_CIRCLE && s_MaixCAM_ID.ucElement==CIRCLE)
    {
        APP_MAIXCAM_Parse_CIRCLE_CAL();
        App_Printf( "APP_MAIXCAM_Parse_CIRCLE_CAL\r\n" );
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
        
        s_tMaixCAM_Data.ucStableCount = 0;
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
                
    }
}

/*
*********************************************************************************************************
*    函 数 名: APP_MAIXCAM_Parse_CIRCLE_CAL
*    功能说明: 色环校准状态
*    形    参: 无
*    返 回 值: 无
*********************************************************************************************************
*/
static void APP_MAIXCAM_Parse_CIRCLE_CAL(void)
{
    uint8_t isInCenter = 0;

    s_usPresentX = s_tMaixCAM_Data.usCoord_X;
    s_usPresentY = s_tMaixCAM_Data.usCoord_Y;
    s_tMaixCAM_Data.dCoord_X = s_tMaixCAM_Data.usCoord_X - s_tMaixCAM_Data.usCircleCenter_X;
    s_tMaixCAM_Data.dCoord_Y = s_tMaixCAM_Data.usCoord_Y - s_tMaixCAM_Data.usCircleCenter_Y;

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
            
            s_tMaixCAM_Data.ucStableCount = 0;   //重置计数
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
    static CAR_XYR_T s_tCarXYR = {0};
    static CAR_XYR_T *ptCarXYR = &s_tCarXYR;

    ptCarXYR->WhaleDirX = CARDIRECTION_X(s_tMaixCAM_Data.dCoord_X);
    ptCarXYR->WhalePosX = abs( s_tMaixCAM_Data.dCoord_X ) * 10;    

    ptCarXYR->WhaleDirY = CARDIRECTION_Y(s_tMaixCAM_Data.dCoord_Y);
    ptCarXYR->WhalePosY = abs( s_tMaixCAM_Data.dCoord_Y ) * 10;

    //向消息队列 Queue_HandleCalculateHandle 发送轮子参数
    xQueueOverwrite(Queue_HandleCalculateHandle, (void *)&ptCarXYR);    
}

void APP_MAIXCAM_Cal( uint8_t _ucElement, uint8_t _ucColor, uint8_t _ucIsCalFlag )
{
    s_MaixCAM_ID.ucElement = _ucElement;
    s_MaixCAM_ID.ucColor = _ucColor;
    s_MaixCAM_ID.ucIsCalFlag = _ucIsCalFlag;
    
    xEventGroupSetBits( EventGroups_CarHandle, EventGroupsCarMaixCAM_EN_1 );    //开启摄像头
}
