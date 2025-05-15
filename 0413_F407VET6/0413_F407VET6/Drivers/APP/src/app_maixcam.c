#include "main.h"

#define MAIXCAM_CMD_BLOB        (0x01 << 0)

#define MAIXCAM_CMD_CIRCLE      (0x01 << 4)

static uint8_t (*DirMapping)[4];  // 指向包含 4 个 uint8_t 元素的一维数组的指针

static void APP_MAIXCAM_Parse(uint8_t* pucFrameData, uint8_t ucFrameLength);
static uint8_t APP_MAIXCAM_Parse_BLOB_ArriveAndWaitCal(void);
static uint8_t APP_MAIXCAM_Parse_BLOB_Cal(void);
static uint8_t APP_MAIXCAM_Parse_BLOB_WaitGrab(void);
static uint8_t APP_MAIXCAM_Parse_BLOB_WaitGrab2(void);
static uint8_t APP_MAIXCAM_Parse_BLOB_WaitGrab3(void);

static uint8_t APP_MAIXCAM_Parse_CIRCLE_CalCar(void);
static uint8_t APP_MAIXCAM_Parse_CIRCLE_CalXR(void);

static void APP_MAIXCAM_Parse_BLOB_Grab(void);
static void SendToAPP_MotorWhale(void);
float calculate_hypotenuse(float a, float b);
int32_t int32_constrain(int32_t Value, int32_t minValue, int32_t maxValue);

__IO int16_t g_dX = 0;
__IO int16_t g_dY = 0;
__IO int16_t g_dR = 0;

__IO uint8_t g_C = 0;
__IO uint16_t g_X = 0;
__IO uint16_t g_Y = 0;

MAIXCAM_DATA_T s_tMaixCAM_Data =
{




    .ucCalFlag = CAL,              //是否校准标志位
    .ucMoveFlag = MoveCar,         // 0-移动车; 1-移动X方向和R方向步进电机

    .ucSendFlag = 0,             // 数据设置为无效

    .ucStableCount = 0,          //中心标志位计数

    .usBlobCenter_X = 180,       //转盘区中心X
    .usBlobCenter_Y = 50,       //转盘区中心Y
    .ucBlobCenterErrCal = 3,    //转盘区校准误差允许（范围小，以便精准校准位置）
    .ucBlobCenterErrGrab = 7,   //转盘区抓取误差允许（范围大，以便及时响应抓取）
    .ucBlobStableThreshold = 1,  //转盘区稳定次数阈值，调节该参数以使小车不会随物块移动

    .usCircleCenter_X = 170,     //中间色环区中心点X
    .usCircleCenter_Y = 115,     //中间色环区中心点Y

    .usRedCircleCenter_X = 172,     //红色环区中心点X
    .usRedCircleCenter_Y = 132,     //红色环区中心点Y
    .usGreenCircleCenter_X = 169,   //绿色环区中心点X
    .usGreenCircleCenter_Y = 129,   //绿色环区中心点Y
    .usBlueCircleCenter_X = 169,    //蓝色环区中心点X
    .usBlueCircleCenter_Y = 124,    //蓝色环区中心点Y
    .ucCircleCenterErrCal = 1,   //色环区校准误差允许
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

__IO uint8_t tick_flag = 0;

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
    DirMapping = APP_Handle_GetMapPoint();

    float pid_1[3] = {0.6, 0, 0};
    PID_init(&Maxi_Cam_Blob_X, PID_POSITION, pid_1, 6000, 0.1);
    PID_init(&Maxi_Cam_Blob_Y, PID_POSITION, pid_1, 6000, 0.1);
    float pid_2[3] = {0.9, 0, 0};
    PID_init(&Maxi_Cam_Circle_X, PID_POSITION, pid_2, 6000, 0.1);
    PID_init(&Maxi_Cam_Circle_Y, PID_POSITION, pid_2, 6000, 0.1);
}

MAIXCAM_DATA_T *APP_MAIXCAM_GetPoint(void)
{
    return &s_tMaixCAM_Data;
}

uint8_t *APP_MAIXCAM_GetDirMappingColor(void)
{
    return s_tMaixCAM_Data.BlobDirMappingColor;
}


uint8_t APP_MAIXCAM_GetRotateSeq(void)
{
    return s_tMaixCAM_Data.RotateSeq;
}

uint8_t APP_MAIXCAM_GetTickFlag(void)
{
    return tick_flag;
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
        vTaskDelay(pdMS_TO_TICKS(70));
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

    g_C = s_tMaixCAM_Data.BlobColor;
    g_X = s_tMaixCAM_Data.usCoord_X;
    g_Y = s_tMaixCAM_Data.usCoord_Y;
    
    
    static uint32_t tick = 0;
    switch (s_tMaixCAM_Data.ucStatus)
    {
        case MAIXCAM_STATUS_BLOB_ArriveAndWaitCal:  //到达转盘等待校准
            if (APP_MAIXCAM_Parse_BLOB_ArriveAndWaitCal() == 1)
            {
                App_Printf("EXIT MAIXCAM_STATUS_BLOB_ArriveAndWaitCal\r\n");
                s_tMaixCAM_Data.ucStatus = MAIXCAM_STATUS_BLOB_Cal;
                tick = HAL_GetTick();
                App_Printf("ENTER MAIXCAM_STATUS_BLOB_Cal\r\n");
            }
            break;

        case MAIXCAM_STATUS_BLOB_Cal:
            if (APP_MAIXCAM_Parse_BLOB_Cal() == 1)
            {
                App_Printf("EXIT MAIXCAM_STATUS_BLOB_Cal\r\n");
                APP_MOTOR_ZDT_Move_P(MOTOR_R, MOTOR_R_Direction_CCR,     MOTOR_R_Vel, MOTOR_R_Acc, MOTOR_R_Pos[CIRCLE_R_MIDDLE], 1, 0);
                APP_MOTOR_ZDT_Move_P(MOTOR_X, MOTOR_X_Direction_FORWARD, MOTOR_X_Vel, MOTOR_X_Acc, MOTOR_X_Pos[BLOB_X_LEFT], 1, 0);
                APP_MOTOR_ZDT_Move_P(MOTOR_W, MOTOR_W_Direction_CCR,     MOTOR_W_Vel, MOTOR_W_Acc[1], MOTOR_W_Pos[FORWARD_W], 1, 0);
                APP_MOTOR_ZDT_WaitAck(MOTOR_W, MOTOR_W_Timeout);
                
                s_MaixCAM_ID.ucElement = BLOB;
                s_MaixCAM_ID.ucColor = COLOR_TWO;

                tick = HAL_GetTick() - tick;
                tick_flag = (tick < 2500) ? 1 : 2;     //判断车是否在2500ms内校准完成
                App_Printf("tick-%d  tick_flag-%d\r\n", tick, tick_flag);
                if (tick_flag == 1) 
                {
                    s_tMaixCAM_Data.ucStatus = MAIXCAM_STATUS_Blob_WaitGrab;
                    App_Printf("ENTER MAIXCAM_STATUS_Blob_WaitGrab\r\n");
                }
                else if (tick_flag == 2)
                {
                    s_tMaixCAM_Data.ucStatus = MAIXCAM_STATUS_Blob_WaitGrab3;
                    App_Printf("ENTER MAIXCAM_STATUS_Blob_WaitGrab3\r\n");
                }
            }
            break;

        case MAIXCAM_STATUS_Blob_WaitGrab:
            s_tMaixCAM_Data.BlobColorLeft = _pucFrameData[1];
            s_tMaixCAM_Data.BlobColorRight = _pucFrameData[2];

            if (APP_MAIXCAM_Parse_BLOB_WaitGrab() == 1)
            {
                App_Printf("EXIT MAIXCAM_STATUS_Blob_WaitGrab\r\n");
                
                //清零
                s_tMaixCAM_Data.BlobColorFir = 0;
                s_tMaixCAM_Data.usBlobCoord_X_Fir = 0;
                s_tMaixCAM_Data.usBlobCoord_Y_Fir = 0;
                s_tMaixCAM_Data.BlobColorSec = 0;
                
                s_tMaixCAM_Data.ucStatus = MAIXCAM_STATUS_Idle;
                App_Printf("BlobDirMappingColor : %d%d%d\r\n", s_tMaixCAM_Data.BlobDirMappingColor[LEFT_BLOB], s_tMaixCAM_Data.BlobDirMappingColor[MIDDLE_BLOB], s_tMaixCAM_Data.BlobDirMappingColor[RIGHT_BLOB] );
                App_Printf("ENTER MAIXCAM_STATUS_Idle\r\n");
                xTaskNotifyGive(Task_MainHandle);
            }
            break;

        case MAIXCAM_STATUS_Blob_WaitGrab2:
            if (APP_MAIXCAM_Parse_BLOB_WaitGrab2() == 1)
            {
                App_Printf("EXIT MAIXCAM_STATUS_Blob_WaitGrab2\r\n");
                
                s_tMaixCAM_Data.ucStatus = MAIXCAM_STATUS_Idle;
                App_Printf("ENTER MAIXCAM_STATUS_Idle\r\n");
                xTaskNotifyGive(Task_MainHandle);
            }
            break;

        case MAIXCAM_STATUS_Blob_WaitGrab3:
            s_tMaixCAM_Data.BlobColorLeft = _pucFrameData[1];
            s_tMaixCAM_Data.BlobColorRight = _pucFrameData[2];

            if (APP_MAIXCAM_Parse_BLOB_WaitGrab3() == 1)
            {
                App_Printf("EXIT MAIXCAM_STATUS_Blob_WaitGrab3\r\n");
                
                //清零
                s_tMaixCAM_Data.BlobColorFir = 0;
                s_tMaixCAM_Data.usBlobCoord_X_Fir = 0;
                s_tMaixCAM_Data.usBlobCoord_Y_Fir = 0;
                s_tMaixCAM_Data.BlobColorSec = 0;
                
                s_tMaixCAM_Data.ucStatus = MAIXCAM_STATUS_Idle;
                App_Printf("BlobDirMappingColor : %d%d%d\r\n", s_tMaixCAM_Data.BlobDirMappingColor[LEFT_BLOB], s_tMaixCAM_Data.BlobDirMappingColor[MIDDLE_BLOB], s_tMaixCAM_Data.BlobDirMappingColor[RIGHT_BLOB] );
                App_Printf("ENTER MAIXCAM_STATUS_Idle\r\n");
                xTaskNotifyGive(Task_MainHandle);
            }
            break;

        case MAIXCAM_STATUS_Circle_CalCar:
            if (APP_MAIXCAM_Parse_CIRCLE_CalCar() == 1)
            {
                App_Printf("EXIT APP_MAIXCAM_Parse_Circle_CalCar\r\n");
                s_tMaixCAM_Data.ucStatus = MAIXCAM_STATUS_Idle;
                MAIXCAM_Send0();
                xTaskNotifyGive(Task_MainHandle);
                App_Printf("ENTER MAIXCAM_STATUS_Idle\r\n");
            }
            break;

        case MAIXCAM_STATUS_Circle_CalXR:
            if (APP_MAIXCAM_Parse_CIRCLE_CalXR() == 1)
            {
                App_Printf("EXIT APP_MAIXCAM_Parse_CIRCLE_CalXR\r\n");
                s_tMaixCAM_Data.ucStatus = MAIXCAM_STATUS_Idle;
                MAIXCAM_Send0();
                xTaskNotifyGive(Task_MainHandle);
                App_Printf("ENTER MAIXCAM_STATUS_Idle\r\n");
            }
            break;

        default:
            break;
    }
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
    static uint8_t ArriveFlag = 0;
    static uint8_t BlobMovingFlag = 0;    //1-物块静止  2-物块正在转动
    
    s_tMaixCAM_Data.usPresentX = s_tMaixCAM_Data.usCoord_X;
    s_tMaixCAM_Data.usPresentY = s_tMaixCAM_Data.usCoord_Y;

    App_Printf("curC=%d curX=%d curY=%d\r\n", s_tMaixCAM_Data.BlobColor, s_tMaixCAM_Data.usPresentX, s_tMaixCAM_Data.usPresentY);
    if (!s_tMaixCAM_Data.usPresentX)   //如果收到的X Y数据有一个为0，则说明相机没有识别到，让摄像头重新发送
    {
        App_Printf("X=0\r\n");
        s_tMaixCAM_Data.ucSendFlag = 0;
//        APP_MAIXCAM_SendAndRead(s_MaixCAM_ID.ucElement, s_MaixCAM_ID.ucColor);
        return 0;
    }
    
    //当小车到达时，记录第一次坐标，更新Previous，并延迟100ms直接return
    if (ArriveFlag == 0)
    {
        s_tMaixCAM_Data.usPreviousX = s_tMaixCAM_Data.usPresentX;
        s_tMaixCAM_Data.usPreviousY = s_tMaixCAM_Data.usPresentY;
        ArriveFlag = 1;
//        vTaskDelay(pdMS_TO_TICKS(40));  //等待物块转动，以便下次判断物块前后两次坐标是否一致

        App_Printf("ArriveFlag=1\r\n");
        App_Printf("pvX=%d pvY=%d\r\n", s_tMaixCAM_Data.usPreviousX, s_tMaixCAM_Data.usPreviousY);
        return 0;
    }

    //记录完第一次坐标后，与第二次坐标进行判断
    if (ArriveFlag == 1)
    {
        // 判断坐标是否不动，如果没动，小车等待下个物块到达，根据下个物块进行校准
        if (abs(s_tMaixCAM_Data.usPresentX - s_tMaixCAM_Data.usPreviousX) < 4 
            && abs(s_tMaixCAM_Data.usPresentY - s_tMaixCAM_Data.usPreviousY) < 4)
        {
            BlobMovingFlag = 1;
            s_tMaixCAM_Data.BlobColorFir = s_tMaixCAM_Data.BlobColorFir ? : s_tMaixCAM_Data.BlobColor;
            s_tMaixCAM_Data.usBlobCoord_X_Fir = s_tMaixCAM_Data.usBlobCoord_X_Fir ? : s_tMaixCAM_Data.usPresentX;
            s_tMaixCAM_Data.usBlobCoord_Y_Fir = s_tMaixCAM_Data.usBlobCoord_Y_Fir ? : s_tMaixCAM_Data.usPresentY;
        }
        // 如果动了，则小车等待物块稳定，根据即将到来的物块进行校准
        else
        {
            BlobMovingFlag = 2;
        }
        ArriveFlag = 2;
        App_Printf("ArriveFlag=2\r\n");
        App_Printf("BlobMovingFlag=%d\r\n", BlobMovingFlag);
        App_Printf("FirC=%d FirX=%d FirY=%d\r\n", s_tMaixCAM_Data.BlobColorFir, s_tMaixCAM_Data.usBlobCoord_X_Fir, s_tMaixCAM_Data.usBlobCoord_Y_Fir);
        return 0;
    }

    if (BlobMovingFlag == 1)  //根据下一次物块进行校准
    {
        App_Printf("BF-1\r\n");
        if (s_tMaixCAM_Data.BlobColor != s_tMaixCAM_Data.BlobColorFir) 
        {
            //下一次的物块已静止
            if (abs(s_tMaixCAM_Data.usBlobCoord_X_Fir - s_tMaixCAM_Data.usPresentX) < 20 
                && abs(s_tMaixCAM_Data.usBlobCoord_Y_Fir - s_tMaixCAM_Data.usPresentY) < 20)
            {
                s_tMaixCAM_Data.usPreviousX = 0;
                s_tMaixCAM_Data.usPreviousY = 0;
                ArriveFlag = 0;
                BlobMovingFlag = 0;
        App_Printf("BlobMovingFlag=1 done\r\n");
                return 1;
            }
        }
    }
    else if (BlobMovingFlag == 2) //根据即将到来的物块进行校准
    {
        App_Printf("BF-2\r\n");
        //即将到来的物块已静止
        if (abs(s_tMaixCAM_Data.usPresentX - s_tMaixCAM_Data.usPreviousX) < 5 
            && abs(s_tMaixCAM_Data.usPresentY - s_tMaixCAM_Data.usPreviousY) < 5)
        {
            s_tMaixCAM_Data.usPreviousX = 0;
            s_tMaixCAM_Data.usPreviousY = 0;
            ArriveFlag = 0;
            BlobMovingFlag = 0;
        App_Printf("BlobMovingFlag=2 done\r\n");
            return 1;
        }
    }
    
    s_tMaixCAM_Data.usPreviousX = s_tMaixCAM_Data.usPresentX;
    s_tMaixCAM_Data.usPreviousY = s_tMaixCAM_Data.usPresentY;

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
    s_tMaixCAM_Data.usPresentX = s_tMaixCAM_Data.usCoord_X;
    s_tMaixCAM_Data.usPresentY = s_tMaixCAM_Data.usCoord_Y;

    App_Printf("PsX %d  PsY %d\r\n", s_tMaixCAM_Data.usPresentX, s_tMaixCAM_Data.usPresentY);
    App_Printf("PvX %d  PvY %d\r\n", s_tMaixCAM_Data.usPreviousX, s_tMaixCAM_Data.usPreviousY);
    
    if ( !s_tMaixCAM_Data.usPresentX )   //如果收到的X Y数据有一个为0，则说明相机没有识别到，让摄像头重新发送
    {
        App_Printf("X0 Y0\r\n");
        s_tMaixCAM_Data.ucSendFlag = 0;
//        APP_MAIXCAM_SendAndRead(s_MaixCAM_ID.ucElement, s_MaixCAM_ID.ucColor);
        return 0;
    }

//    if (abs(s_tMaixCAM_Data.usPresentX - s_tMaixCAM_Data.usPreviousX) > 15 
//        || abs(s_tMaixCAM_Data.usPresentY - s_tMaixCAM_Data.usPreviousY) > 15)
//    {
//        App_Printf("X1 Y1\r\n");
//        s_tMaixCAM_Data.usPreviousX = s_tMaixCAM_Data.usPresentX;
//        s_tMaixCAM_Data.usPreviousY = s_tMaixCAM_Data.usPresentY;
//        return 0;
//    }
    s_tMaixCAM_Data.usPreviousX = s_tMaixCAM_Data.usPresentX;
    s_tMaixCAM_Data.usPreviousY = s_tMaixCAM_Data.usPresentY;

    s_tMaixCAM_Data.dCoord_X = s_tMaixCAM_Data.usPresentX - s_tMaixCAM_Data.usBlobCenter_X;
    s_tMaixCAM_Data.dCoord_Y = s_tMaixCAM_Data.usPresentY - s_tMaixCAM_Data.usBlobCenter_Y;
    g_dX = s_tMaixCAM_Data.dCoord_X;
    g_dY = s_tMaixCAM_Data.dCoord_Y;

    // 判断是否位于小范围中心
    if (abs(s_tMaixCAM_Data.dCoord_X) <= s_tMaixCAM_Data.ucBlobCenterErrCal &&
            abs(s_tMaixCAM_Data.dCoord_Y) <= s_tMaixCAM_Data.ucBlobCenterErrCal)
    {
        // 位于小范围中心
        s_tMaixCAM_Data.usPreviousX = 0;
        s_tMaixCAM_Data.usPreviousY = 0;
        s_MaixCAM_ID.ucElement = ELEMENT_NONE;
        s_MaixCAM_ID.ucColor = WHITE;
        s_tMaixCAM_Data.BlobColorSec = s_tMaixCAM_Data.BlobColorSec ? : s_tMaixCAM_Data.BlobColor;
        return 1;
    }

    //移动车身pid校准
    {
        double err_x = (-s_tMaixCAM_Data.dCoord_X - s_tMaixCAM_Data.dCoord_Y) / 2;
        double err_y = (-s_tMaixCAM_Data.dCoord_X + s_tMaixCAM_Data.dCoord_Y) / 2;
        //int speed_temp=(moto_chassis[0].speed_rpm+moto_chassis[1].speed_rpm+moto_chassis[2].speed_rpm+moto_chassis[3].speed_rpm)/4.0;
        Maxi_Cam_Blob_X.set = 0;
        Maxi_Cam_Blob_X.Ki = 0.75 + (fabs(err_x) / 4);
        Maxi_Cam_Blob_Y.Ki = 0.75 + (fabs(err_y) / 4);
        PID_calc(&Maxi_Cam_Blob_X, -err_x, 0);
        PID_calc(&Maxi_Cam_Blob_Y, -err_y, 0);
        //float w = Maxi_Cam_Blob_X.out;

        s_tMaixCAM_Data.dCoord_X = Maxi_Cam_Blob_X.out;
        s_tMaixCAM_Data.dCoord_Y = Maxi_Cam_Blob_Y.out;

        // 发送移动距离量到APP_MotorWhale的消息队列
        SendToAPP_MotorWhale();
        return 2;
    }
}

/*
*********************************************************************************************************
*    函 数 名: APP_MAIXCAM_Parse_BLOB_WaitGrab
*    功能说明: 物块校准状态
*    形    参: 无
*    返 回 值: ret  0-未检测到目标/不进行操作；1-状态成功结束；
*********************************************************************************************************
*/
static uint8_t APP_MAIXCAM_Parse_BLOB_WaitGrab(void)
{
    if (s_tMaixCAM_Data.BlobColorLeft && s_tMaixCAM_Data.BlobColorRight)
    {
        s_tMaixCAM_Data.BlobDirMappingColor[LEFT_BLOB] = s_tMaixCAM_Data.BlobColorLeft;
        s_tMaixCAM_Data.BlobDirMappingColor[RIGHT_BLOB] = s_tMaixCAM_Data.BlobColorRight;
        //数学法，使用三个颜色的数值总和减去其中两个，可得第三个颜色的数值
        s_tMaixCAM_Data.BlobDirMappingColor[MIDDLE_BLOB] = 6 - s_tMaixCAM_Data.BlobDirMappingColor[LEFT_BLOB] - s_tMaixCAM_Data.BlobDirMappingColor[RIGHT_BLOB];

        s_MaixCAM_ID.ucElement = ELEMENT_NONE;
        s_MaixCAM_ID.ucColor = WHITE;
        return 1;
    }
    return 0;
}

/*
*********************************************************************************************************
*    函 数 名: APP_MAIXCAM_Parse_BLOB_WaitGrab
*    功能说明: 物块校准状态
*    形    参: 无
*    返 回 值: ret  0-未检测到目标/不进行操作；1-状态成功结束；
*********************************************************************************************************
*/
static uint8_t APP_MAIXCAM_Parse_BLOB_WaitGrab2(void)
{
    static uint8_t ArriveFlag = 0;
    
    s_tMaixCAM_Data.usPresentX = s_tMaixCAM_Data.usCoord_X;
    s_tMaixCAM_Data.usPresentY = s_tMaixCAM_Data.usCoord_Y;
    App_Printf("PsX %d  PsY %d\r\n", s_tMaixCAM_Data.usPresentX, s_tMaixCAM_Data.usPresentY);

    if (!s_tMaixCAM_Data.usPresentX)   //如果收到的X Y数据有一个为0，则说明相机没有识别到，让摄像头重新发送
    {
        App_Printf("X=0\r\n");
        s_tMaixCAM_Data.ucSendFlag = 0;
//        APP_MAIXCAM_SendAndRead(s_MaixCAM_ID.ucElement, s_MaixCAM_ID.ucColor);
        return 0;
    }
    
    //记录第一次坐标，更新Previous
    if (ArriveFlag == 0)
    {
        s_tMaixCAM_Data.usPreviousX = s_tMaixCAM_Data.usPresentX;
        s_tMaixCAM_Data.usPreviousY = s_tMaixCAM_Data.usPresentY;
        ArriveFlag = 1;
        App_Printf("ArriveFlag = 1;\r\n");
        App_Printf("pvX=%d pvY=%d\r\n", s_tMaixCAM_Data.usPreviousX, s_tMaixCAM_Data.usPreviousY);
        return 0;
    }

    //记录完第一次坐标后，与第二次坐标进行判断
    if (ArriveFlag == 1)
    {
        // 判断坐标是否不动，如果没动，小车等待下个物块到达，根据下个物块进行校准
        if (abs(s_tMaixCAM_Data.usPresentX - s_tMaixCAM_Data.usPreviousX) < 8 
            && abs(s_tMaixCAM_Data.usPresentY - s_tMaixCAM_Data.usPreviousY) < 8)
        {
            s_MaixCAM_ID.ucElement = ELEMENT_NONE;
            s_MaixCAM_ID.ucColor = WHITE;
            
            ArriveFlag = 0;
            return 1;
        }
    }
    
    s_tMaixCAM_Data.usPreviousX = s_tMaixCAM_Data.usPresentX;
    s_tMaixCAM_Data.usPreviousY = s_tMaixCAM_Data.usPresentY;
    
    return 0;
}

static uint8_t APP_MAIXCAM_Parse_BLOB_WaitGrab3(void)
{
    if (s_tMaixCAM_Data.BlobColorLeft==s_tMaixCAM_Data.BlobColorSec || s_tMaixCAM_Data.BlobColorRight==s_tMaixCAM_Data.BlobColorSec)
    {
        s_tMaixCAM_Data.BlobDirMappingColor[LEFT_BLOB] = s_tMaixCAM_Data.BlobColorLeft;
        s_tMaixCAM_Data.BlobDirMappingColor[RIGHT_BLOB] = s_tMaixCAM_Data.BlobColorRight;
        //数学法，使用三个颜色的数值总和减去其中两个，可得第三个颜色的数值
        s_tMaixCAM_Data.BlobDirMappingColor[MIDDLE_BLOB] = 6 - s_tMaixCAM_Data.BlobDirMappingColor[LEFT_BLOB] - s_tMaixCAM_Data.BlobDirMappingColor[RIGHT_BLOB];
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
static uint8_t APP_MAIXCAM_Parse_CIRCLE_CalCar(void)
{
    uint8_t isInCenter = 0;

    s_tMaixCAM_Data.usPresentX = s_tMaixCAM_Data.usCoord_X;
    s_tMaixCAM_Data.usPresentY = s_tMaixCAM_Data.usCoord_Y;

    if (!(s_tMaixCAM_Data.usPresentX && s_tMaixCAM_Data.usPresentY))   //如果收到的X Y数据有一个为0，则说明相机没有识别到，让摄像头重新发送
    {
        s_tMaixCAM_Data.ucSendFlag = 0;
//        APP_MAIXCAM_SendAndRead(s_MaixCAM_ID.ucElement, s_MaixCAM_ID.ucColor);
        return 0;
    }

    s_tMaixCAM_Data.dCoord_X = s_tMaixCAM_Data.usCoord_X - s_tMaixCAM_Data.usCircleCenter_X;
    s_tMaixCAM_Data.dCoord_Y = s_tMaixCAM_Data.usCoord_Y - s_tMaixCAM_Data.usCircleCenter_Y;

    g_dX = s_tMaixCAM_Data.dCoord_X;
    g_dY = s_tMaixCAM_Data.dCoord_Y;

    //坐标是否位于中心
    uint8_t err;
    if (s_MaixCAM_ID.ucElement == BLOB)
    {
        err = s_tMaixCAM_Data.ucBlobCenterErrGrab;
    }
    else if (s_MaixCAM_ID.ucElement == CIRCLE)
    {
        err = s_tMaixCAM_Data.ucCircleCenterErrCal;
    }
    if (abs(s_tMaixCAM_Data.dCoord_X) <= err &&
            abs(s_tMaixCAM_Data.dCoord_Y) <= err)
    {
        // 坐标连续n次位于中心
        if (s_tMaixCAM_Data.ucStableCount < s_tMaixCAM_Data.ucCircleStableThreshold)
        {
            s_tMaixCAM_Data.ucStableCount++;
        }
        else   // 计数达到阈值，认为真正在中心
        {
            s_MaixCAM_ID.ucElement = ELEMENT_NONE;
            s_MaixCAM_ID.ucColor = WHITE;
            s_tMaixCAM_Data.ucStableCount = 0;   //重置计数
            s_tMaixCAM_Data.dCoord_R = 0;
            s_tMaixCAM_Data.dCoord_X = 0;
            
            return 1;
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
        s_tMaixCAM_Data.dCoord_X = Maxi_Cam_Circle_X.out;
        s_tMaixCAM_Data.dCoord_Y = Maxi_Cam_Circle_Y.out;
        SendToAPP_MotorWhale();
    }

    g_dX = s_tMaixCAM_Data.dCoord_X;
    g_dY = s_tMaixCAM_Data.dCoord_Y;
    
    return 0;
}

/*
*********************************************************************************************************
*    函 数 名: APP_MAIXCAM_Parse_CIRCLE_CalXR
*    功能说明: 色环校准X和R方向步进电机
*    形    参: 无
*    返 回 值: 无
*********************************************************************************************************
*/
static uint8_t APP_MAIXCAM_Parse_CIRCLE_CalXR(void)
{
    uint8_t isInCenter = 0;

    float dR = 0, dX = 0;

    if ((s_tMaixCAM_Data.usCoord_X == 0) || (s_tMaixCAM_Data.usCoord_Y == 0))
    {
//        APP_MAIXCAM_SendAndRead(s_MaixCAM_ID.ucElement, s_MaixCAM_ID.ucColor);
        return 0;
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
            App_Printf("******** %d ********* CalOk\r\n", s_MaixCAM_ID.ucColor);

            s_MaixCAM_ID.ucElement = ELEMENT_NONE;
            s_MaixCAM_ID.ucColor = WHITE;
            s_tMaixCAM_Data.ucStableCount = 0;   //重置计数 
            return 1;
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

        APP_MOTOR_ZDT_Move_P(MOTOR_X, MOTOR_X_Direction_FORWARD,  100, 50, MOTOR_X_Pos[DirMapping[Fir][s_MaixCAM_ID.ucColor]], 1, 0);
        APP_MOTOR_ZDT_Move_P(MOTOR_R, MOTOR_R_Direction_CCR,      100, 50, MOTOR_R_Pos[DirMapping[Fir][s_MaixCAM_ID.ucColor]], 1, 0);
        APP_MOTOR_ZDT_WaitAck(MOTOR_X, 500);
        APP_MOTOR_ZDT_WaitAck(MOTOR_R, 500);
//        vTaskDelay(pdMS_TO_TICKS(200));
    }

    g_dX = s_tMaixCAM_Data.dCoord_X;
    g_dR = s_tMaixCAM_Data.dCoord_R;
    
    return 0;
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
    int16_t dCoord_X_CAR = s_tMaixCAM_Data.dCoord_X;
    int16_t dCoord_Y_CAR = s_tMaixCAM_Data.dCoord_Y;
    if (s_tMaixCAM_Data.ucCalFlag == BlobCalFlag)
    {
        APP_CHASSIS_MovePID(CHASSIS_CAL, CHASSIS_CALL_BLOCKING, CARDIRECTION_BLOB_X(dCoord_X_CAR), abs(dCoord_X_CAR) * k);
        APP_CHASSIS_MovePID(CHASSIS_CAL, CHASSIS_CALL_BLOCKING, CARDIRECTION_BLOB_Y(dCoord_Y_CAR), abs(dCoord_Y_CAR) * k);
    }
    else if (s_tMaixCAM_Data.ucCalFlag == CircleCalFlag)
    {
        APP_CHASSIS_MovePID(CHASSIS_CAL, CHASSIS_CALL_BLOCKING, CARDIRECTION_CIRCLE_X(dCoord_X_CAR), abs(dCoord_X_CAR) * k);
        APP_CHASSIS_MovePID(CHASSIS_CAL, CHASSIS_CALL_BLOCKING, CARDIRECTION_CIRCLE_Y(dCoord_Y_CAR), abs(dCoord_Y_CAR) * k);
    }
}

void APP_MAIXCAM_Cal(uint8_t _ucElement, uint8_t _ucColor, uint8_t _ucIsCalFlag, uint8_t _ucMoveFlag)
{
    s_tMaixCAM_Data.ucCalFlag = _ucIsCalFlag;
    s_tMaixCAM_Data.ucMoveFlag = _ucMoveFlag;

    if (s_tMaixCAM_Data.ucCalFlag == BlobCalFlag) 
    {
        s_tMaixCAM_Data.ucStatus = MAIXCAM_STATUS_BLOB_ArriveAndWaitCal;
    }
    else if (s_tMaixCAM_Data.ucCalFlag == BlobGrabFlag) 
    {
        APP_MOTOR_ZDT_Move_P(MOTOR_R, MOTOR_R_Direction_CCR,     MOTOR_R_Vel, MOTOR_R_Acc, 3500, 1, 0);
        APP_MOTOR_ZDT_Move_P(MOTOR_X, MOTOR_X_Direction_FORWARD, MOTOR_X_Vel, MOTOR_X_Acc, MOTOR_X_Pos[BLOB_X_LEFT], 1, 0);
        APP_MOTOR_ZDT_Move_P(MOTOR_W, MOTOR_W_Direction_CCR,     MOTOR_W_Vel, MOTOR_W_Acc[0], MOTOR_W_Pos[FORWARD_W], 1, 0);
        APP_MOTOR_ZDT_WaitAck(MOTOR_W, 500);
        s_tMaixCAM_Data.ucStatus = MAIXCAM_STATUS_Blob_WaitGrab2;
    }
    else if (s_tMaixCAM_Data.ucCalFlag == CircleCalFlag) 
    {
        if (s_tMaixCAM_Data.ucMoveFlag == MoveCar)
        {
            s_tMaixCAM_Data.ucStatus = MAIXCAM_STATUS_Circle_CalCar;
            __NOP();
        }
        else if (s_tMaixCAM_Data.ucMoveFlag == MoveXR)
        {
            s_tMaixCAM_Data.ucStatus = MAIXCAM_STATUS_Circle_CalXR;
            APP_MOTOR_ZDT_Move_P(MOTOR_R, MOTOR_R_Direction_CCR,     200, 100, MOTOR_R_Pos[DirMapping[Fir][_ucColor]], 1, 0);
            APP_MOTOR_ZDT_Move_P(MOTOR_X, MOTOR_X_Direction_FORWARD, 200, 100, MOTOR_X_Pos[DirMapping[Fir][_ucColor]], 1, 0);
            //Y0：最高点
            APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, 500, 220, 8500, 1, 0);
            APP_MOTOR_ZDT_WaitAck(MOTOR_X, MOTOR_X_Timeout);
            APP_MOTOR_ZDT_WaitAck(MOTOR_R, MOTOR_R_Timeout);
            APP_MOTOR_ZDT_WaitAck(MOTOR_Y, MOTOR_Y_Timeout);
        }
    }

    s_MaixCAM_ID.ucElement = _ucElement;
    s_MaixCAM_ID.ucColor = _ucColor;
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
