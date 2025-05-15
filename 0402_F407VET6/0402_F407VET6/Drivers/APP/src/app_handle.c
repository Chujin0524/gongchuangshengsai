#include "main.h"

/*
腕处位置 :
*/
#define  CIRCLE_W_LEFT    1
#define  CIRCLE_W_MIDDLE  2
#define  CIRCLE_W_RIGHT   3
#define  BLOB_W_LEFT      4
#define  BLOB_W_MIDDLE    5
#define  BLOB_W_RIGHT     6
#define  RIHGT_W          7
#define  FORWARD_W        8
static uint8_t MOTOR_W_Acc = 50;
static uint16_t MOTOR_W_Vel = 200;
static uint32_t MOTOR_W_Pos[9] =
{
    0,        //0 : 初始

    6000,     //1 : 指向色环左边
    3750,     //2 : 指向色环中间
    1514,     //3 : 指向色环右边

    6000,     //4 : 指向物块左边
    3750,     //5 : 指向物块中间
    1514,     //6 : 指向物块右边

    3750,     //7 : 朝右，在物料盘上方
    1514      //8 : 朝前
};

/*
转盘位置 :
*/
#define  CIRCLE_R_LEFT    1
#define  CIRCLE_R_MIDDLE  2
#define  CIRCLE_R_RIGHT   3
#define  BLOB_R_LEFT      4
#define  BLOB_R_MIDDLE    5
#define  BLOB_R_RIGHT     6
static uint8_t MOTOR_R_Acc = 50;
static uint16_t MOTOR_R_Vel = 200;
static uint32_t MOTOR_R_Pos[7] =
{
    0,        //0 : 初始

    6000,     //1 : 指向色环左边
    3750,     //2 : 指向色环中间
    1514,     //3 : 指向色环右边

    6000,     //4 : 指向物块左边
    3750,     //5 : 指向物块中间
    1514      //6 : 指向物块右边
};

/*
X位置 :
*/
#define  CIRCLE_X_LEFT    1
#define  CIRCLE_X_MIDDLE  2
#define  CIRCLE_X_RIGHT   3
#define  BLOB_X_LEFT      4
#define  BLOB_X_MIDDLE    5
#define  BLOB_X_RIGHT     6
static uint8_t MOTOR_X_Acc = 50;
static uint16_t MOTOR_X_Vel = 200;
static uint32_t MOTOR_X_Pos[7] =
{
    0,        //0 : 初始

    6000,     //1 : 指向色环左边
    3750,     //2 : 指向色环中间
    1514,     //3 : 指向色环右边

    6000,     //4 : 指向物块左边
    3750,     //5 : 指向物块中间
    1514      //6 : 指向物块右边
};

//Y位置
static uint8_t MOTOR_Y_Acc = 255;
static uint16_t MOTOR_Y_Vel = 700;
static uint32_t MOTOR_Y_Pos[9] =
{
    0,        //0 : 最高点
    0,        //1 : 最高点-300
    0,        //2 : 夹着物块能越过转盘的位置
    1600,     //3 : 夹着物块放到转盘上的位置
    10900,    //4 : 夹着物块放到色环上的位置
    5300,     //5 : 码垛物块的位置
    8500,     //6 : 物块在色环上，夹爪能越过的位置
    0,        //7 :
    1600,     //8 : 从转盘区夹起物块的位置
};

// 色环顺序映射表
// 色环顺序，与贴纸对应
uint8_t DirMapping[2][4] = {0};  // [第i次加工区][红, 绿, 蓝] = [左中右]

uint32_t *APP_Handle_GetPosPoint(uint8_t _Motor)
{
    switch (_Motor)
    {
        case MOTOR_R:
            return MOTOR_R_Pos;

        case MOTOR_X:
            return MOTOR_X_Pos;

        case MOTOR_Y:
            return MOTOR_Y_Pos;

        default:
            return 0;
    }
}

uint8_t (*APP_Handle_GetMapPoint(void))[4]
{
    return DirMapping;
}

// 初始化色环贴纸上RGB对应的方向
void APP_Handle_ColorMapping_Init(uint8_t _time, uint8_t RedDir, uint8_t GreenDir, uint8_t BlueDir)
{
    DirMapping[_time][RED]   = RedDir;
    DirMapping[_time][GREEN] = GreenDir;
    DirMapping[_time][BLUE]  = BlueDir;
}

void APP_Handle_PickRawArea(uint8_t _ucColor, uint8_t _ucIndex)
{
    uint32_t Delay_Wrist = 1000;
    //Y0：最高点
    APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[0], 1, 0);
    APP_MOTOR_ZDT_WaitAck();

    switch (_ucIndex)
    {
        case LEFT_BLOB:
            //控制底盘转盘转动到左边物块方向
            APP_MOTOR_ZDT_Move_P(MOTOR_R, MOTOR_R_Direction_CCR,     MOTOR_R_Vel, MOTOR_R_Acc, MOTOR_R_Pos[BLOB_R_LEFT], 1, 0);
            //X导轨伸出到左边物块
            APP_MOTOR_ZDT_Move_P(MOTOR_X, MOTOR_X_Direction_FORWARD, MOTOR_X_Vel, MOTOR_X_Acc, MOTOR_X_Pos[BLOB_X_LEFT], 1, 0);

            //腕处转到左边物块
            APP_MOTOR_ZDT_Move_P(MOTOR_W, MOTOR_W_Direction_CCR,     MOTOR_W_Vel, MOTOR_W_Acc, MOTOR_W_Pos[BLOB_W_LEFT], 1, 0);
            APP_MOTOR_ZDT_WaitAck();
            break;

        case MIDDLE_BLOB:
            //控制底盘转盘转动到左边物块方向
            APP_MOTOR_ZDT_Move_P(MOTOR_R, MOTOR_R_Direction_CCR,     MOTOR_R_Vel, MOTOR_R_Acc, MOTOR_R_Pos[BLOB_R_MIDDLE], 1, 0);
            //X导轨伸出到左边物块
            APP_MOTOR_ZDT_Move_P(MOTOR_X, MOTOR_X_Direction_FORWARD, MOTOR_X_Vel, MOTOR_X_Acc, MOTOR_X_Pos[BLOB_X_MIDDLE], 1, 0);

            //腕处转到左边物块
            APP_MOTOR_ZDT_Move_P(MOTOR_W, MOTOR_W_Direction_CCR,     MOTOR_W_Vel, MOTOR_W_Acc, MOTOR_W_Pos[BLOB_W_MIDDLE], 1, 0);
            APP_MOTOR_ZDT_WaitAck();
            break;

        case RIGHT_BLOB:
            //控制底盘转盘转动到左边物块方向
            APP_MOTOR_ZDT_Move_P(MOTOR_R, MOTOR_R_Direction_CCR,     MOTOR_R_Vel, MOTOR_R_Acc, MOTOR_R_Pos[BLOB_R_RIGHT], 1, 0);
            //X导轨伸出到左边物块
            APP_MOTOR_ZDT_Move_P(MOTOR_X, MOTOR_X_Direction_FORWARD, MOTOR_X_Vel, MOTOR_X_Acc, MOTOR_X_Pos[BLOB_X_RIGHT], 1, 0);

            //腕处转到左边物块
            APP_MOTOR_ZDT_Move_P(MOTOR_W, MOTOR_W_Direction_CCR,     MOTOR_W_Vel, MOTOR_W_Acc, MOTOR_W_Pos[BLOB_W_RIGHT], 1, 0);
            APP_MOTOR_ZDT_WaitAck();
            break;

        default:
            break;
    }

    //Y8：从转盘区夹起物块的位置
    APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[8], 1, 0);
    APP_MOTOR_ZDT_WaitAck();

    APP_SERVO_Jaw('Z');     //夹爪舵机夹紧物块

    //Y2：夹着物块能越过转盘的位置
    APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[2], 1, 0);
    APP_MOTOR_ZDT_WaitAck();

    //腕处转到物料盘上方
    APP_MOTOR_ZDT_Move_P(MOTOR_W, MOTOR_W_Direction_CCR,     MOTOR_W_Vel, MOTOR_W_Acc, MOTOR_W_Pos[RIHGT_W], 1, 0);
    APP_MOTOR_ZDT_WaitAck();

    //Y3：夹着物块放到物料盘上的位置
    APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[3], 1, 0);
    APP_MOTOR_ZDT_WaitAck();

    APP_SERVO_Jaw('S');     //夹爪舵机松开物块

    //Y0：最高点
    APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[0], 1, 0);
    APP_MOTOR_ZDT_WaitAck();

    xTaskNotifyGive(Task_MainHandle);
}

/*
转盘舵机seq[1]

第i次放置，i＝1，2，3
{
                                腕处舵机右转
    底部转盘seq[i]
    x轴seq[i]
    延迟，等待舵机右转完成
    y2
                                    夹爪舵机抓紧，延迟等待
    y1
                                腕处舵机朝前，延迟等待
                        转盘舵机seq[i+1]
    if ProFir，
        y4
    else if ProSec，
        y3
                                    夹爪舵机松开，延迟等待
     if 第3次，
        if ProFir，
            y5；
        else if ProSec，
            y0
    else
        y1
}
*/
void APP_Handle_Place_All(uint8_t _ucCycle, uint8_t _firstIndex, uint8_t _secondIndex, uint8_t _thirdIndex)
{
    uint32_t Delay_up = 400;
    uint32_t Delay_Wrist = 1000;
    uint8_t ucSeq[3] = {DirMapping[_ucCycle][_firstIndex], DirMapping[_ucCycle][_secondIndex], DirMapping[_ucCycle][_thirdIndex]};
    App_Printf("ColorSeq %d %d %d \r\n", _firstIndex, _secondIndex, _thirdIndex);
    App_Printf("DirSeq %d %d %d \r\n", ucSeq[0], ucSeq[1], ucSeq[2]);
    App_Printf("MOTOR_X_Pos[%d]=%5d   MOTOR_R_Pos[%d]=%5d\r\n", ucSeq[0], MOTOR_X_Pos[ucSeq[0]], ucSeq[0], MOTOR_R_Pos[ucSeq[0]]);
    App_Printf("MOTOR_X_Pos[%d]=%5d   MOTOR_R_Pos[%d]=%5d\r\n", ucSeq[1], MOTOR_X_Pos[ucSeq[1]], ucSeq[1], MOTOR_R_Pos[ucSeq[1]]);
    App_Printf("MOTOR_X_Pos[%d]=%5d   MOTOR_R_Pos[%d]=%5d\r\n", ucSeq[2], MOTOR_X_Pos[ucSeq[2]], ucSeq[2], MOTOR_R_Pos[ucSeq[2]]);
    //      DirMapping[0][] = B G R
    APP_SERVO_Plate(_firstIndex);        //转盘舵机转到对应物块的放置角度

    for (uint8_t i = 0; i < 3; i++)
    {
        //腕处舵机朝右
        APP_SERVO_Wrist('R');

        //控制底盘转盘转动到指定色环方向
        APP_MOTOR_ZDT_Move_P(MOTOR_R, MOTOR_R_Direction_CCR,     MOTOR_R_Vel, MOTOR_R_Acc, MOTOR_R_Pos[ucSeq[i]], 1, 0);

        //X导轨伸出到指定色环
        APP_MOTOR_ZDT_Move_P(MOTOR_X, MOTOR_X_Direction_FORWARD, MOTOR_X_Vel, MOTOR_X_Acc, MOTOR_X_Pos[ucSeq[i]], 1, 0);

        //延迟，等待舵机右转完成
        vTaskDelay(pdMS_TO_TICKS(Delay_Wrist));

        //Y3：夹着物块放到转盘上的位置
        APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[3], 1, 0);
        APP_MOTOR_ZDT_WaitAck();

        //夹爪舵机夹紧物块，内有延迟
        APP_SERVO_Jaw('Z');

        //Y2：夹着物块能越过转盘的位置
        APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[2], 1, 0);
        APP_MOTOR_ZDT_WaitAck();

        //腕处舵机朝前，延迟等待
        APP_SERVO_Wrist('F');
        vTaskDelay(pdMS_TO_TICKS(Delay_Wrist));

        if (i < 2)  APP_SERVO_Plate(ucSeq[i + 1]);    //转盘舵机转到下一个物块的放置角度
        else        APP_SERVO_Plate(ucSeq[0]);        //转盘舵机转到第一个物块的放置角度

        if (_ucCycle == 0)
        {
            //Y4：夹着物块放到色环上的位置
            APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[4], 1, 0);
            APP_MOTOR_ZDT_WaitAck();
        }
        else if (_ucCycle == 1)
        {
            //Y5：码垛物块的位置
            APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[5], 1, 0);
            APP_MOTOR_ZDT_WaitAck();
        }

        //夹爪舵机松开物块，内有延迟
        APP_SERVO_Jaw('S');

        if (i == 2)
        {
            if (_ucCycle == 0)
            {
                //Y5：物块在色环上，夹爪能越过的位置
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[0], 1, 0);
                APP_MOTOR_ZDT_WaitAck();
            }
            else if (_ucCycle == 1)
            {
                //Y1：最高点-300
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0);
                APP_MOTOR_ZDT_WaitAck();
            }
        }
        else
        {
            //Y2：夹着物块能越过转盘的位置
            APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[2], 1, 0);
            APP_MOTOR_ZDT_WaitAck();
        }
    }

    xTaskNotifyGive(Task_MainHandle);
}

void APP_Handle_Place_One(uint8_t _color)
{
    uint32_t Delay_up = 400;
    uint32_t Delay_Wrist = 1000;
    uint8_t ucSeq = DirMapping[Fir][_color];
    App_Printf("Color %d \r\n", _color);
    App_Printf("Dir %d \r\n", ucSeq);
    App_Printf("MOTOR_X_Pos=%5d   MOTOR_R_Pos=%5d\r\n", MOTOR_X_Pos[ucSeq], MOTOR_R_Pos[ucSeq]);
    //      DirMapping[0][] = B G R
    APP_SERVO_Plate(_color);        //转盘舵机转到对应物块的放置角度

    //Y0：最高点
    APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[0], 1, 0);
    APP_MOTOR_ZDT_WaitAck();

    //腕处舵机朝右
    APP_SERVO_Wrist('R');
    //延迟，等待舵机右转完成
    vTaskDelay(pdMS_TO_TICKS(Delay_Wrist));

    //Y3：夹着物块放到转盘上的位置
    APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[3], 1, 0);
    APP_MOTOR_ZDT_WaitAck();

    //夹爪舵机夹紧物块，内有延迟
    APP_SERVO_Jaw('Z');

    //Y2：夹着物块能越过转盘的位置
    APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[2], 1, 0);
    APP_MOTOR_ZDT_WaitAck();

    //腕处舵机朝前，延迟等待
    APP_SERVO_Wrist('F');
    vTaskDelay(pdMS_TO_TICKS(Delay_Wrist));

    //Y4：夹着物块放到色环上的位置
    APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[4], 1, 0);
    APP_MOTOR_ZDT_WaitAck();

    //夹爪舵机松开物块，内有延迟
    APP_SERVO_Jaw('S');

    //Y1：最高点-300
    APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0);
    APP_MOTOR_ZDT_WaitAck();

    xTaskNotifyGive(Task_MainHandle);
}

/*
第i次抓取，i＝1，2，3
{
    腕处舵机朝前
    底部转盘seq[i]
    x轴seq[i]
    if i == 0
        延迟，等待步进电机到位
    else
        延迟，舵机朝前完成
    y4
    夹爪舵机抓紧，延迟等待
    y2
    腕处舵机右转
    y3
    夹爪舵机松开，延迟等待
    if i < 2
        y2
    else
        y1
    if i == 2
        腕处舵机朝前
}
*/
void APP_Handle_Pick_All(uint8_t _firstIndex, uint8_t _secondIndex, uint8_t _thirdIndex)
{
    uint32_t Delay_up = 400;
    uint32_t Delay_Wrist = 400;
    uint32_t Delay_xr = 500;
    uint8_t ucSeq[3] = {DirMapping[Fir][_firstIndex], DirMapping[Fir][_secondIndex], DirMapping[Fir][_thirdIndex]};
    App_Printf("ColorSeq %d %d %d \r\n", _firstIndex, _firstIndex, _firstIndex);
    App_Printf("DirSeq %d %d %d \r\n", ucSeq[0], ucSeq[1], ucSeq[2]);

    for (uint8_t i = 0; i < 3; i++)
    {
        //腕处舵机朝前
        APP_SERVO_Wrist('F');

        //控制底盘转盘转动到指定色环方向
        APP_MOTOR_ZDT_Move_P(MOTOR_R, MOTOR_R_Direction_CCR,     MOTOR_R_Vel, MOTOR_R_Acc, MOTOR_R_Pos[ucSeq[i]], 1, 0);

        //X导轨伸出到指定色环
        APP_MOTOR_ZDT_Move_P(MOTOR_X, MOTOR_X_Direction_FORWARD, MOTOR_X_Vel, MOTOR_X_Acc, MOTOR_X_Pos[ucSeq[i]], 1, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));  //等待X R到位


        if (i == 0)
        {
            //延迟，等待步进电机到位
            vTaskDelay(pdMS_TO_TICKS(Delay_xr));
        }
        else
        {
            //延迟，等待舵机朝前完成
            vTaskDelay(pdMS_TO_TICKS(Delay_Wrist));
        }

        //Y4：夹着物块放到色环上的位置
        APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[4], 1, 0);
        APP_MOTOR_ZDT_WaitAck();

        //夹爪舵机夹紧物块，内有延迟
        APP_SERVO_Jaw('Z');

        //Y2：夹着物块能越过转盘的位置
        APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[2], 1, 0);
        APP_MOTOR_ZDT_WaitAck();

        //腕处舵机朝右
        APP_SERVO_Wrist('R');
        vTaskDelay(pdMS_TO_TICKS(Delay_Wrist));

        //Y3：夹着物块放到转盘上的位置
        APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[3], 1, 0);
        APP_MOTOR_ZDT_WaitAck();

        //夹爪舵机松开物块，内有延迟
        APP_SERVO_Jaw('S');

        if (i < 2)
        {
            //Y2：夹着物块能越过转盘的位置
            APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[2], 1, 0);
            APP_MOTOR_ZDT_WaitAck();
        }
        else
        {
            //Y1：最高点-300
            APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0);
            APP_MOTOR_ZDT_WaitAck();
        }

        if (i == 2)
        {
            //腕处舵机朝前
            APP_SERVO_Wrist('F');

            //控制底盘转盘转动到中间色环方向
            APP_MOTOR_ZDT_Move_P(MOTOR_R, MOTOR_R_Direction_CCR,     MOTOR_R_Vel, MOTOR_R_Acc, MOTOR_R_Pos[MIDDLE], 1, 0);

            //X导轨伸出到中间色环
            APP_MOTOR_ZDT_Move_P(MOTOR_X, MOTOR_X_Direction_FORWARD, MOTOR_X_Vel, MOTOR_X_Acc, MOTOR_X_Pos[MIDDLE], 1, 0);
        }
    }

    xTaskNotifyGive(Task_MainHandle);
}

void APP_Handle_Run(void)
{
    APP_MOTOR_ZDT_Move_P(MOTOR_Y, 0,     MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[0], 1, 0);
    APP_MOTOR_ZDT_Move_P(MOTOR_R, MOTOR_R_Direction_CCR,     MOTOR_R_Vel, MOTOR_R_Acc, MOTOR_R_Pos[MIDDLE], 1, 0);
    APP_MOTOR_ZDT_Move_P(MOTOR_X, MOTOR_X_Direction_FORWARD, MOTOR_X_Vel, MOTOR_X_Acc, 0, 1, 0);
    //    APP_SERVO_Wrist('R');
    APP_MOTOR_ZDT_WaitAck();
}

void APP_Handle_Stay(void)
{
    APP_MOTOR_ZDT_Move_P(MOTOR_R, MOTOR_R_Direction_CCR,     MOTOR_R_Vel, MOTOR_R_Acc, MOTOR_R_Pos[MIDDLE], 1, 0);
    APP_MOTOR_ZDT_Move_P(MOTOR_X, MOTOR_X_Direction_FORWARD, MOTOR_X_Vel, MOTOR_X_Acc, 0, 1, 0);
    APP_SERVO_Wrist('F');
    vTaskDelay(1000);
}
