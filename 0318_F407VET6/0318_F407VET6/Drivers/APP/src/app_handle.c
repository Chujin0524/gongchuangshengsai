#include "main.h"

/*
转盘位置 : 
0 : 初始
1 : 指向左边
2 : 指向中间
3 : 指向右边
4 : 伸/缩
*/
static uint8_t MOTOR_R_Acc = 0; 
static uint16_t MOTOR_R_Vel = 200; 
static uint32_t MOTOR_R_Pos[4] = {0, 4100, 2100, 90};  

/*
X位置 : 
0 : 初始
1 : 指向左边
2 : 指向中间
3 : 指向右边
4 : 
*/
static uint8_t MOTOR_X_Acc = 0; 
static uint16_t MOTOR_X_Vel = 200; 
static uint32_t MOTOR_X_Pos[4] = {2900, 6700, 4200, 7050};    

/*
Y位置 : 
0 : 最高点
1 : 最高点-300
2 : 夹着物块能越过转盘的位置
3 : 夹着物块放到转盘上的位置
4 : 夹着物块放到色环上的位置
5 : 码垛物块的位置
6 : 物块在色环上，夹爪能越过的位置
7 : 
*/
static uint8_t MOTOR_Y_Acc = 0; 
static uint16_t MOTOR_Y_Vel = 800; 
static uint32_t MOTOR_Y_Pos[8] = {0, 300, 0, 1600, 9000, 6000, 8000, 0}; 

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

void APP_Handle_PickRawArea(uint8_t _ucIndex)
{
     //35步进电机控制夹爪距离顶部0.5圈
    APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); APP_MOTOR_ZDT_WaitAck();

    APP_SERVO_Wrist('F');        //腕处舵机朝前
    
    APP_SERVO_Plate(_ucIndex);       //转盘舵机转到对应物块的放置角度
 
    //35步进电机控制夹爪距离顶部4圈
    APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[3], 1, 0); APP_MOTOR_ZDT_WaitAck();
        
    APP_SERVO_Jaw('Z');     //夹爪舵机夹紧物块
   
    //35步进电机控制夹爪距离顶部0.5圈
    APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); APP_MOTOR_ZDT_WaitAck();
   
    APP_SERVO_Wrist('R');          //腕处舵机朝右
    
    //35步进电机控制夹爪距离顶部1圈
    APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[2], 1, 0); APP_MOTOR_ZDT_WaitAck();
    
    APP_SERVO_Jaw('S');     //夹爪舵机松开物块
    
    //35步进电机控制夹爪距离顶部0.5圈
    APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); APP_MOTOR_ZDT_WaitAck();

    APP_SERVO_Wrist('F');        //腕处舵机朝前
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
void APP_Handle_Place_All(uint8_t _time, uint8_t _firstIndex, uint8_t _secondIndex, uint8_t _thirdIndex)
{
    uint32_t Delay_up = 400;
    uint32_t Delay_Wrist = 400;
    uint8_t ucSeq[3] = {DirMapping[_time][_firstIndex], DirMapping[_time][_secondIndex], DirMapping[_time][_thirdIndex]};
    App_Printf("ColorSeq %d %d %d \r\n", _firstIndex, _firstIndex, _firstIndex);
    App_Printf("DirSeq %d %d %d \r\n", ucSeq[0], ucSeq[1], ucSeq[2]);
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
        APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[3], 1, 0); APP_MOTOR_ZDT_WaitAck();
        
        //夹爪舵机夹紧物块，内有延迟
        APP_SERVO_Jaw('Z');            

        //Y2：夹着物块能越过转盘的位置
        APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[2], 1, 0); APP_MOTOR_ZDT_WaitAck();

        //腕处舵机朝前，延迟等待
        APP_SERVO_Wrist('F');           
        vTaskDelay(pdMS_TO_TICKS(Delay_Wrist));
        
        if (i < 2)  APP_SERVO_Plate(ucSeq[i+1]);      //转盘舵机转到下一个物块的放置角度
        else        APP_SERVO_Plate(ucSeq[0]);        //转盘舵机转到第一个物块的放置角度
        
        if (_time == 1)
        {
            //Y4：夹着物块放到色环上的位置
            APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[4], 1, 0); APP_MOTOR_ZDT_WaitAck();
        }
        else if (_time == 2)
        {
            //Y5：码垛物块的位置
            APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[5], 1, 0); APP_MOTOR_ZDT_WaitAck();
        }
    
        //夹爪舵机松开物块，内有延迟
        APP_SERVO_Jaw('S');     

        if (i == 2)
        {
            if (_time == 1)
            {
                //Y5：物块在色环上，夹爪能越过的位置
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[5], 1, 0); APP_MOTOR_ZDT_WaitAck();
            }
            else if (_time == 2)
            {
                //Y1：最高点-300
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); APP_MOTOR_ZDT_WaitAck();
            }
        }
        else
        {
            //Y2：夹着物块能越过转盘的位置
            APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[2], 1, 0); APP_MOTOR_ZDT_WaitAck();            
        }
    }
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
        APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[4], 1, 0); APP_MOTOR_ZDT_WaitAck();

        //夹爪舵机夹紧物块，内有延迟
        APP_SERVO_Jaw('Z');

        //Y2：夹着物块能越过转盘的位置
        APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[2], 1, 0); APP_MOTOR_ZDT_WaitAck();            

        //腕处舵机朝右
        APP_SERVO_Wrist('R');           
        vTaskDelay(pdMS_TO_TICKS(Delay_Wrist));

        //Y3：夹着物块放到转盘上的位置
        APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[3], 1, 0); APP_MOTOR_ZDT_WaitAck();

        //夹爪舵机松开物块，内有延迟
        APP_SERVO_Jaw('S');     
        
        if (i < 2)
        {
            //Y2：夹着物块能越过转盘的位置
            APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[2], 1, 0); APP_MOTOR_ZDT_WaitAck();            
        }
        else
        {
            //Y1：最高点-300
            APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); APP_MOTOR_ZDT_WaitAck();
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
}

void APP_Handle_Suo(void)
{
    APP_MOTOR_ZDT_Move_P(MOTOR_R, MOTOR_R_Direction_CCR,     MOTOR_R_Vel, MOTOR_R_Acc, MOTOR_R_Pos[MIDDLE], 1, 0); 
    APP_MOTOR_ZDT_Move_P(MOTOR_X, MOTOR_X_Direction_FORWARD, MOTOR_X_Vel, MOTOR_X_Acc, MOTOR_X_Pos[MIDDLE], 1, 0);
}
