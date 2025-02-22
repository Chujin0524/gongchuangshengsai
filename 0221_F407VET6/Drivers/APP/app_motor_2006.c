#include "main.h"
#include "bsp_can_2006.h"
#include "bsp_hwt101.h"

void APP_MOTOR_2006_Init( void )
{
//    Emm_V5_Stop_Now(1, 0);
//    Emm_V5_Stop_Now(2, 0);
//    Emm_V5_Stop_Now(3, 0);
//    Emm_V5_Stop_Now(4, 0);
//    Emm_V5_Stop_Now(5, 0);
//    Emm_V5_Stop_Now(6, 0);
//    Emm_V5_Stop_Now(7, 0);
}

void APP_Motor_2006_WhaleControl(void)
{
    static CAR_XYR_T *ptCarXYR = NULL;

    for(;; )
    {
        xQueueReceive(Queue_MotorHandle, &ptCarXYR, portMAX_DELAY); //等待接收轮子参数
        
        App_Printf("-----APP_MotorWhaleControl-----\r\n");
        App_Printf("ADDR: %04X\r\n", ptCarXYR);        
        App_Printf("DirX: %d\r\n", ptCarXYR->WhaleDirX);
        App_Printf("PosX: %d\r\n", ptCarXYR->WhalePosX);
        App_Printf("DirY: %d\r\n", ptCarXYR->WhaleDirY);
        App_Printf("PosY: %d\r\n", ptCarXYR->WhalePosY);
        App_Printf("DirR: %d\r\n", ptCarXYR->WhaleDirR);
        App_Printf("PosR: %d\r\n", ptCarXYR->WhalePosR);
        App_Printf("-----APP_MotorWhaleControl-----\r\n"); 

        if (ptCarXYR->WhalePosX > 0)
        {
            APP_MOTOR_2006_WHALE_MovePID(ptCarXYR->WhaleDirX, ptCarXYR->WhalePosX);
            
            ptCarXYR->WhaleDirX = 0;
            ptCarXYR->WhalePosX = 0;
        }
        if (ptCarXYR->WhalePosY > 0)
        {
            APP_MOTOR_2006_WHALE_MovePID(ptCarXYR->WhaleDirY, ptCarXYR->WhalePosY);
            
            ptCarXYR->WhaleDirY = 0;
            ptCarXYR->WhalePosY = 0;
        }
        if (ptCarXYR->WhalePosR > 0)
        {
            APP_MOTOR_2006_WHALE_MovePID(ptCarXYR->WhaleDirR, ptCarXYR->WhalePosR);
            
            ptCarXYR->WhaleDirR = 0;
            ptCarXYR->WhalePosR = 0;
        }
    }
}

/*
*********************************************************************************************************
*    函 数 名: APP_MOTOR_Move_V
*    功能说明: 电机运动，速度模式，加延时
*    形    参: addr：电机地址
              dir ：方向        ，0为CW，其余值为CCW
              vel ：速度(RPM)   ，范围0 - 5000RPM
              acc ：加速度      ，范围0 - 255，注意：0是直接启动
              snF ：多机同步标志 ，false为不启用，true为启用
*    返 回 值: 无
*********************************************************************************************************
*/
void APP_MOTOR_Move_V(uint8_t _addr, uint8_t _dir, uint16_t _vel, uint8_t _acc, bool _snF)
{
//    Emm_V5_Vel_Control(_addr, _dir, _vel, _acc, _snF); vTaskDelay(10);    
}

/*
*********************************************************************************************************
*    函 数 名: APP_MOTOR_2006_WHALE_Move_P
*    功能说明: 控制小车平移或旋转给定步数
*    形    参: _dir  : 宏定义见bsp.h
*              _clk  : 步数
*    返 回 值: 无
*********************************************************************************************************
*/
void APP_MOTOR_2006_WHALE_Move_P(uint8_t _dir, uint32_t _clk)
{
    static WHALE_PARAMETER_T s_tMotorWhale = {0};
    switch (_dir)
    {
        case CarDirection_Forward:        //前
            //各轮转向
            s_tMotorWhale.ucaWhaleDir[0] = 1;
            s_tMotorWhale.ucaWhaleDir[1] = 1;
            s_tMotorWhale.ucaWhaleDir[2] = 0;
            s_tMotorWhale.ucaWhaleDir[3] = 0;
                
            break;
        
        case CarDirection_Backward:        //后
            //各轮转向
            s_tMotorWhale.ucaWhaleDir[0] = 0;
            s_tMotorWhale.ucaWhaleDir[1] = 0;
            s_tMotorWhale.ucaWhaleDir[2] = 1;
            s_tMotorWhale.ucaWhaleDir[3] = 1;
            
            break;
        
        case CarDirection_Left:            //左
            //各轮转向
            s_tMotorWhale.ucaWhaleDir[0] = 0;
            s_tMotorWhale.ucaWhaleDir[1] = 1;
            s_tMotorWhale.ucaWhaleDir[2] = 1;
            s_tMotorWhale.ucaWhaleDir[3] = 0;
            
            break;
        
        case CarDirection_Right:        //右
            //各轮转向
            s_tMotorWhale.ucaWhaleDir[0] = 1;
            s_tMotorWhale.ucaWhaleDir[1] = 0;
            s_tMotorWhale.ucaWhaleDir[2] = 0;
            s_tMotorWhale.ucaWhaleDir[3] = 1;
            
            break;
        
        case CarDirection_CR:            //顺
            //各轮转向
            s_tMotorWhale.ucaWhaleDir[0] = 0;
            s_tMotorWhale.ucaWhaleDir[1] = 0;
            s_tMotorWhale.ucaWhaleDir[2] = 0;
            s_tMotorWhale.ucaWhaleDir[3] = 0;
            
            break;
        
        case CarDirection_CCR:            //逆
            //各轮转向
            s_tMotorWhale.ucaWhaleDir[0] = 1;
            s_tMotorWhale.ucaWhaleDir[1] = 1;
            s_tMotorWhale.ucaWhaleDir[2] = 1;
            s_tMotorWhale.ucaWhaleDir[3] = 1;
            
            break;
        
        default:
            break;            
    }
    //各轮转速
    s_tMotorWhale.usaWhaleVel[0] = s_tMotorWhale.usCarVel;
    s_tMotorWhale.usaWhaleVel[1] = s_tMotorWhale.usCarVel;
    s_tMotorWhale.usaWhaleVel[2] = s_tMotorWhale.usCarVel;
    s_tMotorWhale.usaWhaleVel[3] = s_tMotorWhale.usCarVel;
    
    s_tMotorWhale.ulCarPos = _clk;      //脉冲数
    
//    APP_MOTOR_2006_Move_P(0x01,s_tMotorWhale.ucaWhaleDir[0], s_tMotorWhale.usaWhaleVel[0], s_tMotorWhale.ucCarAcc, s_tMotorWhale.ulCarPos, 0, 1); 
//    APP_MOTOR_2006_Move_P(0x02,s_tMotorWhale.ucaWhaleDir[1], s_tMotorWhale.usaWhaleVel[1], s_tMotorWhale.ucCarAcc, s_tMotorWhale.ulCarPos, 0, 1); 
//    APP_MOTOR_2006_Move_P(0x03,s_tMotorWhale.ucaWhaleDir[2], s_tMotorWhale.usaWhaleVel[2], s_tMotorWhale.ucCarAcc, s_tMotorWhale.ulCarPos, 0, 1); 
//    APP_MOTOR_2006_Move_P(0x04,s_tMotorWhale.ucaWhaleDir[3], s_tMotorWhale.usaWhaleVel[3], s_tMotorWhale.ucCarAcc, s_tMotorWhale.ulCarPos, 0, 1); 
//    Emm_V5_Synchronous_motion(0x00); vTaskDelay(10);
}

/*
*********************************************************************************************************
*    函 数 名: APP_MOTOR_2006_WHALE_MovePID
*    功能说明: 角度环控制小车平移给定步数
*    形    参: _dir  : 宏定义见bsp.h
*              _clk  : 步数
*    返 回 值: 无
*********************************************************************************************************
*/
void APP_MOTOR_2006_WHALE_MovePID(uint8_t _dir, uint32_t _clk)
{
    static int flag = 0;
    if (flag == 0)
    {
        switch (_dir)
        {
            case CarDirection_Forward:
                Move_Transfrom(0, 500, 0, 0);
                if (delay_distance(_clk) == 1)
                {
                    flag = 1;
                    stop();
                    Weak_mode = 1;
                    return;  // 退出函数
                }
                break;
            case CarDirection_Backward:
                Move_Transfrom(0, -500, 0, 0);
                if (delay_distance(-_clk) == 1)
                {
                    flag = 1;
                    stop();
                    Weak_mode = 1;
                    return;  // 退出函数
                }        
                break;
            case CarDirection_Left:
                Move_Transfrom(-500, 0, 0, 0);
                if (delay_dis_diagnal(_clk) == 1)
                {
                    flag = 1;
                    stop();
                    Weak_mode = 1;
                    return;  // 退出函数
                }
                break;
            case CarDirection_Right:
                Move_Transfrom(500, 0, 0, 0);
                if (delay_dis_diagnal(-_clk) == 1)
                {
                    flag = 1;
                    stop();
                    Weak_mode = 1;
                    return;  // 退出函数
                }        
                break;
            case CarDirection_CCR:
                Move_Transfrom(0, 0, 300, 0);
                if (gyro_z == _clk)
                {
                    flag = 1;
                    stop();
                    Weak_mode = 1;
                    return;  // 退出函数
                }        
                break;
            case CarDirection_CR:
                Move_Transfrom(0, 0, -300, 0);
                if (gyro_z == _clk)
                {
                    flag = 1;
                    stop();
                    Weak_mode = 1;
                    return;  // 退出函数
                }
                break;
        }
    }
}

/*
*********************************************************************************************************
*    函 数 名: APP_MOTOR_2006_WHALE_Status
*    功能说明: 角度环控制小车平移给定步数
*    形    参: _dir  : 宏定义见bsp.h
*              _clk  : 步数
*    返 回 值: 无
*********************************************************************************************************
*/
void APP_MOTOR_2006_WHALE_Status(STATUS_E _ucStatus)
{
    switch (_ucStatus)
    {
    case STATUS_Start_2_Qr:
        APP_MOTOR_2006_WHALE_MovePID(CarDirection_Backward, 2200);

        APP_MOTOR_2006_WHALE_MovePID(CarDirection_Left, 5800);
        break;
    
    case STATUS_Qr_2_RawArea:
        APP_MOTOR_2006_WHALE_MovePID(CarDirection_Left, 8800);
        break;
    
    case STATUS_RawArea_2_ProFirArea:
        APP_MOTOR_2006_WHALE_MovePID(CarDirection_Right, 4500);

        APP_MOTOR_2006_WHALE_MovePID(CarDirection_CCR, 180);//逆旋转180度
    
        APP_MOTOR_2006_WHALE_MovePID(CarDirection_Forward, 16500);
        break;
    
    case STATUS_ProFirArea_2_ProSecArea:
        APP_MOTOR_2006_WHALE_MovePID(CarDirection_Right, 8100);

        APP_MOTOR_2006_WHALE_MovePID(CarDirection_Backward, 8000);

        APP_MOTOR_2006_WHALE_MovePID(CarDirection_CR, 90);//顺旋转90度
        break;
    
    case STATUS_ProSecArea_2_RawArea:
        APP_MOTOR_2006_WHALE_MovePID(CarDirection_Right, 8900);

        APP_MOTOR_2006_WHALE_MovePID(CarDirection_Backward, 3600);

        APP_MOTOR_2006_WHALE_MovePID(CarDirection_CR, 90);//顺旋转90度
        break;
    
    case STATUS_ProSecArea_2_Start:
        APP_MOTOR_2006_WHALE_MovePID(CarDirection_Backward, 17000);

        APP_MOTOR_2006_WHALE_MovePID(CarDirection_Right, 9800);

        APP_MOTOR_2006_WHALE_MovePID(CarDirection_Backward, 1500);
        break;
    
    default:
        break;
    }
}

