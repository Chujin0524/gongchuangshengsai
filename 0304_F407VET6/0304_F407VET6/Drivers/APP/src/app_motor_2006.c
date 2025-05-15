#include "main.h"

void APP_MOTOR_2006_Init( void )
{
    //停车
    Weak_mode = 1;
    set_moto_current(&hcan1, 0, 0, 0, 0 );
}

void APP_Motor_2006_WhaleControl( void )
{
    static CAR_XYR_T *ptCarXYR = NULL;

    for(;; )
    {
        xQueueReceive( Queue_MotorHandle, &ptCarXYR, portMAX_DELAY ); //等待接收轮子参数

        App_Printf( "-----APP_MotorWhaleControl-----\r\n" );
        App_Printf( "ADDR: %04X\r\n", ptCarXYR );
        App_Printf( "DirX: %d\r\n", ptCarXYR->WhaleDirX );
        App_Printf( "PosX: %d\r\n", ptCarXYR->WhalePosX );
        App_Printf( "DirY: %d\r\n", ptCarXYR->WhaleDirY );
        App_Printf( "PosY: %d\r\n", ptCarXYR->WhalePosY );
        App_Printf( "DirR: %d\r\n", ptCarXYR->WhaleDirR );
        App_Printf( "PosR: %d\r\n", ptCarXYR->WhalePosR );
        App_Printf( "-----APP_MotorWhaleControl-----\r\n" );

        if( ptCarXYR->WhalePosX > 0 )
        {
            APP_MOTOR_2006_WHALE_MovePID( ptCarXYR->WhaleDirX, ptCarXYR->WhalePosX );

            ptCarXYR->WhaleDirX = 0;
            ptCarXYR->WhalePosX = 0;
        }
        if( ptCarXYR->WhalePosY > 0 )
        {
            APP_MOTOR_2006_WHALE_MovePID( ptCarXYR->WhaleDirY, ptCarXYR->WhalePosY );

            ptCarXYR->WhaleDirY = 0;
            ptCarXYR->WhalePosY = 0;
        }
        if( ptCarXYR->WhalePosR >= 0 )
        {
            APP_MOTOR_2006_WHALE_MovePID( ptCarXYR->WhaleDirR, ptCarXYR->WhalePosR );//之后可能会有问题

            ptCarXYR->WhaleDirR = 0;
            ptCarXYR->WhalePosR = 0;
        }
    }
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
void APP_MOTOR_2006_WHALE_MovePID( uint8_t _dir, int32_t _clk )
{
    int flag = 0;
    Weak_mode = 0;
    if( flag == 0 )
    {
        switch( _dir )
        {
        case CarDirection_Forward:
            SpeedUP_x(800, 0, _clk, 0.4*_clk, 0.4*_clk, 0);
            Weak_mode = 1;
            break;
        case CarDirection_Backward:
           SpeedUP_x(-800, 0, _clk, 0.4*_clk, 0.4*_clk, 0);
           Weak_mode = 1;
            break;
				case CarDirection_Left:
						SpeedUP_track(800, 0, _clk, 0.4*_clk, 0.4*_clk, 0);
						Weak_mode = 1;
            break;
        case CarDirection_Right:
            SpeedUP_track(-800, 0, _clk, 0.4*_clk, 0.4*_clk, 0);
						Weak_mode = 1;
            break;
        case CarDirection_CCR:
						Move_Transfrom( 0, 0, 0,_clk);
            break;
        case CarDirection_CR:
            Move_Transfrom( 0, 0, 0,_clk);
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
void APP_MOTOR_2006_WHALE_Status( STATUS_E _ucStatus )
{
    switch( _ucStatus )
    {
    case STATUS_Start_2_Qr:
        APP_MOTOR_2006_WHALE_MovePID( CarDirection_Backward, 2200 );

        APP_MOTOR_2006_WHALE_MovePID( CarDirection_Left, 5800 );
        break;

    case STATUS_Qr_2_RawArea:
        APP_MOTOR_2006_WHALE_MovePID( CarDirection_Left, 8800 );
        break;

    case STATUS_RawArea_2_ProFirArea:
        APP_MOTOR_2006_WHALE_MovePID( CarDirection_Right, 4500 );

        APP_MOTOR_2006_WHALE_MovePID( CarDirection_CCR, 180 ); //逆旋转180度

        APP_MOTOR_2006_WHALE_MovePID( CarDirection_Forward, 16500 );
        break;

    case STATUS_ProFirArea_2_ProSecArea:
        APP_MOTOR_2006_WHALE_MovePID( CarDirection_Right, 8100 );

        APP_MOTOR_2006_WHALE_MovePID( CarDirection_Backward, 8000 );

        APP_MOTOR_2006_WHALE_MovePID( CarDirection_CR, 90 ); //顺旋转90度
        break;

    case STATUS_ProSecArea_2_RawArea:
        APP_MOTOR_2006_WHALE_MovePID( CarDirection_Right, 8900 );

        APP_MOTOR_2006_WHALE_MovePID( CarDirection_Backward, 3600 );

        APP_MOTOR_2006_WHALE_MovePID( CarDirection_CR, 90 ); //顺旋转90度
        break;

    case STATUS_ProSecArea_2_Start:
        APP_MOTOR_2006_WHALE_MovePID( CarDirection_Backward, 17000 );

        APP_MOTOR_2006_WHALE_MovePID( CarDirection_Right, 9800 );

        APP_MOTOR_2006_WHALE_MovePID( CarDirection_Backward, 1500 );
        break;

    default:
        break;
    }
}

