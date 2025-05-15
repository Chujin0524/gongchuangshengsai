#include "main.h"

void APP_MOTOR_ZDT_Init( void )
{
    Emm_V5_Stop_Now(5, 0);
    Emm_V5_Stop_Now(6, 0);
    Emm_V5_Stop_Now(7, 0);
}

/*
*********************************************************************************************************
*    函 数 名: APP_MOTOR_ZDT_Move_P
*    功能说明: 电机运动，位置模式，加延时
*    形    参: addr：电机地址
              dir ：方向        ，0为CW，其余值为CCW
              vel ：速度(RPM)   ，范围0 - 5000RPM
              acc ：加速度      ，范围0 - 255，注意：0是直接启动
              clk ：脉冲数      ，范围0- (2^32 - 1)个
              raF ：相位/绝对标志，false为相对运动，true为绝对值运动
              snF ：多机同步标志 ，false为不启用，true为启用
*    返 回 值: 无
*********************************************************************************************************
*/
void APP_MOTOR_ZDT_Move_P(uint8_t _addr, uint8_t _dir, uint16_t _vel, uint8_t _acc, uint32_t _clk, bool _raF, bool _snF)
{
    can_SetAckId(_addr);
    Emm_V5_Pos_Control(_addr, _dir, _vel, _acc, _clk, _raF, _snF); vTaskDelay(10);    
}

/*
*********************************************************************************************************
*    函 数 名: APP_MOTOR_ZDT_WaitAck
*    功能说明: 等待电机应答
*    形    参: 无
*    返 回 值: 无
*********************************************************************************************************
*/
void APP_MOTOR_ZDT_WaitAck(void)
{
    can_WaitAck();
}
