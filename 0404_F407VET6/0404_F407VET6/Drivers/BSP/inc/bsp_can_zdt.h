/*
 * user_can.h
 *
 *  Created on: 2023年8月6日
 *      Author: 71492
 */

#ifndef __BSP_CAN_ZDT_H_
#define __BSP_CAN_ZDT_H_

#define		ABS(x)		((x) > 0 ? (x) : -(x)) 

typedef enum {
	S_VER   = 0,			/* 读取固件版本和对应的硬件版本 */
	S_RL    = 1,			/* 读取读取相电阻和相电感 */
	S_PID   = 2,			/* 读取PID参数 */
	S_VBUS  = 3,			/* 读取总线电压 */
	S_CPHA  = 5,			/* 读取相电流 */
	S_ENCL  = 7,			/* 读取经过线性化校准后的编码器值 */
	S_TPOS  = 8,			/* 读取电机目标位置角度 */
	S_VEL   = 9,			/* 读取电机实时转速 */
	S_CPOS  = 10,			/* 读取电机实时位置角度 */
	S_PERR  = 11,			/* 读取电机位置误差角度 */
	S_FLAG  = 13,			/* 读取使能/到位/堵转状态标志位 */
	S_Conf  = 14,			/* 读取驱动参数 */
	S_State = 15,			/* 读取系统状态参数 */
	S_ORG   = 16,     /* 读取正在回零/回零失败状态标志位 */
}SysParams_t;


/**********************************************************
*** 注意：每个函数的参数的具体说明，请查阅对应函数的注释说明
**********************************************************/
void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF); // 速度模式控制
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF); // 位置模式控制
void Emm_V5_Stop_Now(uint8_t addr, bool snF); // 让电机立即停止运动
void Emm_V5_Synchronous_motion(uint8_t addr); // 触发多机同步开始运动
void Emm_V5_Origin_Set_O(uint8_t addr, bool svF); // 设置挡圈回零的零点位置
void Emm_V5_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF); // 修改回零参数
void Emm_V5_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF); // 发送命令触发回零

void can_SendCmd(__IO uint8_t *cmd, uint8_t len);
void can_SetAckId(uint8_t _addr);
void can_WaitAck(uint8_t _addr);
void CAN2_ReadMsg(void);

#endif /* INC_USER_CAN_H_ */
