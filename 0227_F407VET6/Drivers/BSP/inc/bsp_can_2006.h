#ifndef __BSP_CAN_2006_H_
#define __BSP_CAN_2006_H_

/* Include  ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

/*CAN发送或是接收的ID*/
typedef enum
{

	CAN_2006Moto_ALL_ID = 0x200,
	CAN_2006Moto1_ID = 0x201,
	CAN_2006Moto2_ID = 0x202,
	CAN_2006Moto3_ID = 0x203,
	CAN_2006Moto4_ID = 0x204,
	CAN_2006Moto5_ID = 0x205
}CAN_Message_ID;

#define FILTER_BUF_LEN		5
/*接收到的云台电机的参数结构体*/
typedef struct{
	int16_t	 	speed_rpm;      //转速
	float  	real_current;			//真实电流值
	int16_t  	given_current;  //规定电流值
	uint8_t  	hall;						//
	uint16_t 	angle;				//abs angle range:[0,8191]
	uint16_t 	last_angle;	//abs angle range:[0,8191]
	uint16_t	offset_angle;     //偏角
	int32_t		round_cnt;        //转过的圈数
	int32_t		total_angle;			//总角度
	uint8_t			buf_idx;
	uint8_t			angle_buf[FILTER_BUF_LEN];
	uint8_t			fited_angle;     //适合的角度
	uint8_t			msg_cnt;
}moto_measure_t;

/* Extern  ------------------------------------------------------------------*/
extern moto_measure_t  moto_chassis[];  //实际底盘测量速度
extern CAN_HandleTypeDef hcan1;
extern CAN_RxHeaderTypeDef 	Rx1Message;
extern int Weak_mode;

/* Void  ------------------------------------------------------------------*/
void my_can_filter_init(CAN_HandleTypeDef* hcan);
void my_can_filter_init_recv_all(CAN_HandleTypeDef* _hcan);
void can_filter_recv_special(CAN_HandleTypeDef* hcan, uint8_t filter_number, uint16_t filtered_id);
void get_moto_measure(moto_measure_t *ptr,uint8_t* Data);
void can_receive_onetime(CAN_HandleTypeDef* _hcan, int time);
void set_moto_current(CAN_HandleTypeDef* hcan, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
void CAN1_ReadMsg(void);
float my_abs(float data);
void hwt_cal(int32_t speed0,int32_t speed1,int32_t speed2,int32_t speed3,int32_t w_t);
void stop();
void _DingjiaoPIDTask();
void Move_Transfrom(double vx,double vy,double vw,double yaw);
void _M2006Task();
int delay_distance(int dis);
int delay_dis_diagnal(int dis);
void init_calculate_values();
void wheel_init();
void SpeedUP_track(int32_t speed_max,uint32_t speed_final,uint32_t distance,uint16_t ramp_dis1,uint16_t ramp_dis2,int yaw);
void Ramp_dis(int spd1, int spd2, int dis,int yaw);
void SpeedUP_x(int32_t speed_max,uint32_t speed_final,uint32_t distance,uint16_t ramp_dis1,uint16_t ramp_dis2,int yaw);
void Ramp_disx(int spd1, int spd2, int dis,int yaw);
#endif /* INC_USER_CAN_H_ */
