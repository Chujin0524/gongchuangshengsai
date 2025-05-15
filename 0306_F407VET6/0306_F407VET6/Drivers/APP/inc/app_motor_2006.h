#ifndef __APP_MOTOR_2006_H
#define __APP_MOTOR_2006_H

/* Define  ------------------------------------------------------------------*/
#define MOTOR_LF 1
#define MOTOR_LB 2
#define MOTOR_RF 3
#define MOTOR_RB 4
#define MOTOR_X  5
#define MOTOR_Y  6
#define MOTOR_R  7

/* С�����н����� */
#define CarDirection_Forward   (1 << 0)       //ǰ      1
#define CarDirection_Backward  (1 << 1)       //��      2
#define CarDirection_Left      (1 << 2)       //��      4
#define CarDirection_Right     (1 << 3)       //��      8
#define CarDirection_CR        (1 << 4)       //˳      16
#define CarDirection_CCR       (1 << 5)       //��      32

#define CarDirection_Y         (CarDirection_Forward | CarDirection_Backward)       //Y�����ƶ�
#define CarDirection_X         (CarDirection_Left    | CarDirection_Right)          //X�����ƶ�
#define CarDirection_R         (CarDirection_CR      | CarDirection_CCR)            //��ת

#define CARDIRECTION_Y(Y)     (Y>0) ? CarDirection_Forward : CarDirection_Backward
#define CARDIRECTION_X(X)     (X>0) ? CarDirection_Left    : CarDirection_Right
#define CARDIRECTION_R(R)     (R>0) ? CarDirection_CR      : CarDirection_CCR

/* Include  ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

typedef struct 
{
    uint8_t WhaleDirX;   // X����
    uint32_t WhalePosX;  // X����
    uint8_t WhaleDirY;   // Y����
    uint32_t WhalePosY;  // Y����
    uint8_t WhaleDirR;   // R����  R:Rotate��ת
    uint32_t WhalePosR;  // R����
}CAR_XYR_T;

typedef struct
{
	uint8_t ucCarDir;		//С����ʻ����
	uint16_t usCarVel;		//С����ʻ�ٶ�
	uint32_t ulCarPos;		//С����ʻ����
	uint8_t ucCarAcc;		//С����ʻ���ٶ�
	
	uint8_t ucWhale;		//��ѡ�������	1,2,3
	uint8_t ucWhaleDir;		//������ת����
	uint8_t ucaWhaleAcc[4];	//���ּ��ٶ�
	uint8_t ucaWhaleDir[4];	//������ת����
	uint16_t usaWhaleVel[4];//�����ٶ�
	uint32_t ulaWhalePos[4];//������ʻ����
}WHALE_PARAMETER_T;


void APP_MOTOR_2006_Init( void );
void APP_Motor_2006_WhaleControl(void);
void APP_MOTOR_2006_WaitResponse(uint8_t _addr);
void APP_MOTOR_2006_Move_P(uint8_t _addr, uint8_t _dir, uint16_t _vel, uint8_t _acc, uint32_t _clk, bool _raF, bool _snF);
void APP_MOTOR_2006_Move_V(uint8_t _addr, uint8_t _dir, uint16_t _vel, uint8_t _acc, bool _snF);
void APP_MOTOR_2006_WHALE_Move_P(uint8_t _dir, uint32_t _clk);
void APP_MOTOR_2006_WHALE_MovePID( uint8_t _dir, int32_t _clk );
void APP_MOTOR_2006_WHALE_Status(uint8_t _ucStatus);



#endif