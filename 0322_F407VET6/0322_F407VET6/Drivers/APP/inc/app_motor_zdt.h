#ifndef __APP_MOTOR_ZDT_H
#define __APP_MOTOR_ZDT_H

#define MOTOR_R  5
#define MOTOR_X  6
#define MOTOR_Y  7

/* С�������ת�̷��򣨴����ԣ� */
#define MOTOR_X_Direction_FORWARD    0       //X����  ǰ
#define MOTOR_X_Direction_BACKWARD   1       //X����  ��
#define MOTOR_Y_Direction_UP         1       //Y����  ��
#define MOTOR_Y_Direction_DOWN       0       //Y����  ��
#define MOTOR_R_Direction_CR         0       //Rת��  ˳
#define MOTOR_R_Direction_CCR        1       //Rת��  ��

void APP_MOTOR_ZDT_Init( void );
void APP_MOTOR_ZDT_WaitAck(void);
void APP_MOTOR_ZDT_Move_P(uint8_t _addr, uint8_t _dir, uint16_t _vel, uint8_t _acc, uint32_t _clk, bool _raF, bool _snF);

#endif