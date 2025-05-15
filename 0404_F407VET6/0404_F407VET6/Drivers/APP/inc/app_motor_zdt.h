#ifndef __APP_MOTOR_ZDT_H
#define __APP_MOTOR_ZDT_H

#define MOTOR_R  5
#define MOTOR_X  6
#define MOTOR_Y  7
#define MOTOR_W  8  //W : Wirst

/* 小车导轨和转盘方向（待测试） */
#define MOTOR_X_Direction_FORWARD    0       //X导轨  前
#define MOTOR_X_Direction_BACKWARD   1       //X导轨  后
#define MOTOR_Y_Direction_UP         1       //Y导轨  上
#define MOTOR_Y_Direction_DOWN       0       //Y导轨  下
#define MOTOR_R_Direction_CR         0       //R转盘  顺
#define MOTOR_R_Direction_CCR        1       //R转盘  逆
#define MOTOR_W_Direction_CR         0       //W腕处  顺
#define MOTOR_W_Direction_CCR        1       //W腕处  逆

void APP_MOTOR_ZDT_Init(void);
void APP_MOTOR_ZDT_SetAckId(uint8_t _addr);
void APP_MOTOR_ZDT_WaitAck(uint8_t _addr);
void APP_MOTOR_ZDT_Move_P(uint8_t _addr, uint8_t _dir, uint16_t _vel, uint8_t _acc, uint32_t _clk, bool _raF, bool _snF);

#endif