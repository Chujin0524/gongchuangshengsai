#ifndef __APP_MOTOR_2006_H
#define __APP_MOTOR_2006_H

/* Define  ------------------------------------------------------------------*/
#define MOTOR_LF 1
#define MOTOR_LB 2
#define MOTOR_RF 3
#define MOTOR_RB 4

/* 小车的行进方向 */
#define CarDirection_Forward   (1 << 0)       //前      1
#define CarDirection_Backward  (1 << 1)       //后      2
#define CarDirection_Left      (1 << 2)       //左      4
#define CarDirection_Right     (1 << 3)       //右      8
#define CarDirection_CR        (1 << 4)       //顺      16
#define CarDirection_CCR       (1 << 5)       //逆      32

#define CarDirection_Y         (CarDirection_Forward | CarDirection_Backward)       //Y方向移动
#define CarDirection_X         (CarDirection_Left    | CarDirection_Right)          //X方向移动
#define CarDirection_R         (CarDirection_CR      | CarDirection_CCR)            //旋转

#define CARDIRECTION_Y(Y)     (Y>0) ? CarDirection_Left     :  CarDirection_Right
#define CARDIRECTION_X(X)     (X>0) ? CarDirection_Backward :  CarDirection_Forward
#define CARDIRECTION_R(R)     (R>0) ? CarDirection_CR       :  CarDirection_CCR

/* Include  ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

typedef struct
{
    uint8_t WhaleDirX;   // X方向
    uint32_t WhalePosX;  // X距离
    uint8_t WhaleDirY;   // Y方向
    uint32_t WhalePosY;  // Y距离
    uint8_t WhaleDirR;   // R方向  R:Rotate旋转
    uint32_t WhalePosR;  // R距离
} CAR_XYR_T;

typedef struct
{
    uint8_t ucCarDir;       //小车行驶方向
    uint16_t usCarVel;      //小车行驶速度
    uint32_t ulCarPos;      //小车行驶距离
    uint8_t ucCarAcc;       //小车行驶加速度

    uint8_t ucWhale;        //被选择的轮子    1,2,3
    uint8_t ucWhaleDir;     //轮子旋转方向
    uint8_t ucaWhaleAcc[4]; //各轮加速度
    uint8_t ucaWhaleDir[4]; //各轮旋转方向
    uint16_t usaWhaleVel[4];//各轮速度
    uint32_t ulaWhalePos[4];//各轮行驶距离
} WHALE_PARAMETER_T;


void APP_MOTOR_2006_Init(void);
void APP_Motor_2006_WhaleControl(void);
void APP_MOTOR_2006_WaitResponse(uint8_t _addr);
void APP_MOTOR_2006_Move_P(uint8_t _addr, uint8_t _dir, uint16_t _vel, uint8_t _acc, uint32_t _clk, bool _raF, bool _snF);
void APP_MOTOR_2006_Move_V(uint8_t _addr, uint8_t _dir, uint16_t _vel, uint8_t _acc, bool _snF);
void APP_MOTOR_2006_WHALE_Move_P(uint8_t _dir, uint32_t _clk);
void APP_MOTOR_2006_WHALE_MovePID(uint8_t _dir, int32_t _clk);
void APP_MOTOR_2006_WHALE_Status(uint8_t _ucStatus);



#endif