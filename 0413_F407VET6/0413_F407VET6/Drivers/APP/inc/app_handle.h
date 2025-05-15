#ifndef __APP_HANDLE_H
#define __APP_HANDLE_H

#define MOTOR_X_Pos_Min  0
#define MOTOR_X_Pos_Max  9800
#define MOTOR_R_Pos_Min  0
#define MOTOR_R_Pos_Max  7000

/*
腕处位置 :
*/
#define  CIRCLE_W_LEFT    1
#define  CIRCLE_W_MIDDLE  2
#define  CIRCLE_W_RIGHT   3
#define  BLOB_W_LEFT      4
#define  BLOB_W_MIDDLE    5
#define  BLOB_W_RIGHT     6
#define  RIGHT_W          7
#define  FORWARD_W        8
#define  LEFT_W           9

/*
转盘位置 :
*/
#define  CIRCLE_R_LEFT    1
#define  CIRCLE_R_MIDDLE  2
#define  CIRCLE_R_RIGHT   3
#define  BLOB_R_LEFT      4
#define  BLOB_R_MIDDLE    5
#define  BLOB_R_RIGHT     6

/*
X位置 :
*/
#define  CIRCLE_X_LEFT    1
#define  CIRCLE_X_MIDDLE  2
#define  CIRCLE_X_RIGHT   3
#define  BLOB_X_LEFT      4
#define  BLOB_X_MIDDLE    5
#define  BLOB_X_RIGHT     6

#define BlobPlace   1
#define CirclePlace 2

uint8_t (*APP_Handle_GetMapPoint(void))[4];
void APP_Handle_ColorMapping_Init(uint8_t _time, uint8_t RedDir, uint8_t GreenDir, uint8_t BlueDir);
void APP_Handle_PickRawArea(uint8_t _ucColor, uint8_t _ucIndex);
void APP_Handle_Place_All(uint8_t _ucCycle, uint8_t _firstIndex, uint8_t _secondIndex, uint8_t _thirdIndex);
void APP_Handle_Place_One(uint8_t _color, uint8_t _next_color);
void APP_Handle_Pick_All(uint8_t _firstIndex, uint8_t _secondIndex, uint8_t _thirdIndex);
void APP_Handle_Run(void);
void APP_Handle_Stay(uint8_t _ucCalPlace);

extern uint8_t MOTOR_W_Acc[2];
extern uint16_t MOTOR_W_Vel;
extern uint32_t MOTOR_W_Pos[10];

extern uint8_t  MOTOR_R_Acc;
extern uint16_t MOTOR_R_Vel;
extern uint32_t MOTOR_R_Pos[7];

extern uint8_t  MOTOR_X_Acc;
extern uint16_t MOTOR_X_Vel;
extern uint32_t MOTOR_X_Pos[8];

extern uint8_t  MOTOR_Y_Acc[2];
extern uint16_t MOTOR_Y_Vel[2];
extern uint32_t MOTOR_Y_Pos[9];

#endif