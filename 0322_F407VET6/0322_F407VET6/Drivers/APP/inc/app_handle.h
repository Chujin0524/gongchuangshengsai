#ifndef __APP_HANDLE_H
#define __APP_HANDLE_H

#define MOTOR_X_Pos_Min  0
#define MOTOR_X_Pos_Max  9800
#define MOTOR_R_Pos_Min  0
#define MOTOR_R_Pos_Max  7000

uint32_t *APP_Handle_GetPosPoint(uint8_t _Motor);
uint8_t (*APP_Handle_GetMapPoint(void))[4];
void APP_Handle_ColorMapping_Init(uint8_t _time, uint8_t RedDir, uint8_t GreenDir, uint8_t BlueDir);
void APP_Handle_PickRawArea(uint8_t _ucStatusIndex);
void APP_Handle_Place_All(uint8_t _time, uint8_t _firstIndex, uint8_t _secondIndex, uint8_t _thirdIndex);
void APP_Handle_Pick_All(uint8_t _firstIndex, uint8_t _secondIndex, uint8_t _thirdIndex);
void APP_Handle_Run(void);
void APP_Handle_Stay(void);

#endif