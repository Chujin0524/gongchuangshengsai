#ifndef __APP_BLUETOOTH_H
#define __APP_BLUETOOTH_H

typedef enum
{
	Flag_Idle = 0,           //0
	Flag_Stop,               //1
	Flag_Start,              //2
	Flag_StartTest,          //3
	Flag_ChangeCarPar,       //4
	Flag_ChangeStatus,       //5
	Flag_ServoTest,	         //6
	Flag_ChangeServoPar,     //7
    Flag_GYROSCOPE_ON,       //8
    Flag_GYROSCOPE_OFF,      //9
    Flag_GYROSCOPE_RESET,    //10
	Flag_NEXT,               //11
    Flag_MotorZDT,           //12
    Flag_Servo,              //13
    Flag_Show,               //14
    Flag_Reset,              //15
}FLAG_E;

typedef struct {
    char paramName[32];  // 参数名
    int16_t paramValue;  // 参数值
} PARAMETER_T;


void APP_BLUETOOTH_Init( void );
void APP_BLUETOOTH_Read( void );

#endif