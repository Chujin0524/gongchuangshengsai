#include "main.h"

//250 - 1250
/*
     Jaw       PD12
     Wrist     PD13
     Plate     PD14
*/

static uint16_t s_AnglePlate_R = 550;   //转盘角度  红色物块存放位置
static uint16_t s_AnglePlate_G = 1200;   //转盘角度  绿色物块存放位置
static uint16_t s_AnglePlate_B = 840;  //转盘角度  蓝色物块存放位置
static uint16_t s_AngleJaw_Z   = 830;     //夹爪角度  抓紧物块
static uint16_t s_AngleJaw_S   = 1200;     //夹爪角度  松开物块
static uint16_t s_AngleWrist_F = 1180;      //腕处角度  朝前
static uint16_t s_AngleWrist_R = 610;       //腕处角度  朝右

static uint16_t s_AngleJaw   = 1200;  //夹爪初始角度
static uint16_t s_AngleWrist = 1180;  //腕处初始角度
static uint16_t s_AnglePlate = 1080;  //转盘初始角度

/*
*********************************************************************************************************
*    函 数 名: bsp_InitServo
*    功能说明: 初始化三个舵机，PWM输出为0
*    形    参: 无
*    返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitServo(void)
{
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_JAW);   //Wrist
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_WRIST);   //Jaw
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_PLATE);   //Plate

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_JAW, s_AngleJaw);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_WRIST, s_AngleWrist);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_PLATE, s_AnglePlate);
}

/*
*********************************************************************************************************
*    函 数 名: SERVO_Plate
*    功能说明: 转盘转角
*    形    参: _ucIndex : R,G,B--红，绿，蓝物块对应角度
*    返 回 值: 无
*********************************************************************************************************
*/
void SERVO_Plate(uint8_t _ucIndex)
{
    switch (_ucIndex)
    {
    case 1:
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_PLATE, s_AnglePlate_R);
        break;

    case 2:
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_PLATE, s_AnglePlate_G);
        break;

    case 3:
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_PLATE, s_AnglePlate_B);
        break;

    default:
        break;
    }
}

/*
*********************************************************************************************************
*    函 数 名: SERVO_Jaw
*    功能说明: 夹爪转角
*    形    参: _ucIndex : Z,S--抓紧，松开
*    返 回 值: 无
*********************************************************************************************************
*/
void SERVO_Jaw(uint8_t _ucIndex)
{
    switch (_ucIndex)
    {
    case 'Z':   //抓
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_JAW, s_AngleJaw_Z);
        break;

    case 'S':   //松
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_JAW, s_AngleJaw_S);
        break;

    default:
        break;
    }
}

/*
*********************************************************************************************************
*   函 数 名: SERVO_Wrist
*   功能说明: 腕处转角
*   形    参: _ucIndex : F,R--朝前，朝右
*   返 回 值: 无
*********************************************************************************************************
*/
void SERVO_Wrist(uint8_t _ucIndex)
{
    switch (_ucIndex)
    {
    case 'F':
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_WRIST, s_AngleWrist_F);
        break;

    case 'R':
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_WRIST, s_AngleWrist_R);
        break;

    default:
        break;
    }
}

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
