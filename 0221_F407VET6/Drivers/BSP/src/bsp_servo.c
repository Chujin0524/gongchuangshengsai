#include "main.h"

//250 - 1250
/*
     Jaw       PD12
     Wrist     PD13
     Plate     PD14
*/

#define TIM_CHANNEL_JAW    TIM_CHANNEL_1
#define TIM_CHANNEL_WRIST  TIM_CHANNEL_2
#define TIM_CHANNEL_PLATE  TIM_CHANNEL_3

static uint16_t s_AnglePlate_R = 350;   //ת�̽Ƕ�  ��ɫ�����λ��
static uint16_t s_AnglePlate_G = 780;   //ת�̽Ƕ�  ��ɫ�����λ��
static uint16_t s_AnglePlate_B = 1250;  //ת�̽Ƕ�  ��ɫ�����λ��
static uint16_t s_AngleJaw_Z   = 640;     //��צ�Ƕ�  ץ�����
static uint16_t s_AngleJaw_S   = 950;     //��צ�Ƕ�  �ɿ����
static uint16_t s_AngleWrist_F = 1160;      //�󴦽Ƕ�  ��ǰ
static uint16_t s_AngleWrist_R = 640;       //�󴦽Ƕ�  ����

static uint16_t s_AngleJaw   = 0;  //ת�̳�ʼ�Ƕ�
static uint16_t s_AngleWrist = 0;  //��צ��ʼ�Ƕ�
static uint16_t s_AnglePlate = 0;  //�󴦳�ʼ�Ƕ�

/*
*********************************************************************************************************
*	�� �� ��: bsp_InitServo
*	����˵��: ��ʼ�����������PWM���Ϊ0
*	��    ��: ��
*	�� �� ֵ: ��
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
*	�� �� ��: SERVO_ChangeAngle
*	����˵��: ����Ƕȸı�
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void SERVO_ChangeAngle(void)
{
	/*
	50Hz---20ms
	
	0.5ms--2.5ms
	2.5%---12.5%
	250----1250
	750ֹͣ
	*/
//	ServoJaw(s_AngleJaw);
//	ServoWrist(s_AngleWrist);
//	ServoPlate(s_AnglePlate);	
}

/*
*********************************************************************************************************
*	�� �� ��: SERVO_Plate
*	����˵��: ת��ת��
*	��    ��: _ucIndex : R,G,B--�죬�̣�������Ӧ�Ƕ�
*	�� �� ֵ: ��
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
*	�� �� ��: SERVO_Jaw
*	����˵��: ��צת��
*	��    ��: _ucIndex : Z,S--ץ�����ɿ�
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void SERVO_Jaw(uint8_t _ucIndex)
{	
	switch (_ucIndex)
	{
    case 'Z':   //ץ
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_JAW, s_AngleJaw_Z);
        break;
    
    case 'S':   //��
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_JAW, s_AngleJaw_S);
        break;
    
    default:
        break;
	}
}

/*
*********************************************************************************************************
*   �� �� ��: SERVO_Wrist
*   ����˵��: ��ת��
*   ��    ��: _ucIndex : F,R--��ǰ������
*   �� �� ֵ: ��
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

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
