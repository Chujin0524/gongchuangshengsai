#ifndef __BSP_SERVO_H
#define __BSP_SERVO_H

#define TIM_CHANNEL_JAW    TIM_CHANNEL_3
#define TIM_CHANNEL_PLATE  TIM_CHANNEL_2
#define TIM_CHANNEL_WRIST  TIM_CHANNEL_1

void bsp_InitServo(void);

void SERVO_Plate(uint8_t _ucIndex);
void SERVO_Jaw(uint8_t _ucIndex);
void SERVO_Wrist(uint8_t _ucIndex);

#endif

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
