#ifndef __BSP_SERVO_H
#define __BSP_SERVO_H

void bsp_InitServo(void);

void SERVO_ChangeAngle(void);
void SERVO_Plate(uint8_t _ucIndex);
void SERVO_Jaw(uint8_t _ucIndex);
void SERVO_Wrist(uint8_t _ucIndex);

#endif

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
