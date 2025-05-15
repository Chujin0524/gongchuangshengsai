#ifndef __KEYLED_H
#define __KEYLED_H

#include "main.h"

typedef enum
{
    KEY_NONE,
    KEY,
    KEY_LEFT,
    KEY_RIGHT
} eKEYS; 

#define KEY_MAX_TIMEOUT 0
eKEYS ScanKey(uint32_t timeout);

#ifdef LED_Pin
#define LED_ON()      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET)
#define LED_OFF()     HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET)
#define LED_TOGGLE()  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin)
#endif /*LED_Pin*/

#ifdef LED_R_Pin
#define LED_R_ON()      HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET)
#define LED_R_OFF()     HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET)
#define LED_R_TOGGLE()  HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin)
#endif /*LED_R_Pin*/

#ifdef LED_G_Pin
#define LED_G_ON()      HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET)
#define LED_G_OFF()     HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET)
#define LED_G_TOGGLE()  HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin)
#endif /*LED_G_Pin*/

#ifdef LED_B_Pin
#define LED_B_ON()      HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET)
#define LED_B_OFF()     HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET)
#define LED_B_TOGGLE()  HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin)
#endif /*LED_B_Pin*/

#endif
