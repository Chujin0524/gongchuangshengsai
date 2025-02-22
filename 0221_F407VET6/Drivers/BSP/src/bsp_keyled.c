#include "main.h"

eKEYS ScanKey(uint32_t timeout)
{
    eKEYS curKey = KEY_NONE;
    uint32_t start_tick = 0;
    if (timeout != 0)
    {
        start_tick = HAL_GetTick(); 
    }
    while (1)
    {
        if (timeout != 0)
        {
            if ((HAL_GetTick() - start_tick) > timeout)
                break;
        }
        
#ifdef Key_Pin
        if (HAL_GPIO_ReadPin(Key_GPIO_Port, Key_Pin) == GPIO_PIN_RESET)
        {
            HAL_Delay(20);
            if (HAL_GPIO_ReadPin(Key_GPIO_Port, Key_Pin) == GPIO_PIN_RESET)
            {
                curKey = KEY;
                while (HAL_GPIO_ReadPin(Key_GPIO_Port, Key_Pin) == GPIO_PIN_RESET);
            }
            break;
        }
#endif /*Key_Pin*/
        
#ifdef KeyLeft_Pin
        if (HAL_GPIO_ReadPin(KeyLeft_GPIO_Port, KeyLeft_Pin) == GPIO_PIN_SET)
        {
            HAL_Delay(20);
            if (HAL_GPIO_ReadPin(KeyLeft_GPIO_Port, KeyLeft_Pin) == GPIO_PIN_SET)
            {
                curKey = KEY_LEFT;
                while (HAL_GPIO_ReadPin(KeyLeft_GPIO_Port, KeyLeft_Pin) == GPIO_PIN_SET);
            }
            break;
        }
#endif /*KeyLeft_Pin*/
        
#ifdef KeyRight_Pin
        if (HAL_GPIO_ReadPin(KeyRight_GPIO_Port, KeyRight_Pin) == GPIO_PIN_SET)
        {
            HAL_Delay(20);
            if (HAL_GPIO_ReadPin(KeyRight_GPIO_Port, KeyRight_Pin) == GPIO_PIN_SET)
            {
                curKey = KEY_RIGHT;
                while (HAL_GPIO_ReadPin(KeyRight_GPIO_Port, KeyRight_Pin) == GPIO_PIN_SET);
            }
            break;
        }
#endif /*KeyRight_Pin*/
    }
    
    return curKey;
}