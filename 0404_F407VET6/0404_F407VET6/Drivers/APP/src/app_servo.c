#include "main.h"

void APP_SERVO_Jaw(uint8_t _index)
{
    SERVO_Jaw(_index);
    vTaskDelay(pdMS_TO_TICKS(400));
}

void APP_SERVO_Wrist(uint8_t _index)
{
    SERVO_Wrist(_index);
    //    vTaskDelay(pdMS_TO_TICKS(100));
}

void APP_SERVO_Plate(uint8_t _index)
{
    SERVO_Plate(_index);
    vTaskDelay(pdMS_TO_TICKS(100));
}
