#ifndef __TASK_CAR_H
#define __TASK_CAR_H

void App_Printf(char* format, ...);
void app_Init(void);
void AppObjCreate(void);

extern volatile int16_t g_observer_cnt1;
extern volatile int16_t g_observer_cnt2;

#endif