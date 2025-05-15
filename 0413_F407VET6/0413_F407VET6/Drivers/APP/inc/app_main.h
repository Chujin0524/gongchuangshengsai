#ifndef __APP_MAIN_H
#define __APP_MAIN_H

typedef enum
{
    STATUS_Idle = 0,

    STATUS_Start_2_Qr = 1,                //从起点到二维码

    STATUS_QrRead = 10,                   //二维码处读取

    STATUS_Qr_2_RawArea = 2,              //从二维码到原料区
    STATUS_RawAreaCal = 20,               //原料区校准
    STATUS_RawAreaGrab = 21,              //原料区抓取

    STATUS_RawArea_2_ProFirArea = 3,      //从原料区到粗加工区
    STATUS_ProFirAreaCalAndPlace = 30,    //粗加工区校准和放置
    STATUS_ProFirAreaGrab = 31,           //粗加工区抓取

    STATUS_ProFirArea_2_ProSecArea = 4,   //从粗加工区到细加工区
    STATUS_ProSecAreaCal = 40,            //细加工区校准
    STATUS_ProSecAreaPlace = 41,          //细加工区放置

    STATUS_ProSecArea_2_RawArea = 5,      //从细加工区到原料区

    STATUS_ProSecArea_2_Start = 6,        //从细加工区到起点
} STATUS_E;

typedef struct {
    uint32_t start_time;
    char duration_str[20];  // 存储 "X分Y秒"
} StateTimer;

void APP_MainStatusChange(void);
void APP_MainStatusRunning(void);

void Timer_Start(StateTimer *timer);
void Timer_GetDuration(StateTimer *timer);

#endif