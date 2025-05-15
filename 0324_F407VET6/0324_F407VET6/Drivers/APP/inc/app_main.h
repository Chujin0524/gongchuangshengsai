#ifndef __APP_MAIN_H
#define __APP_MAIN_H

typedef enum
{
    STATUS_Idle = 0,

    STATUS_Start_2_Qr = 1,                    //从起点到二维码

    STATUS_QrRead = 10,                       //二维码处读取

    STATUS_Qr_2_RawArea = 2,                  //从二维码到原料区
    STATUS_RawAreaCal = 20,          //原料区校准
    STATUS_RawAreaGrab0 = 21,            //原料区第一次抓取
    STATUS_RawAreaGrab1 = 22,            //原料区第二次抓取
    STATUS_RawAreaGrab2 = 23,            //原料区第三次抓取

    STATUS_RawArea_2_ProFirArea = 3,      //从原料区到粗加工区
    STATUS_ProFirAreaCal = 30,         //粗加工区校准
    STATUS_ProFirAreaPlace = 31,       //粗加工区放置
    STATUS_ProFirAreaGrab = 32,        //粗加工区抓取

    STATUS_ProFirArea_2_ProSecArea = 4,  //从粗加工区到细加工区
    STATUS_ProSecAreaCal = 40,           //细加工区校准
    STATUS_ProSecAreaPlace = 41,         //细加工区放置

    STATUS_ProSecArea_2_RawArea = 5,     //从细加工区到原料区

    STATUS_ProSecArea_2_Start = 6,         //从细加工区到起点
} STATUS_E;

void APP_MainStatusChange( void );
void APP_MainStatusRunning( void );

#endif