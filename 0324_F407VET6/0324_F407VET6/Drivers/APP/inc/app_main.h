#ifndef __APP_MAIN_H
#define __APP_MAIN_H

typedef enum
{
    STATUS_Idle = 0,

    STATUS_Start_2_Qr = 1,                    //����㵽��ά��

    STATUS_QrRead = 10,                       //��ά�봦��ȡ

    STATUS_Qr_2_RawArea = 2,                  //�Ӷ�ά�뵽ԭ����
    STATUS_RawAreaCal = 20,          //ԭ����У׼
    STATUS_RawAreaGrab0 = 21,            //ԭ������һ��ץȡ
    STATUS_RawAreaGrab1 = 22,            //ԭ�����ڶ���ץȡ
    STATUS_RawAreaGrab2 = 23,            //ԭ����������ץȡ

    STATUS_RawArea_2_ProFirArea = 3,      //��ԭ�������ּӹ���
    STATUS_ProFirAreaCal = 30,         //�ּӹ���У׼
    STATUS_ProFirAreaPlace = 31,       //�ּӹ�������
    STATUS_ProFirAreaGrab = 32,        //�ּӹ���ץȡ

    STATUS_ProFirArea_2_ProSecArea = 4,  //�Ӵּӹ�����ϸ�ӹ���
    STATUS_ProSecAreaCal = 40,           //ϸ�ӹ���У׼
    STATUS_ProSecAreaPlace = 41,         //ϸ�ӹ�������

    STATUS_ProSecArea_2_RawArea = 5,     //��ϸ�ӹ�����ԭ����

    STATUS_ProSecArea_2_Start = 6,         //��ϸ�ӹ��������
} STATUS_E;

void APP_MainStatusChange( void );
void APP_MainStatusRunning( void );

#endif