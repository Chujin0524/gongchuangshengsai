#ifndef __APP_MAIN_H
#define __APP_MAIN_H

typedef enum
{
    STATUS_Idle = 0,

    STATUS_Start_2_Qr = 1,                //����㵽��ά��

    STATUS_QrRead = 10,                   //��ά�봦��ȡ

    STATUS_Qr_2_RawArea = 2,              //�Ӷ�ά�뵽ԭ����
    STATUS_RawAreaCal = 20,               //ԭ����У׼
    STATUS_RawAreaGrab = 21,              //ԭ����ץȡ

    STATUS_RawArea_2_ProFirArea = 3,      //��ԭ�������ּӹ���
    STATUS_ProFirAreaCalAndPlace = 30,    //�ּӹ���У׼�ͷ���
    STATUS_ProFirAreaGrab = 31,           //�ּӹ���ץȡ

    STATUS_ProFirArea_2_ProSecArea = 4,   //�Ӵּӹ�����ϸ�ӹ���
    STATUS_ProSecAreaCal = 40,            //ϸ�ӹ���У׼
    STATUS_ProSecAreaPlace = 41,          //ϸ�ӹ�������

    STATUS_ProSecArea_2_RawArea = 5,      //��ϸ�ӹ�����ԭ����

    STATUS_ProSecArea_2_Start = 6,        //��ϸ�ӹ��������
} STATUS_E;

void APP_MainStatusChange( void );
void APP_MainStatusRunning( void );

#endif