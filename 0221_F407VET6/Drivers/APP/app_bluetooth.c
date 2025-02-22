#include "main.h"

static void APP_BLUETOOTH_Parse( uint8_t* pucFrameData, uint8_t ucFrameLength );

void APP_BLUETOOTH_Init( void )
{
    bsp_InitBLUETOOTH( APP_BLUETOOTH_Parse );    //ע������ص�����
}

void APP_BLUETOOTH_Read( void )
{
    if( BLUETOOTH_ReadData() )  //�ɹ�����һ֡�������������ý����ص�����APP_BLUETOOTH_Parse
    {
//        LED_TOGGLE();
          
        App_Printf("RECEIVE\r\n");
    }
}

/*
*********************************************************************************************************
*    �� �� ��: APP_BLUETOOTH_Parse
*    ����˵��: ���������ص����������ڴ�����յ�������ָ�
*    ��    �Σ�pucFrameData ���յ�����������֡
*             ucFrameLength ����֡�ĳ���
*    �� �� ֵ: ��
*********************************************************************************************************
*/
static void APP_BLUETOOTH_Parse( uint8_t* pucFrameData, uint8_t ucFrameLength )
{
    static char bluetooth_tx_buf[256] = {0};       //������ͻ���
    App_Printf("%s\r\n", pucFrameData);            //�����յ����ַ������ͳ�ȥ
    
    FLAG_E ucFlag = Flag_Idle;
    uint8_t *number_start;
    
    //�����޸ĵĲ���
    static STATUS_E ucStatus = STATUS_Idle;         //״̬
    static CAR_XYR_T s_tCarXYR = {0};               //С���н�����
    static CAR_XYR_T *ptCarXYR = &s_tCarXYR;
    
    // �����ַ���
    if (strncmp((char*)pucFrameData, "Stop", 4) == 0)
    {
        ucFlag = Flag_Stop;
    }
    else if (strncmp((char*)pucFrameData, "Status=", 7) == 0) 
    {
        number_start = pucFrameData + 7;
        ucStatus = atoi((char*)number_start);

        xQueueOverwrite(Queue_StatusHandle, &ucStatus);  //����Ϣ����Queue_StatusHandle���ͳ���״̬
        
        ucFlag = Flag_ChangeStatus;
    }
    else if (strncmp((char*)pucFrameData, "WhaleDirX=", 10) == 0) 
    {
        number_start = pucFrameData + 10;
        ptCarXYR->WhaleDirX = atoi((char*)number_start);
    }
    else if (strncmp((char*)pucFrameData, "WhalePosX=", 10) == 0) 
    {
        number_start = pucFrameData + 10;
        ptCarXYR->WhalePosX = atoi((char*)number_start);

        xQueueOverwrite(Queue_MotorHandle, &ptCarXYR);  //����Ϣ����Queue_MotorHandle�������Ӳ���
        ucFlag = Flag_ChangeCarPar;
    }
    else if (strncmp((char*)pucFrameData, "WhaleDirY=", 10) == 0) 
    {
        number_start = pucFrameData + 10;
        ptCarXYR->WhaleDirY = atoi((char*)number_start);
    }
    else if (strncmp((char*)pucFrameData, "WhalePosY=", 10) == 0) 
    {
        number_start = pucFrameData + 10;
        ptCarXYR->WhalePosY = atoi((char*)number_start);

        xQueueOverwrite(Queue_MotorHandle, &ptCarXYR);  //����Ϣ����Queue_MotorHandle�������Ӳ���
        ucFlag = Flag_ChangeCarPar;
    }
    else if (strncmp((char*)pucFrameData, "WhaleDirR=", 10) == 0) 
    {
        number_start = pucFrameData + 10;
        ptCarXYR->WhaleDirR = atoi((char*)number_start);
    }
    else if (strncmp((char*)pucFrameData, "WhalePosR=", 10) == 0) 
    {
        number_start = pucFrameData + 10;
        ptCarXYR->WhalePosR = atoi((char*)number_start);

        xQueueOverwrite(Queue_MotorHandle, &ptCarXYR);  //����Ϣ����Queue_MotorHandle�������Ӳ���
        ucFlag = Flag_ChangeCarPar;
    }
    else if (strncmp((char*)pucFrameData, "Start", 5) == 0) 
    {
        ucFlag = Flag_Start;
    }
    else if (strncmp((char*)pucFrameData, "Next", 4) == 0) 
    {
        xTaskNotifyGive(Task_MainHandle);
        ucFlag = Flag_NEXT;
    }
    else 
    {
        App_Printf("Unknown command: %s\r\n", pucFrameData);
    }
    App_Printf("Flag : %d\r\n", ucFlag);
    
    if ( ucFlag == Flag_ChangeCarPar)
    {
        App_Printf("-----Flag_ChangeCarPar-----\r\n");
        App_Printf("DirX: %d\r\n", ptCarXYR->WhaleDirX);
        App_Printf("PosX: %d\r\n", ptCarXYR->WhalePosX);
        App_Printf("DirY: %d\r\n", ptCarXYR->WhaleDirY);
        App_Printf("PosY: %d\r\n", ptCarXYR->WhalePosY);
        App_Printf("DirR: %d\r\n", ptCarXYR->WhaleDirR);
        App_Printf("PosR: %d\r\n", ptCarXYR->WhalePosR);
        App_Printf("-----Flag_ChangeCarPar-----\r\n");
    }
}
