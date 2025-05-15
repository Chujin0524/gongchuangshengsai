#include "main.h"
#include "bsp_can_2006.h"
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

        App_Printf( "RECEIVE\r\n" );
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
    static char bluetooth_tx_buf[512] = {0};       //�������ͻ���
    App_Printf( "%s\r\n", pucFrameData );          //�����յ����ַ������ͳ�ȥ

    FLAG_E ucFlag = Flag_Idle;
    uint8_t* number_start;

    //�����޸ĵĲ���
    static STATUS_E ucStatus = STATUS_Idle;         //״̬
    static CAR_XYR_T s_tCarXYR = {0};               //С���н�����
    static CAR_XYR_T *ptCarXYR = &s_tCarXYR;
    static uint16_t s_AngleJaw = 0;
    static uint16_t s_AngleWrist = 0;
    static uint16_t s_AnglePlate = 0;
    static uint8_t s_MotorZDT_ID = 0;
    static uint8_t s_MotorZDT_DIR = 0;
    static uint32_t s_MotorZDT_POS = 0;
    

    // ��ʾ��Ϣ
    if( strncmp(( char* )pucFrameData, "ShowTask", 8 ) == 0 )
    {
        App_Printf( "=================================================\r\n" );
        App_Printf( "Task_Name                      State   Priority  StackFree  TaskNumber\r\n" );
        vTaskList(( char* )&bluetooth_tx_buf );
        App_Printf( "%s\r\n", bluetooth_tx_buf );
        memset( bluetooth_tx_buf, 0, sizeof( bluetooth_tx_buf ) );

        App_Printf( "\r\nName       RunCounter         Used\r\n" );
        vTaskGetRunTimeStats(( char* )&bluetooth_tx_buf );
        App_Printf( "%s\r\n", bluetooth_tx_buf );
        memset( bluetooth_tx_buf, 0, sizeof( bluetooth_tx_buf ) );

        static uint8_t i = 0;
        if( i == 0 )
        {
            xEventGroupSetBits( EventGroups_CarHandle, EventGroupsCarShow_7 );
            i = 1;
        }
        else
        {
            xEventGroupClearBits( EventGroups_CarHandle, EventGroupsCarShow_7 );
            i = 0;
        }


        ucFlag = Flag_Show;
    }
    else if( strncmp(( char* )pucFrameData, "ShowParameter", 13 ) == 0 )
    {
        App_Printf( "g_fGyro_z : %.3f\r\n", g_fGyro_z );
        App_Printf( "distance_r : %.3f\r\n", distance_r );
        App_Printf( "distance_dia : %.3f\r\n", distance_big_dia );
        
        App_Printf( "g_dX : %d\r\n", g_dX);
        App_Printf( "g_dY : %d\r\n", g_dY);
        App_Printf( "g_dR : %d\r\n", g_dR);
       
        ucFlag = Flag_Show;
    }

    // ��ͣ
    else if( strncmp(( char* )pucFrameData, "Stop", 4 ) == 0 )
    {
        vTaskSuspend( Task_PIDHandle );
        set_moto_current(&hcan1, 0, 0, 0, 0 );
        ucFlag = Flag_Stop;
    }

    // ��λ
    else if( strncmp(( char* )pucFrameData, "Reset", 5 ) == 0 )
    {
        NVIC_SystemReset();
        ucFlag = Flag_Reset;
    }

    // �޸�����״̬
    else if( strncmp(( char* )pucFrameData, "Status=", 7 ) == 0 )
    {
        number_start = pucFrameData + 7;
        ucStatus = atoi(( char* )number_start );

        xQueueOverwrite( Queue_StatusHandle, &ucStatus ); //����Ϣ����Queue_StatusHandle���ͳ���״̬

        ucFlag = Flag_ChangeStatus;
    }


    //��ʻ����
    else if( strncmp(( char* )pucFrameData, "WhaleDirX=", 10 ) == 0 )
    {
        number_start = pucFrameData + 10;
        ptCarXYR->WhaleDirX = atoi(( char* )number_start );
    }
    else if( strncmp(( char* )pucFrameData, "WhalePosX=", 10 ) == 0 )
    {
        number_start = pucFrameData + 10;
        ptCarXYR->WhalePosX = atoi(( char* )number_start );

        xQueueOverwrite( Queue_MotorHandle, &ptCarXYR ); //����Ϣ����Queue_MotorHandle�������Ӳ���
        ucFlag = Flag_ChangeCarPar;
    }
    else if( strncmp(( char* )pucFrameData, "WhaleDirY=", 10 ) == 0 )
    {
        number_start = pucFrameData + 10;
        ptCarXYR->WhaleDirY = atoi(( char* )number_start );
    }
    else if( strncmp(( char* )pucFrameData, "WhalePosY=", 10 ) == 0 )
    {
        number_start = pucFrameData + 10;
        ptCarXYR->WhalePosY = atoi(( char* )number_start );

        xQueueOverwrite( Queue_MotorHandle, &ptCarXYR ); //����Ϣ����Queue_MotorHandle�������Ӳ���
        ucFlag = Flag_ChangeCarPar;
    }
    else if( strncmp(( char* )pucFrameData, "WhaleDirR=", 10 ) == 0 )
    {
        number_start = pucFrameData + 10;
        ptCarXYR->WhaleDirR = atoi(( char* )number_start );
    }
    else if( strncmp(( char* )pucFrameData, "WhalePosR=", 10 ) == 0 )
    {
        number_start = pucFrameData + 10;
        ptCarXYR->WhalePosR = atoi(( char* )number_start );

        xQueueOverwrite( Queue_MotorHandle, &ptCarXYR ); //����Ϣ����Queue_MotorHandle�������Ӳ���
        ucFlag = Flag_ChangeCarPar;
    }

    //����
    else if( strncmp(( char* )pucFrameData, "Start", 5 ) == 0 )
    {
        ucFlag = Flag_Start;
    }


    //������һ��
    else if( strncmp(( char* )pucFrameData, "Next", 4 ) == 0 )
    {
        xTaskNotifyGive( Task_MainHandle );
        ucFlag = Flag_NEXT;
    }


    //zdt����
    //5 : max5300      1��
    //6 : max10000     0ǰ
    //7 : max8000      0��
    else if( strncmp(( char* )pucFrameData, "M_ZDTid=", 8 ) == 0 )
    {
        number_start = pucFrameData + 8;
        s_MotorZDT_ID = atoi(( char* )number_start );

        ucFlag = Flag_MotorZDT;
    }
    else if( strncmp(( char* )pucFrameData, "M_ZDTdir=", 9 ) == 0 )
    {
        number_start = pucFrameData + 9;
        s_MotorZDT_DIR = atoi(( char* )number_start );
        ucFlag = Flag_MotorZDT;
    }
    else if( strncmp(( char* )pucFrameData, "M_ZDTpos=", 9 ) == 0 )
    {
        number_start = pucFrameData + 9;
        s_MotorZDT_POS = atoi(( char* )number_start );
        ucFlag = Flag_MotorZDT;
    }
    else if( strncmp(( char* )pucFrameData, "M_ZDTstart", 10 ) == 0 )
    {
        APP_MOTOR_ZDT_Move_P( s_MotorZDT_ID, s_MotorZDT_DIR, 1000, 0, s_MotorZDT_POS, 1, 0 );
        APP_MOTOR_ZDT_WaitAck();
        
        ucFlag = Flag_MotorZDT;
    }
    else if( strncmp(( char* )pucFrameData, "M_ZDTstartnw", 12 ) == 0 )  //start no wait
    {
        APP_MOTOR_ZDT_Move_P( s_MotorZDT_ID, s_MotorZDT_DIR, 50, 0, s_MotorZDT_POS, 1, 0 );

        ucFlag = Flag_MotorZDT;
    }


    //�������
    else if( strncmp(( char* )pucFrameData, "ServoJaw=", 9 ) == 0 )
    {
        number_start = pucFrameData + 9;
        s_AngleJaw = atoi(( char* )number_start );
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_JAW, s_AngleJaw);
        ucFlag = Flag_Servo;
    }
    else if( strncmp(( char* )pucFrameData, "ServoWrist=", 11 ) == 0 )
    {
        number_start = pucFrameData + 11;
        s_AngleWrist = atoi(( char* )number_start );
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_WRIST, s_AngleWrist );
        ucFlag = Flag_Servo;
    }
    else if( strncmp(( char* )pucFrameData, "ServoPlate=", 11 ) == 0 )
    {
        number_start = pucFrameData + 11;
        s_AnglePlate = atoi(( char* )number_start );
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_PLATE, s_AnglePlate );
        ucFlag = Flag_Servo;
    }

    else if( strncmp(( char* )pucFrameData, "Handle_Test=", 12 ) == 0 )
    {
        number_start = pucFrameData + 12;
        uint8_t test= atoi(( char* )number_start );
        switch (test)
        {
            case 1:
                APP_Handle_Place_All( 1, 1, 2, 3 );
                break;
            case 2:
                APP_Handle_Place_All( 2, 1, 2, 3 );
                break;
            case 3:
                APP_Handle_Pick_All( 1, 2, 3 );
                break;
            case 4:
                APP_MAIXCAM_Cal( CIRCLE, BLUE,  0, MoveXR );        //ʶ����ɫԲ������У׼��ʹX��R����ƶ�
                break;
            case 5:
                APP_MAIXCAM_Cal( CIRCLE, RED,   0, MoveXR );        //ʶ���ɫԲ������У׼��ʹX��R����ƶ�
                break;
            case 6:
                APP_MAIXCAM_Cal( CIRCLE, GREEN, 0, MoveXR );        //ʶ����ɫԲ������У׼��ʹX��R����ƶ�
                break;
            case 7:
                vTaskDelay(5000);
                APP_MOTOR_ZDT_Move_P( 7, 0, 800, 0, 10500, 1, 0 ); APP_MOTOR_ZDT_WaitAck();  //�������
                APP_SERVO_Jaw('Z');        //�н�
                APP_MOTOR_ZDT_Move_P( 7, 0, 800, 0, 300, 1, 0 ); APP_MOTOR_ZDT_WaitAck();    //�������
                APP_SERVO_Wrist('R'); vTaskDelay(400);     //������
                APP_MOTOR_ZDT_Move_P( 7, 0, 800, 0, 3000, 1, 0 ); APP_MOTOR_ZDT_WaitAck();   //����ת���Ϸ�
                APP_SERVO_Jaw('S');       //�ɿ�
                APP_MOTOR_ZDT_Move_P( 7, 0, 800, 0, 300, 1, 0 ); APP_MOTOR_ZDT_WaitAck();    //�������
                APP_SERVO_Wrist('F'); vTaskDelay(400);    //��ǰ
                break;
            case 8:
                vTaskDelay(5000);
                APP_Handle_Pick_All(3, 2, 1);
                break;
            case 9:
                vTaskDelay(5000);
                APP_Handle_Place_All(1, 3, 2, 1);
                break;
            default:
                break;
        }
        ucFlag = Flag_Test;
    }


    else
    {
        App_Printf( "Unknown command: %s\r\n", pucFrameData );
    }
    App_Printf( "Flag : %d\r\n", ucFlag );

    if( ucFlag == Flag_ChangeCarPar )
    {
        App_Printf( "-----Flag_ChangeCarPar-----\r\n" );
        App_Printf( "DirX: %d\r\n", ptCarXYR->WhaleDirX );
        App_Printf( "PosX: %d\r\n", ptCarXYR->WhalePosX );
        App_Printf( "DirY: %d\r\n", ptCarXYR->WhaleDirY );
        App_Printf( "PosY: %d\r\n", ptCarXYR->WhalePosY );
        App_Printf( "DirR: %d\r\n", ptCarXYR->WhaleDirR );
        App_Printf( "PosR: %d\r\n", ptCarXYR->WhalePosR );
        App_Printf( "-----Flag_ChangeCarPar-----\r\n" );
    }
}
