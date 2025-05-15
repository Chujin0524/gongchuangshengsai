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
    UNUSED(ucFrameLength);

    static char bluetooth_tx_buf[512] = {0};       //�������ͻ���
    App_Printf( "%s\r\n", pucFrameData );          //�����յ����ַ������ͳ�ȥ

    FLAG_E ucFlag = Flag_Idle;
    uint8_t* number_start;

    //�����޸ĵĲ���
    static STATUS_E ucStatus = STATUS_Idle;         //״̬

    static uint8_t chassis_dir = 0;        //���̷���
    static int64_t chassis_dis = 0;        //���̾���

    static uint16_t s_AngleJaw = 0;        //��צ���
    static uint16_t s_AngleWrist = 0;      //�󴦶��
    static uint16_t s_AnglePlate = 0;      //ת�̶��

    static uint8_t s_MotorZDT_ID = 0;      //�������ID
    static uint8_t s_MotorZDT_DIR = 0;     //�������ת��
    static uint32_t s_MotorZDT_POS = 0;    //�����������


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
            xEventGroupClearBits( EventGroups_CarHandle, EventGroupsCarShow_7 );
            i = 1;
        }
        else
        {
            xEventGroupSetBits( EventGroups_CarHandle, EventGroupsCarShow_7 );
            i = 0;
        }


        ucFlag = Flag_Show;
    }
    else if( strncmp(( char* )pucFrameData, "ShowParameter", 13 ) == 0 )
    {
//        App_Printf( "g_fGyro_z : %.3f\r\n", g_fGyro_z );
        App_Printf( "chassis_yaw : %.3f\r\n", chassis_move.chassis_yaw );

        App_Printf( "g_dX : %d\r\n", g_dX);
        App_Printf( "g_dY : %d\r\n", g_dY);
        App_Printf( "g_dR : %d\r\n", g_dR);

        ucFlag = Flag_Show;
    }

    // ��ͣ
    else if( strncmp(( char* )pucFrameData, "Stop", 4 ) == 0 )
    {
        vTaskSuspend( Task_ChassisControlHandle );
        set_motor_current( 0, 0, 0, 0 );
        ucFlag = Flag_Stop;
    }
    //����
    else if( strncmp(( char* )pucFrameData, "Start", 5 ) == 0 )
    {
        for (uint16_t i = 0; i < 1; i++)
        {
            set_motor_current( -300, -300, 300, 300 );
            vTaskDelay(10);
        }
        ucFlag = Flag_Start;
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


    //������ʻ
    else if( strncmp(( char* )pucFrameData, "WhaleDir=", 9 ) == 0 )
    {
        number_start = pucFrameData + 9;
        chassis_dir = atoi(( char* )number_start );
    }
    else if( strncmp(( char* )pucFrameData, "WhalePos=", 9 ) == 0 )
    {
        number_start = pucFrameData + 9;
        chassis_dis = atoi(( char* )number_start );

        //����������APP_CHASSIS_MovePID�����ƶ�
        APP_CHASSIS_MovePID(CHASSIS_CALL_NON_BLOCKING, chassis_dir, chassis_dis);
//        APP_CHASSIS_MovePID(CHASSIS_CALL_BLOCKING, chassis_dir, chassis_dis); //������תʱ�ã��ǵ�����תʱ��Ҫʹ�ã����������޷������������޷�����ͣ

        ucFlag = Flag_ChangeCarPar;
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
        APP_MOTOR_ZDT_Move_P( s_MotorZDT_ID, s_MotorZDT_DIR, 800, 0, s_MotorZDT_POS, 1, 0 );
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
                APP_Handle_Place_All( 1, RED, GREEN, BLUE );    //��һ��ȫ�����ò���
                break;
            case 2:
                APP_Handle_Place_All( 2, RED, GREEN, BLUE );    //�ڶ���ȫ�����ò���
                break;
            case 3:
                APP_Handle_Pick_All( 1, 2, 3 );        //ȫ��ץȡ����
                break;
            case 4:
                APP_MAIXCAM_Cal( CIRCLE, RED,   0, MoveXR );        //ʶ����ɫԲ������У׼��ʹX��R����ƶ�
                break;
            case 5:
                APP_MAIXCAM_Cal( CIRCLE, GREEN, 0, MoveXR );        //ʶ���ɫԲ������У׼��ʹX��R����ƶ�
                break;
            case 6:
                APP_MAIXCAM_Cal( CIRCLE, BLUE,  0, MoveXR );        //ʶ����ɫԲ������У׼��ʹX��R����ƶ�
                break;
            case 7:                  //��һ������
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
                APP_Handle_Run();
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
}
