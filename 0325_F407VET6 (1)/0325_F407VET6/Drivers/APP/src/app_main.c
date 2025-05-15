#include "main.h"

static STATUS_E s_ucStatus = STATUS_Idle;     //ȫ������״̬

/*
*********************************************************************************************************
*   �� �� ��: APP_MainStatusChange
*   ����˵��: ���ݶ����е�״̬��������ȫ������״̬�����������û���µ�״ֵ̬���򱣳ֵ�ǰ״̬���䡣
*   ��    �Σ���
*   �� �� ֵ: ��
*********************************************************************************************************
*/
void APP_MainStatusChange( void )
{
    STATUS_E ucStatus = STATUS_Idle;
    BaseType_t xResult = pdFAIL;

    xResult = xQueueReceive( Queue_StatusHandle, ( void* )&ucStatus, 0 ); //����״̬����

    if( xResult == pdPASS )    //�������������Ϣ�����л�״̬Ϊ����ֵ
    {
        s_ucStatus = ucStatus;
        App_Printf( "s_ucStatus : %d\r\n", s_ucStatus );
    }
    else
    {
//        s_ucStatus = s_ucStatus;     //���ֲ���
    }
}

/*
*********************************************************************************************************
*   �� �� ��: APP_MainStatusRunning
*   ����˵��: ���ݵ�ǰ��ȫ������״ִ̬����Ӧ���߼���
*   ��    �Σ���
*   �� �� ֵ: ��
*********************************************************************************************************
*/
void APP_MainStatusRunning( void )
{
    static uint8_t ucCycle = 0; //����Ȧ��
    uint8_t ucaQRScan_buff[32] = {0};    //��ά����ջ���ָ�룬�ַ�����
    uint8_t ucaGrabSequence[6] = {1, 2, 3, 2, 1, 3};
//    uint8_t ucaGrabSequence[6] = {0};    //ץȡ˳��������

    static uint8_t ucSendFlag = 0;        //ǰ����
    uint32_t pulNotificationValue = 0;       //����ֵ֪ͨ

    switch( s_ucStatus )
    {
    case STATUS_Idle:
//            LED_R_TOGGLE();

        vTaskDelay( pdMS_TO_TICKS( 500 ) );
        break;

    case STATUS_Start_2_Qr:
        //ǰ����
        App_Printf( "ENTER STATUS_Start_2_Qr\r\n" );
        APP_Handle_Run();

        //״̬����
        APP_CHASSIS_Status( STATUS_Start_2_Qr );

        //����
        xTaskNotifyWait( 0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY );
        s_ucStatus = STATUS_QrRead;
        s_ucStatus = STATUS_Qr_2_RawArea;
//            s_ucStatus = STATUS_Idle;
        App_Printf( "EXIT STATUS_Start_2_Qr\r\n" );
        break;

    case STATUS_QrRead:
        //ǰ����
        if( ucSendFlag == 0 )
        {
            App_Printf( "ENTER STATUS_QrRead\r\n" );
            ucSendFlag = 1;
        }

        if( QR_SendAndRead( ucaQRScan_buff ) ) //��ά���ȡץȡ˳��
        {
            printf( "GrabSequence String : %s\r\n", ucaQRScan_buff );
            uint8_t j = 0;
            for( uint8_t i = 0; i < 6; i++ )
            {
                if( ucaQRScan_buff[j] == '+' ) j++;
                ucaGrabSequence[i] = ucaQRScan_buff[j++] - '0';   //����ά����ջ���ת��Ϊ���͵�ץȡ˳��
            }

            SCREEN_Dispaly( ucaQRScan_buff );     //��Ļ��ʾ
            xTaskNotifyGive( Task_MainHandle );

            //����
            xTaskNotifyWait( 0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY );
            s_ucStatus = STATUS_Qr_2_RawArea;
//            s_ucStatus = STATUS_Idle;
            App_Printf( "EXIT STATUS_QrRead\r\n" );
            ucSendFlag = 0;
        }
        break;

    case STATUS_Qr_2_RawArea:
        //ǰ����
        App_Printf( "ENTER STATUS_Qr_2_RawArea\r\n" );
        APP_Handle_Run();

        //״̬����
        APP_CHASSIS_Status( STATUS_Qr_2_RawArea );

        //����
        xTaskNotifyWait( 0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY );
        s_ucStatus = STATUS_RawAreaCal;
//            s_ucStatus = STATUS_Idle;
        App_Printf( "EXIT STATUS_Qr_2_RawArea\r\n" );
        break;

    case STATUS_RawAreaCal:
        //ǰ����
        App_Printf( "ENTER STATUS_RawAreaCal\r\n" );
        APP_Handle_Stay();

        //״̬����
        APP_MAIXCAM_Cal( BLOB, RED, CAL, MoveCar );

        //����
        xTaskNotifyWait( 0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY );
        s_ucStatus = STATUS_RawAreaGrab;
        s_ucStatus = STATUS_RawArea_2_ProFirArea;
//        s_ucStatus = STATUS_Idle;
        App_Printf( "EXIT STATUS_RawAreaCal\r\n" );
        break;

    case STATUS_RawAreaGrab:
        //ǰ����
        App_Printf( "ENTER STATUS_RawAreaGrab\r\n" );

        //״̬����
        for( uint8_t i = 0; i < 3; i++ )
        {
            //ʶ��ץȡ��i + 3 * ucCycle�����
            APP_MAIXCAM_Cal( BLOB, ucaGrabSequence[i + 3 * ucCycle], GRAB, 0 );
            xTaskNotifyWait( 0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY );
            APP_Handle_PickRawArea( ucaGrabSequence[i + 3 * ucCycle] );
            xTaskNotifyWait( 0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY );
        }

        //����
        App_Printf( "EXIT STATUS_RawAreaGrab\r\n" );
        s_ucStatus = STATUS_RawArea_2_ProFirArea;

        break;

    case STATUS_RawArea_2_ProFirArea:
        //ǰ����
        App_Printf( "ENTER STATUS_RawArea_2_ProFirArea\r\n" );
        APP_Handle_Run();

        //״̬����
        APP_CHASSIS_Status( STATUS_RawArea_2_ProFirArea );

        //����
        xTaskNotifyWait( 0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY );
        s_ucStatus = STATUS_ProFirAreaCalAndPlace;
//            s_ucStatus = STATUS_Idle;
        App_Printf( "EXIT STATUS_RawArea_2_ProFirArea\r\n" );
        break;

    case STATUS_ProFirAreaCalAndPlace:
        //ǰ����
        App_Printf( "ENTER STATUS_ProFirAreaCal\r\n" );
        APP_Handle_Stay();

        //״̬����
        APP_MAIXCAM_Cal( CIRCLE, GREEN, CAL, MoveCar );     //ʶ����ɫԲ������У׼��ʹС���ƶ�

        //����1���ȴ�С��У׼�����ɫɫ��
        xTaskNotifyWait( 0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY );

//        //����2��ÿУ׼һ��ɫ������һ�η���
//        for( uint8_t i = 0; i < 3; i++ )
//        {
//            //ʶ��ɫ������У׼��ʹX��R����ƶ�
//            APP_MAIXCAM_Cal( CIRCLE, ucaGrabSequence[0 + 3 * ucCycle],  0, MoveXR );
//            xTaskNotifyWait( 0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY );

//            //����У׼֮���X��R������һ�����
//            APP_Handle_Place_One( ucaGrabSequence[0 + 3 * ucCycle] );
//            xTaskNotifyWait( 0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY );
//        }

        //������
//        APP_MAIXCAM_Cal( CIRCLE, GREEN,  0, MoveXR );      //ʶ����ɫԲ������У׼��ʹX��R����ƶ�
//        xTaskNotifyWait( 0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY );
//        APP_Handle_Place_One(GREEN);
//        xTaskNotifyWait( 0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY );

//        APP_MAIXCAM_Cal( CIRCLE, RED,  0, MoveXR );        //ʶ���ɫԲ������У׼��ʹX��R����ƶ�
//        xTaskNotifyWait( 0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY );
//        APP_Handle_Place_One(RED);
//        xTaskNotifyWait( 0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY );

//        APP_MAIXCAM_Cal( CIRCLE, BLUE,  0, MoveXR );        //ʶ���ɫԲ������У׼��ʹX��R����ƶ�
//        xTaskNotifyWait( 0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY );
//        APP_Handle_Place_One(BLUE);
//        xTaskNotifyWait( 0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY );

        s_ucStatus = STATUS_ProFirAreaGrab;
        s_ucStatus = STATUS_ProFirArea_2_ProSecArea;
//        s_ucStatus = STATUS_Idle;
        App_Printf( "EXIT STATUS_ProFirAreaCalAndPlace\r\n" );
        break;

    case STATUS_ProFirAreaGrab:
        //ǰ����
        App_Printf( "ENTER STATUS_ProFirAreaGrab\r\n" );

        //״̬����
        APP_Handle_Pick_All( ucaGrabSequence[0 + 3 * ucCycle], ucaGrabSequence[1 + 3 * ucCycle], ucaGrabSequence[2 + 3 * ucCycle] );

        //����
        xTaskNotifyWait( 0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY );
        s_ucStatus = STATUS_ProFirArea_2_ProSecArea;
//        s_ucStatus = STATUS_Idle;
        App_Printf( "EXIT STATUS_ProFirAreaGrab\r\n" );

        break;

    case STATUS_ProFirArea_2_ProSecArea:
        //ǰ����
        App_Printf( "ENTER STATUS_ProFirArea_2_ProSecArea\r\n" );
        APP_Handle_Run();

        //״̬����
        APP_CHASSIS_Status( STATUS_ProFirArea_2_ProSecArea );

        //����
        xTaskNotifyWait( 0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY );
        s_ucStatus = STATUS_ProSecAreaCal;
//            s_ucStatus = STATUS_Idle;
        App_Printf( "EXIT STATUS_ProFirArea_2_ProSecArea\r\n" );
        break;

    case STATUS_ProSecAreaCal:
        //ǰ����
        App_Printf( "ENTER STATUS_ProSecAreaCal\r\n" );
        APP_Handle_Stay();

        //״̬����
        APP_MAIXCAM_Cal( CIRCLE, GREEN, CAL, MoveCar );

        //����
        xTaskNotifyWait( 0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY );

        s_ucStatus = STATUS_ProSecAreaPlace;
        if( ucCycle == 0 ) s_ucStatus = STATUS_ProSecArea_2_RawArea;
        else if( ucCycle == 1 ) s_ucStatus = STATUS_ProSecArea_2_Start;
//        s_ucStatus = STATUS_Idle;
        App_Printf( "EXIT STATUS_ProSecAreaCal\r\n" );
        break;

    case STATUS_ProSecAreaPlace:
        //ǰ����
        App_Printf( "ENTER STATUS_ProSecAreaPlace\r\n" );

        //״̬����
        APP_Handle_Place_All( ucCycle, ucaGrabSequence[0 + 3 * ucCycle], ucaGrabSequence[1 + 3 * ucCycle], ucaGrabSequence[2 + 3 * ucCycle] );

        //����
        xTaskNotifyWait( 0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY );
        if( ucCycle == 0 ) s_ucStatus = STATUS_ProSecArea_2_RawArea;
        else if( ucCycle == 1 ) s_ucStatus = STATUS_ProSecArea_2_Start;
//            s_ucStatus = STATUS_Idle;
        App_Printf( "EXIT STATUS_ProSecAreaPlace\r\n" );

        break;

    case STATUS_ProSecArea_2_RawArea:
        //ǰ����
        App_Printf( "ENTER STATUS_ProSecArea_2_RawArea\r\n" );
        APP_Handle_Run();

        //״̬����
        APP_CHASSIS_Status( STATUS_ProSecArea_2_RawArea );

        //����
        xTaskNotifyWait( 0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY );
        ucCycle = 1;
        s_ucStatus = STATUS_RawAreaCal;
//            s_ucStatus = STATUS_Idle;
        App_Printf( "EXIT STATUS_ProSecArea_2_RawArea\r\n" );
        break;

    case STATUS_ProSecArea_2_Start:
        //ǰ����
        App_Printf( "ENTER STATUS_ProSecArea_2_Start\r\n" );
        APP_Handle_Run();

        //״̬����
        APP_CHASSIS_Status( STATUS_ProSecArea_2_Start );

        //����
        xTaskNotifyWait( 0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY );
        s_ucStatus = STATUS_Idle;
        App_Printf( "EXIT STATUS_ProSecArea_2_Start\r\n" );
        break;

    default:
        /* ������״̬������ */
        break;
    }
}

