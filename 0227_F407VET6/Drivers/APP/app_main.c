#include "main.h"

static STATUS_E s_ucStatus = STATUS_Idle;     //全局运行状态

/*
*********************************************************************************************************
*   函 数 名: APP_MainStatusChange
*   功能说明: 根据队列中的状态参数更新全局运行状态。如果队列中没有新的状态值，则保持当前状态不变。
*   形    参：无
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_MainStatusChange( void )
{
    STATUS_E ucStatus = STATUS_Idle;
    BaseType_t xResult = pdFAIL;

    xResult = xQueueReceive( Queue_StatusHandle, ( void* )&ucStatus, 0 ); //接收状态参数

    if( xResult == pdPASS )    //如果队列里有消息，则切换状态为队列值
    {
        s_ucStatus = ucStatus;
        App_Printf( "s_ucStatus : %d\r\n", s_ucStatus ); ////////////测试通过////////////
    }
    else
    {
//        s_ucStatus = s_ucStatus;     //保持不变
    }
}

/*
*********************************************************************************************************
*   函 数 名: APP_MainStatusRunning
*   功能说明: 根据当前的全局运行状态执行相应的逻辑。
*   形    参：无
*   返 回 值: 无
*********************************************************************************************************
*/
void APP_MainStatusRunning( void )
{
    static uint8_t ucCycle = 0; //运行圈数
    uint8_t ucaQRScan_buff[32] = {0};    //二维码接收缓存指针，字符串型
    uint8_t ucaGrabSequence[6] = {1, 2, 3, 2, 1, 3};
//    uint8_t ucaGrabSequence[6] = {0};    //抓取顺序，整数型

    static uint8_t ucSendFlag = 0;        //前处理
    uint32_t pulNotificationValue = 0;       //任务通知值

    uint8_t ucStatusIndex = 0;  //物块索引

    switch( s_ucStatus )
    {
    case STATUS_Idle:
//            LED_R_TOGGLE();

        vTaskDelay( pdMS_TO_TICKS( 500 ) );
        break;

    case STATUS_Start_2_Qr:
        //前处理
        App_Printf( "ENTER STATUS_Start_2_Qr\r\n" );

        //状态动作
        APP_MOTOR_2006_WHALE_Status( STATUS_Start_2_Qr );

        //后处理
        xTaskNotifyWait( 0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY );
        s_ucStatus = STATUS_QrRead;
//            s_ucStatus = STATUS_Idle;
        App_Printf( "EXIT STATUS_Start_2_Qr\r\n" );
        break;

    case STATUS_QrRead:
        //前处理
        if (ucSendFlag == 0)
        {
            App_Printf( "ENTER STATUS_QrRead\r\n" );
            ucSendFlag = 1;
        }

        if( QR_SendAndRead( ucaQRScan_buff ) ) //二维码读取抓取顺序
        {
            printf( "GrabSequence String : %s\r\n", ucaQRScan_buff );
            uint8_t j = 0;
            for( uint8_t i = 0; i < 6; i++ )
            {
                if( ucaQRScan_buff[j] == '+' ) j++;
                ucaGrabSequence[i] = ucaQRScan_buff[j++] - '0';   //将二维码接收缓存转换为整型的抓取顺序
            }

            SCREEN_Dispaly( ucaQRScan_buff );     //屏幕显示
            xTaskNotifyGive(Task_MainHandle);
            
            //后处理
            xTaskNotifyWait( 0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY );
            s_ucStatus = STATUS_Qr_2_RawArea;
//            s_ucStatus = STATUS_Idle;
            App_Printf( "EXIT STATUS_QrRead\r\n" );
            ucSendFlag = 0;
        }
        break;

    case STATUS_Qr_2_RawArea:
        //前处理
        App_Printf( "ENTER STATUS_Qr_2_RawArea\r\n" );

        //状态动作
        APP_MOTOR_2006_WHALE_Status( STATUS_Qr_2_RawArea );

        //后处理
        xTaskNotifyWait( 0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY );
        s_ucStatus = STATUS_RawAreaCal;
//            s_ucStatus = STATUS_Idle;
        App_Printf( "EXIT STATUS_Qr_2_RawArea\r\n" );
        break;

    case STATUS_RawAreaCal:
        //前处理
        App_Printf( "ENTER STATUS_RawAreaCal\r\n" );

        //状态动作
        APP_MAIXCAM_Cal( BLOB, ucaGrabSequence[2+3*ucCycle], 1 );

        //后处理
        xTaskNotifyWait( 0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY );
        s_ucStatus = STATUS_RawAreaGrab0;
//            s_ucStatus = STATUS_Idle;
        App_Printf( "EXIT STATUS_RawAreaCal\r\n" );
        break;

    case STATUS_RawAreaGrab0:
    case STATUS_RawAreaGrab1:
    case STATUS_RawAreaGrab2:
        //前处理
        ucStatusIndex = s_ucStatus - STATUS_RawAreaGrab0 + 3 * ucCycle; // 计算当前状态对应的索引

        //状态动作
        App_Printf( "ENTER STATUS_RawAreaGrab%d\r\n", ucStatusIndex );

        APP_MAIXCAM_Cal( BLOB, ucaGrabSequence[ucStatusIndex], 0 );

        //后处理1
        xTaskNotifyWait( 0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY );
        App_Printf( "ENTER STATUS_RawAreaGrab%d After\r\n", ucStatusIndex );
        {
            /*
            控制竖直导杆下降，夹爪舵机夹取物块，升起竖直导杆，
            旋转腕处舵机，根据 ucaGrabSequence ，旋转转盘至指定物块颜色，
            降下竖直导杆，松开夹爪，使物块放置到转盘，升起竖直导杆，
            旋转腕处舵机
            */
//            APP_Handle_PickRawArea( ucaGrabSequence[ucStatusIndex] );

            //后处理2
            xTaskNotifyWait( 0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY );
            {
                s_ucStatus = ( s_ucStatus < STATUS_RawAreaGrab2 ) ? ( STATUS_E )( s_ucStatus + 1 ) : STATUS_RawArea_2_ProFirArea;

                if( s_ucStatus == STATUS_RawArea_2_ProFirArea )
                {
                    xEventGroupClearBits( EventGroups_CarHandle, EventGroupsCarMaixCAM_EN_1 );

//                sprintf( maixcam_tx_buf, "mode=0" );
//                MAIXCAM_SendData(( uint8_t* )maixcam_tx_buf, strlen( maixcam_tx_buf ) );
                }
            }
        }

        break;

    case STATUS_RawArea_2_ProFirArea:
        //前处理
        App_Printf( "ENTER STATUS_RawArea_2_ProFirArea\r\n" );

        //状态动作
        APP_MOTOR_2006_WHALE_Status( STATUS_RawArea_2_ProFirArea );

        //后处理
        xTaskNotifyWait( 0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY );
        s_ucStatus = STATUS_ProFirAreaCal;
//            s_ucStatus = STATUS_Idle;
        App_Printf( "EXIT STATUS_RawArea_2_ProFirArea\r\n" );
        break;

    case STATUS_ProFirAreaCal:
        //前处理
        App_Printf( "ENTER STATUS_ProFirAreaCal\r\n" );

        //状态动作
        APP_MAIXCAM_Cal( CIRCLE, GREEN, 1 );

        //后处理
        xTaskNotifyWait( 0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY );
        xEventGroupClearBits( EventGroups_CarHandle, EventGroupsCarMaixCAM_EN_1 );

        s_ucStatus = STATUS_ProFirAreaPlace;
//            s_ucStatus = STATUS_Idle;
        App_Printf( "EXIT STATUS_ProFirAreaCal\r\n" );
        break;

    case STATUS_ProFirAreaPlace:
        //前处理
        App_Printf( "ENTER STATUS_ProFirAreaPlace\r\n" );

        //状态动作
//        APP_Handle_Place_All( 1, ucaGrabSequence[0 + 3 * ucCycle], ucaGrabSequence[1 + 3 * ucCycle], ucaGrabSequence[2 + 3 * ucCycle] );

        //后处理
        xTaskNotifyWait( 0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY );
        s_ucStatus = STATUS_ProFirAreaGrab;
//            s_ucStatus = STATUS_Idle;
        App_Printf( "EXIT STATUS_ProFirAreaPlace\r\n" );

        break;

    case STATUS_ProFirAreaGrab:
        //前处理
        App_Printf( "ENTER STATUS_ProFirAreaGrab\r\n" );

        //状态动作
//        APP_Handle_Pick_All( ucaGrabSequence[0 + 3 * ucCycle], ucaGrabSequence[1 + 3 * ucCycle], ucaGrabSequence[2 + 3 * ucCycle] );

        //后处理
        xTaskNotifyWait( 0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY );
        s_ucStatus = STATUS_ProFirArea_2_ProSecArea;
//            s_ucStatus = STATUS_Idle;
        App_Printf( "EXIT STATUS_ProFirAreaGrab\r\n" );

        break;

    case STATUS_ProFirArea_2_ProSecArea:
        //前处理
        App_Printf( "ENTER STATUS_ProFirArea_2_ProSecArea\r\n" );

        //状态动作
        APP_MOTOR_2006_WHALE_Status( STATUS_ProFirArea_2_ProSecArea );

        //后处理
        xTaskNotifyWait( 0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY );
        s_ucStatus = STATUS_ProSecAreaCal;
//            s_ucStatus = STATUS_Idle;
        App_Printf( "EXIT STATUS_ProFirArea_2_ProSecArea\r\n" );
        break;

    case STATUS_ProSecAreaCal:
        //前处理
        App_Printf( "ENTER STATUS_ProSecAreaCal\r\n" );

        //状态动作
        xEventGroupSetBits( EventGroups_CarHandle, EventGroupsCarMaixCAM_EN_1 );
        APP_MAIXCAM_Cal( CIRCLE, GREEN, 1 );

        //后处理
        xTaskNotifyWait( 0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY );
        xEventGroupClearBits( EventGroups_CarHandle, EventGroupsCarMaixCAM_EN_1 );

        s_ucStatus = STATUS_ProSecAreaPlace;
//            s_ucStatus = STATUS_Idle;
        App_Printf( "EXIT STATUS_ProSecAreaCal\r\n" );
        break;

    case STATUS_ProSecAreaPlace:
        //前处理
        App_Printf( "ENTER STATUS_ProSecAreaPlace\r\n" );

        //状态动作
        APP_Handle_Place_All( ucCycle, ucaGrabSequence[0 + 3 * ucCycle], ucaGrabSequence[1 + 3 * ucCycle], ucaGrabSequence[2 + 3 * ucCycle] );

        //后处理
        xTaskNotifyWait( 0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY );
        if (ucCycle == 0 ) s_ucStatus = STATUS_ProSecArea_2_RawArea;
        else if (ucCycle == 1 ) s_ucStatus = STATUS_ProSecArea_2_Start;
//            s_ucStatus = STATUS_Idle;
        App_Printf( "EXIT STATUS_ProSecAreaPlace\r\n" );

        break;

    case STATUS_ProSecArea_2_RawArea:
        //前处理
        App_Printf( "ENTER STATUS_ProSecArea_2_RawArea\r\n" );

        //状态动作
        APP_MOTOR_2006_WHALE_Status( STATUS_ProSecArea_2_RawArea );

        //后处理
        xTaskNotifyWait( 0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY );
        ucCycle = 1;
        s_ucStatus = STATUS_RawAreaCal;
//            s_ucStatus = STATUS_Idle;
        App_Printf( "EXIT STATUS_ProSecArea_2_RawArea\r\n" );
        break;

    case STATUS_ProSecArea_2_Start:
        //前处理
        App_Printf( "ENTER STATUS_ProSecArea_2_Start\r\n" );

        //状态动作
        APP_MOTOR_2006_WHALE_Status( STATUS_ProSecArea_2_Start );

        //后处理
        xTaskNotifyWait( 0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY );
        s_ucStatus = STATUS_Idle;
        App_Printf( "EXIT STATUS_ProSecArea_2_Start\r\n" );
        break;

    default:
        /* 其它的状态不处理 */
        break;
    }
}

