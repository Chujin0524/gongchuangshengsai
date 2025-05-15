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
void APP_MainStatusChange(void)
{
    STATUS_E ucStatus = STATUS_Idle;
    BaseType_t xResult = pdFAIL;

    xResult = xQueueReceive(Queue_StatusHandle, (void*)&ucStatus, 0);     //接收状态参数

    if (xResult == pdPASS)     //如果队列里有消息，则切换状态为队列值
    {
        s_ucStatus = ucStatus;
        App_Printf("s_ucStatus : %d\r\n", s_ucStatus);
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
void APP_MainStatusRunning(void)
{
    static uint8_t ucCycle = 0; //运行圈数
    uint8_t ucaQRScan_buff[32] = {0};    //二维码接收缓存指针，字符串型
    uint8_t ucaGrabSequence[6] = {1, 2, 3, 2, 1, 3};
    //    uint8_t ucaGrabSequence[6] = {0};    //抓取顺序，整数型

    static uint8_t ucSendFlag = 0;        //前处理
    uint32_t pulNotificationValue = 0;       //任务通知值

    switch (s_ucStatus)
    {
        case STATUS_Idle:
            //            LED_R_TOGGLE();

            vTaskDelay(pdMS_TO_TICKS(500));
            break;

        case STATUS_Start_2_Qr:
            //前处理
            App_Printf("ENTER STATUS_Start_2_Qr\r\n");
            APP_Handle_Run();

            //状态动作
            APP_CHASSIS_Status(STATUS_Start_2_Qr);

            //后处理
            xTaskNotifyWait(0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY);
            s_ucStatus = STATUS_QrRead;
            s_ucStatus = STATUS_Qr_2_RawArea;
            //            s_ucStatus = STATUS_Idle;
            App_Printf("EXIT STATUS_Start_2_Qr\r\n");
            break;

        case STATUS_QrRead:

            //前处理
            if (ucSendFlag == 0)
            {
                App_Printf("ENTER STATUS_QrRead\r\n");
                ucSendFlag = 1;
            }

            if (QR_SendAndRead(ucaQRScan_buff))    //二维码读取抓取顺序
            {
                printf("GrabSequence String : %s\r\n", ucaQRScan_buff);
                uint8_t j = 0;

                for (uint8_t i = 0; i < 6; i++)
                {
                    if (ucaQRScan_buff[j] == '+') j++;

                    ucaGrabSequence[i] = ucaQRScan_buff[j++] - '0';   //将二维码接收缓存转换为整型的抓取顺序
                }

                SCREEN_Dispaly(ucaQRScan_buff);       //屏幕显示
                xTaskNotifyGive(Task_MainHandle);

                //后处理
                xTaskNotifyWait(0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY);
                s_ucStatus = STATUS_Qr_2_RawArea;
                //            s_ucStatus = STATUS_Idle;
                App_Printf("EXIT STATUS_QrRead\r\n");
                ucSendFlag = 0;
            }

            break;

        case STATUS_Qr_2_RawArea:
            //前处理
            App_Printf("ENTER STATUS_Qr_2_RawArea\r\n");
            APP_Handle_Run();

            //状态动作
            APP_CHASSIS_Status(STATUS_Qr_2_RawArea);

            //后处理
            xTaskNotifyWait(0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY);
            s_ucStatus = STATUS_RawAreaCal;
            //            s_ucStatus = STATUS_Idle;
            App_Printf("EXIT STATUS_Qr_2_RawArea\r\n");
            break;

        case STATUS_RawAreaCal:
            //前处理
            App_Printf("ENTER STATUS_RawAreaCal\r\n");
            APP_Handle_Stay();

            //状态动作
            APP_MAIXCAM_Cal(BLOB, COLOR_ALL, BlobCalFlag, MoveCar);

            //后处理
            xTaskNotifyWait(0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY);
            s_ucStatus = STATUS_RawAreaGrab;
            //            s_ucStatus = STATUS_RawArea_2_ProFirArea;
            //        s_ucStatus = STATUS_Idle;
            App_Printf("EXIT STATUS_RawAreaCal\r\n");
            break;

        case STATUS_RawAreaGrab:
        {
            //前处理
            App_Printf("ENTER STATUS_RawAreaGrab\r\n");

            //状态动作1 获取颜色位置映射
            uint8_t *BlobDirMappingColor = APP_MAIXCAM_GetDirMappingColor();
            uint8_t BlobColorMappingDir[4];
            for(uint8_t color = RED; color < COLOR_COUNT; color++)
            {
                for(uint8_t dir = LEFT_BLOB; dir < DIR_COUNT; dir++)
                {
                    if(BlobDirMappingColor[dir] == color)
                    {
                        BlobColorMappingDir[color] = dir;
                        break;  // 找到第一个匹配的就退出内层循环
                    }
                }
            }
            App_Printf("BlobDirMappingColor : %d%d%d\r\n", BlobDirMappingColor[LEFT_BLOB], BlobDirMappingColor[MIDDLE_BLOB], BlobDirMappingColor[RIGHT_BLOB] );
            App_Printf("BlobColorMappingDir : %d%d%d\r\n", BlobColorMappingDir[RED], BlobColorMappingDir[GREEN], BlobColorMappingDir[BLUE] );
            
            //状态动作2 根据颜色及位置进行抓取
//            APP_Handle_PickRawArea(ucaGrabSequence[0+3*ucCycle], BlobColorMappingDir[ucaGrabSequence[0+3*ucCycle]]);
//            xTaskNotifyWait(0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY);
//            APP_Handle_PickRawArea(ucaGrabSequence[1+3*ucCycle], BlobColorMappingDir[ucaGrabSequence[1+3*ucCycle]]);
//            xTaskNotifyWait(0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY);
//            APP_Handle_PickRawArea(ucaGrabSequence[2+3*ucCycle], BlobColorMappingDir[ucaGrabSequence[2+3*ucCycle]]);
//            xTaskNotifyWait(0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY);

            //后处理
            App_Printf("EXIT STATUS_RawAreaGrab\r\n");
            s_ucStatus = STATUS_RawArea_2_ProFirArea;
            s_ucStatus = STATUS_Idle;

            break;
        }
        
        case STATUS_RawArea_2_ProFirArea:
            //前处理
            App_Printf("ENTER STATUS_RawArea_2_ProFirArea\r\n");
            APP_Handle_Run();

            //状态动作
            APP_CHASSIS_Status(STATUS_RawArea_2_ProFirArea);

            //后处理
            xTaskNotifyWait(0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY);
            s_ucStatus = STATUS_ProFirAreaCalAndPlace;
            //            s_ucStatus = STATUS_Idle;
            App_Printf("EXIT STATUS_RawArea_2_ProFirArea\r\n");
            break;

        case STATUS_ProFirAreaCalAndPlace:
            //前处理
            App_Printf("ENTER STATUS_ProFirAreaCal\r\n");
            APP_Handle_Stay();

            //状态动作 粗校准
            APP_MAIXCAM_Cal(CIRCLE, GREEN, CAL, MoveCar);       //识别绿色圆环进行校准，使小车移动
            //后处理1，等待小车粗校准完成绿色色环
            xTaskNotifyWait(0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY);

            //降低Y导轨，进行细校准
            APP_MOTOR_ZDT_Move_P(MOTOR_Y, 0,     300, 100, 7500, 1, 0);
            APP_MOTOR_ZDT_WaitAck();

            //状态动作 细校准
            APP_MAIXCAM_Cal(CIRCLE, GREEN, CAL, MoveCar);       //识别绿色圆环进行校准，使小车移动
            //后处理2，等待小车细校准完成绿色色环
            xTaskNotifyWait(0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY);

            //后处理2，每校准一个色环进行一次放置
            for (uint8_t i = 0; i < 3; i++)
            {
                //降低Y导轨，进行细校准
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, 0,     300, 100, 7500, 1, 0);
                APP_MOTOR_ZDT_WaitAck();

                //识别色环进行校准，使X、R电机移动
                APP_MAIXCAM_Cal(CIRCLE, ucaGrabSequence[i + 3 * ucCycle],  0, MoveXR);
                xTaskNotifyWait(0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY);

                //根据校准之后的X、R，放置一个物块
                APP_Handle_Place_One(ucaGrabSequence[i + 3 * ucCycle]);
                xTaskNotifyWait(0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY);
            }

            //测试用
            //        APP_MAIXCAM_Cal( CIRCLE, GREEN,  0, MoveXR );      //识别绿色圆环进行校准，使X、R电机移动
            //        xTaskNotifyWait( 0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY );
            //        APP_Handle_Place_One(GREEN);
            //        xTaskNotifyWait( 0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY );

            //        APP_MAIXCAM_Cal( CIRCLE, RED,  0, MoveXR );        //识别红色圆环进行校准，使X、R电机移动
            //        xTaskNotifyWait( 0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY );
            //        APP_Handle_Place_One(RED);
            //        xTaskNotifyWait( 0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY );

            //        APP_MAIXCAM_Cal( CIRCLE, BLUE,  0, MoveXR );        //识别红色圆环进行校准，使X、R电机移动
            //        xTaskNotifyWait( 0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY );
            //        APP_Handle_Place_One(BLUE);
            //        xTaskNotifyWait( 0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY );

            s_ucStatus = STATUS_ProFirAreaGrab;
            //            s_ucStatus = STATUS_ProFirArea_2_ProSecArea;
            //            s_ucStatus = STATUS_Idle;
            App_Printf("EXIT STATUS_ProFirAreaCalAndPlace\r\n");
            break;

        case STATUS_ProFirAreaGrab:
            //前处理
            App_Printf("ENTER STATUS_ProFirAreaGrab\r\n");

            //状态动作
            APP_Handle_Pick_All(ucaGrabSequence[0 + 3 * ucCycle], ucaGrabSequence[1 + 3 * ucCycle], ucaGrabSequence[2 + 3 * ucCycle]);

            //后处理
            xTaskNotifyWait(0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY);
            s_ucStatus = STATUS_ProFirArea_2_ProSecArea;
            //        s_ucStatus = STATUS_Idle;
            App_Printf("EXIT STATUS_ProFirAreaGrab\r\n");

            break;

        case STATUS_ProFirArea_2_ProSecArea:
            //前处理
            App_Printf("ENTER STATUS_ProFirArea_2_ProSecArea\r\n");
            APP_Handle_Run();

            //状态动作
            APP_CHASSIS_Status(STATUS_ProFirArea_2_ProSecArea);

            //后处理
            xTaskNotifyWait(0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY);
            s_ucStatus = STATUS_ProSecAreaCal;
            //            s_ucStatus = STATUS_Idle;
            App_Printf("EXIT STATUS_ProFirArea_2_ProSecArea\r\n");
            break;

        case STATUS_ProSecAreaCal:
            //前处理
            App_Printf("ENTER STATUS_ProSecAreaCal\r\n");
            APP_Handle_Stay();

            //状态动作
            APP_MAIXCAM_Cal(CIRCLE, GREEN, CAL, MoveCar);

            //后处理
            xTaskNotifyWait(0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY);

            s_ucStatus = STATUS_ProSecAreaPlace;

            //            if (ucCycle == 0) s_ucStatus = STATUS_ProSecArea_2_RawArea;
            //            else if (ucCycle == 1) s_ucStatus = STATUS_ProSecArea_2_Start;

            //        s_ucStatus = STATUS_Idle;
            App_Printf("EXIT STATUS_ProSecAreaCal\r\n");
            break;

        case STATUS_ProSecAreaPlace:
            //前处理
            App_Printf("ENTER STATUS_ProSecAreaPlace\r\n");

            //状态动作
            APP_Handle_Place_All(ucCycle, ucaGrabSequence[0 + 3 * ucCycle], ucaGrabSequence[1 + 3 * ucCycle], ucaGrabSequence[2 + 3 * ucCycle]);

            //后处理
            xTaskNotifyWait(0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY);

            if (ucCycle == 0) s_ucStatus = STATUS_ProSecArea_2_RawArea;
            else if (ucCycle == 1) s_ucStatus = STATUS_ProSecArea_2_Start;

            //            s_ucStatus = STATUS_Idle;
            App_Printf("EXIT STATUS_ProSecAreaPlace\r\n");

            break;

        case STATUS_ProSecArea_2_RawArea:
            //前处理
            App_Printf("ENTER STATUS_ProSecArea_2_RawArea\r\n");
            APP_Handle_Run();

            //状态动作
            APP_CHASSIS_Status(STATUS_ProSecArea_2_RawArea);

            //后处理
            xTaskNotifyWait(0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY);
            ucCycle = 1;
            s_ucStatus = STATUS_RawAreaCal;
            //            s_ucStatus = STATUS_Idle;
            App_Printf("EXIT STATUS_ProSecArea_2_RawArea\r\n");
            break;

        case STATUS_ProSecArea_2_Start:
            //前处理
            App_Printf("ENTER STATUS_ProSecArea_2_Start\r\n");
            APP_Handle_Run();

            //状态动作
            APP_CHASSIS_Status(STATUS_ProSecArea_2_Start);

            //后处理
            xTaskNotifyWait(0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY);
            s_ucStatus = STATUS_Idle;
            App_Printf("EXIT STATUS_ProSecArea_2_Start\r\n");
            break;

        default:
            /* 其它的状态不处理 */
            break;
    }
}

