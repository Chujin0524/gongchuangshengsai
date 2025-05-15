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
    uint8_t RotateSeq = 1;  //1-顺时针  0-逆时针
    static uint8_t ucCycle = 0; //运行圈数
    uint8_t ucaQRScan_buff[32] = {0};    //二维码接收缓存指针，字符串型
    static uint8_t ucaGrabSequence[6] = {3, 1, 2, 3, 2, 1};
    static uint8_t ucObstacle = 0;
    //    uint8_t ucaGrabSequence[6] = {0};    //抓取顺序，整数型

    static uint8_t ucSendFlag = 0;        //前处理
    uint32_t pulNotificationValue = 0;       //任务通知值
    
    static uint8_t TickFlag = 0;       //计数标志位
    
    static StateTimer run_timer;

    switch (s_ucStatus)
    {
        case STATUS_Idle:
            //            LED_R_TOGGLE();

            vTaskDelay(pdMS_TO_TICKS(500));
            break;

        case STATUS_Start_2_Qr:
            //前处理
            Timer_Start(&run_timer);  // 开始计时

            App_Printf("ENTER STATUS_Start_2_Qr\r\n");
            APP_Handle_Run();

            //状态动作
            APP_CHASSIS_Status(STATUS_Start_2_Qr);

            //后处理
            xTaskNotifyWait(0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY);
            s_ucStatus = STATUS_QrRead;
//            s_ucStatus = STATUS_Qr_2_RawArea;
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
                printf("GrabSequence String : %s\r\n", ucaQRScan_buff);  //打印出来了GrabSequence String : 123+231
                uint8_t j = 0;

                for (uint8_t i = 0; i < 6; i++)
                {
                    if (ucaQRScan_buff[j] == '+') j++;

                    ucaGrabSequence[i] = ucaQRScan_buff[j++] - '0';   //将二维码接收缓存转换为整型的抓取顺序
                }
                ucObstacle = ucaGrabSequence[0];

                SCREEN_Dispaly(ucaQRScan_buff);       //屏幕显示
                xTaskNotifyGive(Task_MainHandle);

                //后处理
                xTaskNotifyWait(0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY);
                s_ucStatus = STATUS_Qr_2_RawArea;
//                s_ucStatus = STATUS_Idle;
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
//            s_ucStatus = STATUS_RawArea_2_ProFirArea;
            //            s_ucStatus = STATUS_Idle;
            App_Printf("EXIT STATUS_Qr_2_RawArea\r\n");
            break;

        case STATUS_RawAreaCal:
            //前处理
            App_Printf("ENTER STATUS_RawAreaCal\r\n");
            APP_Handle_Stay(BlobPlace);
            vTaskDelay(pdMS_TO_TICKS(300));

            //状态动作
            APP_MAIXCAM_Cal(BLOB, COLOR_ALL, BlobCalFlag, MoveCar);
        
            //后处理
            xTaskNotifyWait(0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY);
            TickFlag = APP_MAIXCAM_GetTickFlag();
            s_ucStatus = STATUS_RawAreaGrab;
//            s_ucStatus = STATUS_RawArea_2_ProFirArea;
//            s_ucStatus = STATUS_Idle;
            App_Printf("EXIT STATUS_RawAreaCal-%d\r\n", TickFlag);
            Timer_GetDuration(&run_timer);  // 更新计时
            App_Printf("RunTime: %s\n", run_timer.duration_str);
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
            APP_Handle_PickRawArea(ucaGrabSequence[0+3*ucCycle], BlobColorMappingDir[ucaGrabSequence[0+3*ucCycle]]);
            xTaskNotifyWait(0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY);

            if (TickFlag == 1)    //如果车校准的快，先抓一个再抓两个
            {
            App_Printf("ENTER TickFlag == 1\r\n");
                vTaskDelay(pdMS_TO_TICKS(200));   //待调
                //状态动作3 等待转盘转完并停止一圈
                APP_MAIXCAM_Cal(BLOB, COLOR_ALL, BlobGrabFlag, MoveCar);
                xTaskNotifyWait(0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY);
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

                if (RotateSeq == 1 || RotateSeq == 0) 
                {
                    // 处理两个需要更新的位置（1+3*ucCycle 和 2+3*ucCycle）
                    for (int i = 1; i <= 2; i++) 
                    {
                        int idx = ucaGrabSequence[i + 3 * ucCycle];
                        int currentDir = BlobColorMappingDir[idx];
                        
                        if (RotateSeq == 1) 
                        {
                            // 顺时针旋转: LEFT→RIGHT, MIDDLE→LEFT, RIGHT→MIDDLE
                            BlobColorMappingDir[idx] = (currentDir == LEFT_BLOB) ? RIGHT_BLOB :
                                                      (currentDir == MIDDLE_BLOB) ? LEFT_BLOB :
                                                      (currentDir == RIGHT_BLOB) ? MIDDLE_BLOB : currentDir;
                        } 
                        else 
                        {
                            // 逆时针旋转: LEFT→MIDDLE, MIDDLE→RIGHT, RIGHT→LEFT
                            BlobColorMappingDir[idx] = (currentDir == LEFT_BLOB) ? MIDDLE_BLOB :
                                                      (currentDir == MIDDLE_BLOB) ? RIGHT_BLOB :
                                                      (currentDir == RIGHT_BLOB) ? LEFT_BLOB : currentDir;
                        }
                    }
                }
                APP_Handle_PickRawArea(ucaGrabSequence[1+3*ucCycle], BlobColorMappingDir[ucaGrabSequence[1+3*ucCycle]]);
                xTaskNotifyWait(0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY);
                APP_Handle_PickRawArea(ucaGrabSequence[2+3*ucCycle], BlobColorMappingDir[ucaGrabSequence[2+3*ucCycle]]);
                xTaskNotifyWait(0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY);
            }
            else if (TickFlag == 2)    //如果车校准的慢，先抓两个再抓一个
            {
            App_Printf("ENTER TickFlag == 2\r\n");
                APP_Handle_PickRawArea(ucaGrabSequence[1+3*ucCycle], BlobColorMappingDir[ucaGrabSequence[1+3*ucCycle]]);
                xTaskNotifyWait(0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY);
                
                if (RotateSeq == 1)
                {
                    if (BlobColorMappingDir[ucaGrabSequence[2+3*ucCycle]] == LEFT_BLOB) BlobColorMappingDir[ucaGrabSequence[2+3*ucCycle]] = RIGHT_BLOB;
                    else if (BlobColorMappingDir[ucaGrabSequence[2+3*ucCycle]] == RIGHT_BLOB) BlobColorMappingDir[ucaGrabSequence[2+3*ucCycle]] = MIDDLE_BLOB;
                    else if (BlobColorMappingDir[ucaGrabSequence[2+3*ucCycle]] == MIDDLE_BLOB) BlobColorMappingDir[ucaGrabSequence[2+3*ucCycle]] = LEFT_BLOB;
                }
                else if (RotateSeq == 0)
                {
                    if (BlobColorMappingDir[ucaGrabSequence[2+3*ucCycle]] == LEFT_BLOB) BlobColorMappingDir[ucaGrabSequence[2+3*ucCycle]] = MIDDLE_BLOB;
                    else if (BlobColorMappingDir[ucaGrabSequence[2+3*ucCycle]] == RIGHT_BLOB) BlobColorMappingDir[ucaGrabSequence[2+3*ucCycle]] = LEFT_BLOB;
                    else if (BlobColorMappingDir[ucaGrabSequence[2+3*ucCycle]] == MIDDLE_BLOB) BlobColorMappingDir[ucaGrabSequence[2+3*ucCycle]] = RIGHT_BLOB;
                }
                vTaskDelay(pdMS_TO_TICKS(2100));   //待调
                APP_Handle_PickRawArea(ucaGrabSequence[2+3*ucCycle], BlobColorMappingDir[ucaGrabSequence[2+3*ucCycle]]);
                xTaskNotifyWait(0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY);
            }


//            APP_Handle_PickRawArea(RED, LEFT_BLOB);
//            xTaskNotifyWait(0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY);
//            APP_Handle_PickRawArea(GREEN, MIDDLE_BLOB);
//            xTaskNotifyWait(0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY);
//            APP_Handle_PickRawArea(BLUE, RIGHT_BLOB);
//            xTaskNotifyWait(0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY);

            //后处理
            App_Printf("EXIT STATUS_RawAreaGrab\r\n");
            
            if (ucObstacle != 2) s_ucStatus = STATUS_RawArea_2_ProFirArea_2;
            else s_ucStatus = STATUS_RawArea_2_ProFirArea_1;
//            s_ucStatus = STATUS_Idle;

            Timer_GetDuration(&run_timer);  // 更新计时
            App_Printf("RunTime: %s\n", run_timer.duration_str);
            break;
        }
        
        case STATUS_RawArea_2_ProFirArea_2:
            //前处理
            App_Printf("ENTER STATUS_RawArea_2_ProFirArea_2\r\n");
            APP_Handle_Run();

            //状态动作
            APP_CHASSIS_Status(STATUS_RawArea_2_ProFirArea_2);

            //后处理
            xTaskNotifyWait(0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY);
            s_ucStatus = STATUS_ProFirAreaCalAndPlace;
//            s_ucStatus = STATUS_ProFirArea_2_ProSecArea;
//            s_ucStatus = STATUS_Idle;
            App_Printf("EXIT STATUS_RawArea_2_ProFirArea_2\r\n");
            break;
        
        case STATUS_RawArea_2_ProFirArea_1:
            //前处理
            App_Printf("ENTER STATUS_RawArea_2_ProFirArea_1\r\n");
            APP_Handle_Run();

            //状态动作
            APP_CHASSIS_Status(STATUS_RawArea_2_ProFirArea_1);

            //后处理
            xTaskNotifyWait(0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY);
            s_ucStatus = STATUS_ProFirAreaCalAndPlace;
//            s_ucStatus = STATUS_ProFirArea_2_ProSecArea;
//            s_ucStatus = STATUS_Idle;
            App_Printf("EXIT STATUS_RawArea_2_ProFirArea_1\r\n");
            break;

        case STATUS_ProFirAreaCalAndPlace:
            //前处理
            App_Printf("ENTER STATUS_ProFirAreaCal\r\n");
            APP_Handle_Stay(CirclePlace);

            APP_SERVO_Plate(ucaGrabSequence[0 + 3 * ucCycle]);
//            APP_SERVO_Plate(GREEN);
        
            //状态动作 粗校准
            if (ucCycle == 0)      
            {
                APP_MAIXCAM_Cal(CIRCLE, GREEN, CircleCalFlag, MoveCar);       //识别绿色圆环进行校准，使小车移动
                xTaskNotifyWait(0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY);
                
                //状态动作 细校准
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, 500, 220, 8500, 1, 0);
                APP_MOTOR_ZDT_WaitAck(MOTOR_Y, MOTOR_Y_Timeout);
                APP_MAIXCAM_Cal(CIRCLE, GREEN, CircleCalFlag, MoveCar);       //识别绿色圆环进行校准，使小车移动
                //后处理1，等待小车粗校准完成绿色色环
                xTaskNotifyWait(0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY);

                for (uint8_t i = 0; i < 3; i++)
                {
                    if (ucaGrabSequence[i + 3 * ucCycle] != GREEN)
                    {
                        //细校准
                        APP_MAIXCAM_Cal(CIRCLE, ucaGrabSequence[i + 3 * ucCycle], CircleCalFlag, MoveXR);       //识别圆环进行校准，使小车移动
                        xTaskNotifyWait(0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY);

                    }
                    else
                    {
                        APP_MOTOR_ZDT_Move_P(MOTOR_R, MOTOR_R_Direction_CCR,     MOTOR_R_Vel, MOTOR_R_Acc, 3700, 1, 0);
                        APP_MOTOR_ZDT_Move_P(MOTOR_X, MOTOR_X_Direction_FORWARD, MOTOR_X_Vel, MOTOR_X_Acc, 0, 1, 0);
                        APP_MOTOR_ZDT_WaitAck(MOTOR_R, MOTOR_R_Timeout);
                        APP_MOTOR_ZDT_WaitAck(MOTOR_X, MOTOR_X_Timeout);
                    }
                    //根据校准之后的X、R，放置一个物块
                    if (i < 2) APP_Handle_Place_One(ucaGrabSequence[i + 3 * ucCycle], ucaGrabSequence[i+1 + 3 * ucCycle]);
                    else APP_Handle_Place_One(ucaGrabSequence[i + 3 * ucCycle], 0);
                    xTaskNotifyWait(0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY);
                }
                
                if (ucObstacle != 2) s_ucStatus = STATUS_ProFirArea_2_RawArea_2;
                else s_ucStatus = STATUS_ProFirArea_2_RawArea_1;
            }
            else if (ucCycle == 1) 
            {
                APP_MAIXCAM_Cal(BLOB, GREEN, CircleCalFlag, MoveCar);       //识别绿色圆环进行校准，使小车移动
                xTaskNotifyWait(0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY);

                //放三个
                APP_Handle_Place_All(0, ucaGrabSequence[0 + 3 * ucCycle], ucaGrabSequence[1 + 3 * ucCycle], ucaGrabSequence[2 + 3 * ucCycle]);
                xTaskNotifyWait(0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY);

                //状态动作
                APP_Handle_Pick_All(ucaGrabSequence[0 + 3 * 0], ucaGrabSequence[1 + 3 * 0], ucaGrabSequence[2 + 3 * 0]);
                xTaskNotifyWait(0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY);

                if (ucObstacle != 2) s_ucStatus = STATUS_ProFirArea_2_ProSecArea_2;
                else s_ucStatus = STATUS_ProFirArea_2_ProSecArea_3;
            }
            
//            s_ucStatus = STATUS_ProFirArea_2_ProSecArea;
//            s_ucStatus = STATUS_Idle;
            App_Printf("EXIT STATUS_ProFirAreaCalAndPlace\r\n");
            Timer_GetDuration(&run_timer);  // 更新计时
            App_Printf("RunTime: %s\n", run_timer.duration_str);
            break;

        case STATUS_ProFirArea_2_RawArea_2:
            //前处理
            App_Printf("ENTER STATUS_ProFirArea_2_RawArea_2\r\n");
            APP_Handle_Run();

            //状态动作
            APP_CHASSIS_Status(STATUS_ProFirArea_2_RawArea_2);

            //后处理
            xTaskNotifyWait(0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY);
            s_ucStatus = STATUS_RawAreaCal;
            //            s_ucStatus = STATUS_Idle;
            App_Printf("EXIT STATUS_ProFirArea_2_RawArea_2\r\n");
            Timer_GetDuration(&run_timer);  // 更新计时
            App_Printf("RunTime: %s\n", run_timer.duration_str);
            break;

        case STATUS_ProFirArea_2_RawArea_1:
            //前处理
            App_Printf("ENTER STATUS_ProFirArea_2_RawArea_1\r\n");
            APP_Handle_Run();

            //状态动作
            APP_CHASSIS_Status(STATUS_ProFirArea_2_RawArea_1);

            //后处理
            xTaskNotifyWait(0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY);
            s_ucStatus = STATUS_RawAreaCal;
            //            s_ucStatus = STATUS_Idle;
            App_Printf("EXIT STATUS_ProFirArea_2_RawArea_1\r\n");
            Timer_GetDuration(&run_timer);  // 更新计时
            App_Printf("RunTime: %s\n", run_timer.duration_str);
            break;

        case STATUS_ProFirArea_2_ProSecArea_2:
            //前处理
            App_Printf("ENTER STATUS_ProFirArea_2_ProSecArea_2\r\n");
            APP_Handle_Run();

            //状态动作
            APP_CHASSIS_Status(STATUS_ProFirArea_2_ProSecArea_2);

            //后处理
            xTaskNotifyWait(0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY);
            s_ucStatus = STATUS_ProSecAreaCal;
            //            s_ucStatus = STATUS_Idle;
            App_Printf("EXIT STATUS_ProFirArea_2_ProSecArea_2\r\n");
            Timer_GetDuration(&run_timer);  // 更新计时
            App_Printf("RunTime: %s\n", run_timer.duration_str);
            break;

        case STATUS_ProFirArea_2_ProSecArea_3:
            //前处理
            App_Printf("ENTER STATUS_ProFirArea_2_ProSecArea_3\r\n");
            APP_Handle_Run();

            //状态动作
            APP_CHASSIS_Status(STATUS_ProFirArea_2_ProSecArea_3);

            //后处理
            xTaskNotifyWait(0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY);
            s_ucStatus = STATUS_ProSecAreaCal;
            //            s_ucStatus = STATUS_Idle;
            App_Printf("EXIT STATUS_ProFirArea_2_ProSecArea_3\r\n");
            Timer_GetDuration(&run_timer);  // 更新计时
            App_Printf("RunTime: %s\n", run_timer.duration_str);
            break;

        case STATUS_ProSecAreaCal:
            //前处理
            App_Printf("ENTER STATUS_ProSecAreaCal\r\n");
            APP_Handle_Stay(CirclePlace);

            APP_SERVO_Plate(ucaGrabSequence[0 + 3 * 0]);

            //状态动作 粗校准
//            if (ucCycle == 0)      
            {
                APP_MAIXCAM_Cal(CIRCLE, GREEN, CircleCalFlag, MoveCar);       //识别绿色圆环进行校准，使小车移动
                xTaskNotifyWait(0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY);
                
                //状态动作 细校准
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, 500, 220, 8500, 1, 0);
                APP_MOTOR_ZDT_WaitAck(MOTOR_Y, MOTOR_Y_Timeout);
                APP_MAIXCAM_Cal(CIRCLE, GREEN, CircleCalFlag, MoveCar);       //识别绿色圆环进行校准，使小车移动
                //后处理1，等待小车粗校准完成绿色色环
                xTaskNotifyWait(0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY);
            }

            s_ucStatus = STATUS_ProSecAreaPlace;

//            if (ucCycle == 0) s_ucStatus = STATUS_ProSecArea_2_RawArea;
//            else if (ucCycle == 1) s_ucStatus = STATUS_ProSecArea_2_Start;

//            APP_Handle_Place_One(GREEN, 0);
//            xTaskNotifyWait(0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY);
//            
//            s_ucStatus = STATUS_Idle;
            App_Printf("EXIT STATUS_ProSecAreaCal\r\n");
            Timer_GetDuration(&run_timer);  // 更新计时
            App_Printf("RunTime: %s\n", run_timer.duration_str);
            break;

        case STATUS_ProSecAreaPlace:
            //前处理
            App_Printf("ENTER STATUS_ProSecAreaPlace\r\n");

            //后处理2，每校准一个色环进行一次放置
            for (uint8_t i = 0; i < 3; i++)
            {
                if (ucaGrabSequence[i + 3 * 0] != GREEN)
                {
                    //细校准
                    APP_MAIXCAM_Cal(CIRCLE, ucaGrabSequence[i + 3 * 0], CircleCalFlag, MoveXR);       //识别圆环进行校准，使小车移动
                    xTaskNotifyWait(0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY);

                }
                else
                {
                    APP_MOTOR_ZDT_Move_P(MOTOR_R, MOTOR_R_Direction_CCR,     MOTOR_R_Vel, MOTOR_R_Acc, 3700, 1, 0);
                    APP_MOTOR_ZDT_Move_P(MOTOR_X, MOTOR_X_Direction_FORWARD, MOTOR_X_Vel, MOTOR_X_Acc, 0, 1, 0);
                    APP_MOTOR_ZDT_WaitAck(MOTOR_R, MOTOR_R_Timeout);
                    APP_MOTOR_ZDT_WaitAck(MOTOR_X, MOTOR_X_Timeout);
                }
                //根据校准之后的X、R，放置一个物块
                if (i < 2) APP_Handle_Place_One(ucaGrabSequence[i + 3 * 0], ucaGrabSequence[i+1 + 3 * 0]);
                else APP_Handle_Place_One(ucaGrabSequence[i + 3 * 0], 0);
                xTaskNotifyWait(0x00000000UL, 0xFFFFFFFFUL, &pulNotificationValue, portMAX_DELAY);
            }

//            s_ucStatus = STATUS_Idle;
            s_ucStatus = STATUS_ProSecArea_2_Start;
            App_Printf("EXIT STATUS_ProSecAreaPlace\r\n");

            Timer_GetDuration(&run_timer);  // 更新计时
            App_Printf("RunTime: %s\n", run_timer.duration_str);
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
        
            //显示时间
            xEventGroupClearBits(EventGroups_CarHandle, EventGroupsCarShow_7);
            Timer_GetDuration(&run_timer);  // 更新计时
            App_Printf("RunTime: %s\n", run_timer.duration_str);

            break;

        default:
            /* 其它的状态不处理 */
            break;
    }
}

// 开始计时
void Timer_Start(StateTimer *timer) 
{
    timer->start_time = HAL_GetTick();
}

// 获取当前计时（更新 duration_str）
void Timer_GetDuration(StateTimer *timer) 
{
    uint32_t current_time = HAL_GetTick();
    uint32_t duration_ms = current_time - timer->start_time;
    
    uint32_t seconds = duration_ms / 1000;
    uint32_t minutes = seconds / 60;
    uint32_t remaining_seconds = seconds % 60;
    
    sprintf(timer->duration_str, "%d min %d s\r\n", minutes, remaining_seconds);
}


