#include "main.h"

static void APP_BLUETOOTH_Parse( uint8_t* pucFrameData, uint8_t ucFrameLength );

void APP_BLUETOOTH_Init( void )
{
    bsp_InitBLUETOOTH( APP_BLUETOOTH_Parse );    //注册解析回调函数
}

void APP_BLUETOOTH_Read( void )
{
    if( BLUETOOTH_ReadData() )  //成功接收一帧数据流，并调用解析回调函数APP_BLUETOOTH_Parse
    {
//        LED_TOGGLE();
          
        App_Printf("RECEIVE\r\n");
    }
}

/*
*********************************************************************************************************
*    函 数 名: APP_BLUETOOTH_Parse
*    功能说明: 蓝牙解析回调函数，用于处理接收到的蓝牙指令。
*    形    参：pucFrameData 接收到的蓝牙数据帧
*             ucFrameLength 数据帧的长度
*    返 回 值: 无
*********************************************************************************************************
*/
static void APP_BLUETOOTH_Parse( uint8_t* pucFrameData, uint8_t ucFrameLength )
{
    static char bluetooth_tx_buf[256] = {0};       //相机发送缓存
    App_Printf("%s\r\n", pucFrameData);            //将接收到的字符串发送出去
    
    FLAG_E ucFlag = Flag_Idle;
    uint8_t *number_start;
    
    //可能修改的参数
    static STATUS_E ucStatus = STATUS_Idle;         //状态
    static CAR_XYR_T s_tCarXYR = {0};               //小车行进参数
    static CAR_XYR_T *ptCarXYR = &s_tCarXYR;
    
    // 急停
    if (strncmp((char*)pucFrameData, "Stop", 4) == 0)
    {
        ucFlag = Flag_Stop;
    }


    // 修改运行状态
    else if (strncmp((char*)pucFrameData, "Status=", 7) == 0) 
    {
        number_start = pucFrameData + 7;
        ucStatus = atoi((char*)number_start);

        xQueueOverwrite(Queue_StatusHandle, &ucStatus);  //向消息队列Queue_StatusHandle发送程序状态
        
        ucFlag = Flag_ChangeStatus;
    }


    //行驶方向
    else if (strncmp((char*)pucFrameData, "WhaleDirX=", 10) == 0) 
    {
        number_start = pucFrameData + 10;
        ptCarXYR->WhaleDirX = atoi((char*)number_start);
    }
    else if (strncmp((char*)pucFrameData, "WhalePosX=", 10) == 0) 
    {
        number_start = pucFrameData + 10;
        ptCarXYR->WhalePosX = atoi((char*)number_start);

        xQueueOverwrite(Queue_MotorHandle, &ptCarXYR);  //向消息队列Queue_MotorHandle发送轮子参数
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

        xQueueOverwrite(Queue_MotorHandle, &ptCarXYR);  //向消息队列Queue_MotorHandle发送轮子参数
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

        xQueueOverwrite(Queue_MotorHandle, &ptCarXYR);  //向消息队列Queue_MotorHandle发送轮子参数
        ucFlag = Flag_ChangeCarPar;
    }
    
    //启动
    else if (strncmp((char*)pucFrameData, "Start", 5) == 0) 
    {
        ucFlag = Flag_Start;
    }


    //测试下一步
    else if (strncmp((char*)pucFrameData, "Next", 4) == 0) 
    {
        xTaskNotifyGive(Task_MainHandle);
        ucFlag = Flag_NEXT;
    }


    //zdt测试
    else if (strncmp((char*)pucFrameData, "M_ZDT=", 6) == 0) 
    {
        number_start = pucFrameData + 6;
        uint8_t addr = atoi((char*)number_start);
        APP_MOTOR_ZDT_Move_P(addr, 0, 10, 0, 3200, 0, 0);
        APP_MOTOR_ZDT_WaitAck(addr);
        ucFlag = Flag_MotorZDT;
    }
    
    
    //舵机测试
    else if (strncmp((char*)pucFrameData, "ServoJaw=", 9) == 0) 
    {
        number_start = pucFrameData + 9;
        uint8_t act = *number_start;
        APP_SERVO_Jaw(act);
        ucFlag = Flag_Servo;
    }
    else if (strncmp((char*)pucFrameData, "ServoWrist=", 11) == 0) 
    {
        number_start = pucFrameData + 11;
        uint8_t act = *number_start;
        APP_SERVO_Wrist(act);
        ucFlag = Flag_Servo;
    }
    else if (strncmp((char*)pucFrameData, "ServoPlate=", 11) == 0) 
    {
        number_start = pucFrameData + 11;
        uint8_t act = atoi((char*)number_start);
        APP_SERVO_Plate(act);
        ucFlag = Flag_Servo;
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
