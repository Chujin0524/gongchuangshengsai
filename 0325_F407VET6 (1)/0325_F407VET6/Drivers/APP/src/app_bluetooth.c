#include "main.h"
#include "bsp_can_2006.h"
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

        App_Printf( "RECEIVE\r\n" );
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
    UNUSED(ucFrameLength);

    static char bluetooth_tx_buf[512] = {0};       //蓝牙发送缓存
    App_Printf( "%s\r\n", pucFrameData );          //将接收到的字符串发送出去

    FLAG_E ucFlag = Flag_Idle;
    uint8_t* number_start;

    //可能修改的参数
    static STATUS_E ucStatus = STATUS_Idle;         //状态

    static uint8_t chassis_dir = 0;        //底盘方向
    static int64_t chassis_dis = 0;        //底盘距离

    static uint16_t s_AngleJaw = 0;        //夹爪舵机
    static uint16_t s_AngleWrist = 0;      //腕处舵机
    static uint16_t s_AnglePlate = 0;      //转盘舵机

    static uint8_t s_MotorZDT_ID = 0;      //步进电机ID
    static uint8_t s_MotorZDT_DIR = 0;     //步进电机转向
    static uint32_t s_MotorZDT_POS = 0;    //步进电机距离


    // 显示信息
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

    // 急停
    else if( strncmp(( char* )pucFrameData, "Stop", 4 ) == 0 )
    {
        vTaskSuspend( Task_ChassisControlHandle );
        set_motor_current( 0, 0, 0, 0 );
        ucFlag = Flag_Stop;
    }
    //启动
    else if( strncmp(( char* )pucFrameData, "Start", 5 ) == 0 )
    {
        for (uint16_t i = 0; i < 1; i++)
        {
            set_motor_current( -300, -300, 300, 300 );
            vTaskDelay(10);
        }
        ucFlag = Flag_Start;
    }

    // 复位
    else if( strncmp(( char* )pucFrameData, "Reset", 5 ) == 0 )
    {
        NVIC_SystemReset();
        ucFlag = Flag_Reset;
    }

    // 修改运行状态
    else if( strncmp(( char* )pucFrameData, "Status=", 7 ) == 0 )
    {
        number_start = pucFrameData + 7;
        ucStatus = atoi(( char* )number_start );

        xQueueOverwrite( Queue_StatusHandle, &ucStatus ); //向消息队列Queue_StatusHandle发送程序状态

        ucFlag = Flag_ChangeStatus;
    }


    //底盘行驶
    else if( strncmp(( char* )pucFrameData, "WhaleDir=", 9 ) == 0 )
    {
        number_start = pucFrameData + 9;
        chassis_dir = atoi(( char* )number_start );
    }
    else if( strncmp(( char* )pucFrameData, "WhalePos=", 9 ) == 0 )
    {
        number_start = pucFrameData + 9;
        chassis_dis = atoi(( char* )number_start );

        //非阻塞调用APP_CHASSIS_MovePID进行移动
        APP_CHASSIS_MovePID(CHASSIS_CALL_NON_BLOCKING, chassis_dir, chassis_dis);
//        APP_CHASSIS_MovePID(CHASSIS_CALL_BLOCKING, chassis_dir, chassis_dis); //调试旋转时用，非调试旋转时不要使用，否则蓝牙无法工作，导致无法程序急停

        ucFlag = Flag_ChangeCarPar;
    }

    //测试下一步
    else if( strncmp(( char* )pucFrameData, "Next", 4 ) == 0 )
    {
        xTaskNotifyGive( Task_MainHandle );
        ucFlag = Flag_NEXT;
    }


    //zdt测试
    //5 : max5300      1逆
    //6 : max10000     0前
    //7 : max8000      0下
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


    //舵机测试
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
                APP_Handle_Place_All( 1, RED, GREEN, BLUE );    //第一次全部放置测试
                break;
            case 2:
                APP_Handle_Place_All( 2, RED, GREEN, BLUE );    //第二次全部放置测试
                break;
            case 3:
                APP_Handle_Pick_All( 1, 2, 3 );        //全部抓取测试
                break;
            case 4:
                APP_MAIXCAM_Cal( CIRCLE, RED,   0, MoveXR );        //识别蓝色圆环进行校准，使X、R电机移动
                break;
            case 5:
                APP_MAIXCAM_Cal( CIRCLE, GREEN, 0, MoveXR );        //识别红色圆环进行校准，使X、R电机移动
                break;
            case 6:
                APP_MAIXCAM_Cal( CIRCLE, BLUE,  0, MoveXR );        //识别绿色圆环进行校准，使X、R电机移动
                break;
            case 7:                  //放一个测试
                vTaskDelay(5000);
                APP_MOTOR_ZDT_Move_P( 7, 0, 800, 0, 10500, 1, 0 ); APP_MOTOR_ZDT_WaitAck();  //降到最底
                APP_SERVO_Jaw('Z');        //夹紧
                APP_MOTOR_ZDT_Move_P( 7, 0, 800, 0, 300, 1, 0 ); APP_MOTOR_ZDT_WaitAck();    //升到最高
                APP_SERVO_Wrist('R'); vTaskDelay(400);     //手腕朝右
                APP_MOTOR_ZDT_Move_P( 7, 0, 800, 0, 3000, 1, 0 ); APP_MOTOR_ZDT_WaitAck();   //降到转盘上分
                APP_SERVO_Jaw('S');       //松开
                APP_MOTOR_ZDT_Move_P( 7, 0, 800, 0, 300, 1, 0 ); APP_MOTOR_ZDT_WaitAck();    //升到最高
                APP_SERVO_Wrist('F'); vTaskDelay(400);    //朝前
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
