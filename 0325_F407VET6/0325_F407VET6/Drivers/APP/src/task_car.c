#include "main.h"
//#include "app_motor_2006.h"
//#include "bsp_can_2006.h"
/*
*********************************************************************************************************
*                                          变量定义
*********************************************************************************************************
*/
volatile uint32_t g_observer_cnt1 = 0; //
volatile uint32_t g_observer_cnt2 = 0; //
//StreamBufferHandle_t StreamBuffer_MaixCAM_AND_HC = NULL;

/*
*********************************************************************************************************
*                                          函数声明
*********************************************************************************************************
*/
void AppTask_Main( void* argument );
void AppTask_BlueTooth( void* argument );
void AppTask_MaixCAM( void* argument );
void AppTask_ChassisControl( void* argument );
void AppTask_HWT101( void* argument );

/*
*********************************************************************************************************
*                                      应用初始化
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*    函 数 名: app_Init
*    功能说明: 初始化所有的应用
*    形    参：无
*    返 回 值: 无
*********************************************************************************************************
*/
void app_Init( void )
{
    APP_BLUETOOTH_Init();    //初始化蓝牙app
    APP_MAIXCAM_Init();      //初始化摄像头app
    APP_HWT101_Init();       //初始化陀螺仪app
    APP_CHASSIS_Init();   //初始化2006电机app
    APP_MOTOR_ZDT_Init();    //初始化ZDT电机app
    APP_Handle_ColorMapping_Init( Fir, RIGHT, MIDDLE, LEFT ); //初始化第一次加工区色环贴纸上RGB对应的方向
    APP_Handle_ColorMapping_Init( Sec, RIGHT, MIDDLE, LEFT ); //初始化第二次加工区色环贴纸上RGB对应的方向

    hwt_init();

//    APP_CHASSIS_MovePID(CHASSIS_CALL_NON_BLOCKING, CarDirection_Forward, 1);
    HAL_Delay(2000);
//    APP_Handle_Run();
//    APP_SERVO_Wrist('R');
}

/*
*********************************************************************************************************
*    函 数 名: AppObjCreate
*    功能说明: 此函数是创建用户自定义的任务通信量，是为了防止在freertos.c中被CubeMX覆盖
*    形    参：无
*    返 回 值: 无
*********************************************************************************************************
*/
void AppObjCreate( void )
{
    /* 创建流缓冲区句柄 */
//    StreamBuffer_MaixCAM_AND_HC = xStreamBufferCreate( 32, 4 );
}

/*
*********************************************************************************************************
*                                           主任务
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*    函 数 名: AppTask_Main
*    功能说明: 主任务，负责处理主状态切换和运行逻辑。
*    形    参：无
*    返 回 值: 无
*********************************************************************************************************
*/
void AppTask_Main( void* argument )
{
    UNUSED( argument );

//    init_task_cycle_counter();
//    __super_loop_monitor__(100) {
    for(;; )
    {
        APP_MainStatusChange();  //判断是否需要根据蓝牙指令切换状态，用于调试
        APP_MainStatusRunning();
        vTaskDelay( pdMS_TO_TICKS( 10 ) );
    }
}

/*显示任务*/
void AppTask_Show( void* argument )
{
    UNUSED( argument );

    EventBits_t uxBits;
    const TickType_t xFrequency = 50;

    static uint32_t *MOTOR_R_Pos = NULL;
    static uint32_t *MOTOR_X_Pos = NULL;
    MOTOR_R_Pos = APP_Handle_GetPosPoint(MOTOR_R);
    MOTOR_X_Pos = APP_Handle_GetPosPoint(MOTOR_X);
    xEventGroupSetBits( EventGroups_CarHandle, EventGroupsCarShow_7 );  //用于决定显示任务是否工作
    for(;; )
    {
        uxBits = xEventGroupWaitBits(
                     EventGroups_CarHandle,
                     EventGroupsCarShow_7,
                     pdFALSE,        // 不清除标志位，即开启蓝牙开关之后，就不会因为该函数关闭
                     pdFALSE,        // pdTRUE与运算  pdFALSE或运算
                     portMAX_DELAY );

        if(( uxBits & ( EventGroupsCarShow_7 ) ) == ( EventGroupsCarShow_7 ) )
        {
//            PRINT(fGyro_z, "%.3f", g_fGyro_z);
//            PRINT(moto_rpm, "%d,%d,%d,%d", motor_chassis[0].speed_rpm
//                                         , motor_chassis[1].speed_rpm
//                                         , motor_chassis[2].speed_rpm
//                                         , motor_chassis[3].speed_rpm);
//            PRINT(moto_set, "%d,%d,%d,%d", (int)motor_pid[0].set
//                                         , (int)motor_pid[1].set
//                                         , (int)motor_pid[2].set
//                                         , (int)motor_pid[3].set);
//            PRINT(moto1, "%d,%d", (int)motor_pid[0].set, motor_chassis[0].speed_rpm);
//            PRINT(moto2, "%d,%d", (int)motor_pid[1].set, motor_chassis[1].speed_rpm);
//            PRINT(moto3, "%d,%d", (int)motor_pid[2].set, motor_chassis[2].speed_rpm);
//            PRINT(moto4, "%d,%d", (int)motor_pid[3].set, motor_chassis[3].speed_rpm);
//            set_motor_current(&hcan1, 0, 0, 100, 0 );
//            PRINT( moto1, "%d,%d,%d,%d,%d,%d"
//                   , motor_chassis[2].last_angle
//                   , motor_chassis[2].angle
//                   , motor_chassis[2].speed_rpm
//                   , motor_chassis[2].real_current
//                   , motor_chassis[2].round_cnt
//                   , motor_chassis[2].total_angle );
//            PRINT( total_angle, "%d,%d,%d,%d"
//                   , motor_chassis[0].total_angle
//                   , motor_chassis[1].total_angle
//                   , motor_chassis[2].total_angle
//                   , motor_chassis[3].total_angle );
            PRINT( speed, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f"
                   , chassis_move.motor_chassis[0].speed
                   , chassis_move.motor_chassis[1].speed
                   , chassis_move.motor_chassis[2].speed
                   , chassis_move.motor_chassis[3].speed
                   , chassis_move.vx_set
                   , -chassis_move.vx_set
                   , chassis_move.vy_set
                   , -chassis_move.vy_set);
            PRINT( wz, "%.3f,%.3f"
                   , g_fGyro_z
                   , chassis_move.wz_set);
            PRINT( speed_set, "%.3f,%.3f,%.3f,%.3f"
                   , chassis_move.motor_chassis[0].speed_set
                   , chassis_move.motor_chassis[1].speed_set
                   , chassis_move.motor_chassis[2].speed_set
                   , chassis_move.motor_chassis[3].speed_set);
            PRINT( give_current, "%d,%d,%d,%d"
                   , chassis_move.motor_chassis[0].give_current
                   , chassis_move.motor_chassis[1].give_current
                   , chassis_move.motor_chassis[2].give_current
                   , chassis_move.motor_chassis[3].give_current);
            PRINT( total_angle_offset, "%lld,%lld,%lld,%lld"
                   , chassis_move.motor_chassis[0].chassis_motor_measure->total_angle_offset
                   , chassis_move.motor_chassis[1].chassis_motor_measure->total_angle_offset
                   , chassis_move.motor_chassis[2].chassis_motor_measure->total_angle_offset
                   , chassis_move.motor_chassis[3].chassis_motor_measure->total_angle_offset);
            PRINT( MOTOR_R_Pos, "%d,%d,%d"
                   , MOTOR_R_Pos[LEFT]
                   , MOTOR_R_Pos[MIDDLE]
                   , MOTOR_R_Pos[RIGHT]);
            PRINT( MOTOR_X_Pos, "%d,%d,%d"
                   , MOTOR_X_Pos[LEFT]
                   , MOTOR_X_Pos[MIDDLE]
                   , MOTOR_X_Pos[RIGHT]);
            PRINT( dis_set, "%lld,%lld"
                   , chassis_move.dis
                   , chassis_move.dis_set);
            PRINT( g_observer, "%d,%d"
                   , g_observer_cnt1
                   , g_observer_cnt2);
        }
        else
        {
//            LED_R_ON(); //应该不会进入此判断
        }
        vTaskDelay( pdMS_TO_TICKS( xFrequency ) );
    }
}

/*
*********************************************************************************************************
*                                          通信任务
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*    函 数 名: AppTask_BlueTooth
*    功能说明: 蓝牙通信任务，负责初始化蓝牙模块并接收数据。
*    形    参：无
*    返 回 值: 无
*********************************************************************************************************
*/
void AppTask_BlueTooth( void* argument )
{
    UNUSED( argument );

    EventBits_t uxBits;
    const TickType_t xFrequency = 10;

    xEventGroupSetBits( EventGroups_CarHandle, EventGroupsCarBlueTooth_EN_0 );  //用于决定蓝牙是否工作
    for(;; )
    {
        uxBits = xEventGroupWaitBits(
                     EventGroups_CarHandle,
                     EventGroupsCarBlueTooth_EN_0,
                     pdFALSE,        // 不清除标志位，即开启蓝牙开关之后，就不会因为该函数关闭
                     pdFALSE,        // pdTRUE与运算  pdFALSE或运算
                     portMAX_DELAY );

        if(( uxBits & ( EventGroupsCarBlueTooth_EN_0 ) ) == ( EventGroupsCarBlueTooth_EN_0 ) )
        {
            APP_BLUETOOTH_Read();
        }
        else
        {
//            LED_R_ON(); //应该不会进入此判断
        }
        vTaskDelay( pdMS_TO_TICKS( xFrequency ) );
    }
}

/*
*********************************************************************************************************
*    函 数 名: AppTask_MaixCAM
*    功能说明: MaixCAM 任务，负责初始化摄像头并处理图像数据。
*    形    参：无
*    返 回 值: 无
*********************************************************************************************************
*/
void AppTask_MaixCAM( void* argument )
{
    UNUSED( argument );

    APP_MaixCAMControl();
}

/*
*********************************************************************************************************
*    函 数 名: AppTask_ChassisControl
*    功能说明: 底盘接收任务，根据队列中的方向和位置信息控制轮子电机运动。
*    形    参：无
*    返 回 值: 无
*********************************************************************************************************
*/
void AppTask_ChassisControl( void* argument )
{
    UNUSED( argument );

    APP_CHASSIS_Control();
}

/*
*********************************************************************************************************
*                                          处理任务
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*    函 数 名: AppTask_HWT101
*    功能说明: HWT101 任务，负责解析陀螺仪数据
*    形    参：无
*    返 回 值: 无
*********************************************************************************************************
*/
void AppTask_HWT101( void* argument )
{
    UNUSED( argument );
    const TickType_t xFrequency = 5;

    for(;; )
    {
        if( APP_HWT101_Read() )
        {

        }

        vTaskDelay( pdMS_TO_TICKS( xFrequency ) );
    }
}

/*
*********************************************************************************************************
*    函 数 名: App_Printf
*    功能说明: 安全打印函数，使用互斥量保护打印操作。
*    形    参：无
*    返 回 值: 无
*********************************************************************************************************
*/
void App_Printf( char* format, ... )
{
    char buf_str[1024 + 1];
    va_list v_args;


    va_start( v_args, format );
    ( void )vsnprintf(( char* )&buf_str[0],
                      ( size_t ) sizeof( buf_str ),
                      ( char const* ) format,
                      v_args );
    va_end( v_args );

    /* Mutex_PrintHandle */
    osMutexAcquire( Mutex_PrintHandle, portMAX_DELAY );

    printf( "%s", buf_str );

    osMutexRelease( Mutex_PrintHandle );
}
