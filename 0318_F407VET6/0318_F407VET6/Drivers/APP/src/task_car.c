#include "main.h"
//#include "app_motor_2006.h"
//#include "bsp_can_2006.h"
/*
*********************************************************************************************************
*                                          变量定义
*********************************************************************************************************
*/
volatile int postion_flag = 0; //
//StreamBufferHandle_t StreamBuffer_MaixCAM_AND_HC = NULL;

/*
*********************************************************************************************************
*                                          函数声明
*********************************************************************************************************
*/
void AppTask_Main( void* argument );
void AppTask_BlueTooth( void* argument );
void AppTask_MaixCAM( void* argument );
void AppTask_MotorWhale( void* argument );
void AppTask_PID( void* argument );
void AppTask_HandleMove( void* argument );
void AppTask_Key( void* argument );
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
    APP_MOTOR_2006_Init();   //初始化2006电机app
    APP_MOTOR_ZDT_Init();    //初始化ZDT电机app
    APP_Handle_ColorMapping_Init( Fir, RIGHT, MIDDLE, LEFT ); //初始化第一次加工区色环贴纸上RGB对应的方向
    APP_Handle_ColorMapping_Init( Sec, RIGHT, MIDDLE, LEFT ); //初始化第二次加工区色环贴纸上RGB对应的方向
    hwt_init();
    
    Emm_V5_Origin_Trigger_Return(5, 0, 0);    //触发转盘回零
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
//              Weak_mode = 0;
        APP_MainStatusChange();  //判断是否需要根据蓝牙指令切换状态，用于调试
        APP_MainStatusRunning();
//          Move_Transfrom(0,0,0,0);
//          HAL_Delay(1000);
//          Move_Transfrom(0,0,0,90);
//          HAL_Delay(1000);
//          Move_Transfrom(0,0,0,180);
//          HAL_Delay(1000);
//          Move_Transfrom(0,0,0,-90);
//          HAL_Delay(1000);
//          Move_Transfrom(0,0,0,0);
        //SpeedUP_track(-800, 0, 100, 20, 20, 0);
        vTaskDelay( pdMS_TO_TICKS( 10 ) );
    }
}

/*显示任务*/
void AppTask_Show( void* argument )
{
    EventBits_t uxBits;
    const TickType_t xFrequency = 50;

//    xEventGroupSetBits( EventGroups_CarHandle, EventGroupsCarShow_7 );  //用于决定显示任务是否工作
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
//            PRINT(moto_rpm, "%d,%d,%d,%d", moto_chassis[0].speed_rpm
//                                         , moto_chassis[1].speed_rpm
//                                         , moto_chassis[2].speed_rpm
//                                         , moto_chassis[3].speed_rpm);
//            PRINT(moto_set, "%d,%d,%d,%d", (int)motor_pid[0].set
//                                         , (int)motor_pid[1].set
//                                         , (int)motor_pid[2].set
//                                         , (int)motor_pid[3].set);
//            PRINT(moto1, "%d,%d", (int)motor_pid[0].set, moto_chassis[0].speed_rpm);
//            PRINT(moto2, "%d,%d", (int)motor_pid[1].set, moto_chassis[1].speed_rpm);
//            PRINT(moto3, "%d,%d", (int)motor_pid[2].set, moto_chassis[2].speed_rpm);
//            PRINT(moto4, "%d,%d", (int)motor_pid[3].set, moto_chassis[3].speed_rpm);
//            set_moto_current(&hcan1, 0, 0, 100, 0 );
//            PRINT( moto1, "%d,%d,%d,%d,%d,%d"
//                   , moto_chassis[2].last_angle
//                   , moto_chassis[2].angle
//                   , moto_chassis[2].speed_rpm
//                   , moto_chassis[2].real_current
//                   , moto_chassis[2].round_cnt
//                   , moto_chassis[2].total_angle );
            PRINT( total_angle, "%d,%d,%d,%d"
                   , moto_chassis[0].total_angle 
                   , moto_chassis[1].total_angle 
                   , moto_chassis[2].total_angle 
                   , moto_chassis[3].total_angle );
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
*    函 数 名: AppTask_MotorWhale
*    功能说明: 轮子电机控制任务，根据队列中的方向和位置信息控制轮子电机运动。
*    形    参：无
*    返 回 值: 无
*********************************************************************************************************
*/
void AppTask_MotorWhale( void* argument )
{
    UNUSED( argument );

    APP_Motor_2006_WhaleControl();
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
*    函 数 名: AppTask_PID
*    功能说明: PID 控制任务，待实现。
*    形    参：无
*    返 回 值: 无
*********************************************************************************************************
*/
void AppTask_PID( void* argument )
{
    UNUSED( argument );
    const TickType_t xFrequency = 10;

    for(;; )
    {
        //待做
        if( postion_flag == 0 )
        {
            taskENTER_CRITICAL();
            _DingjiaoPIDTask();
            _M2006Task();
            taskEXIT_CRITICAL();
        }
        else
        {
            taskENTER_CRITICAL();
            //Postion_PID();
            stop();
            _M2006Task();
            taskEXIT_CRITICAL();
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
