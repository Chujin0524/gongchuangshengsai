#include "main.h"
//#include "app_motor_2006.h"
//#include "bsp_can_2006.h"
/*
*********************************************************************************************************
*                                          ��������
*********************************************************************************************************
*/
volatile uint32_t g_observer_cnt1 = 0; //
volatile uint32_t g_observer_cnt2 = 0; //
//StreamBufferHandle_t StreamBuffer_MaixCAM_AND_HC = NULL;

/*
*********************************************************************************************************
*                                          ��������
*********************************************************************************************************
*/
void AppTask_Main( void* argument );
void AppTask_BlueTooth( void* argument );
void AppTask_MaixCAM( void* argument );
void AppTask_ChassisControl( void* argument );
void AppTask_HWT101( void* argument );

/*
*********************************************************************************************************
*                                      Ӧ�ó�ʼ��
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*    �� �� ��: app_Init
*    ����˵��: ��ʼ�����е�Ӧ��
*    ��    �Σ���
*    �� �� ֵ: ��
*********************************************************************************************************
*/
void app_Init( void )
{
    APP_BLUETOOTH_Init();    //��ʼ������app
    APP_MAIXCAM_Init();      //��ʼ������ͷapp
    APP_HWT101_Init();       //��ʼ��������app
    APP_CHASSIS_Init();   //��ʼ��2006���app
    APP_MOTOR_ZDT_Init();    //��ʼ��ZDT���app
    APP_Handle_ColorMapping_Init( Fir, RIGHT, MIDDLE, LEFT ); //��ʼ����һ�μӹ���ɫ����ֽ��RGB��Ӧ�ķ���
    APP_Handle_ColorMapping_Init( Sec, RIGHT, MIDDLE, LEFT ); //��ʼ���ڶ��μӹ���ɫ����ֽ��RGB��Ӧ�ķ���

    hwt_init();

//    APP_CHASSIS_MovePID(CHASSIS_CALL_NON_BLOCKING, CarDirection_Forward, 1);
    HAL_Delay(2000);
//    APP_Handle_Run();
//    APP_SERVO_Wrist('R');
}

/*
*********************************************************************************************************
*    �� �� ��: AppObjCreate
*    ����˵��: �˺����Ǵ����û��Զ��������ͨ��������Ϊ�˷�ֹ��freertos.c�б�CubeMX����
*    ��    �Σ���
*    �� �� ֵ: ��
*********************************************************************************************************
*/
void AppObjCreate( void )
{
    /* ��������������� */
//    StreamBuffer_MaixCAM_AND_HC = xStreamBufferCreate( 32, 4 );
}

/*
*********************************************************************************************************
*                                           ������
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*    �� �� ��: AppTask_Main
*    ����˵��: �����񣬸�������״̬�л��������߼���
*    ��    �Σ���
*    �� �� ֵ: ��
*********************************************************************************************************
*/
void AppTask_Main( void* argument )
{
    UNUSED( argument );

//    init_task_cycle_counter();
//    __super_loop_monitor__(100) {
    for(;; )
    {
        APP_MainStatusChange();  //�ж��Ƿ���Ҫ��������ָ���л�״̬�����ڵ���
        APP_MainStatusRunning();
        vTaskDelay( pdMS_TO_TICKS( 10 ) );
    }
}

/*��ʾ����*/
void AppTask_Show( void* argument )
{
    UNUSED( argument );

    EventBits_t uxBits;
    const TickType_t xFrequency = 50;

    static uint32_t *MOTOR_R_Pos = NULL;
    static uint32_t *MOTOR_X_Pos = NULL;
    MOTOR_R_Pos = APP_Handle_GetPosPoint(MOTOR_R);
    MOTOR_X_Pos = APP_Handle_GetPosPoint(MOTOR_X);
    xEventGroupSetBits( EventGroups_CarHandle, EventGroupsCarShow_7 );  //���ھ�����ʾ�����Ƿ���
    for(;; )
    {
        uxBits = xEventGroupWaitBits(
                     EventGroups_CarHandle,
                     EventGroupsCarShow_7,
                     pdFALSE,        // �������־λ����������������֮�󣬾Ͳ�����Ϊ�ú����ر�
                     pdFALSE,        // pdTRUE������  pdFALSE������
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
//            LED_R_ON(); //Ӧ�ò��������ж�
        }
        vTaskDelay( pdMS_TO_TICKS( xFrequency ) );
    }
}

/*
*********************************************************************************************************
*                                          ͨ������
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*    �� �� ��: AppTask_BlueTooth
*    ����˵��: ����ͨ�����񣬸����ʼ������ģ�鲢�������ݡ�
*    ��    �Σ���
*    �� �� ֵ: ��
*********************************************************************************************************
*/
void AppTask_BlueTooth( void* argument )
{
    UNUSED( argument );

    EventBits_t uxBits;
    const TickType_t xFrequency = 10;

    xEventGroupSetBits( EventGroups_CarHandle, EventGroupsCarBlueTooth_EN_0 );  //���ھ��������Ƿ���
    for(;; )
    {
        uxBits = xEventGroupWaitBits(
                     EventGroups_CarHandle,
                     EventGroupsCarBlueTooth_EN_0,
                     pdFALSE,        // �������־λ����������������֮�󣬾Ͳ�����Ϊ�ú����ر�
                     pdFALSE,        // pdTRUE������  pdFALSE������
                     portMAX_DELAY );

        if(( uxBits & ( EventGroupsCarBlueTooth_EN_0 ) ) == ( EventGroupsCarBlueTooth_EN_0 ) )
        {
            APP_BLUETOOTH_Read();
        }
        else
        {
//            LED_R_ON(); //Ӧ�ò��������ж�
        }
        vTaskDelay( pdMS_TO_TICKS( xFrequency ) );
    }
}

/*
*********************************************************************************************************
*    �� �� ��: AppTask_MaixCAM
*    ����˵��: MaixCAM ���񣬸����ʼ������ͷ������ͼ�����ݡ�
*    ��    �Σ���
*    �� �� ֵ: ��
*********************************************************************************************************
*/
void AppTask_MaixCAM( void* argument )
{
    UNUSED( argument );

    APP_MaixCAMControl();
}

/*
*********************************************************************************************************
*    �� �� ��: AppTask_ChassisControl
*    ����˵��: ���̽������񣬸��ݶ����еķ����λ����Ϣ�������ӵ���˶���
*    ��    �Σ���
*    �� �� ֵ: ��
*********************************************************************************************************
*/
void AppTask_ChassisControl( void* argument )
{
    UNUSED( argument );

    APP_CHASSIS_Control();
}

/*
*********************************************************************************************************
*                                          ��������
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*    �� �� ��: AppTask_HWT101
*    ����˵��: HWT101 ���񣬸����������������
*    ��    �Σ���
*    �� �� ֵ: ��
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
*    �� �� ��: App_Printf
*    ����˵��: ��ȫ��ӡ������ʹ�û�����������ӡ������
*    ��    �Σ���
*    �� �� ֵ: ��
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
