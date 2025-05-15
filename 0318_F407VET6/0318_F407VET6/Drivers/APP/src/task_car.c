#include "main.h"
//#include "app_motor_2006.h"
//#include "bsp_can_2006.h"
/*
*********************************************************************************************************
*                                          ��������
*********************************************************************************************************
*/
volatile int postion_flag = 0; //
//StreamBufferHandle_t StreamBuffer_MaixCAM_AND_HC = NULL;

/*
*********************************************************************************************************
*                                          ��������
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
    APP_MOTOR_2006_Init();   //��ʼ��2006���app
    APP_MOTOR_ZDT_Init();    //��ʼ��ZDT���app
    APP_Handle_ColorMapping_Init( Fir, RIGHT, MIDDLE, LEFT ); //��ʼ����һ�μӹ���ɫ����ֽ��RGB��Ӧ�ķ���
    APP_Handle_ColorMapping_Init( Sec, RIGHT, MIDDLE, LEFT ); //��ʼ���ڶ��μӹ���ɫ����ֽ��RGB��Ӧ�ķ���
    hwt_init();
    
    Emm_V5_Origin_Trigger_Return(5, 0, 0);    //����ת�̻���
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
//              Weak_mode = 0;
        APP_MainStatusChange();  //�ж��Ƿ���Ҫ��������ָ���л�״̬�����ڵ���
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

/*��ʾ����*/
void AppTask_Show( void* argument )
{
    EventBits_t uxBits;
    const TickType_t xFrequency = 50;

//    xEventGroupSetBits( EventGroups_CarHandle, EventGroupsCarShow_7 );  //���ھ�����ʾ�����Ƿ���
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
*    �� �� ��: AppTask_MotorWhale
*    ����˵��: ���ӵ���������񣬸��ݶ����еķ����λ����Ϣ�������ӵ���˶���
*    ��    �Σ���
*    �� �� ֵ: ��
*********************************************************************************************************
*/
void AppTask_MotorWhale( void* argument )
{
    UNUSED( argument );

    APP_Motor_2006_WhaleControl();
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
*    �� �� ��: AppTask_PID
*    ����˵��: PID �������񣬴�ʵ�֡�
*    ��    �Σ���
*    �� �� ֵ: ��
*********************************************************************************************************
*/
void AppTask_PID( void* argument )
{
    UNUSED( argument );
    const TickType_t xFrequency = 10;

    for(;; )
    {
        //����
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
