#include "main.h"

static void chassis_init( chassis_move_t* chassis_move_init );
static void chassis_set_mode( chassis_move_t* chassis_move_mode );
static void chassis_feedback_update( chassis_move_t* chassis_move_update );
static void chassis_set_contorl( chassis_move_t* chassis_move_control );
static void chassis_control_loop( chassis_move_t* chassis_move_control_loop );

//�����˶�����
chassis_move_t chassis_move;

/**
  * @brief          �������񣬼�� CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: ��
  * @retval         none
  */
void APP_CHASSIS_Control( void )
{
    //����һ��ʱ��
    vTaskDelay( CHASSIS_TASK_INIT_TIME );
    //���̳�ʼ��
    chassis_init(&chassis_move );

    while( 1 )
    {
        //���õ��̿���ģʽ
        chassis_set_mode(&chassis_move );
        //�������ݸ���
        chassis_feedback_update(&chassis_move );
        //���̿���������
        chassis_set_contorl(&chassis_move );
        //���̿���PID����
        chassis_control_loop(&chassis_move );

        //���Ϳ��Ƶ���
        set_motor_current( chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current,
                           chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current );

        //ϵͳ��ʱ
        vTaskDelay( CHASSIS_CONTROL_TIME_MS );
    }
}

/**
  * @brief          ��ʼ��"chassis_move"����������pid��ʼ���� ң����ָ���ʼ����3508���̵��ָ���ʼ������̨�����ʼ���������ǽǶ�ָ���ʼ��
  * @param[out]     chassis_move_init:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_init( chassis_move_t* chassis_move_init )
{
    if( chassis_move_init == NULL )
    {
        return;
    }

    //�����ٶȻ�pidֵ
    const static float motor_speed_pid[3] = {M2006_MOTOR_SPEED_PID_KP, M2006_MOTOR_SPEED_PID_KI, M2006_MOTOR_SPEED_PID_KD};

    //���̽Ƕ�pidֵ
    const static float chassis_yaw_pid[3] = {CHASSIS_FOLLOW_CHASSIS_PID_KP, CHASSIS_FOLLOW_CHASSIS_PID_KI, CHASSIS_FOLLOW_CHASSIS_PID_KD};

    const static float chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static float chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};

    //���̿���״̬Ϊԭʼ
    chassis_move_init->chassis_mode = CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW;
    //��ȡ��������̬��ָ��
    chassis_move_init->chassis_INS_angle = APP_HWT101_GetAnglePoint();

    //��ȡ���̵������ָ�룬��ʼ��PID
    for( uint8_t i = 0; i < 4; i++ )
    {
        chassis_move_init->motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point( i );
        PID_init(&chassis_move_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid, M2006_MOTOR_SPEED_PID_MAX_OUT, M2006_MOTOR_SPEED_PID_MAX_IOUT );
    }
    //��ʼ���Ƕ�PID
    PID_init(&chassis_move_init->chassis_angle_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_FOLLOW_CHASSIS_PID_MAX_OUT, CHASSIS_FOLLOW_CHASSIS_PID_MAX_IOUT );

    //��һ���˲�����б����������
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter );
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter );

    //��� ��С�ٶ�
    chassis_move_init->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

    chassis_move_init->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
    chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;

    //����һ������
    chassis_feedback_update( chassis_move_init );
}

/**
  * @brief          ���õ��̿���ģʽ����Ҫ��'chassis_behaviour_mode_set'�����иı�
  * @param[out]     chassis_move_mode:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_set_mode( chassis_move_t* chassis_move_mode )
{
    if( chassis_move_mode == NULL )
    {
        return;
    }
    //in file "chassis_behaviour.c"
    chassis_behaviour_mode_set( chassis_move_mode );
}

/**
  * @brief          ���̲������ݸ��£���������ٶȣ�ŷ���Ƕȣ��������ٶ�
  * @param[out]     chassis_move_update:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_feedback_update( chassis_move_t* chassis_move_update )
{
    if( chassis_move_update == NULL )
    {
        return;
    }

    for( uint8_t i = 0; i < 4; i++ )
    {
        //���µ���ٶ�
        chassis_move_update->motor_chassis[i].speed = chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm / ReductionRatio;
    }
    // ����ĸ�������ٶ��Ƿ�С�� threshold 0.5��������е���ٶȶ�С�� 0.5���ͷ��ź���
    bool all_motors_below_threshold = true;
    for (uint8_t i = 0; i < 4 && (all_motors_below_threshold = (chassis_move_update->motor_chassis[i].speed < 0.5)); i++);
    if (all_motors_below_threshold) xSemaphoreGive(SemMoveHandle);

    //���������̬�Ƕ�, �����������������������ⲿ�ִ���
    chassis_move_update->chassis_yaw = *( chassis_move_update->chassis_INS_angle );
}

/**
  * @brief          ����ң����ͨ��ֵ����������ͺ����ٶ�
  *
  * @param[out]     vx_set: �����ٶ�ָ��
  * @param[out]     vy_set: �����ٶ�ָ��
  * @param[out]     chassis_move: "chassis_move" ����ָ��
  * @retval         none
  */
void chassis_control_vector( float* vx_set, float* vy_set, chassis_move_t* chassis_move )
{
    if( chassis_move == NULL || vx_set == NULL || vy_set == NULL )
    {
        return;
    }

    float vx_set_channel=0, vy_set_channel=0;

    float v1 = 180;
    float v2 = 50;
    float v_max = 0;
    int64_t dis_long = 650000;  //dis_set���ڴ�ֵ����v1�ٶȣ���֮v2�ٶ�
    float dis_dec_k = 0.94;     //���پ����ϵ�������ߵ�dis_dec_k*dis_set֮�󣬿�ʼ����

    if ( chassis_move->dis_set >= dis_long)    //����������ʱ�ٶ��ǿ��
    {
        v_max = v1;
    }
    else                                       //�̾�������ʱ�ٶ�������
    {
        v_max = v2;
    }

    switch( chassis_move->dir )
    {
    case CarDirection_Forward:
        chassis_move->dis =
            (- chassis_move->motor_chassis[0].chassis_motor_measure->total_angle_offset
             - chassis_move->motor_chassis[1].chassis_motor_measure->total_angle_offset
             + chassis_move->motor_chassis[2].chassis_motor_measure->total_angle_offset
             + chassis_move->motor_chassis[3].chassis_motor_measure->total_angle_offset ) / 4;
        vx_set_channel = v_max;
        //����������650000��ʼ����
        if( chassis_move->dis_set >= dis_long && chassis_move->dis > dis_dec_k*chassis_move->dis_set)
        {
            vx_set_channel = 0;
        }
        break;
    case CarDirection_Backward:
        chassis_move->dis =
            (+ chassis_move->motor_chassis[0].chassis_motor_measure->total_angle_offset
             + chassis_move->motor_chassis[1].chassis_motor_measure->total_angle_offset
             - chassis_move->motor_chassis[2].chassis_motor_measure->total_angle_offset
             - chassis_move->motor_chassis[3].chassis_motor_measure->total_angle_offset ) / 4;
        vx_set_channel = -v_max;
        //����������650000��ʼ����
        if( chassis_move->dis_set >= dis_long && chassis_move->dis > dis_dec_k*chassis_move->dis_set)
        {
            vx_set_channel = -0;
        }
        break;
    case CarDirection_Left:
        chassis_move->dis =
            (+ chassis_move->motor_chassis[0].chassis_motor_measure->total_angle_offset
             - chassis_move->motor_chassis[1].chassis_motor_measure->total_angle_offset
             - chassis_move->motor_chassis[2].chassis_motor_measure->total_angle_offset
             + chassis_move->motor_chassis[3].chassis_motor_measure->total_angle_offset ) / 4;
        vy_set_channel = -v_max;
        //����������650000��ʼ����
        if( chassis_move->dis_set >= dis_long && chassis_move->dis > dis_dec_k*chassis_move->dis_set)
        {
            vy_set_channel = -0;
        }
        break;
    case CarDirection_Right:
        chassis_move->dis =
            (- chassis_move->motor_chassis[0].chassis_motor_measure->total_angle_offset
             + chassis_move->motor_chassis[1].chassis_motor_measure->total_angle_offset
             + chassis_move->motor_chassis[2].chassis_motor_measure->total_angle_offset
             - chassis_move->motor_chassis[3].chassis_motor_measure->total_angle_offset ) / 4;
        vy_set_channel = v_max;
        //����������650000��ʼ����
        if( chassis_move->dis_set >= dis_long && chassis_move->dis > dis_dec_k*chassis_move->dis_set)
        {
            vy_set_channel = 0;
        }
        break;
    }

    //һ�׵�ͨ�˲�����б����Ϊ�����ٶ�����
    first_order_filter_cali(&chassis_move->chassis_cmd_slow_set_vx, vx_set_channel );
    first_order_filter_cali(&chassis_move->chassis_cmd_slow_set_vy, vy_set_channel );

    if ( chassis_move->dis >= chassis_move->dis_set)   //ֱ��ͣ��
    {
        chassis_move->chassis_cmd_slow_set_vx.out = 0.0f;
        chassis_move->chassis_cmd_slow_set_vy.out = 0.0f;
    }

    *vx_set = chassis_move->chassis_cmd_slow_set_vx.out;
    *vy_set = chassis_move->chassis_cmd_slow_set_vy.out;
}

/**
  * @brief          ���õ��̿�������ֵ, ���˶�����ֵ��ͨ��chassis_behaviour_control_set�������õ�
  * @param[out]     chassis_move_update:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_set_contorl( chassis_move_t* chassis_move_control )
{
    if( chassis_move_control == NULL )
    {
        return;
    }

    float vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;
    //��ȡ������������ֵ
    chassis_behaviour_control_set(&vx_set, &vy_set, &angle_set, chassis_move_control );

    if( chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW )
    {
        float delat_angle = 0.0f;
        //���õ��̿��ƵĽǶ�
        delat_angle = theta_format( chassis_move.chassis_yaw_set - chassis_move.chassis_yaw );
        deadband_limit(delat_angle, delat_angle, CHASSIS_YAW_DEADLINE);
        g_fGyro_z = delat_angle;

        //������ת�Ľ��ٶ�
        chassis_move_control->wz_set = PID_calc(&chassis_move_control->chassis_angle_pid, 0.0f, delat_angle );
//        chassis_move_control->wz_set = 0; //����������У׼

        //�ٶ��޷�
        chassis_move_control->vx_set = fp32_constrain( vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed );
        chassis_move_control->vy_set = fp32_constrain( vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed );
    }
}

/**
  * @brief          �ĸ������ٶ���ͨ�������������������
  * @param[in]      vx_set: �����ٶ�
  * @param[in]      vy_set: �����ٶ�
  * @param[in]      wz_set: ��ת�ٶ�
  * @param[out]     wheel_speed: �ĸ������ٶ�
  * @retval         none
  */
static void chassis_vector_to_wheel_speed( const float vx_set, const float vy_set, const float wz_set, float wheel_speed[4] )
{
    wheel_speed[0] = -vx_set - vy_set - wz_set;
    wheel_speed[1] = -vx_set + vy_set - wz_set;
    wheel_speed[2] =  vx_set + vy_set - wz_set;
    wheel_speed[3] =  vx_set - vy_set - wz_set;
}

/**
  * @brief          ����ѭ�������ݿ����趨ֵ������������ֵ�����п���
  * @param[out]     chassis_move_control_loop:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_control_loop( chassis_move_t* chassis_move_control_loop )
{
    float max_vector = 0.0f, vector_rate = 0.0f;
    float temp = 0.0f;
    float wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t i = 0;

    //ȫ�����˶��ֽ�
    chassis_vector_to_wheel_speed( chassis_move_control_loop->vx_set,
                                   chassis_move_control_loop->vy_set,
                                   chassis_move_control_loop->wz_set, wheel_speed );

    //�������ӿ�������ٶȣ�������������ٶ�
    for( i = 0; i < 4; i++ )
    {
        chassis_move_control_loop->motor_chassis[i].speed_set = wheel_speed[i];
        temp = fabs( chassis_move_control_loop->motor_chassis[i].speed_set );
        if( max_vector < temp )
        {
            max_vector = temp;
        }
    }

    if( max_vector > MAX_WHEEL_SPEED )
    {
        vector_rate = MAX_WHEEL_SPEED / max_vector;
        for( i = 0; i < 4; i++ )
        {
            chassis_move_control_loop->motor_chassis[i].speed_set *= vector_rate;
        }
    }

    //����pid
    for( i = 0; i < 4; i++ )
    {
        PID_calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].speed, chassis_move_control_loop->motor_chassis[i].speed_set );
    }

    //��ֵ����ֵ
    for( i = 0; i < 4; i++ )
    {
        chassis_move_control_loop->motor_chassis[i].give_current = ( int16_t )( chassis_move_control_loop->motor_speed_pid[i].out );
    }
}

void APP_CHASSIS_Init( void )
{
    //ͣ��
    set_motor_current( 0, 0, 0, 0 );

    //��ȡ��ʼ������ֵ
    get_motor_offset();
}

/*
*********************************************************************************************************
*    �� �� ��: APP_CHASSIS_MovePID
*    ����˵��: �ǶȻ�����С��ƽ�Ƹ�������
*    ��    ��: _dir  : �궨���bsp.h
*              _clk  : ����
*    �� �� ֵ: ��
*********************************************************************************************************
*/
void APP_CHASSIS_MovePID( chassis_block_mode_e _block_mode, uint8_t _dir, int64_t _clk )
{
    int64_t k = 1000;
    _clk *= k;
    chassis_move.dir = _dir;
    chassis_move.dis_set = chassis_move.dis_remaining = _clk;
    set_motor_zero_angle();

    if( chassis_move.dir == CarDirection_Rotate)    //��ת
    {
        chassis_move.dis_set = chassis_move.dis_remaining = 0;
        chassis_move.chassis_yaw_set = _clk / k;    //��צָ��Ķ�������ʱ��Ϊ��

        float delat_angle = 0.0f;
        delat_angle = theta_format( chassis_move.chassis_yaw_set - chassis_move.chassis_yaw );   //����Ƕ�ƫ��
        while(fabs(delat_angle) >= 0.1)     //���Ƕ�ƫ�����0.1ʱ�����ڸó���
        {
            delat_angle = theta_format( chassis_move.chassis_yaw_set - chassis_move.chassis_yaw );
        }
        return;
    }

    if( _block_mode == CHASSIS_CALL_BLOCKING )      //����ģʽ���ȴ������ٶ�Ϊ0
    {
        for(uint16_t i = 0; i< 10; i++)
        {
            xSemaphoreTake(SemMoveHandle, portMAX_DELAY);
        }
    }
    else if( _block_mode == CHASSIS_CALL_NON_BLOCKING )
    {

    }
}

void APP_CHASSIS_Status( STATUS_E _ucStatus )
{
    switch( _ucStatus )
    {
    case STATUS_Start_2_Qr:
        APP_CHASSIS_MovePID( CHASSIS_CALL_BLOCKING, CarDirection_Left, 150 );
        APP_CHASSIS_MovePID( CHASSIS_CALL_BLOCKING, CarDirection_Forward, 600 );
        break;

    case STATUS_Qr_2_RawArea:
        APP_CHASSIS_MovePID( CHASSIS_CALL_BLOCKING, CarDirection_Forward, 800 );
        break;

    case STATUS_RawArea_2_ProFirArea:
        APP_CHASSIS_MovePID( CHASSIS_CALL_BLOCKING, CarDirection_Backward, 350 );
        APP_CHASSIS_MovePID( CHASSIS_CALL_BLOCKING, CarDirection_Rotate, 90 );     //��צ����90��
        APP_CHASSIS_MovePID( CHASSIS_CALL_BLOCKING, CarDirection_Forward, 1700 );
        APP_CHASSIS_MovePID( CHASSIS_CALL_BLOCKING, CarDirection_Rotate, 180 );    //��צ����180��
        break;

    case STATUS_ProFirArea_2_ProSecArea:
        APP_CHASSIS_MovePID( CHASSIS_CALL_BLOCKING, CarDirection_Rotate, 0 );      //��צ����0��
        APP_CHASSIS_MovePID( CHASSIS_CALL_BLOCKING, CarDirection_Forward, 850 );
        APP_CHASSIS_MovePID( CHASSIS_CALL_BLOCKING, CarDirection_Rotate, 270 );    //��צ����270��
        APP_CHASSIS_MovePID( CHASSIS_CALL_BLOCKING, CarDirection_Forward, 850 );
        APP_CHASSIS_MovePID( CHASSIS_CALL_BLOCKING, CarDirection_Rotate, 90 );     //��צ����90��
        break;

    case STATUS_ProSecArea_2_RawArea:
        APP_CHASSIS_MovePID( CHASSIS_CALL_BLOCKING, CarDirection_Rotate, 270 );    //��צ����270��
        APP_CHASSIS_MovePID( CHASSIS_CALL_BLOCKING, CarDirection_Forward, 850 );
        APP_CHASSIS_MovePID( CHASSIS_CALL_BLOCKING, CarDirection_Rotate, 180 );    //��צ����180��
        APP_CHASSIS_MovePID( CHASSIS_CALL_BLOCKING, CarDirection_Forward, 350 );
        APP_CHASSIS_MovePID( CHASSIS_CALL_BLOCKING, CarDirection_Rotate, 0 );      //��צ����0��
        break;

    case STATUS_ProSecArea_2_Start:
        APP_CHASSIS_MovePID( CHASSIS_CALL_BLOCKING, CarDirection_Rotate, 270 );    //��צ����270��
        APP_CHASSIS_MovePID( CHASSIS_CALL_BLOCKING, CarDirection_Forward, 700 );
        APP_CHASSIS_MovePID( CHASSIS_CALL_BLOCKING, CarDirection_Rotate, 180 );    //��צ����180��
        APP_CHASSIS_MovePID( CHASSIS_CALL_BLOCKING, CarDirection_Forward, 1700 );
        APP_CHASSIS_MovePID( CHASSIS_CALL_BLOCKING, CarDirection_Left, 400 );
        break;

    default:
        break;
    }
    xTaskNotifyGive( Task_MainHandle );
}
