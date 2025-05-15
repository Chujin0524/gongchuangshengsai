#include "main.h"

static void chassis_init(chassis_move_t* chassis_move_init);
static void chassis_set_mode(chassis_move_t* chassis_move_mode);
static void chassis_feedback_update(chassis_move_t* chassis_move_update);
static void chassis_set_contorl(chassis_move_t* chassis_move_control);
static void chassis_control_loop(chassis_move_t* chassis_move_control_loop);

//底盘运动数据
chassis_move_t chassis_move;

/**
  * @brief          底盘任务，间隔 CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */
void APP_CHASSIS_Control(void)
{
    //空闲一段时间
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    //底盘初始化
    chassis_init(&chassis_move);

    while (1)
    {
        //设置底盘控制模式
        chassis_set_mode(&chassis_move);
        //底盘数据更新
        chassis_feedback_update(&chassis_move);
        //底盘控制量设置
        chassis_set_contorl(&chassis_move);
        //底盘控制PID计算
        chassis_control_loop(&chassis_move);

        //发送控制电流
        set_motor_current(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current,
                          chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current);

        //系统延时
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);
    }
}

/**
  * @brief          初始化"chassis_move"变量，包括pid初始化， 遥控器指针初始化，3508底盘电机指针初始化，云台电机初始化，陀螺仪角度指针初始化
  * @param[out]     chassis_move_init:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_init(chassis_move_t* chassis_move_init)
{
    if (chassis_move_init == NULL)
    {
        return;
    }

    //底盘速度环pid值
    const static float motor_speed_pid[3] = {M2006_MOTOR_SPEED_PID_KP, M2006_MOTOR_SPEED_PID_KI, M2006_MOTOR_SPEED_PID_KD};

    //底盘角度pid值
    const static float chassis_yaw_pid[3] = {CHASSIS_FOLLOW_CHASSIS_PID_KP, CHASSIS_FOLLOW_CHASSIS_PID_KI, CHASSIS_FOLLOW_CHASSIS_PID_KD};

    const static float chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static float chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};

    const static float chassis_x_order_filter_cal[1] = {CHASSIS_ACCEL_X_NUM_CAL};
    const static float chassis_y_order_filter_cal[1] = {CHASSIS_ACCEL_Y_NUM_CAL};

    //底盘开机状态为原始
    chassis_move_init->chassis_mode = CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW;
    //获取陀螺仪姿态角指针
    chassis_move_init->chassis_INS_angle = APP_HWT101_GetAnglePoint();

    //获取底盘电机数据指针，初始化PID
    for (uint8_t i = 0; i < 4; i++)
    {
        chassis_move_init->motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
        PID_init(&chassis_move_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid, M2006_MOTOR_SPEED_PID_MAX_OUT, M2006_MOTOR_SPEED_PID_MAX_IOUT);
    }

    //初始化角度PID
    PID_init(&chassis_move_init->chassis_angle_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_FOLLOW_CHASSIS_PID_MAX_OUT, CHASSIS_FOLLOW_CHASSIS_PID_MAX_IOUT);

    //用一阶滤波代替斜波函数生成，大步快速提速降速用
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);

    //校准用，小步慢速提速降速用
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx_cal, CHASSIS_CONTROL_TIME, chassis_x_order_filter_cal);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy_cal, CHASSIS_CONTROL_TIME, chassis_y_order_filter_cal);

    //最大 最小速度
    chassis_move_init->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

    chassis_move_init->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
    chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;

    //更新一下数据
    chassis_feedback_update(chassis_move_init);
}

/**
  * @brief          设置底盘控制模式，主要在'chassis_behaviour_mode_set'函数中改变
  * @param[out]     chassis_move_mode:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_set_mode(chassis_move_t* chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }

    //in file "chassis_behaviour.c"
    chassis_behaviour_mode_set(chassis_move_mode);
}

/**
  * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
  * @param[out]     chassis_move_update:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_feedback_update(chassis_move_t* chassis_move_update)
{
    if (chassis_move_update == NULL)
    {
        return;
    }

    for (uint8_t i = 0; i < 4; i++)
    {
        //更新电机速度
        chassis_move_update->motor_chassis[i].speed = chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm / ReductionRatio;
    }

    // 检查四个电机的速度是否都小于 threshold 0.5。如果所有电机速度都小于 0.5，释放信号量
    bool all_motors_below_threshold = true;

    for (uint8_t i = 0; i < 4 && (all_motors_below_threshold = (chassis_move_update->motor_chassis[i].speed < 0.5)); i++);

    if (all_motors_below_threshold) xSemaphoreGive(SemMoveHandle);

    //计算底盘姿态角度, 如果底盘上有陀螺仪请更改这部分代码
    chassis_move_update->chassis_yaw = *(chassis_move_update->chassis_INS_angle);
}

/**
  * @brief          根据遥控器通道值，计算纵向和横移速度
  *
  * @param[out]     vx_set: 纵向速度指针
  * @param[out]     vy_set: 横向速度指针
  * @param[out]     chassis_move: "chassis_move" 变量指针
  * @retval         none
  */
void chassis_control_vector(float* vx_set, float* vy_set, chassis_move_t* chassis_move)
{
    if (chassis_move == NULL || vx_set == NULL || vy_set == NULL)
    {
        return;
    }

    float vx_set_channel = 0, vy_set_channel = 0;

    float v1 = 180;
    float v2 = 50;
    float v_max = 0;
    int64_t dis_long = 650000;  //dis_set大于此值，以v1速度，反之v2速度
    float dis_dec_k = 0.94;     //减速距离的系数，行走到dis_dec_k*dis_set之后，开始减速


    if (chassis_move->move_mode == CHASSIS_LONG)
    {
        if (chassis_move->dis_set >= dis_long)     //长距离行走时速度是快的
        {
            v_max = v1;
        }
        else                                       //短距离行走时速度是慢的
        {
            v_max = v2;
        }
    }
    else if (chassis_move->move_mode == CHASSIS_CAL)
    {
        v_max = v2;
    }

    switch (chassis_move->dir)
    {
        case CarDirection_Forward:
            chassis_move->dis =
                (- chassis_move->motor_chassis[0].chassis_motor_measure->total_angle_offset
                 - chassis_move->motor_chassis[1].chassis_motor_measure->total_angle_offset
                 + chassis_move->motor_chassis[2].chassis_motor_measure->total_angle_offset
                 + chassis_move->motor_chassis[3].chassis_motor_measure->total_angle_offset) / 4;
            vx_set_channel = v_max;

            //长距离的最后650000开始减速
            if (chassis_move->dis_set >= dis_long && chassis_move->dis > dis_dec_k * chassis_move->dis_set)
            {
                vx_set_channel = 0;
            }

            break;

        case CarDirection_Backward:
            chassis_move->dis =
                (+ chassis_move->motor_chassis[0].chassis_motor_measure->total_angle_offset
                 + chassis_move->motor_chassis[1].chassis_motor_measure->total_angle_offset
                 - chassis_move->motor_chassis[2].chassis_motor_measure->total_angle_offset
                 - chassis_move->motor_chassis[3].chassis_motor_measure->total_angle_offset) / 4;
            vx_set_channel = -v_max;

            //长距离的最后650000开始减速
            if (chassis_move->dis_set >= dis_long && chassis_move->dis > dis_dec_k * chassis_move->dis_set)
            {
                vx_set_channel = -0;
            }

            break;

        case CarDirection_Left:
            chassis_move->dis =
                (+ chassis_move->motor_chassis[0].chassis_motor_measure->total_angle_offset
                 - chassis_move->motor_chassis[1].chassis_motor_measure->total_angle_offset
                 - chassis_move->motor_chassis[2].chassis_motor_measure->total_angle_offset
                 + chassis_move->motor_chassis[3].chassis_motor_measure->total_angle_offset) / 4;
            vy_set_channel = -v_max;

            //长距离的最后650000开始减速
            if (chassis_move->dis_set >= dis_long && chassis_move->dis > dis_dec_k * chassis_move->dis_set)
            {
                vy_set_channel = -0;
            }

            break;

        case CarDirection_Right:
            chassis_move->dis =
                (- chassis_move->motor_chassis[0].chassis_motor_measure->total_angle_offset
                 + chassis_move->motor_chassis[1].chassis_motor_measure->total_angle_offset
                 + chassis_move->motor_chassis[2].chassis_motor_measure->total_angle_offset
                 - chassis_move->motor_chassis[3].chassis_motor_measure->total_angle_offset) / 4;
            vy_set_channel = v_max;

            //长距离的最后650000开始减速
            if (chassis_move->dis_set >= dis_long && chassis_move->dis > dis_dec_k * chassis_move->dis_set)
            {
                vy_set_channel = 0;
            }

            break;
    }


    //一阶低通滤波代替斜波作为底盘速度输入
    float vx_s = 0;
    float vy_s = 0;

    if (chassis_move->move_mode == CHASSIS_LONG)
    {
        first_order_filter_cali(&chassis_move->chassis_cmd_slow_set_vx, vx_set_channel);
        first_order_filter_cali(&chassis_move->chassis_cmd_slow_set_vy, vy_set_channel);

        if (chassis_move->dis >= chassis_move->dis_set)    //直接停车
        {
            chassis_move->chassis_cmd_slow_set_vx.out = 0.0f;
            chassis_move->chassis_cmd_slow_set_vy.out = 0.0f;
        }

        vx_s = chassis_move->chassis_cmd_slow_set_vx.out;
        vy_s = chassis_move->chassis_cmd_slow_set_vy.out;
    }
    else if (chassis_move->move_mode == CHASSIS_CAL)
    {
        first_order_filter_cali(&chassis_move->chassis_cmd_slow_set_vx_cal, vx_set_channel);
        first_order_filter_cali(&chassis_move->chassis_cmd_slow_set_vy_cal, vy_set_channel);

        if (chassis_move->dis >= chassis_move->dis_set)    //直接停车
        {
            chassis_move->chassis_cmd_slow_set_vx_cal.out = 0.0f;
            chassis_move->chassis_cmd_slow_set_vy_cal.out = 0.0f;
        }

        vx_s = chassis_move->chassis_cmd_slow_set_vx_cal.out;
        vy_s = chassis_move->chassis_cmd_slow_set_vy_cal.out;
    }


    *vx_set = vx_s;
    *vy_set = vy_s;
}

/**
  * @brief          设置底盘控制设置值, 三运动控制值是通过chassis_behaviour_control_set函数设置的
  * @param[out]     chassis_move_update:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_set_contorl(chassis_move_t* chassis_move_control)
{
    if (chassis_move_control == NULL)
    {
        return;
    }

    float vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;
    //获取三个控制设置值
    chassis_behaviour_control_set(&vx_set, &vy_set, &angle_set, chassis_move_control);

    if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW)
    {
        float delat_angle = 0.0f;
        //设置底盘控制的角度
        delat_angle = theta_format(chassis_move.chassis_yaw_set - chassis_move.chassis_yaw);
        deadband_limit(delat_angle, delat_angle, CHASSIS_YAW_DEADLINE);
        g_fGyro_z = delat_angle;

        //计算旋转的角速度
        chassis_move_control->wz_set = PID_calc(&chassis_move_control->chassis_angle_pid, 0.0f, delat_angle);
        //        chassis_move_control->wz_set = 0; //忽略陀螺仪校准

        //速度限幅
        chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
}

/**
  * @brief          四个麦轮速度是通过三个参数计算出来的
  * @param[in]      vx_set: 纵向速度
  * @param[in]      vy_set: 横向速度
  * @param[in]      wz_set: 旋转速度
  * @param[out]     wheel_speed: 四个麦轮速度
  * @retval         none
  */
static void chassis_vector_to_wheel_speed(const float vx_set, const float vy_set, const float wz_set, float wheel_speed[4])
{
    wheel_speed[0] = -vx_set - vy_set - wz_set;
    wheel_speed[1] = -vx_set + vy_set - wz_set;
    wheel_speed[2] =  vx_set + vy_set - wz_set;
    wheel_speed[3] =  vx_set - vy_set - wz_set;
}

/**
  * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
  * @param[out]     chassis_move_control_loop:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_control_loop(chassis_move_t* chassis_move_control_loop)
{
    float max_vector = 0.0f, vector_rate = 0.0f;
    float temp = 0.0f;
    float wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t i = 0;

    //全向轮运动分解
    chassis_vector_to_wheel_speed(chassis_move_control_loop->vx_set,
                                  chassis_move_control_loop->vy_set,
                                  chassis_move_control_loop->wz_set, wheel_speed);

    //计算轮子控制最大速度，并限制其最大速度
    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis[i].speed_set = wheel_speed[i];
        temp = fabs(chassis_move_control_loop->motor_chassis[i].speed_set);

        if (max_vector < temp)
        {
            max_vector = temp;
        }
    }

    if (max_vector > MAX_WHEEL_SPEED)
    {
        vector_rate = MAX_WHEEL_SPEED / max_vector;

        for (i = 0; i < 4; i++)
        {
            chassis_move_control_loop->motor_chassis[i].speed_set *= vector_rate;
        }
    }

    //计算pid
    for (i = 0; i < 4; i++)
    {
        PID_calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].speed, chassis_move_control_loop->motor_chassis[i].speed_set);
    }

    //赋值电流值
    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_move_control_loop->motor_speed_pid[i].out);
    }
}

void APP_CHASSIS_Init(void)
{
    //停车
    set_motor_current(0, 0, 0, 0);

    //获取初始编码器值
    get_motor_offset();
}

/*
*********************************************************************************************************
*    函 数 名: APP_CHASSIS_MovePID
*    功能说明: 角度环控制小车平移给定步数
*    形    参: _dir  : 宏定义见bsp.h
*              _clk  : 步数
*    返 回 值: 无
*********************************************************************************************************
*/
void APP_CHASSIS_MovePID(chassis_move_mode_e _move_mode, chassis_block_mode_e _block_mode, uint8_t _dir, int64_t _clk)
{
    int64_t k = 1000;
    _clk *= k;
    chassis_move.move_mode = _move_mode;
    chassis_move.dir = _dir;
    chassis_move.dis_set = chassis_move.dis_remaining = _clk;
    set_motor_zero_angle();

    if (chassis_move.dir == CarDirection_Rotate)    //旋转
    {
        chassis_move.dis_set = chassis_move.dis_remaining = 0;
        chassis_move.chassis_yaw_set = _clk / k;    //夹爪指向的度数，逆时针为正

        float delat_angle = 0.0f;
        delat_angle = theta_format(chassis_move.chassis_yaw_set - chassis_move.chassis_yaw);     //计算角度偏差

        while (fabs(delat_angle) >= 0.1)    //当角度偏差大于0.1时阻塞在该程序
        {
            delat_angle = theta_format(chassis_move.chassis_yaw_set - chassis_move.chassis_yaw);
        }

        return;
    }
    else
    {
        for (int i = 0; i < 4; i++)
        {
            // 短暂施加相同小电流，消除静摩擦
            chassis_move.motor_chassis[i].give_current = 1000;  // 实验确定最佳值
        }

        vTaskDelay(20);  // 等待电机微动同步
        set_motor_current(0, 0, 0, 0);
    }

    if (_block_mode == CHASSIS_CALL_BLOCKING)       //阻塞模式，等待车的速度为0
    {
        for (uint16_t i = 0; i < 10; i++)
        {
            xSemaphoreTake(SemMoveHandle, portMAX_DELAY);
        }
    }
    else if (_block_mode == CHASSIS_CALL_NON_BLOCKING)
    {

    }
}

void APP_CHASSIS_Status(STATUS_E _ucStatus)
{
    switch (_ucStatus)
    {
        case STATUS_Start_2_Qr:
            APP_CHASSIS_MovePID(CHASSIS_LONG, CHASSIS_CALL_BLOCKING, CarDirection_Left, 150);
            APP_CHASSIS_MovePID(CHASSIS_LONG, CHASSIS_CALL_BLOCKING, CarDirection_Forward, 600);
            break;

        case STATUS_Qr_2_RawArea:
            APP_CHASSIS_MovePID(CHASSIS_LONG, CHASSIS_CALL_BLOCKING, CarDirection_Forward, 800);
            break;

        case STATUS_RawArea_2_ProFirArea:
            APP_CHASSIS_MovePID(CHASSIS_LONG, CHASSIS_CALL_BLOCKING, CarDirection_Backward, 350);
            APP_CHASSIS_MovePID(CHASSIS_LONG, CHASSIS_CALL_BLOCKING, CarDirection_Rotate, 90);       //夹爪朝向90度
            APP_CHASSIS_MovePID(CHASSIS_LONG, CHASSIS_CALL_BLOCKING, CarDirection_Forward, 1700);
            APP_CHASSIS_MovePID(CHASSIS_LONG, CHASSIS_CALL_BLOCKING, CarDirection_Rotate, 180);      //夹爪朝向180度
            break;

        case STATUS_ProFirArea_2_ProSecArea:
            APP_CHASSIS_MovePID(CHASSIS_LONG, CHASSIS_CALL_BLOCKING, CarDirection_Rotate, 0);        //夹爪朝向0度
            APP_CHASSIS_MovePID(CHASSIS_LONG, CHASSIS_CALL_BLOCKING, CarDirection_Forward, 850);
            APP_CHASSIS_MovePID(CHASSIS_LONG, CHASSIS_CALL_BLOCKING, CarDirection_Rotate, 270);      //夹爪朝向270度
            APP_CHASSIS_MovePID(CHASSIS_LONG, CHASSIS_CALL_BLOCKING, CarDirection_Forward, 850);
            APP_CHASSIS_MovePID(CHASSIS_LONG, CHASSIS_CALL_BLOCKING, CarDirection_Rotate, 90);       //夹爪朝向90度
            break;

        case STATUS_ProSecArea_2_RawArea:
            APP_CHASSIS_MovePID(CHASSIS_LONG, CHASSIS_CALL_BLOCKING, CarDirection_Rotate, 270);      //夹爪朝向270度
            APP_CHASSIS_MovePID(CHASSIS_LONG, CHASSIS_CALL_BLOCKING, CarDirection_Forward, 850);
            APP_CHASSIS_MovePID(CHASSIS_LONG, CHASSIS_CALL_BLOCKING, CarDirection_Rotate, 180);      //夹爪朝向180度
            APP_CHASSIS_MovePID(CHASSIS_LONG, CHASSIS_CALL_BLOCKING, CarDirection_Forward, 850);
            APP_CHASSIS_MovePID(CHASSIS_LONG, CHASSIS_CALL_BLOCKING, CarDirection_Rotate, 0);        //夹爪朝向0度
            break;

        case STATUS_ProSecArea_2_Start:
            APP_CHASSIS_MovePID(CHASSIS_LONG, CHASSIS_CALL_BLOCKING, CarDirection_Rotate, 270);      //夹爪朝向270度
            APP_CHASSIS_MovePID(CHASSIS_LONG, CHASSIS_CALL_BLOCKING, CarDirection_Forward, 700);
            APP_CHASSIS_MovePID(CHASSIS_LONG, CHASSIS_CALL_BLOCKING, CarDirection_Rotate, 180);      //夹爪朝向180度
            APP_CHASSIS_MovePID(CHASSIS_LONG, CHASSIS_CALL_BLOCKING, CarDirection_Forward, 1700);
            APP_CHASSIS_MovePID(CHASSIS_LONG, CHASSIS_CALL_BLOCKING, CarDirection_Left, 400);
            break;

        default:
            break;
    }

    xTaskNotifyGive(Task_MainHandle);
}
