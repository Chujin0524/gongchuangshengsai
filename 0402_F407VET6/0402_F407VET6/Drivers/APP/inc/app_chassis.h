#ifndef __APP_CHASSIS_H
#define __APP_CHASSIS_H

//任务开始空闲一段时间
#define CHASSIS_TASK_INIT_TIME 357

#define CHASSIS_ACCEL_X_NUM 0.1f
#define CHASSIS_ACCEL_Y_NUM 0.1f

#define CHASSIS_ACCEL_X_NUM_CAL 0.333333f
#define CHASSIS_ACCEL_Y_NUM_CAL 0.333333f

#define CHASSIS_YAW_DEADLINE 0.1   //陀螺仪角度死区

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f

//底盘任务控制间隔 5ms
#define CHASSIS_CONTROL_TIME_MS 5
//底盘任务控制间隔 0.005s
#define CHASSIS_CONTROL_TIME 0.005f
//底盘3508最大can发送电流值
#define MAX_MOTOR_CAN_CURRENT 6000.0f

//单个底盘电机最大速度
#define MAX_WHEEL_SPEED 1000.0f
//底盘运动过程最大前进速度
#define NORMAL_MAX_CHASSIS_SPEED_X 1000.0f
//底盘运动过程最大平移速度
#define NORMAL_MAX_CHASSIS_SPEED_Y 1000.0f

#define CHASSIS_WZ_SET_SCALE 0.1f

//底盘电机速度环PID
#define ReductionRatio 36.0f      //M2006电机减速比
#define M2006_MOTOR_SPEED_PID_KP 450.0f
#define M2006_MOTOR_SPEED_PID_KI 50.0f
#define M2006_MOTOR_SPEED_PID_KD 0.0f
#define M2006_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M2006_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

//底盘旋转跟随PID
#define CHASSIS_FOLLOW_CHASSIS_PID_KP 6.0f
#define CHASSIS_FOLLOW_CHASSIS_PID_KI 0.0f
#define CHASSIS_FOLLOW_CHASSIS_PID_KD 0.0f
#define CHASSIS_FOLLOW_CHASSIS_PID_MAX_OUT 100.0f
#define CHASSIS_FOLLOW_CHASSIS_PID_MAX_IOUT 0.0f

#define MOTOR_LF 1
#define MOTOR_LB 2
#define MOTOR_RF 3
#define MOTOR_RB 4

/* 小车的行进方向 */
#define CarDirection_Forward   (1 << 0)       //前      1
#define CarDirection_Backward  (1 << 1)       //后      2
#define CarDirection_Left      (1 << 2)       //左      4
#define CarDirection_Right     (1 << 3)       //右      8
#define CarDirection_Rotate    (1 << 4)       //转      16

#define CarDirection_Y         (CarDirection_Forward | CarDirection_Backward)       //Y方向移动
#define CarDirection_X         (CarDirection_Left    | CarDirection_Right)          //X方向移动

#define CARDIRECTION_Y(Y)     (Y>0) ? CarDirection_Left     :  CarDirection_Right
#define CARDIRECTION_X(X)     (X>0) ? CarDirection_Backward :  CarDirection_Forward

#define deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }


typedef enum
{
    CHASSIS_CALL_BLOCKING,       //阻塞调用
    CHASSIS_CALL_NON_BLOCKING,   //非阻塞调用
} chassis_block_mode_e;

typedef enum
{
    CHASSIS_LONG,   //长距离走
    CHASSIS_CAL,   //校准走
} chassis_move_mode_e;

typedef enum
{
    CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW,   //底盘会跟随云台相对角度
    CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW,  //底盘有底盘角度控制闭环
    CHASSIS_VECTOR_NO_FOLLOW_YAW,       //底盘有旋转速度控制
    CHASSIS_VECTOR_RAW,                 //control-current will be sent to CAN bus derectly.
} chassis_mode_e;

typedef struct
{
    const motor_measure_t *chassis_motor_measure;
    float accel;
    float speed;
    float speed_set;
    int16_t give_current;
} chassis_motor_t;

typedef struct
{
    const float *chassis_INS_angle;            //获取陀螺仪解算出的欧拉角指针
    chassis_mode_e chassis_mode;               //state machine. 底盘控制状态机
    chassis_mode_e last_chassis_mode;          //last state machine.底盘上次控制状态机
    chassis_motor_t motor_chassis[4];          //底盘电机数据
    PID_TypeDef motor_speed_pid[4];           //底盘电机速度pid
    PID_TypeDef chassis_angle_pid;            //底盘跟随角度pid

    first_order_filter_type_t chassis_cmd_slow_set_vx;  //使用一阶低通滤波减缓设定值
    first_order_filter_type_t chassis_cmd_slow_set_vy;  //使用一阶低通滤波减缓设定值

    first_order_filter_type_t chassis_cmd_slow_set_vx_cal;  //使用一阶低通滤波减缓设定值，校准慢速用
    first_order_filter_type_t chassis_cmd_slow_set_vy_cal;  //使用一阶低通滤波减缓设定值，校准慢速用

    int64_t dis_set ;                  //设置行驶距离
    int64_t dis_remaining ;            //剩余行驶距离
    int64_t dis;                       //目前行驶距离
    int64_t last_dis;                       //上次行驶距离
    uint8_t dir;                       //小车方向
    float vx;                          //底盘速度 前进方向 前为正，单位 m/s
    float vy;                          //底盘速度 左右方向 左为正  单位 m/s
    float wz;                          //底盘旋转角速度，逆时针为正 单位 rad/s
    float vx_set;                      //底盘设定速度 前进方向 前为正，单位 m/s
    float vy_set;                      //底盘设定速度 左右方向 左为正，单位 m/s
    float wz_set;                      //底盘设定旋转角速度，逆时针为正 单位 rad/s
    float chassis_relative_angle;      //底盘与云台的相对角度，单位 rad
    float chassis_relative_angle_set;  //设置相对云台控制角度
    float chassis_yaw_set;

    float vx_max_speed;  //前进方向最大速度 单位m/s
    float vx_min_speed;  //后退方向最大速度 单位m/s
    float vy_max_speed;  //左方向最大速度 单位m/s
    float vy_min_speed;  //右方向最大速度 单位m/s
    float chassis_yaw;   //陀螺仪和云台电机叠加的yaw角度
    float chassis_pitch; //陀螺仪和云台电机叠加的pitch角度
    float chassis_roll;  //陀螺仪和云台电机叠加的roll角度

    chassis_move_mode_e move_mode;  //移动模式
} chassis_move_t;

typedef struct
{
    uint8_t WhaleDirX;   // X方向
    uint32_t WhalePosX;  // X距离
    uint8_t WhaleDirY;   // Y方向
    uint32_t WhalePosY;  // Y距离
    uint8_t WhaleDirR;   // R方向  R:Rotate旋转
    uint32_t WhalePosR;  // R距离
} CAR_XYR_T;

void APP_CHASSIS_Init(void);
void APP_CHASSIS_Control(void);
void APP_CHASSIS_MovePID(chassis_move_mode_e _move_mode, chassis_block_mode_e _block_mode, uint8_t _dir, int64_t _clk);
void APP_CHASSIS_Status(uint8_t _ucStatus);
void chassis_control_vector(float* vx_set, float* vy_set, chassis_move_t* chassis_move);

extern chassis_move_t chassis_move;

#endif