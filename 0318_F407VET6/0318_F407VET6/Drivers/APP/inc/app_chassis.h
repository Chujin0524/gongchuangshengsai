#ifndef __APP_CHASSIS_H
#define __APP_CHASSIS_H

//任务开始空闲一段时间
#define CHASSIS_TASK_INIT_TIME 357

#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f

//摇杆死区
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f


#define MOTOR_DISTANCE_TO_CENTER 0.2f

//底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//底盘任务控制间隔 0.002s
#define CHASSIS_CONTROL_TIME 0.002f
//底盘任务控制频率，尚未使用这个宏
#define CHASSIS_CONTROL_FREQUENCE 500.0f
//底盘3508最大can发送电流值
#define MAX_MOTOR_CAN_CURRENT 10000.0f

//m3508转化成底盘速度(m/s)的比例，
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

//单个底盘电机最大速度
#define MAX_WHEEL_SPEED 4.0f
//底盘运动过程最大前进速度
#define NORMAL_MAX_CHASSIS_SPEED_X 2.0f
//底盘运动过程最大平移速度
#define NORMAL_MAX_CHASSIS_SPEED_Y 1.5f

#define CHASSIS_WZ_SET_SCALE 0.1f

//chassis motor speed PID
//底盘电机速度环PID
#define M3505_MOTOR_SPEED_PID_KP 15000.0f
#define M3505_MOTOR_SPEED_PID_KI 10.0f
#define M3505_MOTOR_SPEED_PID_KD 0.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

//chassis follow angle PID
//底盘旋转跟随PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 40.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 6.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.2f

typedef struct
{
  const moto_measure_t *chassis_motor_measure;
  float accel;
  float speed;
  float speed_set;
  int16_t give_current;
} chassis_motor_t;

typedef struct
{
  const float *chassis_INS_angle;            //获取陀螺仪解算出的欧拉角指针
  chassis_motor_t motor_chassis[4];          //底盘电机数据
  PID_TypeDef motor_speed_pid[4];           //底盘电机速度pid
  PID_TypeDef chassis_angle_pid;            //底盘跟随角度pid

  first_order_filter_type_t chassis_cmd_slow_set_vx;  //使用一阶低通滤波减缓设定值
  first_order_filter_type_t chassis_cmd_slow_set_vy;  //使用一阶低通滤波减缓设定值

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

} chassis_move_t;

void APP_CHASSIS_Init( void );
void APP_CHASSIS_WhaleControl(void);
void APP_CHASSIS_MovePID( uint8_t _dir, int32_t _clk );
void APP_CHASSIS_Status(uint8_t _ucStatus);



#endif