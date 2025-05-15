#ifndef __APP_CHASSIS_H
#define __APP_CHASSIS_H

//����ʼ����һ��ʱ��
#define CHASSIS_TASK_INIT_TIME 357

#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f

//ҡ������
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f


#define MOTOR_DISTANCE_TO_CENTER 0.2f

//����������Ƽ�� 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//����������Ƽ�� 0.002s
#define CHASSIS_CONTROL_TIME 0.002f
//�����������Ƶ�ʣ���δʹ�������
#define CHASSIS_CONTROL_FREQUENCE 500.0f
//����3508���can���͵���ֵ
#define MAX_MOTOR_CAN_CURRENT 10000.0f

//m3508ת���ɵ����ٶ�(m/s)�ı�����
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

//�������̵������ٶ�
#define MAX_WHEEL_SPEED 4.0f
//�����˶��������ǰ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_X 2.0f
//�����˶��������ƽ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_Y 1.5f

#define CHASSIS_WZ_SET_SCALE 0.1f

//chassis motor speed PID
//���̵���ٶȻ�PID
#define M3505_MOTOR_SPEED_PID_KP 15000.0f
#define M3505_MOTOR_SPEED_PID_KI 10.0f
#define M3505_MOTOR_SPEED_PID_KD 0.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

//chassis follow angle PID
//������ת����PID
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
  const float *chassis_INS_angle;            //��ȡ�����ǽ������ŷ����ָ��
  chassis_motor_t motor_chassis[4];          //���̵������
  PID_TypeDef motor_speed_pid[4];           //���̵���ٶ�pid
  PID_TypeDef chassis_angle_pid;            //���̸���Ƕ�pid

  first_order_filter_type_t chassis_cmd_slow_set_vx;  //ʹ��һ�׵�ͨ�˲������趨ֵ
  first_order_filter_type_t chassis_cmd_slow_set_vy;  //ʹ��һ�׵�ͨ�˲������趨ֵ

  float vx;                          //�����ٶ� ǰ������ ǰΪ������λ m/s
  float vy;                          //�����ٶ� ���ҷ��� ��Ϊ��  ��λ m/s
  float wz;                          //������ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
  float vx_set;                      //�����趨�ٶ� ǰ������ ǰΪ������λ m/s
  float vy_set;                      //�����趨�ٶ� ���ҷ��� ��Ϊ������λ m/s
  float wz_set;                      //�����趨��ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
  float chassis_relative_angle;      //��������̨����ԽǶȣ���λ rad
  float chassis_relative_angle_set;  //���������̨���ƽǶ�
  float chassis_yaw_set;             

  float vx_max_speed;  //ǰ����������ٶ� ��λm/s
  float vx_min_speed;  //���˷�������ٶ� ��λm/s
  float vy_max_speed;  //��������ٶ� ��λm/s
  float vy_min_speed;  //�ҷ�������ٶ� ��λm/s
  float chassis_yaw;   //�����Ǻ���̨������ӵ�yaw�Ƕ�
  float chassis_pitch; //�����Ǻ���̨������ӵ�pitch�Ƕ�
  float chassis_roll;  //�����Ǻ���̨������ӵ�roll�Ƕ�

} chassis_move_t;

void APP_CHASSIS_Init( void );
void APP_CHASSIS_WhaleControl(void);
void APP_CHASSIS_MovePID( uint8_t _dir, int32_t _clk );
void APP_CHASSIS_Status(uint8_t _ucStatus);



#endif