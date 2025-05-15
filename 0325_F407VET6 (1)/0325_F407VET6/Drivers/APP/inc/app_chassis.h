#ifndef __APP_CHASSIS_H
#define __APP_CHASSIS_H

//����ʼ����һ��ʱ��
#define CHASSIS_TASK_INIT_TIME 357

#define CHASSIS_ACCEL_X_NUM 0.1f
#define CHASSIS_ACCEL_Y_NUM 0.1f

#define CHASSIS_YAW_DEADLINE 0.1   //�����ǽǶ�����

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f

//����������Ƽ�� 5ms
#define CHASSIS_CONTROL_TIME_MS 5
//����������Ƽ�� 0.005s
#define CHASSIS_CONTROL_TIME 0.005f
//����3508���can���͵���ֵ
#define MAX_MOTOR_CAN_CURRENT 6000.0f

//�������̵������ٶ�
#define MAX_WHEEL_SPEED 1000.0f
//�����˶��������ǰ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_X 1000.0f
//�����˶��������ƽ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_Y 1000.0f

#define CHASSIS_WZ_SET_SCALE 0.1f

//���̵���ٶȻ�PID
#define ReductionRatio 36.0f      //M2006������ٱ�
#define M2006_MOTOR_SPEED_PID_KP 450.0f
#define M2006_MOTOR_SPEED_PID_KI 50.0f
#define M2006_MOTOR_SPEED_PID_KD 0.0f
#define M2006_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M2006_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

//������ת����PID
#define CHASSIS_FOLLOW_CHASSIS_PID_KP 6.0f
#define CHASSIS_FOLLOW_CHASSIS_PID_KI 0.0f
#define CHASSIS_FOLLOW_CHASSIS_PID_KD 0.0f
#define CHASSIS_FOLLOW_CHASSIS_PID_MAX_OUT 100.0f
#define CHASSIS_FOLLOW_CHASSIS_PID_MAX_IOUT 0.0f

#define MOTOR_LF 1
#define MOTOR_LB 2
#define MOTOR_RF 3
#define MOTOR_RB 4

/* С�����н����� */
#define CarDirection_Forward   (1 << 0)       //ǰ      1
#define CarDirection_Backward  (1 << 1)       //��      2
#define CarDirection_Left      (1 << 2)       //��      4
#define CarDirection_Right     (1 << 3)       //��      8
#define CarDirection_Rotate    (1 << 4)       //ת      16

#define CarDirection_Y         (CarDirection_Forward | CarDirection_Backward)       //Y�����ƶ�
#define CarDirection_X         (CarDirection_Left    | CarDirection_Right)          //X�����ƶ�

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
  CHASSIS_CALL_BLOCKING,       //��������
  CHASSIS_CALL_NON_BLOCKING ,  //����������
} chassis_block_mode_e;

typedef enum
{
  CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW,   //���̻������̨��ԽǶ�
  CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW,  //�����е��̽Ƕȿ��Ʊջ�
  CHASSIS_VECTOR_NO_FOLLOW_YAW,       //��������ת�ٶȿ���
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
  const float *chassis_INS_angle;            //��ȡ�����ǽ������ŷ����ָ��
  chassis_mode_e chassis_mode;               //state machine. ���̿���״̬��
  chassis_mode_e last_chassis_mode;          //last state machine.�����ϴο���״̬��
  chassis_motor_t motor_chassis[4];          //���̵������
  PID_TypeDef motor_speed_pid[4];           //���̵���ٶ�pid
  PID_TypeDef chassis_angle_pid;            //���̸���Ƕ�pid

  first_order_filter_type_t chassis_cmd_slow_set_vx;  //ʹ��һ�׵�ͨ�˲������趨ֵ
  first_order_filter_type_t chassis_cmd_slow_set_vy;  //ʹ��һ�׵�ͨ�˲������趨ֵ

  int64_t dis_set ;                  //������ʻ����
  int64_t dis_remaining ;            //ʣ����ʻ����
  int64_t dis;                       //Ŀǰ��ʻ����
  int64_t last_dis;                       //�ϴ���ʻ����
  uint8_t dir;                       //С������
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

typedef struct 
{
    uint8_t WhaleDirX;   // X����
    uint32_t WhalePosX;  // X����
    uint8_t WhaleDirY;   // Y����
    uint32_t WhalePosY;  // Y����
    uint8_t WhaleDirR;   // R����  R:Rotate��ת
    uint32_t WhalePosR;  // R����
}CAR_XYR_T;

void APP_CHASSIS_Init( void );
void APP_CHASSIS_Control( void );
void APP_CHASSIS_MovePID( chassis_block_mode_e _block_mode, uint8_t _dir, int64_t _clk );
void APP_CHASSIS_Status(uint8_t _ucStatus);
void chassis_control_vector( float* vx_set, float* vy_set, chassis_move_t* chassis_move );

extern chassis_move_t chassis_move;

#endif