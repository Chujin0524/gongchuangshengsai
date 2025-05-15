#ifndef __BSP_CAN_2006_H_
#define __BSP_CAN_2006_H_

#include <stdint.h>
#include <stdbool.h>

/* CAN ���ͻ���յ� ID */
typedef enum {
    CAN_2006Moto_ALL_ID = 0x200,
    CAN_2006Moto1_ID = 0x201,
    CAN_2006Moto2_ID = 0x202,
    CAN_2006Moto3_ID = 0x203,
    CAN_2006Moto4_ID = 0x204,
    CAN_2006Moto5_ID = 0x205
} CAN_Message_ID;

#define FILTER_BUF_LEN 5

/* ���յ�����̨����Ĳ����ṹ�� */
typedef struct {
    int16_t speed_rpm;          // ת��
    float real_current;         // ��ʵ����ֵ
    int16_t given_current;      // �涨����ֵ
    uint8_t hall;               // �����ź�
    uint16_t angle;             // ���ԽǶȷ�Χ��[0, 8191]
    uint16_t last_angle;        // �ϴξ��ԽǶ�
    uint16_t offset_angle;      // ƫ��
    int32_t round_cnt;          // ת����Ȧ��
    int32_t total_angle;        // �ܽǶ�
    uint8_t buf_idx;            // ����������
    uint8_t angle_buf[FILTER_BUF_LEN]; // �ǶȻ�����
    uint8_t fited_angle;        // �ʺϵĽǶ�
    uint8_t msg_cnt;            // ��Ϣ����
} moto_measure_t;

/* Extern �������� */
extern volatile int Weak_mode;
extern volatile float distance_r;
extern volatile float distance_big_dia;
/* CAN ��ʼ�������� */
void my_can_filter_init(CAN_HandleTypeDef* hcan);
void my_can_filter_init_recv_all(CAN_HandleTypeDef* _hcan);
void can_filter_recv_special(CAN_HandleTypeDef* hcan, uint8_t filter_number, uint16_t filtered_id);

/* CAN �����շ� */
void CAN1_ReadMsg(void);
void set_moto_current(CAN_HandleTypeDef* hcan, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
void can_receive_onetime(CAN_HandleTypeDef* _hcan, int time);

/* ������ݴ��� */
void get_moto_measure(moto_measure_t* ptr, uint8_t* Data);

/* �˶����� */
void Move_Transfrom(double vx, double vy, double vw, double yaw);
void _M2006Task();
void _DingjiaoPIDTask();
void hwt_cal(int32_t speed0, int32_t speed1, int32_t speed2, int32_t speed3, int32_t w_t);
void stop();

/* ����������ӳ� */
int delay_distance(int dis);
int delay_dis_diagnal(int dis);

/* ��ʼ����������� */
void wheel_init();
void init_calculate_values();

/* �ٶȿ�����б�º��� */
void SpeedUP_track(int32_t speed_max, uint32_t speed_final, uint32_t distance, uint16_t ramp_dis1, uint16_t ramp_dis2, int yaw);
void Ramp_dis(int spd1, int spd2, int dis, int yaw);
void SpeedUP_x(int32_t speed_max, uint32_t speed_final, uint32_t distance, uint16_t ramp_dis1, uint16_t ramp_dis2, int yaw);
void Ramp_disx(int spd1, int spd2, int dis, int yaw);

/* ���ߺ��� */
float my_abs(float data);

#endif /* __BSP_CAN_2006_H_ */