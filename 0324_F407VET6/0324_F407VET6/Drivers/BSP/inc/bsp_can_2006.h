#ifndef __BSP_CAN_2006_H_
#define __BSP_CAN_2006_H_

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
    int16_t real_current;         // ��ʵ����ֵ
    int16_t given_current;      // �涨����ֵ
    uint8_t hall;               // �����ź�
    uint16_t angle;             // ���ԽǶȷ�Χ��[0, 8191]
    uint16_t last_angle;        // �ϴξ��ԽǶ�
    uint16_t offset_angle;      // ƫ��
    int32_t round_cnt;          // ת����Ȧ��
    int64_t total_angle;        // �ܽǶ�
    int64_t last_total_angle;        // �ϴ��ܽǶ�
    int64_t total_angle_offset;        // �ܽǶ�ƫ��
    int64_t total_angle_zero;   //�ܽǶ����
    uint8_t buf_idx;            // ����������
    uint8_t angle_buf[FILTER_BUF_LEN]; // �ǶȻ�����
    uint8_t fited_angle;        // �ʺϵĽǶ�
    uint8_t msg_cnt;            // ��Ϣ����
} motor_measure_t;

/* CAN �����շ� */
void CAN1_ReadMsg(void);
void get_motor_measure(motor_measure_t* ptr, uint8_t* Data);
void get_motor_offset(void);
void set_motor_zero_angle(void);
void set_motor_current( int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);

#endif /* __BSP_CAN_2006_H_ */