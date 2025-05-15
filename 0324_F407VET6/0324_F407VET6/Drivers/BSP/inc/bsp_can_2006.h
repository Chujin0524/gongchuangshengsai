#ifndef __BSP_CAN_2006_H_
#define __BSP_CAN_2006_H_

/* CAN 发送或接收的 ID */
typedef enum {
    CAN_2006Moto_ALL_ID = 0x200,
    CAN_2006Moto1_ID = 0x201,
    CAN_2006Moto2_ID = 0x202,
    CAN_2006Moto3_ID = 0x203,
    CAN_2006Moto4_ID = 0x204,
    CAN_2006Moto5_ID = 0x205
} CAN_Message_ID;

#define FILTER_BUF_LEN 5

/* 接收到的云台电机的参数结构体 */
typedef struct {
    int16_t speed_rpm;          // 转速
    int16_t real_current;         // 真实电流值
    int16_t given_current;      // 规定电流值
    uint8_t hall;               // 霍尔信号
    uint16_t angle;             // 绝对角度范围：[0, 8191]
    uint16_t last_angle;        // 上次绝对角度
    uint16_t offset_angle;      // 偏角
    int32_t round_cnt;          // 转过的圈数
    int64_t total_angle;        // 总角度
    int64_t last_total_angle;        // 上次总角度
    int64_t total_angle_offset;        // 总角度偏差
    int64_t total_angle_zero;   //总角度零点
    uint8_t buf_idx;            // 缓冲区索引
    uint8_t angle_buf[FILTER_BUF_LEN]; // 角度缓冲区
    uint8_t fited_angle;        // 适合的角度
    uint8_t msg_cnt;            // 消息计数
} motor_measure_t;

/* CAN 数据收发 */
void CAN1_ReadMsg(void);
void get_motor_measure(motor_measure_t* ptr, uint8_t* Data);
void get_motor_offset(void);
void set_motor_zero_angle(void);
void set_motor_current( int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);

#endif /* __BSP_CAN_2006_H_ */