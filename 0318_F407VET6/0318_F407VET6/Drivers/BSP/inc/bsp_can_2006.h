#ifndef __BSP_CAN_2006_H_
#define __BSP_CAN_2006_H_

#include <stdint.h>
#include <stdbool.h>

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
    int32_t total_angle;        // 总角度
    uint8_t buf_idx;            // 缓冲区索引
    uint8_t angle_buf[FILTER_BUF_LEN]; // 角度缓冲区
    uint8_t fited_angle;        // 适合的角度
    uint8_t msg_cnt;            // 消息计数
} moto_measure_t;

/* CAN 数据收发 */
void CAN1_ReadMsg(void);
void get_moto_measure(moto_measure_t* ptr, uint8_t* Data);
void set_moto_current(CAN_HandleTypeDef* hcan, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
const moto_measure_t *get_chassis_motor_measure_point(uint8_t i);



/* Extern 变量声明 */
extern volatile int Weak_mode;
extern volatile float distance_r;
extern volatile float distance_big_dia;
extern PID_TypeDef motor_pid[4];
extern moto_measure_t moto_chassis[4];
/* CAN 初始化与配置 */


/* 电机数据处理 */

/* 运动控制 */
void Move_Transfrom(double vx, double vy, double vw, double yaw);
void _M2006Task();
void _DingjiaoPIDTask();
void hwt_cal(int32_t speed0, int32_t speed1, int32_t speed2, int32_t speed3, int32_t w_t);
void stop();

/* 距离计算与延迟 */
int delay_distance(int dis);
int delay_dis_diagnal(int dis);
int delay_angle( int dis );
int delay_angle_diagnal( int dis );
/* 初始化与参数计算 */
void wheel_init();
void init_calculate_values();

/* 速度控制与斜坡函数 */
void SpeedUP_track(int32_t speed_max, uint32_t speed_final, uint32_t distance, uint16_t ramp_dis1, uint16_t ramp_dis2, int yaw);
void Ramp_dis(int spd1, int spd2, int dis, int yaw);
void SpeedUP_x(int32_t speed_max, uint32_t speed_final, uint32_t distance, uint16_t ramp_dis1, uint16_t ramp_dis2, int yaw);
void Ramp_disx(int spd1, int spd2, int dis, int yaw);

/* 工具函数 */
float my_abs(float data);

#endif /* __BSP_CAN_2006_H_ */