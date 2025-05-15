#ifndef __BSP_PID_H
#define __BSP_PID_H

enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct
{
    uint8_t mode;
    float Kp;
    float Ki;
    float Kd;

    float max_out;
    float max_iout;

    float set;
    float fdb;

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];
    float error[3];

} PID_TypeDef;

void PID_init(PID_TypeDef *pid, uint8_t mode, const float PID[3], float max_out, float max_iout);
float PID_calc(PID_TypeDef *pid, float ref, float set);
void PID_clear(PID_TypeDef *pid);

void PID_set(PID_TypeDef *pid,float Kp,float Ki,float Kd);
#endif
