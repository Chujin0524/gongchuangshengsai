#ifndef BSP_PID_H
#define BSP_PID_H
#include "main.h"
typedef float fp32;


enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct
{
    uint8_t mode;
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;
    fp32 max_iout;

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];
    fp32 error[3];

} PID_TypeDef;

extern void PID_init(PID_TypeDef *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);


extern fp32 PID_calc(PID_TypeDef *pid, fp32 ref, fp32 set);

extern void PID_clear(PID_TypeDef *pid);


void PID_set(PID_TypeDef *pid,float Kp,float Ki,float Kd);
#endif
