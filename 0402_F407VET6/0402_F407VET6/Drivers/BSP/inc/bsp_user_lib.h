#ifndef __BSP_USER_LIB_H
#define __BSP_USER_LIB_H

typedef struct
{
    float input;        //输入数据
    float out;          //输出数据
    float min_value;    //限幅最小值
    float max_value;    //限幅最大值
    float frame_period; //时间间隔
} ramp_function_source_t;

typedef struct
{
    float input;        //输入数据
    float out;          //滤波输出的数据
    float num[1];       //滤波参数
    float frame_period; //滤波的时间间隔 单位 s
} first_order_filter_type_t;


//快速开方
float invSqrt(float num);

//斜波函数初始化
void ramp_init(ramp_function_source_t *ramp_source_type, float frame_period, float max, float min);

//斜波函数计算
void ramp_calc(ramp_function_source_t *ramp_source_type, float input);
//一阶滤波初始化
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, float frame_period, const float num[1]);
//一阶滤波计算
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, float input);
//绝对限制
void abs_limit(float *num, float Limit);
//判断符号位
float sign(float value);
//浮点死区
float fp32_deadline(float Value, float minValue, float maxValue);
//int16死区
int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue);
//限幅函数
float fp32_constrain(float Value, float minValue, float maxValue);
//限幅函数
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);
//循环限幅函数
float loop_fp32_constrain(float Input, float minValue, float maxValue);
//角度 °限幅 180 ~ -180
float theta_format(float Ang);

//弧度格式化为-PI~PI
#define rad_format(Ang) loop_fp32_constrain((Ang), -PI, PI)

#endif
