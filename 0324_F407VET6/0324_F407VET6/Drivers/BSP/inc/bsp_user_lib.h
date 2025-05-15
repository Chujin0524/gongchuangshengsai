#ifndef __BSP_USER_LIB_H
#define __BSP_USER_LIB_H

typedef struct
{
    float input;        //��������
    float out;          //�������
    float min_value;    //�޷���Сֵ
    float max_value;    //�޷����ֵ
    float frame_period; //ʱ����
} ramp_function_source_t;

typedef struct
{
    float input;        //��������
    float out;          //�˲����������
    float num[1];       //�˲�����
    float frame_period; //�˲���ʱ���� ��λ s
} first_order_filter_type_t;


//���ٿ���
float invSqrt(float num);

//б��������ʼ��
void ramp_init(ramp_function_source_t *ramp_source_type, float frame_period, float max, float min);

//б����������
void ramp_calc(ramp_function_source_t *ramp_source_type, float input);
//һ���˲���ʼ��
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, float frame_period, const float num[1]);
//һ���˲�����
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, float input);
//��������
void abs_limit(float *num, float Limit);
//�жϷ���λ
float sign(float value);
//��������
float fp32_deadline(float Value, float minValue, float maxValue);
//int16����
int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue);
//�޷�����
float fp32_constrain(float Value, float minValue, float maxValue);
//�޷�����
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);
//ѭ���޷�����
float loop_fp32_constrain(float Input, float minValue, float maxValue);
//�Ƕ� ���޷� 180 ~ -180
float theta_format(float Ang);

//���ȸ�ʽ��Ϊ-PI~PI
#define rad_format(Ang) loop_fp32_constrain((Ang), -PI, PI)

#endif
