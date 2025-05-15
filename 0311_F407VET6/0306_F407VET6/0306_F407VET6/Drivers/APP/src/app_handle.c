#include "main.h"

/*
ת��λ�� : 
0 : ��ʼ
1 : ָ�����
2 : ָ���м�
3 : ָ���ұ�
4 : ��/��
*/
static uint8_t MOTOR_R_Acc = 10; 
static uint16_t MOTOR_R_Vel = 250; 
static uint32_t MOTOR_R_Pos[4] = {0, 4100, 2100, 90};  

/*
Xλ�� : 
0 : ��ʼ
1 : ָ�����
2 : ָ���м�
3 : ָ���ұ�
4 : 
*/
static uint8_t MOTOR_X_Acc = 10; 
static uint16_t MOTOR_X_Vel = 250; 
static uint32_t MOTOR_X_Pos[4] = {2900, 6700, 4200, 7050};    

/*
Yλ�� : 
0 : ��ߵ�
1 : ��ߵ�-300
2 : ���������Խ��ת�̵�λ��
3 : �������ŵ�ת���ϵ�λ��
4 : �������ŵ�ɫ���ϵ�λ��
5 : �������λ��
6 : �����ɫ���ϣ���צ��Խ����λ��
7 : 
*/
static uint8_t MOTOR_Y_Acc = 10; 
static uint16_t MOTOR_Y_Vel = 250; 
static uint32_t MOTOR_Y_Pos[8] = {0, 300, 1500, 1700, 10000, 6000, 9000, 0}; 

//void APP_HandleMove(void)
//{
//    static CAR_XYR_T *ptCarXYR = NULL;
//    static int16_t k = 1;                     //���ص�ת��Ϊ�ƶ�����ı���    

//    for(;; )
//    {
//        xQueueReceive(Queue_HandleCalculateHandle, &ptCarXYR, portMAX_DELAY);  //�ȴ��������Ӳ���
////        pWhaleParameter->WhalePosX *= k;     //X���ص�ת��Ϊ�ƶ�����
////        pWhaleParameter->WhalePosY *= k;     //Y���ص�ת��Ϊ�ƶ�����

//        xQueueOverwrite(Queue_MotorHandle, (void *)&ptCarXYR);  //����Ϣ����Queue_MotorHandle�������Ӳ���
//    }
//}

void APP_Handle_PickRawArea(uint8_t _ucIndex)
{
     //35����������Ƽ�צ���붥��0.5Ȧ
    APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

    APP_SERVO_Wrist('F');        //�󴦶����ǰ
    
    APP_SERVO_Plate(_ucIndex);       //ת�̶��ת����Ӧ���ķ��ýǶ�
 
    //35����������Ƽ�צ���붥��4Ȧ
    APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[3], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
        
    APP_SERVO_Jaw('Z');     //��צ����н����
   
    //35����������Ƽ�צ���붥��0.5Ȧ
    APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
   
    APP_SERVO_Wrist('R');          //�󴦶������
    
    //35����������Ƽ�צ���붥��1Ȧ
    APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[2], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
    
    APP_SERVO_Jaw('S');     //��צ����ɿ����
    
    //35����������Ƽ�צ���붥��0.5Ȧ
    APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

    APP_SERVO_Wrist('F');        //�󴦶����ǰ
}

/*
ת�̶��seq[1]

��i�η��ã�i��1��2��3
{
                                �󴦶����ת
    �ײ�ת��seq[i]
    x��seq[i]
    �ӳ٣��ȴ������ת���
    y2
                                    ��צ���ץ�����ӳٵȴ�
    y1
                                �󴦶����ǰ���ӳٵȴ�
                        ת�̶��seq[i+1]
    if ProFir��
        y4
    else if ProSec��
        y3
                                    ��צ����ɿ����ӳٵȴ�
     if ��3�Σ�
        if ProFir��
            y5��
        else if ProSec��
            y0
    else
        y1
}
*/
void APP_Handle_Place_All(uint8_t _time, uint8_t _firstIndex, uint8_t _secondIndex, uint8_t _thirdIndex)
{
    uint32_t Delay_up = 400;
    uint32_t Delay_Wrist = 100;
    uint8_t ucSeq[3] = {_firstIndex, _secondIndex, _thirdIndex};
    printf("Seq %d %d %d \r\n", ucSeq[0], ucSeq[1], ucSeq[2]);
    
    APP_SERVO_Plate(_firstIndex);        //ת�̶��ת����Ӧ���ķ��ýǶ�
    
    for (uint8_t i = 0; i < 3; i++)
    {    
        //�󴦶������
        APP_SERVO_Wrist('R');           

        //���Ƶ���ת��ת����ָ��ɫ������
        APP_MOTOR_ZDT_Move_P(MOTOR_R, MOTOR_R_Direction_CR,      MOTOR_R_Vel, MOTOR_R_Acc, MOTOR_R_Pos[ucSeq[i]], 1, 0); 

        //X���������ָ��ɫ��
        APP_MOTOR_ZDT_Move_P(MOTOR_X, MOTOR_X_Direction_FORWARD, MOTOR_X_Vel, MOTOR_X_Acc, MOTOR_X_Pos[ucSeq[i]], 1, 0);

        //�ӳ٣��ȴ������ת���
        vTaskDelay(pdMS_TO_TICKS(Delay_Wrist));

        //Y3���������ŵ�ת���ϵ�λ��
        APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[3], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
        
        //��צ����н���飬�����ӳ�
        APP_SERVO_Jaw('Z');            

        //Y2�����������Խ��ת�̵�λ��
        APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[2], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

        //�󴦶����ǰ���ӳٵȴ�
        APP_SERVO_Wrist('F');           
        vTaskDelay(pdMS_TO_TICKS(Delay_Wrist));
        
        if (i < 2)  APP_SERVO_Plate(ucSeq[i+1]);      //ת�̶��ת����һ�����ķ��ýǶ�
        else        APP_SERVO_Plate(ucSeq[0]);        //ת�̶��ת����һ�����ķ��ýǶ�
        
        if (_time == 1)
        {
            //Y4���������ŵ�ɫ���ϵ�λ��
            APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[4], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
        }
        else if (_time == 2)
        {
            //Y5���������λ��
            APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[5], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
        }
    
        //��צ����ɿ���飬�����ӳ�
        APP_SERVO_Jaw('S');     

        if (i == 2)
        {
            if (_time == 1)
            {
                //Y5�������ɫ���ϣ���צ��Խ����λ��
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[5], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
            }
            else if (_time == 2)
            {
                //Y1����ߵ�-300
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
            }
        }
        else
        {
            //Y2�����������Խ��ת�̵�λ��
            APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[2], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);            
        }
    }
}

/*
��i��ץȡ��i��1��2��3
{
    �󴦶����ǰ
    �ײ�ת��seq[i]
    x��seq[i]
    if i == 0
        �ӳ٣��ȴ����������λ
    else
        �ӳ٣������ǰ���
    y4
    ��צ���ץ�����ӳٵȴ�
    y2
    �󴦶����ת
    y3
    ��צ����ɿ����ӳٵȴ�
    if i < 2
        y2
    else
        y1
    if i == 2
        �󴦶����ǰ
}
*/
void APP_Handle_Pick_All(uint8_t _firstIndex, uint8_t _secondIndex, uint8_t _thirdIndex)
{
    uint32_t Delay_up = 400;
    uint32_t Delay_Wrist = 100;
    uint32_t Delay_xr = 1000;
    uint8_t ucSeq[3] = {_firstIndex, _secondIndex, _thirdIndex};
    printf("Seq %d %d %d \r\n", ucSeq[0], ucSeq[1], ucSeq[2]);
    
    for (uint8_t i = 0; i < 3; i++)
    {    
        //�󴦶����ǰ
        APP_SERVO_Wrist('F');           

        //���Ƶ���ת��ת����ָ��ɫ������
        APP_MOTOR_ZDT_Move_P(MOTOR_R, MOTOR_R_Direction_CR,      MOTOR_R_Vel, MOTOR_R_Acc, MOTOR_R_Pos[ucSeq[i]], 1, 0); 

        //X���������ָ��ɫ��
        APP_MOTOR_ZDT_Move_P(MOTOR_X, MOTOR_X_Direction_FORWARD, MOTOR_X_Vel, MOTOR_X_Acc, MOTOR_X_Pos[ucSeq[i]], 1, 0);
        
        if (i == 0)
        {
            //�ӳ٣��ȴ����������λ
            vTaskDelay(pdMS_TO_TICKS(Delay_xr));
        }
        else
        {
            //�ӳ٣��ȴ������ǰ���
            vTaskDelay(pdMS_TO_TICKS(Delay_Wrist));
        }
        
        //Y4���������ŵ�ɫ���ϵ�λ��
        APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[4], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

        //��צ����н���飬�����ӳ�
        APP_SERVO_Jaw('Z');

        //Y2�����������Խ��ת�̵�λ��
        APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[2], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);            

        //�󴦶������
        APP_SERVO_Wrist('R');           
        vTaskDelay(pdMS_TO_TICKS(Delay_Wrist));

        //Y3���������ŵ�ת���ϵ�λ��
        APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[3], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

        //��צ����ɿ���飬�����ӳ�
        APP_SERVO_Jaw('S');     
        
        if (i < 2)
        {
            //Y2�����������Խ��ת�̵�λ��
            APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[2], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);            
        }
        else
        {
            //Y1����ߵ�-300
            APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
        }
        
        if (i == 2)
        {
            //�󴦶����ǰ
            APP_SERVO_Wrist('F');           
        }
    }
}
