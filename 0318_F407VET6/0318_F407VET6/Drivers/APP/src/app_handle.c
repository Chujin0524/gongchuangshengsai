#include "main.h"

/*
ת��λ�� : 
0 : ��ʼ
1 : ָ�����
2 : ָ���м�
3 : ָ���ұ�
4 : ��/��
*/
static uint8_t MOTOR_R_Acc = 0; 
static uint16_t MOTOR_R_Vel = 200; 
static uint32_t MOTOR_R_Pos[4] = {0, 4100, 2100, 90};  

/*
Xλ�� : 
0 : ��ʼ
1 : ָ�����
2 : ָ���м�
3 : ָ���ұ�
4 : 
*/
static uint8_t MOTOR_X_Acc = 0; 
static uint16_t MOTOR_X_Vel = 200; 
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
static uint8_t MOTOR_Y_Acc = 0; 
static uint16_t MOTOR_Y_Vel = 800; 
static uint32_t MOTOR_Y_Pos[8] = {0, 300, 0, 1600, 9000, 6000, 8000, 0}; 

// ɫ��˳��ӳ���
// ɫ��˳������ֽ��Ӧ
uint8_t DirMapping[2][4] = {0};  // [��i�μӹ���][��, ��, ��] = [������]

uint32_t *APP_Handle_GetPosPoint(uint8_t _Motor)
{
    switch (_Motor)
    {
        case MOTOR_R:
            return MOTOR_R_Pos;
        case MOTOR_X:
            return MOTOR_X_Pos;
        case MOTOR_Y:
            return MOTOR_Y_Pos;
        default:
            return 0;
    }
}

uint8_t (*APP_Handle_GetMapPoint(void))[4]
{
    return DirMapping;
}

// ��ʼ��ɫ����ֽ��RGB��Ӧ�ķ���
void APP_Handle_ColorMapping_Init(uint8_t _time, uint8_t RedDir, uint8_t GreenDir, uint8_t BlueDir)
{
    DirMapping[_time][RED]   = RedDir;
    DirMapping[_time][GREEN] = GreenDir;
    DirMapping[_time][BLUE]  = BlueDir;
}

void APP_Handle_PickRawArea(uint8_t _ucIndex)
{
     //35����������Ƽ�צ���붥��0.5Ȧ
    APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); APP_MOTOR_ZDT_WaitAck();

    APP_SERVO_Wrist('F');        //�󴦶����ǰ
    
    APP_SERVO_Plate(_ucIndex);       //ת�̶��ת����Ӧ���ķ��ýǶ�
 
    //35����������Ƽ�צ���붥��4Ȧ
    APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[3], 1, 0); APP_MOTOR_ZDT_WaitAck();
        
    APP_SERVO_Jaw('Z');     //��צ����н����
   
    //35����������Ƽ�צ���붥��0.5Ȧ
    APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); APP_MOTOR_ZDT_WaitAck();
   
    APP_SERVO_Wrist('R');          //�󴦶������
    
    //35����������Ƽ�צ���붥��1Ȧ
    APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[2], 1, 0); APP_MOTOR_ZDT_WaitAck();
    
    APP_SERVO_Jaw('S');     //��צ����ɿ����
    
    //35����������Ƽ�צ���붥��0.5Ȧ
    APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); APP_MOTOR_ZDT_WaitAck();

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
    uint32_t Delay_Wrist = 400;
    uint8_t ucSeq[3] = {DirMapping[_time][_firstIndex], DirMapping[_time][_secondIndex], DirMapping[_time][_thirdIndex]};
    App_Printf("ColorSeq %d %d %d \r\n", _firstIndex, _firstIndex, _firstIndex);
    App_Printf("DirSeq %d %d %d \r\n", ucSeq[0], ucSeq[1], ucSeq[2]);
//      DirMapping[0][] = B G R
    APP_SERVO_Plate(_firstIndex);        //ת�̶��ת����Ӧ���ķ��ýǶ�
    
    for (uint8_t i = 0; i < 3; i++)
    {    
        //�󴦶������
        APP_SERVO_Wrist('R');           

        //���Ƶ���ת��ת����ָ��ɫ������
        APP_MOTOR_ZDT_Move_P(MOTOR_R, MOTOR_R_Direction_CCR,     MOTOR_R_Vel, MOTOR_R_Acc, MOTOR_R_Pos[ucSeq[i]], 1, 0); 

        //X���������ָ��ɫ��
        APP_MOTOR_ZDT_Move_P(MOTOR_X, MOTOR_X_Direction_FORWARD, MOTOR_X_Vel, MOTOR_X_Acc, MOTOR_X_Pos[ucSeq[i]], 1, 0);

        //�ӳ٣��ȴ������ת���
        vTaskDelay(pdMS_TO_TICKS(Delay_Wrist));

        //Y3���������ŵ�ת���ϵ�λ��
        APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[3], 1, 0); APP_MOTOR_ZDT_WaitAck();
        
        //��צ����н���飬�����ӳ�
        APP_SERVO_Jaw('Z');            

        //Y2�����������Խ��ת�̵�λ��
        APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[2], 1, 0); APP_MOTOR_ZDT_WaitAck();

        //�󴦶����ǰ���ӳٵȴ�
        APP_SERVO_Wrist('F');           
        vTaskDelay(pdMS_TO_TICKS(Delay_Wrist));
        
        if (i < 2)  APP_SERVO_Plate(ucSeq[i+1]);      //ת�̶��ת����һ�����ķ��ýǶ�
        else        APP_SERVO_Plate(ucSeq[0]);        //ת�̶��ת����һ�����ķ��ýǶ�
        
        if (_time == 1)
        {
            //Y4���������ŵ�ɫ���ϵ�λ��
            APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[4], 1, 0); APP_MOTOR_ZDT_WaitAck();
        }
        else if (_time == 2)
        {
            //Y5���������λ��
            APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[5], 1, 0); APP_MOTOR_ZDT_WaitAck();
        }
    
        //��צ����ɿ���飬�����ӳ�
        APP_SERVO_Jaw('S');     

        if (i == 2)
        {
            if (_time == 1)
            {
                //Y5�������ɫ���ϣ���צ��Խ����λ��
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[5], 1, 0); APP_MOTOR_ZDT_WaitAck();
            }
            else if (_time == 2)
            {
                //Y1����ߵ�-300
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); APP_MOTOR_ZDT_WaitAck();
            }
        }
        else
        {
            //Y2�����������Խ��ת�̵�λ��
            APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[2], 1, 0); APP_MOTOR_ZDT_WaitAck();            
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
    uint32_t Delay_Wrist = 400;
    uint32_t Delay_xr = 500;
    uint8_t ucSeq[3] = {DirMapping[Fir][_firstIndex], DirMapping[Fir][_secondIndex], DirMapping[Fir][_thirdIndex]};
    App_Printf("ColorSeq %d %d %d \r\n", _firstIndex, _firstIndex, _firstIndex);
    App_Printf("DirSeq %d %d %d \r\n", ucSeq[0], ucSeq[1], ucSeq[2]);
    
    for (uint8_t i = 0; i < 3; i++)
    {    
        //�󴦶����ǰ
        APP_SERVO_Wrist('F');           

        //���Ƶ���ת��ת����ָ��ɫ������
        APP_MOTOR_ZDT_Move_P(MOTOR_R, MOTOR_R_Direction_CCR,     MOTOR_R_Vel, MOTOR_R_Acc, MOTOR_R_Pos[ucSeq[i]], 1, 0); 

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
        APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[4], 1, 0); APP_MOTOR_ZDT_WaitAck();

        //��צ����н���飬�����ӳ�
        APP_SERVO_Jaw('Z');

        //Y2�����������Խ��ת�̵�λ��
        APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[2], 1, 0); APP_MOTOR_ZDT_WaitAck();            

        //�󴦶������
        APP_SERVO_Wrist('R');           
        vTaskDelay(pdMS_TO_TICKS(Delay_Wrist));

        //Y3���������ŵ�ת���ϵ�λ��
        APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[3], 1, 0); APP_MOTOR_ZDT_WaitAck();

        //��צ����ɿ���飬�����ӳ�
        APP_SERVO_Jaw('S');     
        
        if (i < 2)
        {
            //Y2�����������Խ��ת�̵�λ��
            APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[2], 1, 0); APP_MOTOR_ZDT_WaitAck();            
        }
        else
        {
            //Y1����ߵ�-300
            APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); APP_MOTOR_ZDT_WaitAck();
        }
        
        if (i == 2)
        {
            //�󴦶����ǰ
            APP_SERVO_Wrist('F');
            
            //���Ƶ���ת��ת�����м�ɫ������
            APP_MOTOR_ZDT_Move_P(MOTOR_R, MOTOR_R_Direction_CCR,     MOTOR_R_Vel, MOTOR_R_Acc, MOTOR_R_Pos[MIDDLE], 1, 0); 

            //X����������м�ɫ��
            APP_MOTOR_ZDT_Move_P(MOTOR_X, MOTOR_X_Direction_FORWARD, MOTOR_X_Vel, MOTOR_X_Acc, MOTOR_X_Pos[MIDDLE], 1, 0);
        }
    }
}

void APP_Handle_Suo(void)
{
    APP_MOTOR_ZDT_Move_P(MOTOR_R, MOTOR_R_Direction_CCR,     MOTOR_R_Vel, MOTOR_R_Acc, MOTOR_R_Pos[MIDDLE], 1, 0); 
    APP_MOTOR_ZDT_Move_P(MOTOR_X, MOTOR_X_Direction_FORWARD, MOTOR_X_Vel, MOTOR_X_Acc, MOTOR_X_Pos[MIDDLE], 1, 0);
}
