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
2 : ָ�����
3 : ָ�����
4 : 
*/
static uint8_t MOTOR_X_Acc = 10; 
static uint16_t MOTOR_X_Vel = 250; 
static uint32_t MOTOR_X_Pos[4] = {2900, 6700, 4200, 7050};    

/*
Yλ�� : 
0 : ��ʼ
1 : ����
2 : ת���Ϸ�
3 : ԭ����
4 : �ӹ�����һ��
5 : �ӹ����ڶ���
6 : ������������֮���צ�����ĵ�λ��
7 : 
*/
static uint8_t MOTOR_Y_Acc = 10; 
static uint16_t MOTOR_Y_Vel = 250; 
static uint32_t MOTOR_Y_Pos[8] = {0, 350, 1500, 7100, 12700, 8150, 3500, 1500}; 

void APP_HandleMove(void)
{
    static CAR_XYR_T *ptCarXYR = NULL;
    static int16_t k = 1;                     //���ص�ת��Ϊ�ƶ�����ı���    

    for(;; )
    {
        xQueueReceive(Queue_HandleCalculateHandle, &ptCarXYR, portMAX_DELAY);  //�ȴ��������Ӳ���
//        pWhaleParameter->WhalePosX *= k;     //X���ص�ת��Ϊ�ƶ�����
//        pWhaleParameter->WhalePosY *= k;     //Y���ص�ת��Ϊ�ƶ�����

        xQueueOverwrite(Queue_MotorHandle, (void *)&ptCarXYR);  //����Ϣ����Queue_MotorHandle�������Ӳ���
    }    
}

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
������ת�󴦶������ͬʱ�����X�ᵼ�죬����ucaGrabSequence[0+3*ucCycle]����תת����ָ�������ɫ��
��ֱ�����½�����צ�����ȡ��飬
������ֱ���ˣ�������ת�󴦶������ֱ�����½���ͬʱ����ת����ת�̣�
�ɿ���צ������鵽ɫ����������ֱ����

������ֱ���˵�ͬʱ����ת����ת�̻����ƶ�X�ᵼ�죬
������ת�󴦶������ͬʱ���ƶ�X�ᵼ�����ת����ת�̣�����ucaGrabSequence[1+3*ucCycle]����תת����ָ�������ɫ��
��ֱ�����½�����צ�����ȡ��飬
������ֱ���ˣ�������ת�󴦶������ֱ�����½���ͬʱ����ת����ת�̣�
�ɿ���צ������鵽ɫ����������ֱ����

������ֱ���˵�ͬʱ����ת����ת�̻����ƶ�X�ᵼ�죬
������ת�󴦶������ͬʱ���ƶ�X�ᵼ�����ת����ת�̣�����ucaGrabSequence[2+3*ucCycle]����תת����ָ�������ɫ��
��ֱ�����½�����צ�����ȡ��飬
������ֱ���ˣ�������ת�󴦶������ֱ�����½���ͬʱ����ת����ת�̣�
�ɿ���צ������鵽ɫ����������ֱ����

�����������ý���������ֻ����һ��߶ȣ��Ա����ץȡ�����ƶ����̾���
*/
void APP_Handle_Place_All(uint8_t _time, uint8_t _firstIndex, uint8_t _secondIndex, uint8_t _thirdIndex)
{
    uint32_t Delay_up = 400;
    printf("Seq %d %d %d \r\n", _firstIndex, _secondIndex, _thirdIndex);
    if (_firstIndex == 1 || _firstIndex == 3)
    {
        //��һ��
        {
            APP_SERVO_Jaw('S');            //��צ����ɿ����
            APP_SERVO_Plate(_firstIndex);        //ת�̶��ת����Ӧ���ķ��ýǶ�
            
            //35����������Ƽ�צ���붥��0.5Ȧ    
            APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
            APP_SERVO_Wrist('R');           //�󴦶������

            //X���������ָ��ɫ��
            APP_MOTOR_ZDT_Move_P(MOTOR_X, MOTOR_X_Direction_FORWARD, MOTOR_X_Vel, MOTOR_X_Acc, MOTOR_X_Pos[_firstIndex], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_X);
            
            //35����������Ƽ�צ���붥��1Ȧ
            APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[2], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

            //���Ƶ���ת��ת����ָ��ɫ������
            APP_MOTOR_ZDT_Move_P(MOTOR_R, MOTOR_R_Direction_CR, MOTOR_R_Vel, MOTOR_R_Acc, MOTOR_R_Pos[_firstIndex], 1, 0); 

            APP_SERVO_Jaw('Z');         //��צ����н����
         
            //35����������Ƽ�צ���붥��0.5Ȧ
            APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

            APP_SERVO_Wrist('F');           //�󴦶����ǰ    
          
            //35����������Ƽ�צ���붥��4Ȧ
            APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[4], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
            APP_SERVO_Jaw('S');         //��צ996���ת30�ȣ��ɿ����
                 
            //35����������Ƽ�צ���붥��1Ȧ    
            APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); 
            vTaskDelay(Delay_up);

            APP_MOTOR_ZDT_WaitAck(MOTOR_R);
            //���Ƶ���ת��ת����ָ��ɫ������
            APP_MOTOR_ZDT_Move_P(MOTOR_R, MOTOR_R_Direction_CR, MOTOR_R_Vel, MOTOR_R_Acc, MOTOR_R_Pos[_secondIndex], 1, 0);
        }
        if (_secondIndex == 2) //�ڶ������м��
        {
            //�ڶ���
            {
                APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
                
                APP_SERVO_Plate(_secondIndex);        //ת�̶��ת����Ӧ���ķ��ýǶ�
                APP_SERVO_Wrist('R');           //�󴦶������
                APP_MOTOR_ZDT_Move_P(MOTOR_X, MOTOR_X_Direction_FORWARD, MOTOR_X_Vel, MOTOR_X_Acc, MOTOR_X_Pos[_secondIndex], 1, 0);                   //X���������ָ��ɫ��
                
                
                //35����������Ƽ�צ���붥��1Ȧ
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[2], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

                APP_SERVO_Jaw('Z');            //��צ����н����
             
                //35����������Ƽ�צ���붥��0.5Ȧ
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
                ////printf("3-5\r\n");

                APP_SERVO_Wrist('F');            //�󴦶����ǰ    
                
              
                //35����������Ƽ�צ���붥��4Ȧ
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[4], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
                APP_SERVO_Jaw('S');            //��צ996���ת30�ȣ��ɿ����
                     
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0);            //35����������Ƽ�צ���붥��1Ȧ    
                vTaskDelay(500);
                APP_MOTOR_ZDT_Move_P(MOTOR_X, MOTOR_X_Direction_FORWARD, MOTOR_X_Vel, MOTOR_X_Acc, MOTOR_X_Pos[_thirdIndex], 1, 0);                   //X���������ָ��ɫ��
            }
            //������
            {
                APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
                
                APP_SERVO_Plate(_thirdIndex);        //ת�̶��ת����Ӧ���ķ��ýǶ�

                APP_SERVO_Wrist('R');            //�󴦶������            
                APP_MOTOR_ZDT_WaitAck(MOTOR_R);            
                APP_MOTOR_ZDT_Move_P(MOTOR_R, MOTOR_R_Direction_CR, MOTOR_R_Vel, MOTOR_R_Acc, MOTOR_R_Pos[_thirdIndex], 1, 0);     //���Ƶ���ת��ת����ָ��ɫ������           
                
                
                //35����������Ƽ�צ���붥��1Ȧ
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[2], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
                ////printf("3-4\r\n");

                APP_SERVO_Jaw('Z');            //��צ����н����
             
                //35����������Ƽ�צ���붥��0.5Ȧ
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
                ////printf("3-5\r\n");

                APP_SERVO_Wrist('F');            //�󴦶����ǰ    
                
              
                //35����������Ƽ�צ���붥��4Ȧ
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[4], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
                APP_SERVO_Jaw('S');            //��צ996���ת30�ȣ��ɿ����
                     
                //35����������Ƽ�צ���붥��1Ȧ    
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[6], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

                APP_MOTOR_ZDT_WaitAck(MOTOR_R);            
            }
        }
        else    //�ڶ������ұߵ�
        {
            //�ڶ���
            {
                APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
                
                APP_SERVO_Plate(_secondIndex);        //ת�̶��ת����Ӧ���ķ��ýǶ�

                APP_SERVO_Wrist('R');            //�󴦶������
                APP_MOTOR_ZDT_Move_P(MOTOR_X, MOTOR_X_Direction_FORWARD, MOTOR_X_Vel, MOTOR_X_Acc, MOTOR_X_Pos[_secondIndex], 1, 0);                   //X���������ָ��ɫ��
                
                
                //35����������Ƽ�צ���붥��1Ȧ
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[2], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

                APP_SERVO_Jaw('Z');            //��צ����н����
             
                //35����������Ƽ�צ���붥��0.5Ȧ
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
                ////printf("3-5\r\n");

                APP_SERVO_Wrist('F');            //�󴦶����ǰ    
                
              
                //35����������Ƽ�צ���붥��4Ȧ
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[4], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
                APP_SERVO_Jaw('S');            //��צ996���ת30�ȣ��ɿ����
                     
                //35����������Ƽ�צ���붥��1Ȧ
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0);                
                vTaskDelay(Delay_up);

                APP_MOTOR_ZDT_WaitAck(MOTOR_R);            
                APP_MOTOR_ZDT_Move_P(MOTOR_R, MOTOR_R_Direction_CR, MOTOR_R_Vel, MOTOR_R_Acc, MOTOR_R_Pos[_thirdIndex], 1, 0);     //���Ƶ���ת��ת����ָ��ɫ������           
            }
            //������
            {
                APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
                
                APP_SERVO_Plate(_thirdIndex);        //ת�̶��ת����Ӧ���ķ��ýǶ�

                APP_SERVO_Wrist('R');            //�󴦶������
                APP_MOTOR_ZDT_Move_P(MOTOR_X, MOTOR_X_Direction_FORWARD, MOTOR_X_Vel, MOTOR_X_Acc, MOTOR_X_Pos[_thirdIndex], 1, 0);                   //X���������ָ��ɫ��
                
                
                //35����������Ƽ�צ���붥��1Ȧ
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[2], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

                APP_SERVO_Jaw('Z');            //��צ����н����
             
                //35����������Ƽ�צ���붥��0.5Ȧ
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

                APP_SERVO_Wrist('F');            //�󴦶����ǰ    
                
              
                //35����������Ƽ�צ���붥��4Ȧ
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[4], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
                APP_SERVO_Jaw('S');            //��צ996���ת30�ȣ��ɿ����
                     
                //35����������Ƽ�צ���붥��1Ȧ
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[6], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
                APP_MOTOR_ZDT_WaitAck(MOTOR_R);            
            }
        }
    }
    if (_firstIndex == 2)
    {
        //��һ��
        {
            APP_SERVO_Jaw('S');            //��צ����ɿ����
            APP_SERVO_Plate(_firstIndex);        //ת�̶��ת����Ӧ���ķ��ýǶ�
            
            //35����������Ƽ�צ���붥��0.5Ȧ    
            APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
            APP_SERVO_Wrist('R');            //�󴦶������
            
            
            //35����������Ƽ�צ���붥��1Ȧ
            APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[2], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

            APP_SERVO_Jaw('Z');            //��צ����н����
         
            //35����������Ƽ�צ���붥��0.5Ȧ
            APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

            APP_SERVO_Wrist('F');            //�󴦶����ǰ    
            
          
            //35����������Ƽ�צ���붥��4Ȧ
            APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[4], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
            APP_SERVO_Jaw('S');            //��צ996���ת30�ȣ��ɿ����
                 
            //35����������Ƽ�צ���붥��1Ȧ    
            APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0);            
            vTaskDelay(Delay_up);
            APP_MOTOR_ZDT_Move_P(MOTOR_X, MOTOR_X_Direction_FORWARD, MOTOR_X_Vel, MOTOR_X_Acc, MOTOR_X_Pos[_secondIndex], 1, 0);                   //X���������ָ��ɫ��
        }
//        if (_secondIndex == 2) //�ڶ������м��
        {
            //�ڶ���
            {
                APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
                
                APP_SERVO_Plate(_secondIndex);        //ת�̶��ת����Ӧ���ķ��ýǶ�
                APP_SERVO_Wrist('R');            //�󴦶������
                APP_MOTOR_ZDT_Move_P(MOTOR_R, MOTOR_R_Direction_CR, MOTOR_R_Vel, MOTOR_R_Acc, MOTOR_R_Pos[_secondIndex], 1, 0);     //���Ƶ���ת��ת����ָ��ɫ������           
                
                
                //35����������Ƽ�צ���붥��1Ȧ
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[2], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

                APP_SERVO_Jaw('Z');            //��צ����н����
             
                //35����������Ƽ�צ���붥��0.5Ȧ
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

                APP_SERVO_Wrist('F');            //�󴦶����ǰ    
                
              
                //35����������Ƽ�צ���붥��4Ȧ
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[4], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
                APP_SERVO_Jaw('S');            //��צ996���ת30�ȣ��ɿ����
                     
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0);            //35����������Ƽ�צ���붥��1Ȧ    
                vTaskDelay(Delay_up);

                APP_MOTOR_ZDT_WaitAck(MOTOR_R);
                APP_MOTOR_ZDT_Move_P(MOTOR_R, MOTOR_R_Direction_CR, MOTOR_R_Vel, MOTOR_R_Acc, MOTOR_R_Pos[_thirdIndex], 1, 0);     //���Ƶ���ת��ת����ָ��ɫ������       
                vTaskDelay(500);
            }
            //������
            {
                APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
                
                APP_SERVO_Plate(_thirdIndex);        //ת�̶��ת����Ӧ���ķ��ýǶ�
                APP_SERVO_Wrist('R');            //�󴦶������            
                APP_MOTOR_ZDT_Move_P(MOTOR_X, MOTOR_X_Direction_FORWARD, MOTOR_X_Vel, MOTOR_X_Acc, MOTOR_X_Pos[_thirdIndex], 1, 0);                   //X���������ָ��ɫ��
                
                
                //35����������Ƽ�צ���붥��1Ȧ
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[2], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

                APP_SERVO_Jaw('Z');            //��צ����н����
             
                //35����������Ƽ�צ���붥��0.5Ȧ
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

                APP_SERVO_Wrist('F');            //�󴦶����ǰ    
                
              
                //35����������Ƽ�צ���붥��4Ȧ
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[4], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
                APP_SERVO_Jaw('S');            //��צ996���ת30�ȣ��ɿ����
                     
                //35����������Ƽ�צ���붥��1Ȧ
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[6], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

                APP_MOTOR_ZDT_WaitAck(MOTOR_R);            
            }
        }       
    }
}

void APP_Handle_Pick_All(uint8_t _firstIndex, uint8_t _secondIndex, uint8_t _thirdIndex)
{
    uint32_t Delay_up = 400;
    uint32_t Delay_turn = 500;
    printf("Seq %d %d %d \r\n", _firstIndex, _secondIndex, _thirdIndex);
    if (_firstIndex == 1 || _firstIndex == 3)
    {
        //��һ��
        {
            if (_thirdIndex == 2)   //��������õ����м��
            {
                APP_MOTOR_ZDT_Move_P(MOTOR_X, MOTOR_X_Direction_FORWARD, MOTOR_X_Vel, MOTOR_X_Acc, MOTOR_X_Pos[_firstIndex], 1, 0);                   //X���������ָ��ɫ��
                vTaskDelay(Delay_turn);
                APP_MOTOR_ZDT_Move_P(MOTOR_R, MOTOR_R_Direction_CR, MOTOR_R_Vel, MOTOR_R_Acc, MOTOR_R_Pos[_firstIndex], 1, 0);     //���Ƶ���ת��ת����ָ��ɫ������
                APP_MOTOR_ZDT_WaitAck(MOTOR_R);
            }
            else
            {
                APP_MOTOR_ZDT_Move_P(MOTOR_R, MOTOR_R_Direction_CR, MOTOR_R_Vel, MOTOR_R_Acc, MOTOR_R_Pos[_firstIndex], 1, 0);     //���Ƶ���ת��ת����ָ��ɫ������
                APP_MOTOR_ZDT_WaitAck(MOTOR_R);
                APP_MOTOR_ZDT_Move_P(MOTOR_X, MOTOR_X_Direction_FORWARD, MOTOR_X_Vel, MOTOR_X_Acc, MOTOR_X_Pos[_firstIndex], 1, 0);                   //X���������ָ��ɫ��
            }
            
            //35����������Ƽ�צ���붥��4Ȧ
            APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[4], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
            APP_SERVO_Jaw('Z');            //��צ996���ת30�ȣ��ɿ����
                 
            //35����������Ƽ�צ���붥��1Ȧ
            APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0);                
            vTaskDelay(Delay_up);

            APP_MOTOR_ZDT_Move_P(MOTOR_R, MOTOR_R_Direction_CR, MOTOR_R_Vel, MOTOR_R_Acc, MOTOR_R_Pos[_secondIndex], 1, 0);     //���Ƶ���ת��ת����ָ��ɫ������           
        }
        if (_secondIndex == 2) //�ڶ������м��
        {
            //�ڶ���
            {
                APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
                
                APP_SERVO_Plate(_firstIndex);        //ת�̶��ת����Ӧ���ķ��ýǶ�
                APP_SERVO_Wrist('R');            //�󴦶������
                APP_MOTOR_ZDT_Move_P(MOTOR_X, MOTOR_X_Direction_FORWARD, MOTOR_X_Vel, MOTOR_X_Acc, MOTOR_X_Pos[_secondIndex], 1, 0);                   //X���������ָ��ɫ��
                
                
                //35����������Ƽ�צ���붥��1Ȧ
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[2], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

                APP_SERVO_Jaw('S');            //��צ����н����
             
                //35����������Ƽ�צ���붥��0.5Ȧ
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

                APP_SERVO_Wrist('F');            //�󴦶����ǰ    
                
              
                //35����������Ƽ�צ���붥��4Ȧ
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[4], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
                APP_SERVO_Jaw('Z');            //��צ996���ת30�ȣ��ɿ����
                     
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0);            //35����������Ƽ�צ���붥��1Ȧ    
                vTaskDelay(Delay_up);
                APP_MOTOR_ZDT_Move_P(MOTOR_X, MOTOR_X_Direction_FORWARD, MOTOR_X_Vel, MOTOR_X_Acc, MOTOR_X_Pos[_thirdIndex], 1, 0);                   //X���������ָ��ɫ��
            }
            //������
            {
                APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
                
                APP_SERVO_Plate(_secondIndex);        //ת�̶��ת����Ӧ���ķ��ýǶ�

                APP_SERVO_Wrist('R');            //�󴦶������            
                APP_MOTOR_ZDT_WaitAck(MOTOR_R);            
                APP_MOTOR_ZDT_Move_P(MOTOR_R, MOTOR_R_Direction_CR, MOTOR_R_Vel, MOTOR_R_Acc, MOTOR_R_Pos[_thirdIndex], 1, 0);     //���Ƶ���ת��ת����ָ��ɫ������           
                
                
                //35����������Ƽ�צ���붥��1Ȧ
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[2], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

                APP_SERVO_Jaw('S');            //��צ����н����
             
                //35����������Ƽ�צ���붥��0.5Ȧ
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

                APP_SERVO_Wrist('F');            //�󴦶����ǰ    
                
             
                //35����������Ƽ�צ���붥��4Ȧ
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[4], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

                APP_SERVO_Jaw('Z');            //��צ����н����
             
                //35����������Ƽ�צ���붥��0.5Ȧ
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0);        
                vTaskDelay(Delay_up);

                APP_MOTOR_ZDT_WaitAck(MOTOR_R);            
                //���Ƶ���ת��ת����ָ��ɫ������           
                APP_MOTOR_ZDT_Move_P(MOTOR_R, MOTOR_R_Direction_CR, MOTOR_R_Vel, MOTOR_R_Acc, MOTOR_R_Pos[_secondIndex], 1, 0);     //���Ƶ���ת��ת����ָ��ɫ������           
                
                APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
                
                APP_SERVO_Plate(_thirdIndex);        //ת�̶��ת����Ӧ���ķ��ýǶ�
                APP_SERVO_Wrist('R');            //�󴦶������    
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[2], 1, 0);
                
                
                //35����������Ƽ�צ���붥��1Ȧ
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[2], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

                APP_SERVO_Jaw('S');            //��צ����ɿ����
             
                //35����������Ƽ�צ���붥��0.5Ȧ
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

//                APP_SERVO_Wrist('F');            //�󴦶����ǰ    
//                

                APP_MOTOR_ZDT_WaitAck(MOTOR_R);            
            }
        }           
        else    //�ڶ������ұߵ�
        {
            //�ڶ���
            {
                APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
                
                APP_SERVO_Plate(_firstIndex);        //ת�̶��ת����Ӧ���ķ��ýǶ�

                APP_SERVO_Wrist('R');            //�󴦶������
                APP_MOTOR_ZDT_Move_P(MOTOR_X, MOTOR_X_Direction_FORWARD, MOTOR_X_Vel, MOTOR_X_Acc, MOTOR_X_Pos[_secondIndex], 1, 0);                   //X���������ָ��ɫ��
                
                
                //35����������Ƽ�צ���붥��1Ȧ
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[2], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

                APP_SERVO_Jaw('S');            //��צ����н����
             
                //35����������Ƽ�צ���붥��0.5Ȧ
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

                APP_SERVO_Wrist('F');            //�󴦶����ǰ    
                
              
                //35����������Ƽ�צ���붥��4Ȧ
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[4], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
                APP_SERVO_Jaw('Z');            //��צ996���ת30�ȣ��ɿ����
                     
                //35����������Ƽ�צ���붥��0.5Ȧ
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

                APP_MOTOR_ZDT_WaitAck(MOTOR_R);            
                APP_MOTOR_ZDT_Move_P(MOTOR_R, MOTOR_R_Direction_CR, MOTOR_R_Vel, MOTOR_R_Acc, MOTOR_R_Pos[_thirdIndex], 1, 0);     //���Ƶ���ת��ת����ָ��ɫ������           
            }
            //������
            {
                APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
                
                APP_SERVO_Plate(_secondIndex);        //ת�̶��ת����Ӧ���ķ��ýǶ�

                APP_SERVO_Wrist('R');            //�󴦶������
                APP_MOTOR_ZDT_Move_P(MOTOR_X, MOTOR_X_Direction_FORWARD, MOTOR_X_Vel, MOTOR_X_Acc, MOTOR_X_Pos[_thirdIndex], 1, 0);                   //X���������ָ��ɫ��
                
                
                //35����������Ƽ�צ���붥��1Ȧ
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[2], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

                APP_SERVO_Jaw('S');            //��צ����н����
             
                //35����������Ƽ�צ���붥��0.5Ȧ
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

                APP_SERVO_Wrist('F');            //�󴦶����ǰ    
                
              
                //35����������Ƽ�צ���붥��4Ȧ
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[4], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
                APP_SERVO_Jaw('Z');            //��צ996���ת30�ȣ��ɿ����
                     
                //35����������Ƽ�צ���붥��0.5Ȧ
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
                
                APP_SERVO_Plate(_thirdIndex);        //ת�̶��ת����Ӧ���ķ��ýǶ�
                APP_SERVO_Wrist('R');            //�󴦶������
                
                
                //35����������Ƽ�צ���붥��1Ȧ
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[2], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

                APP_SERVO_Jaw('S');            //��צ����н����
             
                //35����������Ƽ�צ���붥��0.5Ȧ
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

//                APP_SERVO_Wrist('F');            //�󴦶����ǰ    
//                
               
                APP_MOTOR_ZDT_WaitAck(MOTOR_R);            
            }
        }
    }
    else if (_firstIndex == 2)
    {
        //��һ��
        {
            {
                APP_MOTOR_ZDT_Move_P(MOTOR_R, MOTOR_R_Direction_CR, MOTOR_R_Vel, MOTOR_R_Acc, MOTOR_R_Pos[_firstIndex], 1, 0);     //���Ƶ���ת��ת����ָ��ɫ������
                APP_MOTOR_ZDT_WaitAck(MOTOR_R);            
                APP_MOTOR_ZDT_Move_P(MOTOR_X, MOTOR_X_Direction_FORWARD, MOTOR_X_Vel, MOTOR_X_Acc, MOTOR_X_Pos[_firstIndex], 1, 0);                   //X���������ָ��ɫ��
                vTaskDelay(500);
            }
            //35����������Ƽ�צ���붥��4Ȧ
            APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[4], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
            APP_SERVO_Jaw('Z');            //��צ996���ת30�ȣ��ɿ����
                 
            //35����������Ƽ�צ���붥��1Ȧ    
            APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0);            
            vTaskDelay(Delay_up);
            APP_MOTOR_ZDT_Move_P(MOTOR_X, MOTOR_X_Direction_FORWARD, MOTOR_X_Vel, MOTOR_X_Acc, MOTOR_X_Pos[_secondIndex], 1, 0);                   //X���������ָ��ɫ��

            //�ڶ���
            {
                APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
                
                APP_SERVO_Plate(_firstIndex);        //ת�̶��ת����Ӧ���ķ��ýǶ�

                APP_SERVO_Wrist('R');            //�󴦶������            
                APP_MOTOR_ZDT_Move_P(MOTOR_R, MOTOR_R_Direction_CR, MOTOR_R_Vel, MOTOR_R_Acc, MOTOR_R_Pos[_secondIndex], 1, 0);     //���Ƶ���ת��ת����ָ��ɫ������           
                
                
                //35����������Ƽ�צ���붥��1Ȧ
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[2], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

                APP_SERVO_Jaw('S');            //��צ����н����
             
                //35����������Ƽ�צ���붥��0.5Ȧ
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
                ////printf("3-5\r\n");

                APP_SERVO_Wrist('F');            //�󴦶����ǰ    
                
             
                //35����������Ƽ�צ���붥��0.5Ȧ
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[4], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

                APP_SERVO_Jaw('Z');            //��צ����н����
             
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0);        //35����������Ƽ�צ���붥��0.5Ȧ
                vTaskDelay(Delay_up);

                APP_MOTOR_ZDT_WaitAck(MOTOR_R);            
                APP_MOTOR_ZDT_Move_P(MOTOR_R, MOTOR_R_Direction_CR, MOTOR_R_Vel, MOTOR_R_Acc, MOTOR_R_Pos[_thirdIndex], 1, 0);     //���Ƶ���ת��ת����ָ��ɫ������           
                
                APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
                
                APP_SERVO_Plate(_secondIndex);        //ת�̶��ת����Ӧ���ķ��ýǶ�
                APP_SERVO_Wrist('R');            //�󴦶������    
                APP_MOTOR_ZDT_Move_P(MOTOR_X, MOTOR_X_Direction_FORWARD, MOTOR_X_Vel, MOTOR_X_Acc, MOTOR_X_Pos[_thirdIndex], 1, 0);
                
                
                //35����������Ƽ�צ���붥��1Ȧ
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[2], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

                APP_SERVO_Jaw('S');            //��צ����ɿ����
             
                //35����������Ƽ�צ���붥��0.5Ȧ
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

                APP_SERVO_Wrist('F');            //�󴦶����ǰ    
                
                
                //35����������Ƽ�צ���붥��0.5Ȧ
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[4], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

                APP_SERVO_Jaw('Z');            //��צ����н����
             
                //35����������Ƽ�צ���붥��0.5Ȧ
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0);        
                vTaskDelay(Delay_up);

                APP_MOTOR_ZDT_WaitAck(MOTOR_R);            
                //���Ƶ���ת��ת����ָ��ɫ������           
                APP_MOTOR_ZDT_Move_P(MOTOR_R, MOTOR_R_Direction_CR, MOTOR_R_Vel, MOTOR_R_Acc, MOTOR_R_Pos[_secondIndex], 1, 0);     //���Ƶ���ת��ת����ָ��ɫ������           
                
                APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
                
                APP_SERVO_Plate(_thirdIndex);        //ת�̶��ת����Ӧ���ķ��ýǶ�
                APP_SERVO_Wrist('R');            //�󴦶������    
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[2], 1, 0);
                
                
                //35����������Ƽ�צ���붥��1Ȧ
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[2], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

                APP_SERVO_Jaw('S');            //��צ����ɿ����
                
                //35����������Ƽ�צ���붥��0.5Ȧ
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

//                APP_SERVO_Wrist('F');            //�󴦶����ǰ    
//                

                APP_MOTOR_ZDT_WaitAck(MOTOR_R);            
            }
        }
    }
}
