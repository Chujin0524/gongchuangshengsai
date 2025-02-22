#include "main.h"
#include "bsp_can_2006.h"
#include "bsp_hwt101.h"

void APP_MOTOR_2006_Init( void )
{
//    Emm_V5_Stop_Now(1, 0);
//    Emm_V5_Stop_Now(2, 0);
//    Emm_V5_Stop_Now(3, 0);
//    Emm_V5_Stop_Now(4, 0);
//    Emm_V5_Stop_Now(5, 0);
//    Emm_V5_Stop_Now(6, 0);
//    Emm_V5_Stop_Now(7, 0);
}

void APP_Motor_2006_WhaleControl(void)
{
    static CAR_XYR_T *ptCarXYR = NULL;

    for(;; )
    {
        xQueueReceive(Queue_MotorHandle, &ptCarXYR, portMAX_DELAY); //�ȴ��������Ӳ���
        
        App_Printf("-----APP_MotorWhaleControl-----\r\n");
        App_Printf("ADDR: %04X\r\n", ptCarXYR);        
        App_Printf("DirX: %d\r\n", ptCarXYR->WhaleDirX);
        App_Printf("PosX: %d\r\n", ptCarXYR->WhalePosX);
        App_Printf("DirY: %d\r\n", ptCarXYR->WhaleDirY);
        App_Printf("PosY: %d\r\n", ptCarXYR->WhalePosY);
        App_Printf("DirR: %d\r\n", ptCarXYR->WhaleDirR);
        App_Printf("PosR: %d\r\n", ptCarXYR->WhalePosR);
        App_Printf("-----APP_MotorWhaleControl-----\r\n"); 

        if (ptCarXYR->WhalePosX > 0)
        {
            APP_MOTOR_2006_WHALE_MovePID(ptCarXYR->WhaleDirX, ptCarXYR->WhalePosX);
            
            ptCarXYR->WhaleDirX = 0;
            ptCarXYR->WhalePosX = 0;
        }
        if (ptCarXYR->WhalePosY > 0)
        {
            APP_MOTOR_2006_WHALE_MovePID(ptCarXYR->WhaleDirY, ptCarXYR->WhalePosY);
            
            ptCarXYR->WhaleDirY = 0;
            ptCarXYR->WhalePosY = 0;
        }
        if (ptCarXYR->WhalePosR > 0)
        {
            APP_MOTOR_2006_WHALE_MovePID(ptCarXYR->WhaleDirR, ptCarXYR->WhalePosR);
            
            ptCarXYR->WhaleDirR = 0;
            ptCarXYR->WhalePosR = 0;
        }
    }
}

/*
*********************************************************************************************************
*    �� �� ��: APP_MOTOR_Move_V
*    ����˵��: ����˶����ٶ�ģʽ������ʱ
*    ��    ��: addr�������ַ
              dir ������        ��0ΪCW������ֵΪCCW
              vel ���ٶ�(RPM)   ����Χ0 - 5000RPM
              acc �����ٶ�      ����Χ0 - 255��ע�⣺0��ֱ������
              snF �����ͬ����־ ��falseΪ�����ã�trueΪ����
*    �� �� ֵ: ��
*********************************************************************************************************
*/
void APP_MOTOR_Move_V(uint8_t _addr, uint8_t _dir, uint16_t _vel, uint8_t _acc, bool _snF)
{
//    Emm_V5_Vel_Control(_addr, _dir, _vel, _acc, _snF); vTaskDelay(10);    
}

/*
*********************************************************************************************************
*    �� �� ��: APP_MOTOR_2006_WHALE_Move_P
*    ����˵��: ����С��ƽ�ƻ���ת��������
*    ��    ��: _dir  : �궨���bsp.h
*              _clk  : ����
*    �� �� ֵ: ��
*********************************************************************************************************
*/
void APP_MOTOR_2006_WHALE_Move_P(uint8_t _dir, uint32_t _clk)
{
    static WHALE_PARAMETER_T s_tMotorWhale = {0};
    switch (_dir)
    {
        case CarDirection_Forward:        //ǰ
            //����ת��
            s_tMotorWhale.ucaWhaleDir[0] = 1;
            s_tMotorWhale.ucaWhaleDir[1] = 1;
            s_tMotorWhale.ucaWhaleDir[2] = 0;
            s_tMotorWhale.ucaWhaleDir[3] = 0;
                
            break;
        
        case CarDirection_Backward:        //��
            //����ת��
            s_tMotorWhale.ucaWhaleDir[0] = 0;
            s_tMotorWhale.ucaWhaleDir[1] = 0;
            s_tMotorWhale.ucaWhaleDir[2] = 1;
            s_tMotorWhale.ucaWhaleDir[3] = 1;
            
            break;
        
        case CarDirection_Left:            //��
            //����ת��
            s_tMotorWhale.ucaWhaleDir[0] = 0;
            s_tMotorWhale.ucaWhaleDir[1] = 1;
            s_tMotorWhale.ucaWhaleDir[2] = 1;
            s_tMotorWhale.ucaWhaleDir[3] = 0;
            
            break;
        
        case CarDirection_Right:        //��
            //����ת��
            s_tMotorWhale.ucaWhaleDir[0] = 1;
            s_tMotorWhale.ucaWhaleDir[1] = 0;
            s_tMotorWhale.ucaWhaleDir[2] = 0;
            s_tMotorWhale.ucaWhaleDir[3] = 1;
            
            break;
        
        case CarDirection_CR:            //˳
            //����ת��
            s_tMotorWhale.ucaWhaleDir[0] = 0;
            s_tMotorWhale.ucaWhaleDir[1] = 0;
            s_tMotorWhale.ucaWhaleDir[2] = 0;
            s_tMotorWhale.ucaWhaleDir[3] = 0;
            
            break;
        
        case CarDirection_CCR:            //��
            //����ת��
            s_tMotorWhale.ucaWhaleDir[0] = 1;
            s_tMotorWhale.ucaWhaleDir[1] = 1;
            s_tMotorWhale.ucaWhaleDir[2] = 1;
            s_tMotorWhale.ucaWhaleDir[3] = 1;
            
            break;
        
        default:
            break;            
    }
    //����ת��
    s_tMotorWhale.usaWhaleVel[0] = s_tMotorWhale.usCarVel;
    s_tMotorWhale.usaWhaleVel[1] = s_tMotorWhale.usCarVel;
    s_tMotorWhale.usaWhaleVel[2] = s_tMotorWhale.usCarVel;
    s_tMotorWhale.usaWhaleVel[3] = s_tMotorWhale.usCarVel;
    
    s_tMotorWhale.ulCarPos = _clk;      //������
    
//    APP_MOTOR_2006_Move_P(0x01,s_tMotorWhale.ucaWhaleDir[0], s_tMotorWhale.usaWhaleVel[0], s_tMotorWhale.ucCarAcc, s_tMotorWhale.ulCarPos, 0, 1); 
//    APP_MOTOR_2006_Move_P(0x02,s_tMotorWhale.ucaWhaleDir[1], s_tMotorWhale.usaWhaleVel[1], s_tMotorWhale.ucCarAcc, s_tMotorWhale.ulCarPos, 0, 1); 
//    APP_MOTOR_2006_Move_P(0x03,s_tMotorWhale.ucaWhaleDir[2], s_tMotorWhale.usaWhaleVel[2], s_tMotorWhale.ucCarAcc, s_tMotorWhale.ulCarPos, 0, 1); 
//    APP_MOTOR_2006_Move_P(0x04,s_tMotorWhale.ucaWhaleDir[3], s_tMotorWhale.usaWhaleVel[3], s_tMotorWhale.ucCarAcc, s_tMotorWhale.ulCarPos, 0, 1); 
//    Emm_V5_Synchronous_motion(0x00); vTaskDelay(10);
}

/*
*********************************************************************************************************
*    �� �� ��: APP_MOTOR_2006_WHALE_MovePID
*    ����˵��: �ǶȻ�����С��ƽ�Ƹ�������
*    ��    ��: _dir  : �궨���bsp.h
*              _clk  : ����
*    �� �� ֵ: ��
*********************************************************************************************************
*/
void APP_MOTOR_2006_WHALE_MovePID(uint8_t _dir, uint32_t _clk)
{
    static int flag = 0;
    if (flag == 0)
    {
        switch (_dir)
        {
            case CarDirection_Forward:
                Move_Transfrom(0, 500, 0, 0);
                if (delay_distance(_clk) == 1)
                {
                    flag = 1;
                    stop();
                    Weak_mode = 1;
                    return;  // �˳�����
                }
                break;
            case CarDirection_Backward:
                Move_Transfrom(0, -500, 0, 0);
                if (delay_distance(-_clk) == 1)
                {
                    flag = 1;
                    stop();
                    Weak_mode = 1;
                    return;  // �˳�����
                }        
                break;
            case CarDirection_Left:
                Move_Transfrom(-500, 0, 0, 0);
                if (delay_dis_diagnal(_clk) == 1)
                {
                    flag = 1;
                    stop();
                    Weak_mode = 1;
                    return;  // �˳�����
                }
                break;
            case CarDirection_Right:
                Move_Transfrom(500, 0, 0, 0);
                if (delay_dis_diagnal(-_clk) == 1)
                {
                    flag = 1;
                    stop();
                    Weak_mode = 1;
                    return;  // �˳�����
                }        
                break;
            case CarDirection_CCR:
                Move_Transfrom(0, 0, 300, 0);
                if (gyro_z == _clk)
                {
                    flag = 1;
                    stop();
                    Weak_mode = 1;
                    return;  // �˳�����
                }        
                break;
            case CarDirection_CR:
                Move_Transfrom(0, 0, -300, 0);
                if (gyro_z == _clk)
                {
                    flag = 1;
                    stop();
                    Weak_mode = 1;
                    return;  // �˳�����
                }
                break;
        }
    }
}

/*
*********************************************************************************************************
*    �� �� ��: APP_MOTOR_2006_WHALE_Status
*    ����˵��: �ǶȻ�����С��ƽ�Ƹ�������
*    ��    ��: _dir  : �궨���bsp.h
*              _clk  : ����
*    �� �� ֵ: ��
*********************************************************************************************************
*/
void APP_MOTOR_2006_WHALE_Status(STATUS_E _ucStatus)
{
    switch (_ucStatus)
    {
    case STATUS_Start_2_Qr:
        APP_MOTOR_2006_WHALE_MovePID(CarDirection_Backward, 2200);

        APP_MOTOR_2006_WHALE_MovePID(CarDirection_Left, 5800);
        break;
    
    case STATUS_Qr_2_RawArea:
        APP_MOTOR_2006_WHALE_MovePID(CarDirection_Left, 8800);
        break;
    
    case STATUS_RawArea_2_ProFirArea:
        APP_MOTOR_2006_WHALE_MovePID(CarDirection_Right, 4500);

        APP_MOTOR_2006_WHALE_MovePID(CarDirection_CCR, 180);//����ת180��
    
        APP_MOTOR_2006_WHALE_MovePID(CarDirection_Forward, 16500);
        break;
    
    case STATUS_ProFirArea_2_ProSecArea:
        APP_MOTOR_2006_WHALE_MovePID(CarDirection_Right, 8100);

        APP_MOTOR_2006_WHALE_MovePID(CarDirection_Backward, 8000);

        APP_MOTOR_2006_WHALE_MovePID(CarDirection_CR, 90);//˳��ת90��
        break;
    
    case STATUS_ProSecArea_2_RawArea:
        APP_MOTOR_2006_WHALE_MovePID(CarDirection_Right, 8900);

        APP_MOTOR_2006_WHALE_MovePID(CarDirection_Backward, 3600);

        APP_MOTOR_2006_WHALE_MovePID(CarDirection_CR, 90);//˳��ת90��
        break;
    
    case STATUS_ProSecArea_2_Start:
        APP_MOTOR_2006_WHALE_MovePID(CarDirection_Backward, 17000);

        APP_MOTOR_2006_WHALE_MovePID(CarDirection_Right, 9800);

        APP_MOTOR_2006_WHALE_MovePID(CarDirection_Backward, 1500);
        break;
    
    default:
        break;
    }
}

