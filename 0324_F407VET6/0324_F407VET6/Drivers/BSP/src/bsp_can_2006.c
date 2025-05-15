#include "main.h"

/* ȫ�ֱ������� */

motor_measure_t motor_chassis[4] = {0}; // �ĸ����̵���Ĳ�������

CAN_TxHeaderTypeDef Tx1Message;      // CAN ������Ϣͷ
CAN_RxHeaderTypeDef Rx1Message;      // CAN ������Ϣͷ
uint8_t Tx1Data[8] = {0};            // CAN ��������
uint8_t Rx1Data[8] = {0};            // CAN ��������
uint32_t Box = 100;                  // CAN ��������

/*******************************************************************************************
 * @Func    CAN1_ReadMsg
 * @Brief   CAN1 ���ݶ�ȡ����
 * @Param   None
 * @Retval  None
 *******************************************************************************************/
void CAN1_ReadMsg( void )
{
    HAL_CAN_GetRxMessage(&hcan1, CAN_FilterFIFO0, &Rx1Message, Rx1Data );
    switch( Rx1Message.StdId )
    {
    case CAN_2006Moto1_ID:
    case CAN_2006Moto2_ID:
    case CAN_2006Moto3_ID:
    case CAN_2006Moto4_ID:
        {
            static uint8_t i;
            i = Rx1Message.StdId - CAN_2006Moto1_ID;
            get_motor_measure(&motor_chassis[i], Rx1Data );
        }
        break;
    }
}

/*******************************************************************************************
 * @Func    get_motor_measure
 * @Brief   ��ȡ�����������
 * @Param   ptr: ������ݽṹ��ָ��
 *         Data: CAN ���յ�������
 * @Retval  None
 *******************************************************************************************/
void get_motor_measure( motor_measure_t* ptr, uint8_t* Data ) //�ɼ�4���������Ϣ
{
    ptr->last_angle = ptr->angle;
    ptr->angle = ( uint16_t )(( uint16_t )Data[0] << 8 | Data[1] ) ; //ת�ӽǶ�
    ptr->speed_rpm  = ( int16_t )(( int16_t )Data[2] << 8 | Data[3] ); //ת���ٶ�
//    ptr->real_current = ( Data[4] << 8 | Data[5] ) * 5.f / 16384.f; //ת��ת��ת��Ϊ����
    ptr->real_current = ( Data[4] << 8 | Data[5] ); //ת��ת��ת��Ϊ����

    ptr->hall = Data[6];//null

    if( ptr->angle - ptr->last_angle > 4096 )
        ptr->round_cnt --;
    else if( ptr->angle - ptr->last_angle < -4096 )
        ptr->round_cnt ++;

    ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;//�ܽǶ�
    ptr->total_angle_offset = ptr->total_angle - ptr->total_angle_zero;//�������ܽǶ�
}

//����ϵ�ʱ�ĽǶ�ƫ��
void get_motor_offset( void )
{
    for( uint8_t i = 0; i < 4; i++ )
    {
        motor_chassis[i].offset_angle = motor_chassis[i].angle;
    }
}

//���й��������õ�����
void set_motor_zero_angle( void )
{
    for( uint8_t i = 0; i < 4; i++ )
    {
        motor_chassis[i].total_angle_zero = motor_chassis[i].total_angle;
    }
}

/**
  * @brief          ���ص��̵�� 3508�������ָ��
  * @param[in]      i: ������,��Χ[0,3]
  * @retval         �������ָ��
  */
const motor_measure_t* get_chassis_motor_measure_point( uint8_t i )
{
    return &motor_chassis[i];
}

/*******************************************************************************************
 * @Func    set_motor_current
 * @Brief   ���õ������
 * @Param   hcan: CAN ���
 *          iq1-iq4: �ĸ�����ĵ���ֵ
 * @Retval  None
 *******************************************************************************************/
void set_motor_current( int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4 )
{
//���͵���4���������
    Tx1Message.StdId = 0x200;
    Tx1Message.IDE = CAN_ID_STD;
    Tx1Message.RTR = CAN_RTR_DATA;
    Tx1Message.DLC = 0x08;
    Tx1Data[0] = ( iq1 >> 8 ); //1�ŵ����߰�λ
    Tx1Data[1] = iq1;
    Tx1Data[2] = ( iq2 >> 8 ); //2�ŵ����߰�λ
    Tx1Data[3] = iq2;
    Tx1Data[4] = iq3 >> 8;//3�ŵ����߰�λ
    Tx1Data[5] = iq3;
    Tx1Data[6] = iq4 >> 8;//4�ŵ����߰�λ
    Tx1Data[7] = iq4;

    HAL_CAN_AddTxMessage(&hcan1, &Tx1Message, Tx1Data, &Box );
}
