#include "main.h"

/* 全局变量定义 */

motor_measure_t motor_chassis[4] = {0}; // 四个底盘电机的测量数据

CAN_TxHeaderTypeDef Tx1Message;      // CAN 发送消息头
CAN_RxHeaderTypeDef Rx1Message;      // CAN 接收消息头
uint8_t Tx1Data[8] = {0};            // CAN 发送数据
uint8_t Rx1Data[8] = {0};            // CAN 接收数据
uint32_t Box = 100;                  // CAN 发送邮箱

/*******************************************************************************************
 * @Func    CAN1_ReadMsg
 * @Brief   CAN1 数据读取解析
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
 * @Brief   获取电机测量数据
 * @Param   ptr: 电机数据结构体指针
 *         Data: CAN 接收到的数据
 * @Retval  None
 *******************************************************************************************/
void get_motor_measure( motor_measure_t* ptr, uint8_t* Data ) //采集4个电机的信息
{
    ptr->last_angle = ptr->angle;
    ptr->angle = ( uint16_t )(( uint16_t )Data[0] << 8 | Data[1] ) ; //转子角度
    ptr->speed_rpm  = ( int16_t )(( int16_t )Data[2] << 8 | Data[3] ); //转子速度
//    ptr->real_current = ( Data[4] << 8 | Data[5] ) * 5.f / 16384.f; //转子转矩转化为电流
    ptr->real_current = ( Data[4] << 8 | Data[5] ); //转子转矩转化为电流

    ptr->hall = Data[6];//null

    if( ptr->angle - ptr->last_angle > 4096 )
        ptr->round_cnt --;
    else if( ptr->angle - ptr->last_angle < -4096 )
        ptr->round_cnt ++;

    ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;//总角度
    ptr->total_angle_offset = ptr->total_angle - ptr->total_angle_zero;//相对零点总角度
}

//电机上电时的角度偏差
void get_motor_offset( void )
{
    for( uint8_t i = 0; i < 4; i++ )
    {
        motor_chassis[i].offset_angle = motor_chassis[i].angle;
    }
}

//运行过程中设置电机零点
void set_motor_zero_angle( void )
{
    for( uint8_t i = 0; i < 4; i++ )
    {
        motor_chassis[i].total_angle_zero = motor_chassis[i].total_angle;
    }
}

/**
  * @brief          返回底盘电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
const motor_measure_t* get_chassis_motor_measure_point( uint8_t i )
{
    return &motor_chassis[i];
}

/*******************************************************************************************
 * @Func    set_motor_current
 * @Brief   设置电机电流
 * @Param   hcan: CAN 句柄
 *          iq1-iq4: 四个电机的电流值
 * @Retval  None
 *******************************************************************************************/
void set_motor_current( int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4 )
{
//发送底盘4个电机数据
    Tx1Message.StdId = 0x200;
    Tx1Message.IDE = CAN_ID_STD;
    Tx1Message.RTR = CAN_RTR_DATA;
    Tx1Message.DLC = 0x08;
    Tx1Data[0] = ( iq1 >> 8 ); //1号电流高八位
    Tx1Data[1] = iq1;
    Tx1Data[2] = ( iq2 >> 8 ); //2号电流高八位
    Tx1Data[3] = iq2;
    Tx1Data[4] = iq3 >> 8;//3号电流高八位
    Tx1Data[5] = iq3;
    Tx1Data[6] = iq4 >> 8;//4号电流高八位
    Tx1Data[7] = iq4;

    HAL_CAN_AddTxMessage(&hcan1, &Tx1Message, Tx1Data, &Box );
}
