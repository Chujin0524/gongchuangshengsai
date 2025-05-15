#include "main.h"

/* 全局变量定义 */
int yaw0 = 0;                        // 目标 Yaw 角度
volatile int Weak_mode = 1;          // 弱模式标志
volatile float distance_r = 0;       // 平移距离
volatile float distance_angel = 0;   // 角度距离
volatile float distance_big_dia = 0; // 对角线距离
int moto_angle = 0;                  // 电机角度
static float rotate_ratio_f;         // 前轮旋转比例
static float rotate_ratio_b;         // 后轮旋转比例
static float wheel_rpm_ratio;        // 轮子 RPM 比例
float wheel_rpm[4] = {0};            // 四个轮子的 RPM
int dingjiao_mode = 0;               // 定角模式
int dingjiao_flag = 0;               // 定角标志
int32_t w;                           // 差速值

PID_TypeDef motor_pid[4];            // 四个电机的 PID 控制器
PID_TypeDef hwt_pid;                 // HWT101 的 PID 控制器

moto_measure_t moto_chassis[4] = {0}; // 四个底盘电机的测量数据
CAN_TxHeaderTypeDef Tx1Message;      // CAN 发送消息头
CAN_RxHeaderTypeDef Rx1Message;      // CAN 接收消息头
uint8_t Rx1Data[8] = {0};            // CAN 接收数据
uint8_t Tx1Data[8] = {0};            // CAN 发送数据
uint32_t Box = 100;                  // CAN 发送邮箱

/*******************************************************************************************
 * @Func    CAN1_ReadMsg
 * @Brief   CAN1 数据读取解析
 * @Param   None
 * @Retval  None
 *******************************************************************************************/
void CAN1_ReadMsg( void )
{
//    if( HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData ) != HAL_OK )
//    {
//        printf("Parse hcan1 error\r\n");
//        return;
//    }
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
            get_moto_measure(&moto_chassis[i], Rx1Data );
        }
        break;
    }

//    __HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING );
//    printf("*************2006_CAN**************\r\n");
//    printf("StdID = %d\r\n", RxHeader.StdId);
//    printf("ExtID = %d\r\n", RxHeader.ExtId);
//    printf("RTR(0=Data, 2=Remote) = %d\r\n", RxHeader.RTR);
//    printf("IDE(0=Std, 4=Ext) = %d\r\n", RxHeader.IDE);
//    printf("DLC(Data Length) = %d\r\n", RxHeader.DLC);
//    printf("Data = %02X %02X %02X %02X\r\n", RxData[0], RxData[1], RxData[2], RxData[3]);
//    printf("*************2006_CAN***************\r\n");
}

/*******************************************************************************************
 * @Func    get_moto_measure
 * @Brief   获取电机测量数据
 * @Param   ptr: 电机数据结构体指针
 *         Data: CAN 接收到的数据
 * @Retval  None
 *******************************************************************************************/
void get_moto_measure( moto_measure_t* ptr, uint8_t* Data ) //采集5个电机的信息
{
    ptr->last_angle = ptr->angle;
    ptr->angle = ( uint16_t )( Data[0] << 8 | Data[1] ) ; //转子角度
    ptr->speed_rpm  = ( int16_t )( Data[2] << 8 | Data[3] ); //转子速度
    ptr->real_current = ( Data[4] << 8 | Data[5] ) * 5.f / 16384.f; //转子转矩

    ptr->hall = Data[6];//null


    if( ptr->angle - ptr->last_angle > 4096 )
        ptr->round_cnt --;
    else if( ptr->angle - ptr->last_angle < -4096 )
        ptr->round_cnt ++;
    ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;//总角度
}

/*******************************************************************************************
 * @Func    get_moto_offset
 * @Brief   获取电机偏移角度，在system和初始化后使用
 * @Param   ptr: 电机数据结构体指针
 *          Data: CAN 接收到的数据
 * @Retval  None
 *******************************************************************************************/
void get_moto_offset( moto_measure_t* ptr, uint8_t* Data )
{
    ptr->angle = ( uint16_t )( Data[0] << 8 | Data[1] ) ; //转子偏移角度
    ptr->offset_angle = ptr->angle;
}

/*******************************************************************************************
 * @Func    get_total_angle
 * @Brief   更新电机的总角度
            电机上电角度=0， 之后用这个函数更新3510电机的相对开机后（为0）的相对角度。
 * @Param   p: 电机数据结构体指针
 * @Retval  None
 *******************************************************************************************/
void get_total_angle( moto_measure_t* p )
{

    int res1, res2, delta;
    if( p->angle < p->last_angle )          //可能的情况
    {
        res1 = p->angle + 8192 - p->last_angle; //正转，delta=+
        res2 = p->angle - p->last_angle;                //反转    delta=-
    }
    else    //angle > last
    {
        res1 = p->angle - 8192 - p->last_angle ;//反转    delta -
        res2 = p->angle - p->last_angle;                //正转    delta +
    }
    //不管正反转，肯定是转的角度小的那个是真的
    if( ABS( res1 ) < ABS( res2 ) )
        delta = res1;
    else
        delta = res2;

    p->total_angle += delta;
    p->last_angle = p->angle;
}

/*******************************************************************************************
 * @Func    set_moto_current
 * @Brief   设置电机电流
 * @Param   hcan: CAN 句柄
 *          iq1-iq4: 四个电机的电流值
 * @Retval  None
 *******************************************************************************************/
void set_moto_current( CAN_HandleTypeDef* hcan, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4 )
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

    HAL_CAN_AddTxMessage( hcan, &Tx1Message, Tx1Data, &Box );
}

/*******************************************************************************************
 * @Func    wheel_init
 * @Brief   初始化轮子相关参数
 * @Param   None
 * @Retval  None
 *******************************************************************************************/
void wheel_init()
{
//    my_can_filter_init_recv_all(&hcan1 );
//    HAL_CAN_Start(&hcan1 );
//    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING );
    for( int i = 0; i < 4; i++ )
    {
        float pid[3] = {5, 0.5, 0};
        PID_init(&motor_pid[i], PID_DELTA, pid, 16800, 1000 );
    }
    {
        float pid[3] = {60, 0, 0};
        PID_init(&hwt_pid, PID_POSITION, pid, 6000, 0.1 );
    }
    motor_pid[0].set = 0;
    motor_pid[1].set = 0;
    motor_pid[2].set = 0;
    motor_pid[3].set = 0;
    init_calculate_values();
}

/*******************************************************************************************
 * @Func    init_calculate_values
 * @Brief   初始化计算参数
 * @Param   None
 * @Retval  None
 *******************************************************************************************/
void init_calculate_values()
{
    static float WHEELBASE = 0.3f; // 轴距
    static float WHEELTRACK = 0.3f; // 轮距
    static float GIMBAL_OFFSET = 0.0f; // 云台偏移，假设为0
    static float PERIMETER = 3.14159f * 0.07f; // 轮子的周长，假设轮子直径为0.07m
    static float CHASSIS_DECELE_RATIO = 36.0f; // 减速比
    static float RADIAN_COEF = 57.3f; // 弧度转角度的系数

    rotate_ratio_f = (( WHEELBASE + WHEELTRACK ) / 2.0f - GIMBAL_OFFSET ) / RADIAN_COEF;
    rotate_ratio_b = (( WHEELBASE + WHEELTRACK ) / 2.0f + GIMBAL_OFFSET ) / RADIAN_COEF;
    wheel_rpm_ratio = 60.0f / ( PERIMETER * CHASSIS_DECELE_RATIO );
}

/*******************************************************************************************
 * @Func    M2006_CAL
 * @Brief   计算 M2006 电机的距离
 * @Param   None
 * @Retval  None
 *******************************************************************************************/
void M2006_CAL()
{
    distance_r = 22.9 * ( moto_chassis[0].round_cnt - moto_chassis[1].round_cnt - moto_chassis[2].round_cnt + moto_chassis[3].round_cnt ) / 144.0;
    distance_big_dia = 22.9 * ( -moto_chassis[0].round_cnt - moto_chassis[1].round_cnt + moto_chassis[2].round_cnt + moto_chassis[3].round_cnt ) / 144.0;
}

/*
*********************************************************************************************************
*    函 数 名: delay_dis_diagnal
*    功能说明: 平移距离延迟
*    返 回 值: 无
*********************************************************************************************************
*/
int delay_dis_diagnal( int dis )
{
    float dis0 = distance_big_dia;
    if( dis >= 0 )
    {
        while( distance_big_dia < dis0 + dis )
        {

        }
        return 1;
    }
    else
    {
        while( distance_big_dia > dis0 + dis )
        {

        }
        return 1;

    }
}

/*
*********************************************************************************************************
*    函 数 名: delay_distance
*    功能说明: 前后距离延迟
*    返 回 值: 无
*********************************************************************************************************
*/
int delay_distance( int dis )
{
    float dis0 = distance_r;
    if( dis >= 0 )
    {
        while( distance_r < dis0 + dis )
        {

        }
        return 1;
    }
    else
    {
        while( distance_r > dis0 + dis )
        {
            //&&!AutoAim_Data_Receive.turn_flag);
        }
        return 1;

    }
}

/*******************************************************************************************
 * @Func    _M2006Task
 * @Brief   M2006 电机 PID 控制任务
 * @Param   None
 * @Retval  None
 *******************************************************************************************/
void _M2006Task()
{
    M2006_CAL();
    for( int i = 0; i < 4; i++ )
    {
        PID_calc(&motor_pid[i], moto_chassis[i].speed_rpm, motor_pid[i].set );
    }

    if( Weak_mode == 0 )
    {
        set_moto_current(&hcan1, motor_pid[0].out, motor_pid[1].out, motor_pid[2].out, motor_pid[3].out );
    }
    else
    {
        set_moto_current(&hcan1, 0, 0, 0, 0 );
    }
}


/*******************************************************************************************
 * @Func    Move_Transfrom
 * @Brief   M2006四轮运动解析。整车移动量转换为单轮速度
 * @Param   vx: X 轴速度     左+右-
 *          vy: Y 轴速度     前+后-
 *          vw: 旋转速度     逆+顺-
 *          yaw: 目标角度
 * @Retval  None
 *******************************************************************************************/
void Move_Transfrom(double vx,double vy,double vw,double yaw)
{
	wheel_rpm[3] = (vx + vy + vw * rotate_ratio_f) * wheel_rpm_ratio;
	wheel_rpm[2] = (vx - vy - vw * rotate_ratio_f) * wheel_rpm_ratio;
	wheel_rpm[1] = (-vx - vy + vw * rotate_ratio_b) * wheel_rpm_ratio;
	wheel_rpm[0] = (-vx + vy - vw * rotate_ratio_b) * wheel_rpm_ratio;
	yaw0=yaw;
}

/*******************************************************************************************
 * @Func    _DingjiaoPIDTask
 * @Brief   Yaw 角 PID 控制任务
 * @Param   None
 * @Retval  None
 *******************************************************************************************/
void _DingjiaoPIDTask( void )
{
    //static int gyro1 = 0;
    // xQueueReceive( Queue_HandleStraightHandle, &gyro1, portMAX_DELAY );
    //double err = 0;
    static float gyro_z = 0;
    static float err = 0;
    if( APP_HWT101_Read(&gyro_z ) )
    {
        err = gyro_z - yaw0;
        if( err > 180 )
        {
            err = err - 360;
        }
        if( err < -180 )
        {
            err = err + 360;
        }
        //int speed_temp=(moto_chassis[0].speed_rpm+moto_chassis[1].speed_rpm+moto_chassis[2].speed_rpm+moto_chassis[3].speed_rpm)/4.0;
        if( my_abs( err ) <= 1 )
        {
            dingjiao_flag = 1; //表示不需要进行转角调节
        }
        hwt_pid.set = 0;
        hwt_pid.Ki = 0.2 + 0.6 * (( my_abs( err ) -90 ) / 90 );
        //dingjiao_pid.ki=2*(my_abs(err)/180.0);
        PID_calc(&hwt_pid, err, 0 );
        w = hwt_pid.out;
        hwt_cal( wheel_rpm[0], wheel_rpm[1], wheel_rpm[2], wheel_rpm[3], w );
    }
}


void stop()
{
    motor_pid[0].set = 0;
    motor_pid[1].set = 0;
    motor_pid[2].set = 0;
    motor_pid[3].set = 0;
}

void hwt_cal( int32_t speed0, int32_t speed1, int32_t speed2, int32_t speed3, int32_t w_t )
{
    motor_pid[0].set = speed0 - w_t;
    motor_pid[1].set = speed1 - w_t;
    motor_pid[2].set = speed2 - w_t;
    motor_pid[3].set = speed3 - w_t;
}


float my_abs( float data )
{
    return data > 0 ? data : -data;
}

/*
*********************************************************************************************************
*    函 数 名: Ramp_dis
*    功能说明: 梯形加速底层代码，前后
*    形    参：spd1:初始速度
							 spd2:最终速度
*    返 回 值: 无
*********************************************************************************************************
*/

void Ramp_dis(int spd1, int spd2, int dis,int yaw)
{
	if(spd2>=0)
	{
			if(spd1>=0)
			{
					int delta_spd = (spd2 - spd1) / dis;
					for (int i = 1; i <= dis; i++)
					{
							Move_Transfrom(0,spd1 + delta_spd * i, 0, yaw);
							if (spd1 + delta_spd * i==0)
							{
						break;
							}
							else
							{
									if (delay_distance(1)==1)
									{
										continue;
									} // 继续迭代
							}
					}
			}
			else
			{
					int delta_spd = (spd2 - spd1) / dis;
					for (int i = 1; i <= dis; i++)
					{
							Move_Transfrom(0,spd1 + delta_spd * i, 0, yaw);
							if (spd1 + delta_spd * i==0)
							{
						break;
							}
							else
							{
									if (delay_distance(-1)==1)
									{
										continue;
									} // 继续迭代
							}
					}
			}

	}
	else
	{
			int delta_spd = (spd2 - spd1) / dis;
			for (int i = 1; i <= dis; i++)
			{
					Move_Transfrom(0,spd1 + delta_spd * i, 0, yaw);
					if (spd1 + delta_spd * i==0)
					{
						break;
					}
					else
					{
							if (delay_distance(-1)==1)
							{
								continue;
								} // 继续迭代
						}

			}
			
	}
	
Move_Transfrom(0,spd2,0, yaw); // 确保最终速度设置为spd2
}

/*
*********************************************************************************************************
*    函 数 名: SpeedUP_track
*    功能说明: 梯形加速
							前：speed_max+;后：speed_max-
*    形    参：
								speed_max： 期望最大速度
								speed_final： 最终速度，一般为0停止
								distance：总距离
								ramp_dis1：加速距离
								ramp_dis2：减速距离
								yaw：期望角度
*    返 回 值: 无
*********************************************************************************************************
*/
void SpeedUP_track(int32_t speed_max,uint32_t speed_final,uint32_t distance,uint16_t ramp_dis1,uint16_t ramp_dis2,int yaw)
{

	Ramp_dis(0,speed_max,ramp_dis1,yaw);
	if(speed_max>=0)
	{
		if (delay_distance(distance-ramp_dis1-ramp_dis2)==1)
		{
			Ramp_dis(speed_max,speed_final,ramp_dis2,yaw);
		}
	}
	else
	{
		if (delay_distance(-(distance-ramp_dis1-ramp_dis2))==1)
		{
			Ramp_dis(speed_max,speed_final,ramp_dis2,yaw);
		}
	}

}

/*
*********************************************************************************************************
*    函 数 名: Ramp_disx
*    功能说明: 梯形加速底层代码，左右
*    形    参：spd1:初始速度
               spd2:最终速度
*    返 回 值: 无
*********************************************************************************************************
*/
void Ramp_disx(int spd1, int spd2, int dis,int yaw)
{	
if(spd2>=0)
	{
			if(spd1>=0)
			{
					int delta_spd = (spd2 - spd1) / dis;
					for (int i = 1; i <= dis; i++)
					{
							Move_Transfrom(spd1 + delta_spd * i,0, 0, yaw);
							if (spd1 + delta_spd * i==0)
							{
								break;
							}
							else
							{
									if (delay_dis_diagnal(1)==1)
									{
										continue;
									} // 继续迭代
							}
					}
					Move_Transfrom(spd2,0,0, yaw); // 确保最终速度设置为spd2
			}
			else
			{
					int delta_spd = (spd2 - spd1) / dis;
					for (int i = 1; i <= dis; i++)
					{
							Move_Transfrom(spd1 + delta_spd * i,0, 0, yaw);
							if (spd1 + delta_spd * i==0)
							{
								break;
							}
							else
							{
									if (delay_dis_diagnal(-1)==1)
									{
										continue;
									} // 继续迭代
							}
					}
			}

	}
	else
	{
			int delta_spd = (spd2 - spd1) / dis;
			for (int i = 1; i <= dis; i++)
			{
					Move_Transfrom(spd1 + delta_spd * i,0, 0, yaw);
					if (spd1 + delta_spd * i==0)
					{
						break;
					}
					else
					{
							if (delay_dis_diagnal(-1)==1)
							{
								continue;
								} // 继续迭代
						}

			}
			
	}
	
Move_Transfrom(spd2,0,0, yaw); // 确保最终速度设置为spd2			
}
/*
*********************************************************************************************************
*    函 数 名: SpeedUP_x
*    功能说明: 梯形加速，方向左右
                左：speed_max+;右：speed_max-
*    形    参：
               speed_max： 期望最大速度
               speed_final： 最终速度，一般为0停止
               distance：总距离
               ramp_dis1：加速距离
               ramp_dis2：减速距离
               yaw：期望角度
*    返 回 值: 无
*********************************************************************************************************
*/
void SpeedUP_x(int32_t speed_max,uint32_t speed_final,uint32_t distance,uint16_t ramp_dis1,uint16_t ramp_dis2,int yaw)
{
	Ramp_disx(0, speed_max, ramp_dis1, yaw);
	if(speed_max>=0)
	{
		if (delay_dis_diagnal(distance-ramp_dis1-ramp_dis2)==1)
		{
			Ramp_disx(speed_max,speed_final,ramp_dis2,yaw);
		}
	}
	else
	{
		if (delay_dis_diagnal(-(distance-ramp_dis1-ramp_dis2))==1)
		{
			Ramp_disx(speed_max,speed_final,ramp_dis2,yaw);
		}
	}
}

