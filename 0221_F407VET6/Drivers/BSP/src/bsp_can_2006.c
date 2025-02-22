#include "main.h"
#include "bsp_pid.h"
#include "bsp_Emm_V5.h"
#include "bsp_can_2006.h"

static CAN_RxHeaderTypeDef RxHeader;
static uint8_t RxData[8];
__IO static uint8_t addr = 0;
__IO static uint8_t ackFlag = 0;

int yaw0=0;
int Weak_mode=0;
volatile float distance_r=0;
volatile float distance_angel=0;
volatile float distance_big_dia=0;
int moto_angle=0;
static float rotate_ratio_f;
static float rotate_ratio_b;
static float wheel_rpm_ratio;
float wheel_rpm[4] = {0};
int dingjiao_mode=0;
int dingjiao_flag=0;
int32_t w;				//差速


PID_TypeDef motor_pid[4];  //2006电机pid结构体
PID_TypeDef hwt_pid;

moto_measure_t moto_chassis[4] = {0};//4 chassis moto+1 同步带//电机信息结构体储存0-4
CAN_TxHeaderTypeDef		Tx1Message;
CAN_RxHeaderTypeDef 	Rx1Message;
uint8_t Rx1Data[8] = {0};
uint8_t Tx1Data[8] = {0};//1-4电机的发送电流数据
uint32_t Box = 100;
uint32_t x = 0;


/*******************************************************************************************
  * @Func		my_can_filter_init
  * @Brief    CAN1和CAN2滤波器配置
  * @Param		CAN_HandleTypeDef* hcan
  * @Retval		None
  * @Date     2025/2/22
 *******************************************************************************************/
void my_can_filter_init_recv_all(CAN_HandleTypeDef* _hcan)
{
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

}


/*HAL_CAN_RxFifo0MsgPendingCallback回调函数所调用的CAN1数据读取解析*/
void CAN1_ReadMsg(void)
{
//    if( HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData ) != HAL_OK )
//    {
//        printf("Parse hcan1 error\r\n");
//        return;
//    }
			HAL_CAN_GetRxMessage(&hcan1,CAN_FilterFIFO0,&Rx1Message,Rx1Data);
					switch(Rx1Message.StdId)
									{
												case CAN_2006Moto1_ID:
												case CAN_2006Moto2_ID:
												case CAN_2006Moto3_ID:
												case CAN_2006Moto4_ID:
													{
														static uint8_t i;
														i = Rx1Message.StdId - CAN_2006Moto1_ID;
														get_moto_measure(&moto_chassis[i], Rx1Data);
													}
													break;
									}

	__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    printf("*************2006_CAN**************\r\n");
    printf("StdID = %d\r\n", RxHeader.StdId);
    printf("ExtID = %d\r\n", RxHeader.ExtId);
    printf("RTR(0=Data, 2=Remote) = %d\r\n", RxHeader.RTR);
    printf("IDE(0=Std, 4=Ext) = %d\r\n", RxHeader.IDE);
    printf("DLC(Data Length) = %d\r\n", RxHeader.DLC);
    printf("Data = %02X %02X %02X %02X\r\n", RxData[0], RxData[1], RxData[2], RxData[3]);
    printf("*************2006_CAN***************\r\n");
}

void get_moto_measure(moto_measure_t *ptr, uint8_t* Data)//采集5个电机的信息
{

	ptr->last_angle = ptr->angle;
	ptr->angle = (uint16_t)(Data[0]<<8 | Data[1]) ;//转子角度
	ptr->speed_rpm  = (int16_t)(Data[2]<<8 | Data[3]);//转子速度
	ptr->real_current = (Data[4]<<8 | Data[5])*5.f/16384.f;//转子转矩

	ptr->hall = Data[6];//null


	if(ptr->angle - ptr->last_angle > 4096)
		ptr->round_cnt --;
	else if (ptr->angle - ptr->last_angle < -4096)
		ptr->round_cnt ++;
	ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;//总角度
}

/*this function should be called after system+can init *///在system和初始化后使用
void get_moto_offset(moto_measure_t *ptr, uint8_t* Data)
{
	ptr->angle = (uint16_t)(Data[0]<<8 | Data[1]) ;//转子偏移角度
	ptr->offset_angle = ptr->angle;
}

/**
*@bref 电机上电角度=0， 之后用这个函数更新3510电机的相对开机后（为0）的相对角度。
	*/
void get_total_angle(moto_measure_t *p){

	int res1, res2, delta;
	if(p->angle < p->last_angle){			//可能的情况
		res1 = p->angle + 8192 - p->last_angle;	//正转，delta=+
		res2 = p->angle - p->last_angle;				//反转	delta=-
	}else{	//angle > last
		res1 = p->angle - 8192 - p->last_angle ;//反转	delta -
		res2 = p->angle - p->last_angle;				//正转	delta +
	}
	//不管正反转，肯定是转的角度小的那个是真的
	if(ABS(res1)<ABS(res2))
		delta = res1;
	else
		delta = res2;

	p->total_angle += delta;
	p->last_angle = p->angle;
}

void set_moto_current(CAN_HandleTypeDef* hcan, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
//发送底盘4个电机数据
	Tx1Message.StdId = 0x200;
	Tx1Message.IDE = CAN_ID_STD;
	Tx1Message.RTR = CAN_RTR_DATA;
	Tx1Message.DLC = 0x08;
	Tx1Data[0] = (iq1 >> 8);//1号电流高八位
	Tx1Data[1] = iq1;
	Tx1Data[2] = (iq2 >> 8);//2号电流高八位
	Tx1Data[3] = iq2;
	Tx1Data[4] = iq3 >> 8;//3号电流高八位
	Tx1Data[5] = iq3;
	Tx1Data[6] = iq4 >> 8;//4号电流高八位
	Tx1Data[7] = iq4;

	HAL_CAN_AddTxMessage(hcan,&Tx1Message,Tx1Data,&Box);
}

void wheel_init()
{
  my_can_filter_init_recv_all(&hcan1);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
  for(int i=0; i<4; i++)
    {
  		float pid[3]={5,0.5,0};
  		PID_init(&motor_pid[i], PID_DELTA, pid, 16800, 1000);
    }
	{
		float pid[3]={60,0,0};
		PID_init(&hwt_pid, PID_POSITION, pid, 6000, 0.1);
	}
  motor_pid[0].set=0;
  motor_pid[1].set=0;
  motor_pid[2].set=0;
  motor_pid[3].set=0;
}


void init_calculate_values()
{
    static float WHEELBASE = 0.3f; // 轴距
    static float WHEELTRACK = 0.3f; // 轮距
    static float GIMBAL_OFFSET = 0.0f; // 云台偏移，假设为0
    static float PERIMETER = 3.14159f * 0.07f; // 轮子的周长，假设轮子直径为0.07m
    static float CHASSIS_DECELE_RATIO = 36.0f; // 减速比
    static float RADIAN_COEF = 57.3f; // 弧度转角度的系数

    rotate_ratio_f = ((WHEELBASE + WHEELTRACK) / 2.0f - GIMBAL_OFFSET) / RADIAN_COEF;
    rotate_ratio_b = ((WHEELBASE + WHEELTRACK) / 2.0f + GIMBAL_OFFSET) / RADIAN_COEF;
    wheel_rpm_ratio = 60.0f / (PERIMETER * CHASSIS_DECELE_RATIO);
}

void M2006_CAL()
{

	distance_r =22.9*(moto_chassis[0].round_cnt-moto_chassis[1].round_cnt+moto_chassis[2].round_cnt-moto_chassis[3].round_cnt)/144.0;
	distance_big_dia=32.9*(moto_chassis[1].round_cnt-moto_chassis[3].round_cnt)/72.0;
}

//适用于平移算距离
int delay_dis_diagnal(int dis)
{
	int dis0=distance_big_dia;
	if(dis>=0)
	{
		while(distance_big_dia<dis0+dis)
		{

		}
		return 1;
	}
	else
		{
			while(distance_big_dia>dis0+dis)
			{
				
			}
			return 1;

		}
}


int delay_distance(int dis)
{
	int dis0=distance_r;
	if(dis>=0)
	{
		while(distance_r<dis0+dis)
		{

		}
		return 1;
	}
	else
		{
			while(distance_r>dis0+dis)
			{
				//&&!AutoAim_Data_Receive.turn_flag);
			}
			return 1;

		}
}

/*****************************M2006电机pid******************************/
void _M2006Task()
{
	M2006_CAL();
	for(int i=0; i<4; i++)
	{
		PID_calc(&motor_pid[i],moto_chassis[i].speed_rpm,motor_pid[i].set);
	}

	if(Weak_mode==0)
		{
			set_moto_current(&hcan1,motor_pid[0].out,motor_pid[1].out,motor_pid[2].out,motor_pid[3].out);
		}
/****************************************************************************/
else{set_moto_current(&hcan1,0,0,0,0);

	}
}


/*****************************M2006麦轮运动解析******************************/
// 整车移动量转换为单轮速度  x:左-右+ y: 前+后-  z:逆+顺- yaw:目标角度
int chassis_Radius=10;

void Move_Transfrom(double vx,double vy,double vz,double yaw)
{
    /********直接计算方法********/
    float sin_ang = sin(45);
    float cos_ang = cos(45);

    // 计算四个电机的速度
    wheel_rpm[0] = ((-cos_ang - sin_ang) * vx + (-sin_ang + cos_ang) * vy + chassis_Radius * vz) / sqrt(2);
    wheel_rpm[1] = ((-cos_ang + sin_ang) * vx + (-sin_ang - cos_ang) * vy + chassis_Radius * vz) / sqrt(2);
    wheel_rpm[2] = ((cos_ang + sin_ang) * vx + (sin_ang - cos_ang) * vy + chassis_Radius * vz) / sqrt(2);
    wheel_rpm[3] = ((cos_ang - sin_ang) * vx + (sin_ang + cos_ang) * vy + chassis_Radius * vz) / sqrt(2);
    yaw0=yaw;
}

/*****************************Yaw角pid******************************/
void _DingjiaoPIDTask()
{
		static int gyro1=0;
		xQueueReceive(Queue_HandleStraightHandle, &gyro1, portMAX_DELAY);
		double err=gyro1-yaw0;
		if(err > 180)
		{
			err = err-360;
		}
		if(err < -180)
		{
			err = err+360;
		}
		//int speed_temp=(moto_chassis[0].speed_rpm+moto_chassis[1].speed_rpm+moto_chassis[2].speed_rpm+moto_chassis[3].speed_rpm)/4.0;
		if(my_abs(err)<=1)
		{
			dingjiao_flag=1;//表示不需要进行转角调节
		}
		hwt_pid.set=0;
		hwt_pid.Ki=0.2+0.6*((my_abs(err)-90)/90);
		//dingjiao_pid.ki=2*(my_abs(err)/180.0);
		PID_calc(&hwt_pid,err,0);
		w = hwt_pid.out;
		hwt_cal(wheel_rpm[0],wheel_rpm[1],wheel_rpm[2],wheel_rpm[3],w);
}


void stop()
{
	  motor_pid[0].set=0;
	  motor_pid[1].set=0;
	  motor_pid[2].set=0;
	  motor_pid[3].set=0;
}

void hwt_cal(int32_t speed0,int32_t speed1,int32_t speed2,int32_t speed3,int32_t w_t)
{
	motor_pid[0].set = speed0-w_t;
	motor_pid[1].set = speed1-w_t;
	motor_pid[2].set = speed2-w_t;
	motor_pid[3].set = speed3-w_t;
}


float my_abs(float data)
{
	return data>0?data:-data;
}










