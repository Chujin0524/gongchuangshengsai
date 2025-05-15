#include "main.h"
#include "bsp_pid.h"
#include "bsp_Emm_V5.h"
#include "bsp_can_2006.h"
#include "bsp_hwt101.h"
static CAN_RxHeaderTypeDef RxHeader;
static uint8_t RxData[8];
__IO static uint8_t addr = 0;
__IO static uint8_t ackFlag = 0;

int yaw0 = 0;
int Weak_mode = 0;
volatile float distance_r = 0;
volatile float distance_angel = 0;
volatile float distance_big_dia = 0;
int moto_angle = 0;
static float rotate_ratio_f;
static float rotate_ratio_b;
static float wheel_rpm_ratio;
float wheel_rpm[4] = {0};
int dingjiao_mode = 0;
int dingjiao_flag = 0;
int32_t w;              //å·®é€Ÿ
extern int gyro_z;

PID_TypeDef motor_pid[4];  //2006ç”µæœºpidç»“æ„ä½“
PID_TypeDef hwt_pid;

moto_measure_t moto_chassis[4] = {0};//4 chassis moto+1 åŒæ­¥å¸¦//ç”µæœºä¿¡æ¯ç»“æ„ä½“å‚¨å­˜0-4
CAN_TxHeaderTypeDef     Tx1Message;
CAN_RxHeaderTypeDef     Rx1Message;
uint8_t Rx1Data[8] = {0};
uint8_t Tx1Data[8] = {0};//1-4ç”µæœºçš„å‘é€ç”µæµæ•°æ®
uint32_t Box = 100;
uint32_t x = 0;


/*******************************************************************************************
  * @Func       my_can_filter_init
  * @Brief    CAN1å’ŒCAN2æ»¤æ³¢å™¨é…ç½®
  * @Param      CAN_HandleTypeDef* hcan
  * @Retval     None
  * @Date     2025/2/22
 *******************************************************************************************/
void my_can_filter_init_recv_all( CAN_HandleTypeDef* _hcan )
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
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st );
    HAL_CAN_Start(&hcan1 );
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING );

}


/*HAL_CAN_RxFifo0MsgPendingCallbackå›è°ƒå‡½æ•°æ‰€è°ƒç”¨çš„CAN1æ•°æ®è¯»å–è§£æ*/
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

    __HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING );
//    printf("*************2006_CAN**************\r\n");
//    printf("StdID = %d\r\n", RxHeader.StdId);
//    printf("ExtID = %d\r\n", RxHeader.ExtId);
//    printf("RTR(0=Data, 2=Remote) = %d\r\n", RxHeader.RTR);
//    printf("IDE(0=Std, 4=Ext) = %d\r\n", RxHeader.IDE);
//    printf("DLC(Data Length) = %d\r\n", RxHeader.DLC);
//    printf("Data = %02X %02X %02X %02X\r\n", RxData[0], RxData[1], RxData[2], RxData[3]);
//    printf("*************2006_CAN***************\r\n");
}

void get_moto_measure( moto_measure_t* ptr, uint8_t* Data ) //é‡‡é›†5ä¸ªç”µæœºçš„ä¿¡æ¯
{

    ptr->last_angle = ptr->angle;
    ptr->angle = ( uint16_t )( Data[0] << 8 | Data[1] ) ; //è½¬å­è§’åº¦
    ptr->speed_rpm  = ( int16_t )( Data[2] << 8 | Data[3] ); //è½¬å­é€Ÿåº¦
    ptr->real_current = ( Data[4] << 8 | Data[5] ) * 5.f / 16384.f; //è½¬å­è½¬çŸ©

    ptr->hall = Data[6];//null


    if( ptr->angle - ptr->last_angle > 4096 )
        ptr->round_cnt --;
    else if( ptr->angle - ptr->last_angle < -4096 )
        ptr->round_cnt ++;
    ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;//æ€»è§’åº¦
}

/*this function should be called after system+can init *///åœ¨systemå’Œåˆå§‹åŒ–åä½¿ç”¨
void get_moto_offset( moto_measure_t* ptr, uint8_t* Data )
{
    ptr->angle = ( uint16_t )( Data[0] << 8 | Data[1] ) ; //è½¬å­åç§»è§’åº¦
    ptr->offset_angle = ptr->angle;
}

/**
*@bref ç”µæœºä¸Šç”µè§’åº¦=0ï¼Œ ä¹‹åç”¨è¿™ä¸ªå‡½æ•°æ›´æ–°3510ç”µæœºçš„ç›¸å¯¹å¼€æœºåï¼ˆä¸º0ï¼‰çš„ç›¸å¯¹è§’åº¦ã€‚
    */
void get_total_angle( moto_measure_t* p )
{

    int res1, res2, delta;
    if( p->angle < p->last_angle )          //å¯èƒ½çš„æƒ…å†µ
    {
        res1 = p->angle + 8192 - p->last_angle; //æ­£è½¬ï¼Œdelta=+
        res2 = p->angle - p->last_angle;                //åè½¬    delta=-
    }
    else    //angle > last
    {
        res1 = p->angle - 8192 - p->last_angle ;//åè½¬    delta -
        res2 = p->angle - p->last_angle;                //æ­£è½¬    delta +
    }
    //ä¸ç®¡æ­£åè½¬ï¼Œè‚¯å®šæ˜¯è½¬çš„è§’åº¦å°çš„é‚£ä¸ªæ˜¯çœŸçš„
    if( ABS( res1 ) < ABS( res2 ) )
        delta = res1;
    else
        delta = res2;

    p->total_angle += delta;
    p->last_angle = p->angle;
}

void set_moto_current( CAN_HandleTypeDef* hcan, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4 )
{
//å‘é€åº•ç›˜4ä¸ªç”µæœºæ•°æ®
    Tx1Message.StdId = 0x200;
    Tx1Message.IDE = CAN_ID_STD;
    Tx1Message.RTR = CAN_RTR_DATA;
    Tx1Message.DLC = 0x08;
    Tx1Data[0] = ( iq1 >> 8 ); //1å·ç”µæµé«˜å…«ä½
    Tx1Data[1] = iq1;
    Tx1Data[2] = ( iq2 >> 8 ); //2å·ç”µæµé«˜å…«ä½
    Tx1Data[3] = iq2;
    Tx1Data[4] = iq3 >> 8;//3å·ç”µæµé«˜å…«ä½
    Tx1Data[5] = iq3;
    Tx1Data[6] = iq4 >> 8;//4å·ç”µæµé«˜å…«ä½
    Tx1Data[7] = iq4;

    HAL_CAN_AddTxMessage( hcan, &Tx1Message, Tx1Data, &Box );
}

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


void init_calculate_values()
{
    static float WHEELBASE = 0.3f; // è½´è·
    static float WHEELTRACK = 0.3f; // è½®è·
    static float GIMBAL_OFFSET = 0.0f; // äº‘å°åç§»ï¼Œå‡è®¾ä¸º0
    static float PERIMETER = 3.14159f * 0.07f; // è½®å­çš„å‘¨é•¿ï¼Œå‡è®¾è½®å­ç›´å¾„ä¸º0.07m
    static float CHASSIS_DECELE_RATIO = 36.0f; // å‡é€Ÿæ¯”
    static float RADIAN_COEF = 57.3f; // å¼§åº¦è½¬è§’åº¦çš„ç³»æ•°

    rotate_ratio_f = (( WHEELBASE + WHEELTRACK ) / 2.0f - GIMBAL_OFFSET ) / RADIAN_COEF;
    rotate_ratio_b = (( WHEELBASE + WHEELTRACK ) / 2.0f + GIMBAL_OFFSET ) / RADIAN_COEF;
    wheel_rpm_ratio = 60.0f / ( PERIMETER * CHASSIS_DECELE_RATIO );
}

void M2006_CAL()
{

    distance_r = 22.9 * ( moto_chassis[0].round_cnt - moto_chassis[1].round_cnt - moto_chassis[2].round_cnt + moto_chassis[3].round_cnt ) / 144.0;
    distance_big_dia = 22.9 * ( -moto_chassis[0].round_cnt - moto_chassis[1].round_cnt + moto_chassis[2].round_cnt + moto_chassis[3].round_cnt ) / 144.0;
}

/*
*********************************************************************************************************
*    º¯ Êı Ãû: delay_dis_diagnal
*    ¹¦ÄÜËµÃ÷: Æ½ÒÆ¾àÀëÑÓ³Ù
*    ·µ »Ø Öµ: ÎŞ
*********************************************************************************************************
*/

int delay_dis_diagnal( int dis )
{
    int dis0 = distance_big_dia;
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
*    º¯ Êı Ãû: delay_distance
*    ¹¦ÄÜËµÃ÷: Ç°ºó¾àÀëÑÓ³Ù
*    ·µ »Ø Öµ: ÎŞ
*********************************************************************************************************
*/

int delay_distance( int dis )
{
    int dis0 = distance_r;
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

/*****************************M2006ç”µæœºpid******************************/
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
    /****************************************************************************/
    else
    {
        set_moto_current(&hcan1, 0, 0, 0, 0 );
    }
}


/*****************************M2006ÂóÂÖÔË¶¯½âÎö******************************/
// Õû³µÒÆ¶¯Á¿×ª»»Îªµ¥ÂÖËÙ¶È  x:×ó+ÓÒ- y:Äæ+Ë³- (Ç°+ºó-)   z:Äæ+Ë³- yaw:Ä¿±ê½Ç¶È
void Move_Transfrom(double vx,double vy,double vw,double yaw)
{
	wheel_rpm[3] = (vx + vy + vw * rotate_ratio_f) * wheel_rpm_ratio;
	wheel_rpm[2] = (vx - vy - vw * rotate_ratio_f) * wheel_rpm_ratio;
	wheel_rpm[1] = (-vx - vy + vw * rotate_ratio_b) * wheel_rpm_ratio;
	wheel_rpm[0] = (-vx + vy - vw * rotate_ratio_b) * wheel_rpm_ratio;
	yaw0=yaw;
}

/*
*********************************************************************************************************
*    º¯ Êı Ãû: yaw½Çpid
*    ¹¦ÄÜËµÃ÷: ÍÓÂİÒÇĞ£Õı
*    ·µ »Ø Öµ: ÎŞ
*********************************************************************************************************
*/

void _DingjiaoPIDTask(void)
{
    //static int gyro1 = 0;
   // xQueueReceive( Queue_HandleStraightHandle, &gyro1, portMAX_DELAY );
    //double err = 0;
	double err = gyro_z - yaw0;
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
        dingjiao_flag = 1; //è¡¨ç¤ºä¸éœ€è¦è¿›è¡Œè½¬è§’è°ƒèŠ‚
    }
    hwt_pid.set = 0;
    hwt_pid.Ki = 0.2 + 0.6 * (( my_abs( err ) -90 ) / 90 );
    //dingjiao_pid.ki=2*(my_abs(err)/180.0);
    PID_calc(&hwt_pid, err, 0 );
    w = hwt_pid.out;
    hwt_cal( wheel_rpm[0], wheel_rpm[1], wheel_rpm[2], wheel_rpm[3], w );
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
*    º¯ Êı Ãû: Ramp_dis
*    ¹¦ÄÜËµÃ÷: ÌİĞÎ¼ÓËÙµ×²ã´úÂë£¬Ç°ºó
*    ĞÎ    ²Î£ºspd1:³õÊ¼ËÙ¶È
							 spd2:×îÖÕËÙ¶È
*    ·µ »Ø Öµ: ÎŞ
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
									} // ¼ÌĞøµü´ú
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
									} // ¼ÌĞøµü´ú
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
								} // ¼ÌĞøµü´ú
						}

			}
			
	}
	
Move_Transfrom(0,spd2,0, yaw); // È·±£×îÖÕËÙ¶ÈÉèÖÃÎªspd2
}

/*
*********************************************************************************************************
*    º¯ Êı Ãû: SpeedUP_track
*    ¹¦ÄÜËµÃ÷: ÌİĞÎ¼ÓËÙ
							Ç°£ºspeed_max+;ºó£ºspeed_max-
*    ĞÎ    ²Î£º
								speed_max£º ÆÚÍû×î´óËÙ¶È
								speed_final£º ×îÖÕËÙ¶È£¬Ò»°ãÎª0Í£Ö¹
								distance£º×Ü¾àÀë
								ramp_dis1£º¼ÓËÙ¾àÀë
								ramp_dis2£º¼õËÙ¾àÀë
								yaw£ºÆÚÍû½Ç¶È
*    ·µ »Ø Öµ: ÎŞ
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
*    º¯ Êı Ãû: Ramp_disx
*    ¹¦ÄÜËµÃ÷: ÌİĞÎ¼ÓËÙµ×²ã´úÂë£¬×óÓÒ
*    ĞÎ    ²Î£ºspd1:³õÊ¼ËÙ¶È
							 spd2:×îÖÕËÙ¶È
*    ·µ »Ø Öµ: ÎŞ
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
									} // ¼ÌĞøµü´ú
							}
					}
					Move_Transfrom(spd2,0,0, yaw); // È·±£×îÖÕËÙ¶ÈÉèÖÃÎªspd2
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
									} // ¼ÌĞøµü´ú
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
								} // ¼ÌĞøµü´ú
						}

			}
			
	}
	
Move_Transfrom(spd2,0,0, yaw); // È·±£×îÖÕËÙ¶ÈÉèÖÃÎªspd2			
}

/*
*********************************************************************************************************
*    º¯ Êı Ãû: SpeedUP_x
*    ¹¦ÄÜËµÃ÷: ÌİĞÎ¼ÓËÙ£¬·½Ïò×óÓÒ
							×ó£ºspeed_max+;ÓÒ£ºspeed_max-
*    ĞÎ    ²Î£º
								speed_max£º ÆÚÍû×î´óËÙ¶È
								speed_final£º ×îÖÕËÙ¶È£¬Ò»°ãÎª0Í£Ö¹
								distance£º×Ü¾àÀë
								ramp_dis1£º¼ÓËÙ¾àÀë
								ramp_dis2£º¼õËÙ¾àÀë
								yaw£ºÆÚÍû½Ç¶È
*    ·µ »Ø Öµ: ÎŞ
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








