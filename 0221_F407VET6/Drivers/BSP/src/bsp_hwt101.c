#include "bsp_hwt101.h"
#include "stdio.h"
#include <stdbool.h>
#include "string.h"
#include "cmsis_os.h"
#include "main.h"
#define RINGBUFF_hwt101_LEN 500
#define FRAMELENGTH_hwt101 11

int hwt_flag=1;
extern UART_HandleTypeDef huart2;

typedef struct
{
    uint16_t Head;
    uint16_t Tail;
    uint16_t Lenght;
    uint8_t  Ring_data[RINGBUFF_hwt101_LEN];
}RingBuff_t_hwt;

RingBuff_t_hwt ringBuff_hwt101;

//void hwt_init(void)
//{
//    uint8_t tempData[5] = {0xFF, 0xAA, 0x76, 0x00, 0x00};
//    uint8_t unlock[5] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};//解锁寄存器
//    uint8_t get_zero[5] = {0xFF, 0xAA, 0x48, 0x01, 0x00};//自动获取零偏
//    uint8_t save_zero[5] = {0xFF, 0xAA, 0x00, 0x00, 0x00};//保存在寄存器中
//    int i;
//    for (i = 0; i < 5; i++) //解锁寄存器
//    {
//        HAL_UART_Transmit(&huart2, &unlock[i], 1, 100);
//    }

//    for(int test1=0;test1<5000;test1++){
//    		//1s延时
//    }

//    /*for (i = 0; i < 5; i++)//自动零偏
//    {
//        HAL_UART_Transmit(&huart2, &get_zero[i], 1, 100);
//    }

//    for(int test1=0;test1<20000;test1++){
//    	for(int test2=0;test2<30000;test2++){
//    		//31s延时
//    	}
//    }*/

//    for (i = 0; i < 5; i++) // z轴清零
//    {
//        HAL_UART_Transmit(&huart2, &tempData[i], 1, 100);
//    }

//    for(int test1=0;test1<5000;test1++){
//    		//1s延时
//    }

//    for (i = 0; i < 5; i++)//解锁寄存器
//    {
//        HAL_UART_Transmit(&huart2, &unlock[i], 1, 100);
//    }

//    for(int test1=0;test1<5000;test1++){
//    		//1s延时
//    }

//    for (i = 0; i < 5; i++)// 保存
//    {
//        HAL_UART_Transmit(&huart2, &save_zero[i], 1, 100);
//    }

//	//osDelay(500);
//	//set_angle(4, 60);


//}


void initRingBuff_hwt101(void)
{
  //初始化
  ringBuff_hwt101.Head = 0;
  ringBuff_hwt101.Tail = 0;
  ringBuff_hwt101.Lenght = 0;
}


void writeRingBuff_hwt101(uint8_t data)
{
  if(ringBuff_hwt101.Lenght >= RINGBUFF_hwt101_LEN)
  {
    return ;
  }
  ringBuff_hwt101.Ring_data[ringBuff_hwt101.Tail]=data;
  ringBuff_hwt101.Tail = (ringBuff_hwt101.Tail+1)%RINGBUFF_hwt101_LEN;//防止非法越界
  ringBuff_hwt101.Lenght++;

}

void deleteRingBuff_hwt101(uint16_t size)
{
    if(size >= ringBuff_hwt101.Lenght)
    {
        initRingBuff_hwt101();
        return;
    }
    for(int i = 0; i < size; i++)
    {

        if(ringBuff_hwt101.Lenght == 0)//判断非空
        {
        initRingBuff_hwt101();
        return;
        }
        ringBuff_hwt101.Head = (ringBuff_hwt101.Head+1) % RINGBUFF_hwt101_LEN;//防止非法越界
        ringBuff_hwt101.Lenght--;

    }

}


uint8_t read1BFromRingBuff_hwt101(uint16_t position)
{
    uint16_t realPosition_hwt101 = (ringBuff_hwt101.Head + position) % RINGBUFF_hwt101_LEN;

    return ringBuff_hwt101.Ring_data[realPosition_hwt101];
}

uint16_t getRingBuffLenght_hwt101()
{
    return ringBuff_hwt101.Lenght;
}

int gyro_z = 0;

bool isNegativeHex(uint8_t hexValue) {
    // 检查最高位是否为1，如果是，则为负数
    return (hexValue & 0x80) != 0;
}


void data_parsing_hwt101(void)
{
    while (jsize()>= 11)
    {
        if(j(0) != 0x55 || j(1) != 0x53 || j(8) != 	0xE1 || j(9) != 0x27)
    {
        jdelete(1);
    }
    else
    {
        break;
    }

    }

    if(jsize() >= 11&& j(0) == 0x55 && j(1) == 0x53 && j(8) == 0xE1 && j(9) == 0x27)
    {
       if (isNegativeHex(j(7)))
        {
            int gyro_z_l = j(6);
            int gyro_z_h = j(7);
            gyro_z = (gyro_z_h * 256.0 + gyro_z_l- 128.0)/32768.0*180.0-360.0;

        }
        else
        {
            int gyro_z_l = j(6);
            int gyro_z_h = j(7);
            gyro_z = (gyro_z_h * 256.0 + gyro_z_l)/32768.0*180.0;
            //gyro_z=gyro_z;

        }

        jdelete(FRAMELENGTH_hwt101);
    }
		xQueueOverwrite(Queue_HandleStraightHandle,&gyro_z);

}


