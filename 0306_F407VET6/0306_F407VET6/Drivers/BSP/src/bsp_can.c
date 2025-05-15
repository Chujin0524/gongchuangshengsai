#include "main.h"

static void CAN_SetFilters( void );

void bsp_InitCAN(void)
{
    CAN_SetFilters();
    
    HAL_CAN_Start(&hcan1);
    HAL_CAN_Start(&hcan2);

    __HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    __HAL_CAN_ENABLE_IT(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);    
}

static void CAN_SetFilters( void )
{
    CAN_FilterTypeDef canFilter;
    canFilter.SlaveStartFilterBank = 14;

    // 1. 设置CAN1 FIFO0的筛选器
    canFilter.FilterBank = 0;
    canFilter.FilterMode = CAN_FILTERMODE_IDMASK;
    canFilter.FilterScale = CAN_FILTERSCALE_32BIT;

    canFilter.FilterIdHigh = 0x0000;
    canFilter.FilterIdLow = 0x0000;
    canFilter.FilterMaskIdHigh = 0x0000;
    canFilter.FilterMaskIdLow = 0x0000;

    canFilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    canFilter.FilterActivation = CAN_FILTER_ENABLE;
    HAL_CAN_ConfigFilter(&hcan1, &canFilter);

    // 1. 设置CAN2 FIFO0的筛选器
    canFilter.FilterBank = 15;
    canFilter.FilterMode = CAN_FILTERMODE_IDMASK;
    canFilter.FilterScale = CAN_FILTERSCALE_32BIT;

    canFilter.FilterIdHigh = 0x0000;
    canFilter.FilterIdLow = 0x0000;
    canFilter.FilterMaskIdHigh = 0x0000;
    canFilter.FilterMaskIdLow = 0x0000;

    canFilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    canFilter.FilterActivation = CAN_FILTER_ENABLE;
    HAL_CAN_ConfigFilter(&hcan2, &canFilter);
}

void HAL_CAN_RxFifo0MsgPendingCallback( CAN_HandleTypeDef *hcan )
{
    if (hcan == &hcan1)
    {
//        printf("Message received by hcan1\r\n");     
        CAN1_ReadMsg();                                //测试通过，可以进入该函数并打印邮箱信息
    }
    else if (hcan == &hcan2)
    {
//        printf("Message received by hcan2\r\n");
        CAN2_ReadMsg();                                //测试通过，可以进入该函数并打印邮箱信息
    }
}
