#include "main.h"

static CAN_RxHeaderTypeDef RxHeader;
static uint8_t RxData[8];
__IO static uint8_t addr = 0;
__IO static uint8_t ackFlag = 0;

/**
    * @brief   CAN发送多个字节
    * @param   无
    * @retval  无
    */
void can_SendCmd(__IO uint8_t *cmd, uint8_t len)
{
    static uint32_t TxMailbox;
    CAN_TxHeaderTypeDef CAN_TxMsg;
    uint8_t txData[32];
    __IO uint8_t i = 0, j = 0, k = 0, l = 0, packNum = 0;

    // 除去ID地址和功能码后的数据长度
    j = len - 2;

    // 发送数据
    while (i < j)
    {
        // 数据个数
        k = j - i;

        // 填充缓存
        CAN_TxMsg.StdId = 0x00;
        CAN_TxMsg.ExtId = ((uint32_t)cmd[0] << 8) | (uint32_t)packNum;
        txData[0] = cmd[1];
        CAN_TxMsg.IDE = CAN_ID_EXT;
        CAN_TxMsg.RTR = CAN_RTR_DATA;

        // 小于8字节命令
        if (k < 8)
        {
            for (l = 0; l < k; l++, i++)
            {
                txData[l + 1] = cmd[i + 2];
            }
            CAN_TxMsg.DLC = k + 1;
        }
        // 大于8字节命令，分包发送，每包数据最多发送8个字节
        else
        {
            for (l = 0; l < 7; l++, i++)
            {
                txData[l + 1] = cmd[i + 2];
            }
            CAN_TxMsg.DLC = 8;
        }

        // 发送数据
        while (HAL_CAN_AddTxMessage((&hcan2), (CAN_TxHeaderTypeDef *)(&CAN_TxMsg), (uint8_t *)(&txData), (&TxMailbox)) != HAL_OK);

        // 记录发送的第几包的数据
        ++packNum;
    }
}

/**
    * @brief   CAN等待地址为_id的电机应答
    * @param   无
    * @retval  无
    */
void can_WaitAck(uint8_t _addr)
{
    addr = _addr;
    while (ackFlag == 0);
    ackFlag = 0;
}

void CAN2_ReadMsg(void)
{
    if( HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &RxHeader, RxData ) != HAL_OK )
    {
//        printf("Parse hcan2 error\r\n");
        return;
    }
//    printf("*************ZDT_CAN**************\r\n");
//    printf("addr = %02X\r\n", addr);
//    printf("StdID = %02X\r\n", RxHeader.StdId);
//    printf("ExtID = %02X\r\n", RxHeader.ExtId);
//    printf("RTR(0=Data, 2=Remote) = %d\r\n", RxHeader.RTR);
//    printf("IDE(0=Std, 4=Ext) = %d\r\n", RxHeader.IDE);
//    printf("DLC(Data Length) = %d\r\n", RxHeader.DLC);
//    printf("Data = %02X %02X %02X %02X\r\n", RxData[0], RxData[1], RxData[2], RxData[3]);
//    printf("*************ZDT_CAN***************\r\n");
    
    if (addr == (RxHeader.ExtId>>8))
    {
        if (RxData[0] == 0xFD && RxData[1] == 0x9F && RxData[2] == 0x6B)
        {
            ackFlag = 1;
//            printf("ackFlag = 1\r\n");
        }
    }
    memset(&RxHeader, 0, sizeof(RxHeader));
}
