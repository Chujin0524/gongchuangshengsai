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
  * @brief    速度模式
  * @param    addr：电机地址
  * @param    dir ：方向       ，0为CW，其余值为CCW
  * @param    vel ：速度       ，范围0 - 5000RPM
  * @param    acc ：加速度     ，范围0 - 255，注意：0是直接启动
  * @param    snF ：多机同步标志，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
{
  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xF6;                       // 功能码
  cmd[2] =  dir;                        // 方向
  cmd[3] =  (uint8_t)(vel >> 8);        // 速度(RPM)高8位字节
  cmd[4] =  (uint8_t)(vel >> 0);        // 速度(RPM)低8位字节
  cmd[5] =  acc;                        // 加速度，注意：0是直接启动
  cmd[6] =  snF;                        // 多机同步运动标志
  cmd[7] =  0x6B;                       // 校验字节
  
  // 发送命令
  can_SendCmd(cmd, 8);
}

/**
  * @brief    位置模式
  * @param    addr：电机地址
  * @param    dir ：方向        ，0为CW，其余值为CCW
  * @param    vel ：速度(RPM)   ，范围0 - 5000RPM
  * @param    acc ：加速度      ，范围0 - 255，注意：0是直接启动
  * @param    clk ：脉冲数      ，范围0- (2^32 - 1)个
  * @param    raF ：相位/绝对标志，false为相对运动，true为绝对值运动
  * @param    snF ：多机同步标志 ，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF)
{
  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0]  =  addr;                      // 地址
  cmd[1]  =  0xFD;                      // 功能码
  cmd[2]  =  dir;                       // 方向
  cmd[3]  =  (uint8_t)(vel >> 8);       // 速度(RPM)高8位字节
  cmd[4]  =  (uint8_t)(vel >> 0);       // 速度(RPM)低8位字节 
  cmd[5]  =  acc;                       // 加速度，注意：0是直接启动
  cmd[6]  =  (uint8_t)(clk >> 24);      // 脉冲数(bit24 - bit31)
  cmd[7]  =  (uint8_t)(clk >> 16);      // 脉冲数(bit16 - bit23)
  cmd[8]  =  (uint8_t)(clk >> 8);       // 脉冲数(bit8  - bit15)
  cmd[9]  =  (uint8_t)(clk >> 0);       // 脉冲数(bit0  - bit7 )
  cmd[10] =  raF;                       // 相位/绝对标志，false为相对运动，true为绝对值运动
  cmd[11] =  snF;                       // 多机同步运动标志，false为不启用，true为启用
  cmd[12] =  0x6B;                      // 校验字节
  
  // 发送命令
  can_SendCmd(cmd, 13);
}

/**
  * @brief    立即停止（所有控制模式都通用）
  * @param    addr  ：电机地址
  * @param    snF   ：多机同步标志，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Stop_Now(uint8_t addr, bool snF)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xFE;                       // 功能码
  cmd[2] =  0x98;                       // 辅助码
  cmd[3] =  snF;                        // 多机同步运动标志
  cmd[4] =  0x6B;                       // 校验字节
  
  // 发送命令
  can_SendCmd(cmd, 5);
}

/**
  * @brief    多机同步运动
  * @param    addr  ：电机地址
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Synchronous_motion(uint8_t addr)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xFF;                       // 功能码
  cmd[2] =  0x66;                       // 辅助码
  cmd[3] =  0x6B;                       // 校验字节
  
  // 发送命令
  can_SendCmd(cmd, 4);
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
