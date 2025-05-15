/*
 * user_can.h
 *
 *  Created on: 2023年8月6日
 *      Author: 71492
 */

#ifndef __BSP_CAN_ZDT_H_
#define __BSP_CAN_ZDT_H_

void can_SendCmd(__IO uint8_t *cmd, uint8_t len);
void CAN2_ReadMsg(void);
void can_WaitAck(uint8_t _addr);

#endif /* INC_USER_CAN_H_ */
