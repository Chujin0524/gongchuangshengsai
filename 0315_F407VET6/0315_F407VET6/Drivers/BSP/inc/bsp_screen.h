/*
*********************************************************************************************************
*
*	ģ������ : SCREEN ����WIFIģ����������
*	�ļ����� : bsp_SCREEN.h
*	��    �� : V1.0
*	˵    �� : ͷ�ļ�
*
*	Copyright (C), 2014-2015, ���������� www.armfly.com
*
*********************************************************************************************************
*/

#ifndef __BSP_SCREEN_H
#define __BSP_SCREEN_H

#define COM_SCREEN	COM4		/* ѡ�񴮿� */

/* ����������仰, �����յ����ַ����͵����Դ���1 */
//#define SCREEN_TO_COM1_EN

/* ��ģ�鲿�ֺ����õ��������ʱ�����1��ID�� �����������ñ�ģ��ĺ���ʱ����ע��رܶ�ʱ�� TMR_COUNT - 1��
  bsp_StartTimer(3, _usTimeOut);

  TMR_COUNT �� bsp_timer.h �ļ�����
*/
#define SCREEN_TMR_ID	(TMR_COUNT - 1)



/* ���ⲿ���õĺ������� */
void bsp_InitSCREEN(void);
void SCREEN_SendData(uint8_t *_databuf, uint16_t _len);
void SCREEN_Dispaly(uint8_t *_ucaQRScan_buff);
#endif

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
