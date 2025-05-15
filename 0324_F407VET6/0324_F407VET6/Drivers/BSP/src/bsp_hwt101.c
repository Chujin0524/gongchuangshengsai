/*
*********************************************************************************************************
*
*   ģ������ : HWT101 ����WIFIģ����������
*   �ļ����� : bsp_HWT101.c
*   ��    �� : V1.1
*   ˵    �� : ��װ HWT101 ģ����ص�����
*
*********************************************************************************************************
*/

#include "main.h"

/* HWT101 ģ�����ͼ


        UTXD   ---  PB11/USART2_RX
        URXD   ---  PB10/USART2_TX

    ģ��ȱʡ������ 115200

*/

static HWT101_HandleFrameCallback s_HWT101_HandleFrame = NULL;

#define HWT101_FRAMEHEADER0 0x55
#define HWT101_FRAMEHEADER1 0x53
#define HWT101_FRAMETAIL0   0x53
#define HWT101_FRAMETAIL1   0x53
static uint8_t s_ucHWT101FrameData[32] = {0};
static FRAME_T s_tHWT101Frame =
{
    .ucFrameHeader[0] = HWT101_FRAMEHEADER0,
    .ucFrameHeader[1] = HWT101_FRAMEHEADER1,
    .ucFrameTail[0] = HWT101_FRAMETAIL0,
    .ucFrameTail[1] = HWT101_FRAMETAIL1,

    .ucFrameIndex = 0,
    .ucFrameLength = 0,
    .pucFrameData = s_ucHWT101FrameData,
    .ucCheckSum = 0
};

int hwt_flag = 1;

/*
*********************************************************************************************************
*   �� �� ��: bsp_InitHWT101
*   ����˵��: ��ʼ����ά��ɨ��ģ��Ĵ��ڣ�ע�����ݽ����ص��������ú����� bsp_Init() ���á�
*   ��    ��: ��
*   �� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_InitHWT101( HWT101_HandleFrameCallback _HWT101_HandleFrame )
{
    //ʹ�ô���2������bsp_uart_fifo.c�����ʼ�����˴������ٽ��г�ʼ��

    if( _HWT101_HandleFrame != NULL )
    {
        s_HWT101_HandleFrame = _HWT101_HandleFrame;
    }
}
/*
*********************************************************************************************************
*   �� �� ��: hwt_init
*   ����˵��: ��ӡSTM32��HWT101�յ������ݵ�COM1���ڣ���Ҫ���ڸ��ٵ���
*   ��    ��: _ch : �յ�������
*   �� �� ֵ: ��
*********************************************************************************************************
*/
void hwt_init(void)
{
    uint8_t tempData_head[5] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
    uint8_t tempData[5] = {0xFF, 0xAA, 0x76, 0x00, 0x00};
    uint8_t tempData_tail[5] = {0xFF, 0xAA, 0x00, 0x00, 0x00};
    HAL_UART_Transmit_DMA(&huart3, tempData_head,sizeof(tempData_head));
    HAL_Delay(1000);
    HAL_UART_Transmit_DMA(&huart3, tempData,sizeof(tempData));
    HAL_Delay(1000);
    HAL_UART_Transmit_DMA(&huart3, tempData_tail,sizeof(tempData_tail));
    HAL_Delay(1000);
}

/*
*********************************************************************************************************
*   �� �� ��: HWT101_PrintRxData
*   ����˵��: ��ӡSTM32��HWT101�յ������ݵ�COM1���ڣ���Ҫ���ڸ��ٵ���
*   ��    ��: _ch : �յ�������
*   �� �� ֵ: ��
*********************************************************************************************************
*/
static void HWT101_PrintRxData( uint8_t _ch )
{
#ifdef HWT101_TO_COM1_EN
    HAL_UART_Transmit(&huart1, &_ch, 1, 0xffff ); /* �����յ����ݴ�ӡ�����Դ���1 */
#endif
}

/*
*********************************************************************************************************
*   �� �� ��: HWT101_SendData
*   ����˵��: �������ݰ�
*   ��    ��: _databuf ����
*            _len ���ݳ���
*   �� �� ֵ: ��
*********************************************************************************************************
*/
void HWT101_SendData( uint8_t* _databuf, uint16_t _len )
{
    if( _len > 2048 )
    {
        _len = 2048;
    }

    comSendBuf( COM_HWT101, _databuf, _len );
}

/*
*********************************************************************************************************
*   �� �� ��: HWT101_ReadData
*   ����˵��: ��ȡ�������ݣ�֡�����ɹ�����ûص������������ݽ������ú�����app_HWT101�б�����
*   ��    ��:  ��
*   �� �� ֵ: ��
*********************************************************************************************************
*/
uint8_t HWT101_ReadData( void )
{
    static UART_PARSE_STATE_E s_ucState = STATE_IDLE;
    uint8_t ucData = 0;
    uint8_t ret = 0;

    /* �� ���ڶ�ȡ1���ֽ� comGetChar() ������ bsp_uart_fifo.c ʵ�� */
    while( 1 )
    {
        if( comGetChar( COM_HWT101, &ucData ) )
        {
            HWT101_PrintRxData( ucData );

            switch( s_ucState )
            {
            case STATE_IDLE:
                if( ucData == s_tHWT101Frame.ucFrameHeader[0] ) //�յ�֡ͷ0
                {
                    s_ucState = STATE_HEADER_0;
                }
                else
                {
//                         LED_R_ON();
                }
                break;

            case STATE_HEADER_0:
                if( ucData == s_tHWT101Frame.ucFrameHeader[1] ) //�յ�֡ͷ1
                {
                    s_tHWT101Frame.ucFrameIndex = 0;      //����������׼������������
                    s_tHWT101Frame.ucFrameLength = 8;     //֡���������򳤶ȣ��������ݺͰ汾�ţ�
                    s_tHWT101Frame.ucCheckSum = s_tHWT101Frame.ucFrameHeader[0] + s_tHWT101Frame.ucFrameHeader[1];
                    s_ucState = STATE_DATA;
                }
                else
                {
//                         LED_R_ON();
                    s_ucState = STATE_IDLE;
                }
                break;

            case STATE_DATA:
                s_tHWT101Frame.pucFrameData[s_tHWT101Frame.ucFrameIndex++] = ucData;
                if( s_tHWT101Frame.ucFrameIndex <= s_tHWT101Frame.ucFrameLength )    //����֡���ض�
                {
                    s_tHWT101Frame.ucCheckSum += ucData;                                  //�յ�����
                }
                else
                {
                    if( ucData == ( s_tHWT101Frame.ucCheckSum & 0xFF ) )     //У���
                    {
                        // ֡�����ɹ�����������
                        if( s_HWT101_HandleFrame != NULL )
                        {
                            ret = 1;
                            s_HWT101_HandleFrame( s_tHWT101Frame.pucFrameData, s_tHWT101Frame.ucFrameLength );
                            comClearRxFifo(COM_HWT101);
                        }
                    }
                    s_ucState = STATE_IDLE;
                }
                break;

            default:
                s_ucState = STATE_IDLE;
                break;
            }

            continue;   /* ���ܻ������ݣ��������� */
        }

        break;  /* ������ϣ��˳����� */
    }

    return ret;
}











//void hwt_init(void)
//{
//    uint8_t tempData[5] = {0xFF, 0xAA, 0x76, 0x00, 0x00};
//    uint8_t unlock[5] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};//�����Ĵ���
//    uint8_t get_zero[5] = {0xFF, 0xAA, 0x48, 0x01, 0x00};//�Զ���ȡ��ƫ
//    uint8_t save_zero[5] = {0xFF, 0xAA, 0x00, 0x00, 0x00};//�����ڼĴ�����
//    int i;
//    for (i = 0; i < 5; i++) //�����Ĵ���
//    {
//        HAL_UART_Transmit(&huart2, &unlock[i], 1, 100);
//    }

//    for(int test1=0;test1<5000;test1++){
//          //1s��ʱ
//    }

//    /*for (i = 0; i < 5; i++)//�Զ���ƫ
//    {
//        HAL_UART_Transmit(&huart2, &get_zero[i], 1, 100);
//    }

//    for(int test1=0;test1<20000;test1++){
//      for(int test2=0;test2<30000;test2++){
//          //31s��ʱ
//      }
//    }*/

//    for (i = 0; i < 5; i++) // z������
//    {
//        HAL_UART_Transmit(&huart2, &tempData[i], 1, 100);
//    }

//    for(int test1=0;test1<5000;test1++){
//          //1s��ʱ
//    }

//    for (i = 0; i < 5; i++)//�����Ĵ���
//    {
//        HAL_UART_Transmit(&huart2, &unlock[i], 1, 100);
//    }

//    for(int test1=0;test1<5000;test1++){
//          //1s��ʱ
//    }

//    for (i = 0; i < 5; i++)// ����
//    {
//        HAL_UART_Transmit(&huart2, &save_zero[i], 1, 100);
//    }

//  //osDelay(500);
//  //set_angle(4, 60);


//}
