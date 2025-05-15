/*
*********************************************************************************************************
*
*   ģ������ : BLUETOOTH ����WIFIģ����������
*   �ļ����� : bsp_BLUETOOTH.c
*   ��    �� : V1.1
*   ˵    �� : ��װ BLUETOOTH ģ����ص�����
*
*********************************************************************************************************
*/

#include "main.h"

/* BLUETOOTH ģ�����ͼ


        UTXD   ---  PA10/USART1_RX
        URXD   ---  PA9 /USART1_TX

    ģ��ȱʡ������ 115200

*/

static BLUETOOTH_HandleFrameCallback s_BLUETOOTH_HandleFrame = NULL;

#define BLUETOOTH_FRAME_HEADER '<'
#define BLUETOOTH_FRAME_TAIL   '>'
static uint8_t s_ucBLUETOOTHFrameData[64] = {0};
static UART_PARSE_STATE_E s_ucState = STATE_IDLE;
static FRAME_T s_tBLUETOOTHFrame =
{
    .ucFrameHeader[0] = BLUETOOTH_FRAME_HEADER,
    .ucFrameTail[0] = BLUETOOTH_FRAME_TAIL,

    .ucFrameIndex = 0,
    .pucFrameData = s_ucBLUETOOTHFrameData,
};

/*
*********************************************************************************************************
*   �� �� ��: bsp_InitBLUETOOTH
*   ����˵��: ��ʼ����ά��ɨ��ģ��Ĵ��ڣ�ע�����ݽ����ص��������ú����� bsp_Init() ���á�
*   ��    ��: ��
*   �� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_InitBLUETOOTH( BLUETOOTH_HandleFrameCallback _BLUETOOTH_HandleFrame )
{
    //ʹ�ô���4������bsp_uart_fifo.c�����ʼ�����˴������ٽ��г�ʼ��

    if( _BLUETOOTH_HandleFrame != NULL )
    {
        s_BLUETOOTH_HandleFrame = _BLUETOOTH_HandleFrame;
    }
}

/*
*********************************************************************************************************
*   �� �� ��: BLUETOOTH_PrintRxData
*   ����˵��: ��ӡSTM32��BLUETOOTH�յ������ݵ�COM1���ڣ���Ҫ���ڸ��ٵ���
*   ��    ��: _ch : �յ�������
*   �� �� ֵ: ��
*********************************************************************************************************
*/
static void BLUETOOTH_PrintRxData( uint8_t _ch )
{
#ifdef BLUETOOTH_TO_COM1_EN
    HAL_UART_Transmit(&huart1, &_ch, 1, 0xffff ); /* �����յ����ݴ�ӡ�����Դ���1 */
#endif
}

/*
*********************************************************************************************************
*   �� �� ��: BLUETOOTH_SendData
*   ����˵��: �������ݰ�
*   ��    ��: _databuf ����
*            _len ���ݳ���
*   �� �� ֵ: ��
*********************************************************************************************************
*/
void BLUETOOTH_SendData( uint8_t* _databuf, uint16_t _len )
{
    if( _len > 2048 )
    {
        _len = 2048;
    }

    comSendBuf( COM_BLUETOOTH, _databuf, _len );
}

/*
*********************************************************************************************************
*   �� �� ��: BLUETOOTH_ReadData
*   ����˵��: ��ȡ�������ݣ�֡�����ɹ�����ûص������������ݽ������ú�����app_BLUETOOTH�б�����
*   ��    ��:  ��
*   �� �� ֵ: ��
*********************************************************************************************************
*/
uint8_t BLUETOOTH_ReadData( void )
{
    uint8_t ucData = 0;
    uint8_t ret = 0;

    /* �� ���ڶ�ȡ1���ֽ� comGetChar() ������ bsp_uart_fifo.c ʵ�� */
    while( 1 )
    {
        if( comGetChar( COM_BLUETOOTH, &ucData ) )
        {
            BLUETOOTH_PrintRxData( ucData );

            switch( s_ucState )
            {
            case STATE_IDLE:
                if( ucData == s_tBLUETOOTHFrame.ucFrameHeader[0] ) //�յ�֡ͷ
                {
                    s_ucState = STATE_RECEIVING;
                    s_tBLUETOOTHFrame.ucFrameIndex = 0;               //����������׼�����������򣨰�������������ݣ�
                }
                else
                {
//                         LED_R_ON();
                }
                break;

            case STATE_RECEIVING:
                if( s_tBLUETOOTHFrame.ucFrameIndex < ( sizeof( s_ucBLUETOOTHFrameData ) -1 ) ) //��ֹ���������
                {
                    if( ucData == s_tBLUETOOTHFrame.ucFrameTail[0] )           //�յ�֡β
                    {
                        s_tBLUETOOTHFrame.pucFrameData[s_tBLUETOOTHFrame.ucFrameIndex] = '\0'; //����ַ���������

                        // ֡�����ɹ�����������
                        if( s_BLUETOOTH_HandleFrame != NULL )
                        {
                            s_BLUETOOTH_HandleFrame( s_tBLUETOOTHFrame.pucFrameData, s_tBLUETOOTHFrame.ucFrameIndex );

                            //���ճɹ�����λ
                            ret = 1;
                            s_ucState = STATE_IDLE;
                        }
                    }
                    else
                    {
                        s_tBLUETOOTHFrame.pucFrameData[s_tBLUETOOTHFrame.ucFrameIndex++] = ucData; //������յ�������
                    }
                }
                else
                {
                    //�����������״̬��λ
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
