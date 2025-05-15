/*
*********************************************************************************************************
*
*   ģ������ : MAIXCAM ����WIFIģ����������
*   �ļ����� : bsp_MAIXCAM.c
*   ��    �� : V1.1
*   ˵    �� : ��װ MAIXCAM ģ����ص�����
*
*********************************************************************************************************
*/

#include "main.h"

/* MAIXCAM ģ�����ͼ


        UTXD   ---  PA3/USART2_RX
        URXD   ---  PA2/USART2_TX

    ģ��ȱʡ������ 115200

*/

static MAIXCAM_HandleFrameCallback s_MAIXCAM_HandleFrame = NULL;

#define MAIXCAM_FRAMEHEADER 0xFF
#define MAIXCAM_FRAMETAIL   0xFE
static uint8_t s_ucMaixCamFrameData[32] = {0};
static FRAME_T s_tMaixCamFrame =
{
    .ucFrameHeader[0] = MAIXCAM_FRAMEHEADER,
    .ucFrameTail[0] = MAIXCAM_FRAMETAIL,

    .ucFrameIndex = 0,
    .ucFrameLength = 0,
    .pucFrameData = s_ucMaixCamFrameData,
    .ucCheckSum = 0
};

/*
*********************************************************************************************************
*   �� �� ��: bsp_InitMAIXCAM
*   ����˵��: ��ʼ����ά��ɨ��ģ��Ĵ��ڣ�ע�����ݽ����ص��������ú����� bsp_Init() ���á�
*   ��    ��: ��
*   �� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_InitMAIXCAM( MAIXCAM_HandleFrameCallback _MAIXCAM_HandleFrame )
{
    //ʹ�ô���2������bsp_uart_fifo.c�����ʼ�����˴������ٽ��г�ʼ��

    if( _MAIXCAM_HandleFrame != NULL )
    {
        s_MAIXCAM_HandleFrame = _MAIXCAM_HandleFrame;
    }
}

/*
*********************************************************************************************************
*   �� �� ��: MAIXCAM_PrintRxData
*   ����˵��: ��ӡSTM32��MAIXCAM�յ������ݵ�COM1���ڣ���Ҫ���ڸ��ٵ���
*   ��    ��: _ch : �յ�������
*   �� �� ֵ: ��
*********************************************************************************************************
*/
static void MAIXCAM_PrintRxData( uint8_t _ch )
{
#ifdef MAIXCAM_TO_COM1_EN
    HAL_UART_Transmit(&huart1, &_ch, 1, 0xffff ); /* �����յ����ݴ�ӡ�����Դ���1 */
#endif
}

/*
*********************************************************************************************************
*   �� �� ��: MAIXCAM_SendData
*   ����˵��: �������ݰ�
*   ��    ��: _databuf ����
*            _len ���ݳ���
*   �� �� ֵ: ��
*********************************************************************************************************
*/
static void MAIXCAM_SendData( uint8_t* _databuf, uint16_t _len )
{
    if( _len > 2048 )
    {
        _len = 2048;
    }

    comSendBuf( COM_MAIXCAM, _databuf, _len );
}

/*
*********************************************************************************************************
*   �� �� ��: MAIXCAM_SendCmd
*   ����˵��: �����ض�����ɫ��Ԫ�أ���MaixCAM����ָ��ú�����app_maixcam�б�����
*   ��    ��: _ucElement : Ԫ��
*             _ucColor : ��ɫ
*   �� �� ֵ: ��
*********************************************************************************************************
*/
void MAIXCAM_SendCmd( MAIXCAM_ELEMENT _ucElement, MAIXCAM_COLOR _ucColor )
{
    static char maixcam_tx_buf[256] = {0};       //������ͻ���

    switch( _ucElement ) //����_elementѡ��ʶ��ʲô����
    {
    case BLOB: //Բ��
        _ucElement = 1;
        break;

    case CIRCLE:        //���
        _ucElement = 2;
        break;

    case ELEMENT_ALL:       //��Բ
        _ucElement = 3;
        break;

    default:
        break;
    }
    switch( _ucColor )  //����_colorѡ��ʶ��ʲô��ɫ
    {
    case RED:
        _ucColor = 'R';
        break;

    case GREEN:
        _ucColor = 'G';
        break;

    case BLUE:
        _ucColor = 'B';
        break;

    case COLOR_ALL:     //��Բ
        _ucColor = 'A';
        break;

    default:
        break;
    }

    sprintf( maixcam_tx_buf, "mode=%d%c", _ucElement, _ucColor );
    MAIXCAM_SendData(( uint8_t* )maixcam_tx_buf, strlen( maixcam_tx_buf ) );
}

/*
*********************************************************************************************************
*   �� �� ��: MAIXCAM_ReadData
*   ����˵��: ��ȡ�������ݣ�֡�����ɹ�����ûص������������ݽ������ú�����app_maixcam�б�����
*   ��    ��:  ��
*   �� �� ֵ: ��
*********************************************************************************************************
*/
uint8_t MAIXCAM_ReadData( void )
{
    static UART_PARSE_STATE_E s_ucState = STATE_IDLE;
    uint8_t ucData = 0;
    uint8_t ret = 0;

    /* �� ���ڶ�ȡ1���ֽ� comGetChar() ������ bsp_uart_fifo.c ʵ�� */
    while( 1 )
    {
        if( comGetChar( COM_MAIXCAM, &ucData ) )
        {
            MAIXCAM_PrintRxData( ucData );

            switch( s_ucState )
            {
            case STATE_IDLE:
                if( ucData == s_tMaixCamFrame.ucFrameHeader[0] ) //�յ�֡ͷ
                {
                    s_ucState = STATE_HEADER_0;
                }
                else
                {
//                         LED_R_ON();
                }
                break;

            case STATE_HEADER_0:
                s_tMaixCamFrame.ucFrameLength = ucData;         //�յ�֡���������򳤶ȣ���������������ݣ�
                s_ucState = STATE_LENGTH;
                break;

            case STATE_LENGTH:
                s_tMaixCamFrame.ucFrameIndex = 0;               //����������׼�����������򣨰�������������ݣ�
                s_tMaixCamFrame.ucCheckSum = s_tMaixCamFrame.ucFrameHeader[0] + s_tMaixCamFrame.ucFrameLength + ucData;
                s_tMaixCamFrame.pucFrameData[s_tMaixCamFrame.ucFrameIndex++] = ucData; //�յ������루����У�
                s_ucState = STATE_DATA;
                break;

            case STATE_DATA:
                s_tMaixCamFrame.pucFrameData[s_tMaixCamFrame.ucFrameIndex++] = ucData;
                s_tMaixCamFrame.ucCheckSum += ucData;                                  //�յ�����
                if( s_tMaixCamFrame.ucFrameIndex >= s_tMaixCamFrame.ucFrameLength )    //����֡���ض�
                {
//                    s_ucState = STATE_CHECKSUM;          //����CheckSum�ˣ��������
                    s_ucState = STATE_TAIL_0;
                }
                break;

            case STATE_CHECKSUM:
                if( ucData == ( s_tMaixCamFrame.ucCheckSum & 0xFF ) )     //У���
                {
                    s_ucState = STATE_TAIL_0;
                }
                else
                {
//                        LED_R_ON();
                    s_ucState = STATE_IDLE; // У��ʧ��
                }
                break;

            case STATE_TAIL_0:
                if( ucData == s_tMaixCamFrame.ucFrameTail[0] )           //�յ�֡β
                {
                    // ֡�����ɹ�����������
                    if( s_MAIXCAM_HandleFrame != NULL )
                    {
                        ret = 1;
                        s_MAIXCAM_HandleFrame( s_tMaixCamFrame.pucFrameData, s_tMaixCamFrame.ucFrameLength );
                    }
                }
                s_ucState = STATE_IDLE;
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

/*
*********************************************************************************************************
*   �� �� ��: num2color
*   ����˵��: ���ַ�ת��Ϊ��ɫö������
*   ��    ��: _char : �ַ�
*             _ucColor : ��ɫ
*   �� �� ֵ: ��
*********************************************************************************************************
*/
MAIXCAM_COLOR num2color( uint8_t _num )
{
    switch( _num )
    {
    case 1:
        return BLUE;
    case 2:
        return GREEN;
    case 3:
        return RED;
    default:
        return COLOR_NONE;
    }
}
