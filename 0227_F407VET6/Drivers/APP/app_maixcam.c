#include "main.h"

#define MAIXCAM_CMD_BLOB        (0x01 << 0)

#define MAIXCAM_CMD_CIRCLE      (0x01 << 4)

static uint16_t s_usPresentX=0, s_usPresentY=0, s_usPreviousX=0, s_usPreviousY=0;

static void APP_MAIXCAM_Parse( uint8_t* pucFrameData, uint8_t ucFrameLength );
static uint8_t APP_MAIXCAM_IsInCenter( uint8_t _ucCalFlag, uint8_t _ucCmd );
static void APP_MAIXCAM_Parse_BLOB_Cal(void);
static void APP_MAIXCAM_Parse_BLOB_Grab(void);
static void APP_MAIXCAM_Parse_CIRCLE_CAL(void);
static void SendToAPP_MotorWhale(void);

static MAIXCAM_DATA_T s_tMaixCAM_Data =
{
    .ucSendFlag = 0,             // ��������Ϊ��Ч

    .ucCalFlag = 0,              //�Ƿ�У׼��־λ
    .ucStableCount = 0,          //���ı�־λ����

    .usBlobCenter_X = 175,       //ת��������X
    .usBlobCenter_Y = 110,       //ת��������Y
    .ucBlobCenterErrCal = 10,    //ת����У׼���������ΧС���Ա㾫׼У׼λ�ã�
    .ucBlobCenterErrGrab = 30,   //ת����ץȡ���������Χ���Ա㼰ʱ��Ӧץȡ��
    .ucBlobStableThreshold = 3,  //ת�����ȶ�������ֵ�����ڸò�����ʹС������������ƶ�

    .usCircleCenter_X = 138,     //ɫ��������X
    .usCircleCenter_Y = 108,     //ɫ��������Y
    .ucCircleCenterErrCal = 1,   //ɫ����У׼�������
    .ucCircleStableThreshold = 3, //ɫ����У׼��׼���������ڸò�����ʹС����ȷ��׼����

    .dCoord_X = 0,               //X�����
    .dCoord_Y = 0,               //Y�����
};

static MAIXCAM_ID_T s_MaixCAM_ID =
{
    .ucElement = ELEMENT_NONE,
    .ucColor = COLOR_NONE,
    .ucIsCalFlag = 0
};

/*
*********************************************************************************************************
*    �� �� ��: APP_MAIXCAM_Init
*    ����˵��: ��ʼ�� MaixCAM ģ�飬��ע������ص�������
*    ��    ��: ��
*    �� �� ֵ: ��
*********************************************************************************************************
*/
void APP_MAIXCAM_Init( void )
{
    bsp_InitMAIXCAM( APP_MAIXCAM_Parse );    //ע������ص�����
}

/*
*********************************************************************************************************
*    �� �� ��: APP_MaixCAMControl
*    ����˵��: ������
*    ��    ��: ��
*    �� �� ֵ: ��
*********************************************************************************************************
*/
void APP_MaixCAMControl(void)
{
    EventBits_t uxBits;
    
//    xEventGroupSetBits(EventGroups_CarHandle, EventGroupsCarMaixCAM_EN_1);
    for(;; )
    {
        uxBits = xEventGroupWaitBits(
                     EventGroups_CarHandle,
                     EventGroupsCarMaixCAM_EN_1,
                     pdFALSE,        // �������־λ������������ͷ����֮�󣬾Ͳ�����Ϊ�ú����ر�
                     pdFALSE,        // pdTRUE������  pdFALSE������
                     portMAX_DELAY );

        if(( uxBits & ( EventGroupsCarMaixCAM_EN_1 ) ) == ( EventGroupsCarMaixCAM_EN_1 ) )
        {
            APP_MAIXCAM_SendAndRead( s_MaixCAM_ID.ucElement, s_MaixCAM_ID.ucColor, s_MaixCAM_ID.ucIsCalFlag );
        }
        else
        {
//            LED_R_ON(); //Ӧ�ò��������ж�
        }
    }    
}

/*
*********************************************************************************************************
*    �� �� ��: APP_MAIXCAM_SendAndRead
*    ����˵��: ���� MaixCAM �����ȡ���ݡ�
*    ��    ��: _ucElement    Ҫ����Ԫ�����ͣ����� Blob �� Circle��
*              _ucColor      Ҫ������ɫ
*              _ucIsCalFlag  �Ƿ���У׼״̬
*    �� �� ֵ: ��
*********************************************************************************************************
*/
void APP_MAIXCAM_SendAndRead( uint8_t _ucElement, uint8_t _ucColor, uint8_t _ucIsCalFlag )
{
    const TickType_t xFrequency = 100;
    
    if( s_tMaixCAM_Data.ucSendFlag == 0 )       //�����δ����ָ������һ�η���
    {
        MAIXCAM_SendCmd( _ucElement, _ucColor );
        s_tMaixCAM_Data.ucSendFlag = 1;         //���ͱ�־λ��1
        s_tMaixCAM_Data.ucCalFlag = _ucIsCalFlag;
    }
    if (MAIXCAM_ReadData())   //�ɹ�����һ֡�������������ý����ص�����APP_MAIXCAM_Parse
    {
        s_tMaixCAM_Data.ucSendFlag = 0;
//        App_Printf("RECEIVE\r\n");
         vTaskDelay( pdMS_TO_TICKS( xFrequency ) );
    }
}

/*
*********************************************************************************************************
*    �� �� ��: APP_MAIXCAM_Parse
*    ����˵��: ���� MaixCAM ���յ������ݣ������ݹ�����_pucFrameData[0]��ucCalFlag�����������ĸ�����������
*    ��    ��: pucFrameData    ���յ�������֡��
*              _ucFrameLength  ����֡�ĳ���
*    �� �� ֵ: ��
*********************************************************************************************************
*/
static void APP_MAIXCAM_Parse( uint8_t* _pucFrameData, uint8_t _ucFrameLength )
{
    UNUSED( _ucFrameLength );

    s_tMaixCAM_Data.usCoord_X = (( uint16_t )_pucFrameData[1] ) | (( uint16_t )_pucFrameData[2] << 8 );
    s_tMaixCAM_Data.usCoord_Y = (( uint16_t )_pucFrameData[3] ) | (( uint16_t )_pucFrameData[4] << 8 );

    if (_pucFrameData[0]==MAIXCAM_CMD_BLOB && s_MaixCAM_ID.ucElement==BLOB)
    {
        if (s_tMaixCAM_Data.ucCalFlag == 1)
        {
            APP_MAIXCAM_Parse_BLOB_Cal();
            App_Printf( "APP_MAIXCAM_Parse_BLOB_Cal\r\n" );
        }
        else if (s_tMaixCAM_Data.ucCalFlag == 0)
        {
            APP_MAIXCAM_Parse_BLOB_Grab();
            App_Printf( "APP_MAIXCAM_Parse_BLOB_Grab\r\n" );
        }
    }
    else if (_pucFrameData[0]==MAIXCAM_CMD_CIRCLE && s_MaixCAM_ID.ucElement==CIRCLE)
    {
        APP_MAIXCAM_Parse_CIRCLE_CAL();
        App_Printf( "APP_MAIXCAM_Parse_CIRCLE_CAL\r\n" );
    }
}

/*
*********************************************************************************************************
*    �� �� ��: APP_MAIXCAM_Parse_BLOB_Cal
*    ����˵��: ���У׼״̬
*    ��    ��: ��
*    �� �� ֵ: ��
*********************************************************************************************************
*/
static void APP_MAIXCAM_Parse_BLOB_Cal(void)
{
    uint8_t isInCenter = 0;

    s_usPresentX = s_tMaixCAM_Data.usCoord_X;
    s_usPresentY = s_tMaixCAM_Data.usCoord_Y;
    s_tMaixCAM_Data.dCoord_X = s_tMaixCAM_Data.usCoord_X - s_tMaixCAM_Data.usBlobCenter_X;
    s_tMaixCAM_Data.dCoord_Y = s_tMaixCAM_Data.usCoord_Y - s_tMaixCAM_Data.usBlobCenter_Y;

    // �ж������Ƿ�����n�β���
    if (abs(s_usPresentX - s_usPreviousX) < 5 && abs(s_usPresentY - s_usPreviousY) < 5)
    {
        // ��������n�β���
        if (s_tMaixCAM_Data.ucStableCount < s_tMaixCAM_Data.ucBlobStableThreshold)
        {
            s_tMaixCAM_Data.ucStableCount++;
        }
    }
    else
    {
        // �����б仯�����ü���
        s_tMaixCAM_Data.ucStableCount = 0;
    }

    if (s_tMaixCAM_Data.ucStableCount >= s_tMaixCAM_Data.ucBlobStableThreshold)
    {
        // �ж��Ƿ�λ��С��Χ����
        if( abs( s_tMaixCAM_Data.dCoord_X ) <= s_tMaixCAM_Data.ucBlobCenterErrCal &&
            abs( s_tMaixCAM_Data.dCoord_Y ) <= s_tMaixCAM_Data.ucBlobCenterErrCal )
        {
            // λ��С��Χ����
            isInCenter = 1;
        }
    }

    if (isInCenter)
    {
        // ֪ͨAPP_Main���ر�����ͷ
        xTaskNotifyGive(Task_MainHandle);
        xEventGroupClearBits( EventGroups_CarHandle, EventGroupsCarMaixCAM_EN_1 );     
        
        s_tMaixCAM_Data.ucStableCount = 0;
    }
    else if (s_tMaixCAM_Data.ucStableCount >= s_tMaixCAM_Data.ucBlobStableThreshold)
    {
        // �����ƶ���������APP_MotorWhale����Ϣ����
        s_tMaixCAM_Data.dCoord_X = (s_usPresentX + s_usPreviousX) / 2.0 - s_tMaixCAM_Data.usBlobCenter_X;
        s_tMaixCAM_Data.dCoord_Y = (s_usPresentY + s_usPreviousY) / 2.0 - s_tMaixCAM_Data.usBlobCenter_Y;
        SendToAPP_MotorWhale();
        
        s_tMaixCAM_Data.ucStableCount = 0;
    }

    s_usPreviousX = s_usPresentX;
    s_usPreviousY = s_usPresentY;
}

/*
*********************************************************************************************************
*    �� �� ��: APP_MAIXCAM_Parse_BLOB_Grab
*    ����˵��: ���ץȡ״̬
*    ��    ��: ��
*    �� �� ֵ: ��
*********************************************************************************************************
*/
static void APP_MAIXCAM_Parse_BLOB_Grab(void)
{
    uint8_t isInCenter = 0;

    s_usPresentX = s_tMaixCAM_Data.usCoord_X;
    s_usPresentY = s_tMaixCAM_Data.usCoord_Y;
    s_tMaixCAM_Data.dCoord_X = s_tMaixCAM_Data.usCoord_X - s_tMaixCAM_Data.usBlobCenter_X;
    s_tMaixCAM_Data.dCoord_Y = s_tMaixCAM_Data.usCoord_Y - s_tMaixCAM_Data.usBlobCenter_Y;

    // �ж��Ƿ�λ�ڴ�Χ����
    if( abs( s_tMaixCAM_Data.dCoord_X ) <= s_tMaixCAM_Data.ucBlobCenterErrGrab &&
        abs( s_tMaixCAM_Data.dCoord_Y ) <= s_tMaixCAM_Data.ucBlobCenterErrGrab )
    {
        // λ��С��Χ����
        isInCenter = 1;
    }

    if (isInCenter)
    {
        // ֪ͨAPP_Main���ر�����ͷ
        xTaskNotifyGive(Task_MainHandle);
        xEventGroupClearBits( EventGroups_CarHandle, EventGroupsCarMaixCAM_EN_1 );     
                
    }
}

/*
*********************************************************************************************************
*    �� �� ��: APP_MAIXCAM_Parse_CIRCLE_CAL
*    ����˵��: ɫ��У׼״̬
*    ��    ��: ��
*    �� �� ֵ: ��
*********************************************************************************************************
*/
static void APP_MAIXCAM_Parse_CIRCLE_CAL(void)
{
    uint8_t isInCenter = 0;

    s_usPresentX = s_tMaixCAM_Data.usCoord_X;
    s_usPresentY = s_tMaixCAM_Data.usCoord_Y;
    s_tMaixCAM_Data.dCoord_X = s_tMaixCAM_Data.usCoord_X - s_tMaixCAM_Data.usCircleCenter_X;
    s_tMaixCAM_Data.dCoord_Y = s_tMaixCAM_Data.usCoord_Y - s_tMaixCAM_Data.usCircleCenter_Y;

    //�����Ƿ�λ������
    if( abs( s_tMaixCAM_Data.dCoord_X ) <= s_tMaixCAM_Data.ucCircleCenterErrCal &&
        abs( s_tMaixCAM_Data.dCoord_Y ) <= s_tMaixCAM_Data.ucCircleCenterErrCal )
    {
        // ��������n��λ������
        if (s_tMaixCAM_Data.ucStableCount < s_tMaixCAM_Data.ucCircleStableThreshold)
        {
            s_tMaixCAM_Data.ucStableCount++;
        }
        else   // �����ﵽ��ֵ����Ϊ����������
        {
            // ֪ͨAPP_Main���ر�����ͷ
            xTaskNotifyGive(Task_MainHandle);
            xEventGroupClearBits( EventGroups_CarHandle, EventGroupsCarMaixCAM_EN_1 );     
            
            s_tMaixCAM_Data.ucStableCount = 0;   //���ü���
        }
    }
    else
    {
        // ������������������ü���
        s_tMaixCAM_Data.ucStableCount = 0;

        // �����ƶ���������APP_MotorWhale����Ϣ����
//        s_tMaixCAM_Data.dCoord_X = (s_usPresentX + s_usPreviousX) / 2.0 - s_tMaixCAM_Data.usCircleCenter_X;
//        s_tMaixCAM_Data.dCoord_Y = (s_usPresentY + s_usPreviousY) / 2.0 - s_tMaixCAM_Data.usCircleCenter_Y;
        SendToAPP_MotorWhale();
    }
}

/*
*********************************************************************************************************
*    �� �� ��: SendToAPP_MotorWhale
*    ����˵��: ������Ϣ�����ӿ�������
*    ��    ��: ��
*    �� �� ֵ: ��
*********************************************************************************************************
*/
static void SendToAPP_MotorWhale(void)
{
    static CAR_XYR_T s_tCarXYR = {0};
    static CAR_XYR_T *ptCarXYR = &s_tCarXYR;

    ptCarXYR->WhaleDirX = CARDIRECTION_X(s_tMaixCAM_Data.dCoord_X);
    ptCarXYR->WhalePosX = abs( s_tMaixCAM_Data.dCoord_X ) * 10;    

    ptCarXYR->WhaleDirY = CARDIRECTION_Y(s_tMaixCAM_Data.dCoord_Y);
    ptCarXYR->WhalePosY = abs( s_tMaixCAM_Data.dCoord_Y ) * 10;

    //����Ϣ���� Queue_HandleCalculateHandle �������Ӳ���
    xQueueOverwrite(Queue_HandleCalculateHandle, (void *)&ptCarXYR);    
}

void APP_MAIXCAM_Cal( uint8_t _ucElement, uint8_t _ucColor, uint8_t _ucIsCalFlag )
{
    s_MaixCAM_ID.ucElement = _ucElement;
    s_MaixCAM_ID.ucColor = _ucColor;
    s_MaixCAM_ID.ucIsCalFlag = _ucIsCalFlag;
    
    xEventGroupSetBits( EventGroups_CarHandle, EventGroupsCarMaixCAM_EN_1 );    //��������ͷ
}
