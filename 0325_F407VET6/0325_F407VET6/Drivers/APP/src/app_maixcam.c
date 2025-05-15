#include "main.h"

#define MAIXCAM_CMD_BLOB        (0x01 << 0)

#define MAIXCAM_CMD_CIRCLE      (0x01 << 4)

static uint16_t s_usPresentX=0, s_usPresentY=0, s_usPreviousX=0, s_usPreviousY=0;
static uint32_t *MOTOR_X_Pos = NULL;
static uint32_t *MOTOR_R_Pos = NULL;
static uint8_t (*DirMapping)[4];  // ָ����� 4 �� uint8_t Ԫ�ص�һά�����ָ��

static void APP_MAIXCAM_Parse( uint8_t* pucFrameData, uint8_t ucFrameLength );
static void APP_MAIXCAM_Parse_BLOB_Cal(void);
static void APP_MAIXCAM_Parse_BLOB_Grab(void);
static void APP_MAIXCAM_Parse_CIRCLE_CalCar(void);
static void APP_MAIXCAM_Parse_CIRCLE_CalXR(void);
static void SendToAPP_MotorWhale(void);
float calculate_hypotenuse(float a, float b);
int32_t int32_constrain(int32_t Value, int32_t minValue, int32_t maxValue);

__IO int16_t g_dX = 0;
__IO int16_t g_dY = 0;
__IO int16_t g_dR = 0;

static MAIXCAM_DATA_T s_tMaixCAM_Data =
{
    .ucMoveFlag = 0,             // 0-�ƶ���; 1-�ƶ�X�����R���򲽽����
    .ucSendFlag = 0,             // ��������Ϊ��Ч

    .ucCalFlag = 0,              //�Ƿ�У׼��־λ
    .ucStableCount = 0,          //���ı�־λ����

    .usBlobCenter_X = 160,       //ת��������X
    .usBlobCenter_Y = 120,       //ת��������Y
//    .ucBlobCenterErrCal = 10,    //ת����У׼���������ΧС���Ա㾫׼У׼λ�ã�
    .ucBlobCenterErrCal = 10,    //ת����У׼���������ΧС���Ա㾫׼У׼λ�ã�
    .ucBlobCenterErrGrab = 30,   //ת����ץȡ���������Χ���Ա㼰ʱ��Ӧץȡ��
//    .ucBlobStableThreshold = 3,  //ת�����ȶ�������ֵ�����ڸò�����ʹС������������ƶ�
    .ucBlobStableThreshold = 1,  //ת�����ȶ�������ֵ�����ڸò�����ʹС������������ƶ�

    .usRedCircleCenter_X = 139,     //��ɫ�������ĵ�X
    .usRedCircleCenter_Y = 108,     //��ɫ�������ĵ�Y
    .usGreenCircleCenter_X = 140,     //��ɫ�������ĵ�X
    .usGreenCircleCenter_Y = 106,     //��ɫ�������ĵ�Y
    .usBlueCircleCenter_X = 140,     //��ɫ�������ĵ�X
    .usBlueCircleCenter_Y = 106,     //��ɫ�������ĵ�Y
    .ucCircleCenterErrCal = 1,   //ɫ����У׼�������
    .ucCircleStableThreshold = 1, //ɫ����У׼��׼���������ڸò�����ʹС����ȷ��׼����

    .dCoord_X = 0,               //X�����
    .dCoord_Y = 0,               //Y�����
    .dCoord_R = 0,               //R�ǶȲ�
};

static MAIXCAM_ID_T s_MaixCAM_ID =
{
    .ucElement = ELEMENT_NONE,
    .ucColor = WHITE,
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

    MOTOR_R_Pos = APP_Handle_GetPosPoint(MOTOR_R);
    MOTOR_X_Pos = APP_Handle_GetPosPoint(MOTOR_X);
    DirMapping = APP_Handle_GetMapPoint();

}

MAIXCAM_DATA_T *APP_MAIXCAM_GetPoint(void)
{
    return &s_tMaixCAM_Data;
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
            APP_MAIXCAM_SendAndRead( s_MaixCAM_ID.ucElement, s_MaixCAM_ID.ucColor );
        }
        else
        {
//            LED_R_ON(); //Ӧ�ò��������ж�
        }
        vTaskDelay( pdMS_TO_TICKS(100) );
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
void APP_MAIXCAM_SendAndRead( uint8_t _ucElement, uint8_t _ucColor )
{
    if( s_tMaixCAM_Data.ucSendFlag == 0 )       //�����δ����ָ������һ�η��͡���APP_MOTOR_2006_WHALE_MovePID֮������ñ�־λ
    {
        MAIXCAM_SendCmd( _ucElement, _ucColor );

        s_tMaixCAM_Data.ucSendFlag = 1;         //���ͱ�־λ��1
    }
    if (MAIXCAM_ReadData())   //�ɹ�����һ֡�������������ý����ص�����APP_MAIXCAM_Parse
    {
        s_tMaixCAM_Data.ucSendFlag = 0;
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

    if (s_tMaixCAM_Data.ucMoveFlag == MoveCar)      //�ƶ���
    {
        if (_pucFrameData[0]==MAIXCAM_CMD_BLOB && s_MaixCAM_ID.ucElement==BLOB)
        {
            if (s_tMaixCAM_Data.ucCalFlag == CAL)
            {
                APP_MAIXCAM_Parse_BLOB_Cal();
                App_Printf( "APP_MAIXCAM_Parse_BLOB_Cal\r\n" );
            }
            else if (s_tMaixCAM_Data.ucCalFlag == GRAB)
            {
                APP_MAIXCAM_Parse_BLOB_Grab();
                App_Printf( "APP_MAIXCAM_Parse_BLOB_Grab\r\n" );
            }
        }
        else if (_pucFrameData[0]==MAIXCAM_CMD_CIRCLE && s_MaixCAM_ID.ucElement==CIRCLE)
        {
            APP_MAIXCAM_Parse_CIRCLE_CalCar();
            App_Printf( "APP_MAIXCAM_Parse_CIRCLE_CalCar\r\n" );
        }
    }
    else if (s_tMaixCAM_Data.ucMoveFlag == MoveXR)     //�ƶ�X R���
    {
        APP_MAIXCAM_Parse_CIRCLE_CalXR();
        s_tMaixCAM_Data.ucSendFlag = 0;
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
    if (!(s_usPresentX && s_usPresentY))   //����յ���X Y������һ��Ϊ0����˵�����û��ʶ�𵽣�������ͷ���·���
    {
        s_tMaixCAM_Data.ucSendFlag = 0;
        return;
    }

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

        MAIXCAM_Send0();
        s_tMaixCAM_Data.ucStableCount = 0;
        s_usPresentX = 0;
        s_usPresentY = 0;
        s_usPreviousX = 0;
        s_usPreviousY = 0;
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

    g_dX = s_tMaixCAM_Data.dCoord_X;
    g_dY = s_tMaixCAM_Data.dCoord_Y;
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
    if (!(s_usPresentX && s_usPresentY))   //����յ���X Y������һ��Ϊ0����˵�����û��ʶ�𵽣�������ͷ���·���
    {
        s_tMaixCAM_Data.ucSendFlag = 0;
        return;
    }

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

        MAIXCAM_Send0();
    }

    g_dX = s_tMaixCAM_Data.dCoord_X;
    g_dY = s_tMaixCAM_Data.dCoord_Y;
    App_Printf("dX=%d dY=%d\r\n", s_tMaixCAM_Data.dCoord_X, s_tMaixCAM_Data.dCoord_Y);
}

/*
*********************************************************************************************************
*    �� �� ��: APP_MAIXCAM_Parse_CIRCLE_CalCar
*    ����˵��: ɫ��У׼С��
*    ��    ��: ��
*    �� �� ֵ: ��
*********************************************************************************************************
*/
static void APP_MAIXCAM_Parse_CIRCLE_CalCar(void)
{
    uint8_t isInCenter = 0;

    s_usPresentX = s_tMaixCAM_Data.usCoord_X;
    s_usPresentY = s_tMaixCAM_Data.usCoord_Y;
    if (!(s_usPresentX && s_usPresentY))   //����յ���X Y������һ��Ϊ0����˵�����û��ʶ�𵽣�������ͷ���·���
    {
        s_tMaixCAM_Data.ucSendFlag = 0;
        return;
    }

    s_tMaixCAM_Data.dCoord_X = s_tMaixCAM_Data.usCoord_X - s_tMaixCAM_Data.usGreenCircleCenter_X;
    s_tMaixCAM_Data.dCoord_Y = s_tMaixCAM_Data.usCoord_Y - s_tMaixCAM_Data.usGreenCircleCenter_Y;

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

//            MAIXCAM_Send0();
            s_tMaixCAM_Data.ucStableCount = 0;   //���ü���
            s_tMaixCAM_Data.dCoord_R = 0;
            s_tMaixCAM_Data.dCoord_X = 0;
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

    g_dX = s_tMaixCAM_Data.dCoord_X;
    g_dY = s_tMaixCAM_Data.dCoord_Y;
    App_Printf("dX=%d dY=%d\r\n", s_tMaixCAM_Data.dCoord_X, s_tMaixCAM_Data.dCoord_Y);
}

/*
*********************************************************************************************************
*    �� �� ��: APP_MAIXCAM_Parse_CIRCLE_CalXR
*    ����˵��: ɫ��У׼X��R���򲽽����
*    ��    ��: ��
*    �� �� ֵ: ��
*********************************************************************************************************
*/
static void APP_MAIXCAM_Parse_CIRCLE_CalXR(void)
{
    uint8_t isInCenter = 0;

    float dR=0, dX=0;
    if ((s_tMaixCAM_Data.usCoord_X == 0) || (s_tMaixCAM_Data.usCoord_Y == 0))
    {
        return;
    }

    uint16_t usCircleCenter_X;
    uint16_t usCircleCenter_Y;

    switch(s_MaixCAM_ID.ucColor)
    {
    case RED:
        usCircleCenter_X = s_tMaixCAM_Data.usRedCircleCenter_X;       //ɫ�������ĵ�X
        usCircleCenter_Y = s_tMaixCAM_Data.usRedCircleCenter_Y;       //ɫ�������ĵ�Y
        break;
    case GREEN:
        usCircleCenter_X = s_tMaixCAM_Data.usGreenCircleCenter_X;       //ɫ�������ĵ�X
        usCircleCenter_Y = s_tMaixCAM_Data.usGreenCircleCenter_Y;       //ɫ�������ĵ�Y
        break;
    case BLUE:
        usCircleCenter_X = s_tMaixCAM_Data.usBlueCircleCenter_X;       //ɫ�������ĵ�X
        usCircleCenter_Y = s_tMaixCAM_Data.usBlueCircleCenter_Y;       //ɫ�������ĵ�Y
        break;
    default:
        break;
    }

    dR = atan2f(240-s_tMaixCAM_Data.usCoord_Y, s_tMaixCAM_Data.usCoord_X-usCircleCenter_X) * 180.0f / PI - 90;
    dX = calculate_hypotenuse(240-s_tMaixCAM_Data.usCoord_Y, s_tMaixCAM_Data.usCoord_X-usCircleCenter_X) - (240-usCircleCenter_Y);

    s_tMaixCAM_Data.dCoord_R = dR;
    s_tMaixCAM_Data.dCoord_X = dX;

    //�����Ƿ�λ������
//    if( abs( s_tMaixCAM_Data.dCoord_X ) <= s_tMaixCAM_Data.ucCircleCenterErrCal &&
//        abs( s_tMaixCAM_Data.dCoord_Y ) <= s_tMaixCAM_Data.ucCircleCenterErrCal )
    if( abs( s_tMaixCAM_Data.dCoord_R ) <= 1 &&
        abs( s_tMaixCAM_Data.dCoord_X ) <= 1 )
    {
        // ��������n��λ������
        if (s_tMaixCAM_Data.ucStableCount < s_tMaixCAM_Data.ucCircleStableThreshold)
        {
            s_tMaixCAM_Data.ucStableCount++;
        }
        else   // �����ﵽ��ֵ����Ϊ����������
        {
            // ֪ͨAPP_Main���ر�����ͷ
            App_Printf("******** %d ********* CalOk\r\n", s_MaixCAM_ID.ucColor);
            xTaskNotifyGive(Task_MainHandle);
            xEventGroupClearBits( EventGroups_CarHandle, EventGroupsCarMaixCAM_EN_1 );

            s_tMaixCAM_Data.ucStableCount = 0;   //���ü���

            MAIXCAM_Send0();
        }
    }
    else
    {
        // ������������������ü���
        s_tMaixCAM_Data.ucStableCount = 0;

        s_tMaixCAM_Data.dCoord_R = dR * 10;
        s_tMaixCAM_Data.dCoord_X = dX * 10;
        // �ı�X R����ľ��Ծ���
        MOTOR_X_Pos[DirMapping[Fir][s_MaixCAM_ID.ucColor]] += s_tMaixCAM_Data.dCoord_X;
        MOTOR_R_Pos[DirMapping[Fir][s_MaixCAM_ID.ucColor]] += s_tMaixCAM_Data.dCoord_R;
        MOTOR_X_Pos[DirMapping[Fir][s_MaixCAM_ID.ucColor]] = int32_constrain(MOTOR_X_Pos[DirMapping[Fir][s_MaixCAM_ID.ucColor]], MOTOR_X_Pos_Min, MOTOR_X_Pos_Max);    //�޷�
        MOTOR_R_Pos[DirMapping[Fir][s_MaixCAM_ID.ucColor]] = int32_constrain(MOTOR_R_Pos[DirMapping[Fir][s_MaixCAM_ID.ucColor]], MOTOR_R_Pos_Min, MOTOR_R_Pos_Max);    //�޷�

        APP_MOTOR_ZDT_Move_P(MOTOR_X, MOTOR_X_Direction_FORWARD,  20, 100, MOTOR_X_Pos[DirMapping[Fir][s_MaixCAM_ID.ucColor]], 1, 0);
        APP_MOTOR_ZDT_Move_P(MOTOR_R, MOTOR_R_Direction_CCR,      20, 100, MOTOR_R_Pos[DirMapping[Fir][s_MaixCAM_ID.ucColor]], 1, 0);
    }

    g_dX = s_tMaixCAM_Data.dCoord_X;
    g_dR = s_tMaixCAM_Data.dCoord_R;
//    App_Printf("dX=%.5f               dR=%.5f\r\n", dX, dR);
//    App_Printf("dCoord_X=%5d          dCoord_R=%5d\r\n", s_tMaixCAM_Data.dCoord_X, s_tMaixCAM_Data.dCoord_R);
    App_Printf("MOTOR_X_Pos[%d]=%5d   MOTOR_R_Pos[%d]=%5d\r\n", s_MaixCAM_ID.ucColor, MOTOR_X_Pos[DirMapping[Fir][s_MaixCAM_ID.ucColor]], s_MaixCAM_ID.ucColor, MOTOR_R_Pos[DirMapping[Fir][s_MaixCAM_ID.ucColor]]);
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
    float k = 1;
    APP_CHASSIS_MovePID( CHASSIS_CALL_BLOCKING, CARDIRECTION_X(s_tMaixCAM_Data.dCoord_X), abs( s_tMaixCAM_Data.dCoord_X ) * k );
    APP_CHASSIS_MovePID( CHASSIS_CALL_BLOCKING, CARDIRECTION_Y(s_tMaixCAM_Data.dCoord_Y), abs( s_tMaixCAM_Data.dCoord_Y ) * k );
}

void APP_MAIXCAM_Cal( uint8_t _ucElement, uint8_t _ucColor, uint8_t _ucIsCalFlag, uint8_t _ucMoveFlag )
{
    s_MaixCAM_ID.ucElement = _ucElement;
    s_MaixCAM_ID.ucColor = _ucColor;
    s_tMaixCAM_Data.ucCalFlag = _ucIsCalFlag;
    s_tMaixCAM_Data.ucMoveFlag = _ucMoveFlag;

//    if (s_tMaixCAM_Data.ucMoveFlag == MoveCar)
//    {
//        APP_MOTOR_ZDT_Move_P(MOTOR_R, MOTOR_R_Direction_CCR,     100, 250, MOTOR_R_Pos[MIDDLE], 1, 0);
//        APP_MOTOR_ZDT_Move_P(MOTOR_X, MOTOR_X_Direction_FORWARD, 100, 250, MOTOR_X_Pos[MIDDLE], 1, 0);
//        vTaskDelay(500);
//    }
    if (s_tMaixCAM_Data.ucMoveFlag == MoveXR)
    {
        APP_MOTOR_ZDT_Move_P(MOTOR_R, MOTOR_R_Direction_CCR,     100, 250, MOTOR_R_Pos[DirMapping[Fir][s_MaixCAM_ID.ucColor]], 1, 0);
        APP_MOTOR_ZDT_Move_P(MOTOR_X, MOTOR_X_Direction_FORWARD, 100, 250, MOTOR_X_Pos[DirMapping[Fir][s_MaixCAM_ID.ucColor]], 1, 0);
        vTaskDelay(500);
    }
    xEventGroupSetBits( EventGroups_CarHandle, EventGroupsCarMaixCAM_EN_1 );    //��������ͷ
}

float calculate_hypotenuse(float a, float b)
{
    float squared_sum = a * a + b * b;
    float result;
    arm_sqrt_f32(squared_sum, &result);  // ʹ�� CMSIS DSP ���ƽ��������
    return result;
}

//�޷�����
int32_t int32_constrain(int32_t Value, int32_t minValue, int32_t maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}
