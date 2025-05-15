#include "main.h"

static void APP_HWT101_Parse( uint8_t* _pucFrameData, uint8_t _ucFrameLength );

__IO static int16_t sGyro_z = 0;
__IO static float fGyro_z = 0;
__IO float g_fGyro_z = 0;

/*
*********************************************************************************************************
*    �� �� ��: APP_HWT101_Init _
*    ����˵��: ��ʼ�� HWT101 ģ�飬��ע������ص�������
*    ��    ��: ��
*    �� �� ֵ: ��
*********************************************************************************************************
*/
void APP_HWT101_Init( void )
{
    bsp_InitHWT101( APP_HWT101_Parse );    //ע������ص�����
}

/*
*********************************************************************************************************
*    �� �� ��: APP_HWT101_Read
*    ����˵��: ��ȡHWT101���ݡ�
*    ��    ��: _ucElement    Ҫ����Ԫ�����ͣ����� Blob �� Circle��
*              _ucColor      Ҫ������ɫ
*              _ucIsCalFlag  �Ƿ���У׼״̬
*    �� �� ֵ: ��
*********************************************************************************************************
*/
uint8_t APP_HWT101_Read( float *_fGyro_z )
{
    uint8_t ret = 0;
    if (HWT101_ReadData())   //�ɹ�����һ֡�������������ý����ص�����APP_HWT101_Parse
    {
        ret = 1;
        *_fGyro_z = fGyro_z;
//        App_Printf("sGyro_z : %3d %3d     %3d %.3f\r\n", sizeof(sGyro_z), sGyro_z, sizeof(fGyro_z), fGyro_z);
    }
    return ret;
}

/*
*********************************************************************************************************
*    �� �� ��: APP_HWT101_Parse
*    ����˵��: ���� HWT101 ���յ������ݣ������ݹ�����_pucFrameData[0]��ucCalFlag�����������ĸ�����������
*    ��    ��: pucFrameData    ���յ�������֡��
*              _ucFrameLength  ����֡�ĳ���
*    �� �� ֵ: ��
*********************************************************************************************************
*/
static void APP_HWT101_Parse( uint8_t* _pucFrameData, uint8_t _ucFrameLength )
{
//    UNUSED( _ucFrameLength );    
    sGyro_z = ((int16_t)((uint16_t)_pucFrameData[5]<<8) | (uint16_t)_pucFrameData[4]);
    fGyro_z = sGyro_z / 32768.0 * 180.0;
    g_fGyro_z = fGyro_z;
//    App_Printf("sGyro_z : %3d %3d     %3d %.3f\r\n", sizeof(sGyro_z), sGyro_z, sizeof(fGyro_z), fGyro_z);
//    if( IS_NEGATIVE_HEX( _pucFrameData[5] ) )
//    {
//        sGyro_z = ((int16_t)((uint16_t)_pucFrameData[5]<<8) | (uint16_t)_pucFrameData[4]) / 32768.0 * 180.0 - 360.0;
//    }
//    else
//    {
//        sGyro_z = ((int16_t)((uint16_t)_pucFrameData[5]<<8) | (uint16_t)_pucFrameData[4]) / 32768.0 * 180.0;
//    }
}


