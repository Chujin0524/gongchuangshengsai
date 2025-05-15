#include "main.h"

static void APP_HWT101_Parse( uint8_t* _pucFrameData, uint8_t _ucFrameLength );

__IO static int16_t sGyro_z = 0;
static float fGyro_z = 0;
__IO float g_fGyro_z = 0;

/*
*********************************************************************************************************
*    函 数 名: APP_HWT101_Init _
*    功能说明: 初始化 HWT101 模块，并注册解析回调函数。
*    形    参: 无
*    返 回 值: 无
*********************************************************************************************************
*/
void APP_HWT101_Init( void )
{
    bsp_InitHWT101( APP_HWT101_Parse );    //注册解析回调函数
}

float *APP_HWT101_GetAnglePoint(void)
{
    return &fGyro_z;
}

/*
*********************************************************************************************************
*    函 数 名: APP_HWT101_Read
*    功能说明: 读取HWT101数据。
*    形    参: _ucElement    要检测的元素类型（例如 Blob 或 Circle）
*              _ucColor      要检测的颜色
*              _ucIsCalFlag  是否处于校准状态
*    返 回 值: 无
*********************************************************************************************************
*/
uint8_t APP_HWT101_Read( void )
{
    uint8_t ret = 0;
    if (HWT101_ReadData())   //成功接收一帧数据流，并调用解析回调函数APP_HWT101_Parse
    {
        ret = 1;
//        App_Printf("sGyro_z : %3d %3d     %3d %.3f\r\n", sizeof(sGyro_z), sGyro_z, sizeof(fGyro_z), fGyro_z);
    }
    return ret;
}

/*
*********************************************************************************************************
*    函 数 名: APP_HWT101_Parse
*    功能说明: 解析 HWT101 接收到的数据，并根据功能码_pucFrameData[0]和ucCalFlag来决定调用哪个解析函数。
*    形    参: pucFrameData    接收到的数据帧。
*              _ucFrameLength  数据帧的长度
*    返 回 值: 无
*********************************************************************************************************
*/
static void APP_HWT101_Parse( uint8_t* _pucFrameData, uint8_t _ucFrameLength )
{
    UNUSED( _ucFrameLength );    

    sGyro_z = ((int16_t)((uint16_t)_pucFrameData[5]<<8) | (uint16_t)_pucFrameData[4]);
    fGyro_z = sGyro_z / 32768.0 * 180.0;
//    g_fGyro_z = fGyro_z;
}
