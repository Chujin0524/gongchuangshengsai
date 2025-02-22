#include "main.h"

/*
转盘位置 : 
0 : 初始
1 : 指向左边
2 : 指向中间
3 : 指向右边
4 : 伸/缩
*/
static uint8_t MOTOR_R_Acc = 10; 
static uint16_t MOTOR_R_Vel = 250; 
static uint32_t MOTOR_R_Pos[4] = {0, 4100, 2100, 90};  

/*
X位置 : 
0 : 初始
1 : 指向左边
2 : 指向左边
3 : 指向左边
4 : 
*/
static uint8_t MOTOR_X_Acc = 10; 
static uint16_t MOTOR_X_Vel = 250; 
static uint32_t MOTOR_X_Pos[4] = {2900, 6700, 4200, 7050};    

/*
Y位置 : 
0 : 初始
1 : 顶点
2 : 转盘上方
3 : 原料区
4 : 加工区第一次
5 : 加工区第二次
6 : 放完第三个物块之后夹爪上升的低位置
7 : 
*/
static uint8_t MOTOR_Y_Acc = 10; 
static uint16_t MOTOR_Y_Vel = 250; 
static uint32_t MOTOR_Y_Pos[8] = {0, 350, 1500, 7100, 12700, 8150, 3500, 1500}; 

void APP_HandleMove(void)
{
    static CAR_XYR_T *ptCarXYR = NULL;
    static int16_t k = 1;                     //像素点转换为移动距离的倍数    

    for(;; )
    {
        xQueueReceive(Queue_HandleCalculateHandle, &ptCarXYR, portMAX_DELAY);  //等待接收轮子参数
//        pWhaleParameter->WhalePosX *= k;     //X像素点转换为移动距离
//        pWhaleParameter->WhalePosY *= k;     //Y像素点转换为移动距离

        xQueueOverwrite(Queue_MotorHandle, (void *)&ptCarXYR);  //向消息队列Queue_MotorHandle发送轮子参数
    }    
}

void APP_Handle_PickRawArea(uint8_t _ucIndex)
{
     //35步进电机控制夹爪距离顶部0.5圈
    APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

    APP_SERVO_Wrist('F');        //腕处舵机朝前
    
    APP_SERVO_Plate(_ucIndex);       //转盘舵机转到对应物块的放置角度
 
    //35步进电机控制夹爪距离顶部4圈
    APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[3], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
        
    APP_SERVO_Jaw('Z');     //夹爪舵机夹紧物块
   
    //35步进电机控制夹爪距离顶部0.5圈
    APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
   
    APP_SERVO_Wrist('R');          //腕处舵机朝右
    
    //35步进电机控制夹爪距离顶部1圈
    APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[2], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
    
    APP_SERVO_Jaw('S');     //夹爪舵机松开物块
    
    //35步进电机控制夹爪距离顶部0.5圈
    APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

    APP_SERVO_Wrist('F');        //腕处舵机朝前
}

/*
向右旋转腕处舵机，的同时，伸出X轴导轨，根据ucaGrabSequence[0+3*ucCycle]，旋转转盘至指定物块颜色，
竖直导杆下降，夹爪舵机夹取物块，
升起竖直导杆，向左旋转腕处舵机，竖直导杆下降的同时，旋转整体转盘，
松开夹爪放置物块到色环，升起竖直导杆

升起竖直导杆的同时，旋转整体转盘或者移动X轴导轨，
向右旋转腕处舵机，的同时，移动X轴导轨或旋转整体转盘，根据ucaGrabSequence[1+3*ucCycle]，旋转转盘至指定物块颜色，
竖直导杆下降，夹爪舵机夹取物块，
升起竖直导杆，向左旋转腕处舵机，竖直导杆下降的同时，旋转整体转盘，
松开夹爪放置物块到色环，升起竖直导杆

升起竖直导杆的同时，旋转整体转盘或者移动X轴导轨，
向右旋转腕处舵机，的同时，移动X轴导轨或旋转整体转盘，根据ucaGrabSequence[2+3*ucCycle]，旋转转盘至指定物块颜色，
竖直导杆下降，夹爪舵机夹取物块，
升起竖直导杆，向左旋转腕处舵机，竖直导杆下降的同时，旋转整体转盘，
松开夹爪放置物块到色环，升起竖直导杆

第三个物块放置结束，导轨只升起一半高度，以便进行抓取向下移动更短距离
*/
void APP_Handle_Place_All(uint8_t _time, uint8_t _firstIndex, uint8_t _secondIndex, uint8_t _thirdIndex)
{
    uint32_t Delay_up = 400;
    printf("Seq %d %d %d \r\n", _firstIndex, _secondIndex, _thirdIndex);
    if (_firstIndex == 1 || _firstIndex == 3)
    {
        //第一个
        {
            APP_SERVO_Jaw('S');            //夹爪舵机松开物块
            APP_SERVO_Plate(_firstIndex);        //转盘舵机转到对应物块的放置角度
            
            //35步进电机控制夹爪距离顶部0.5圈    
            APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
            APP_SERVO_Wrist('R');           //腕处舵机朝右

            //X导轨伸出到指定色环
            APP_MOTOR_ZDT_Move_P(MOTOR_X, MOTOR_X_Direction_FORWARD, MOTOR_X_Vel, MOTOR_X_Acc, MOTOR_X_Pos[_firstIndex], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_X);
            
            //35步进电机控制夹爪距离顶部1圈
            APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[2], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

            //控制底盘转盘转动到指定色环方向
            APP_MOTOR_ZDT_Move_P(MOTOR_R, MOTOR_R_Direction_CR, MOTOR_R_Vel, MOTOR_R_Acc, MOTOR_R_Pos[_firstIndex], 1, 0); 

            APP_SERVO_Jaw('Z');         //夹爪舵机夹紧物块
         
            //35步进电机控制夹爪距离顶部0.5圈
            APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

            APP_SERVO_Wrist('F');           //腕处舵机朝前    
          
            //35步进电机控制夹爪距离顶部4圈
            APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[4], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
            APP_SERVO_Jaw('S');         //夹爪996舵机转30度，松开物块
                 
            //35步进电机控制夹爪距离顶部1圈    
            APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); 
            vTaskDelay(Delay_up);

            APP_MOTOR_ZDT_WaitAck(MOTOR_R);
            //控制底盘转盘转动到指定色环方向
            APP_MOTOR_ZDT_Move_P(MOTOR_R, MOTOR_R_Direction_CR, MOTOR_R_Vel, MOTOR_R_Acc, MOTOR_R_Pos[_secondIndex], 1, 0);
        }
        if (_secondIndex == 2) //第二个是中间的
        {
            //第二个
            {
                APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
                
                APP_SERVO_Plate(_secondIndex);        //转盘舵机转到对应物块的放置角度
                APP_SERVO_Wrist('R');           //腕处舵机朝右
                APP_MOTOR_ZDT_Move_P(MOTOR_X, MOTOR_X_Direction_FORWARD, MOTOR_X_Vel, MOTOR_X_Acc, MOTOR_X_Pos[_secondIndex], 1, 0);                   //X导轨伸出到指定色环
                
                
                //35步进电机控制夹爪距离顶部1圈
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[2], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

                APP_SERVO_Jaw('Z');            //夹爪舵机夹紧物块
             
                //35步进电机控制夹爪距离顶部0.5圈
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
                ////printf("3-5\r\n");

                APP_SERVO_Wrist('F');            //腕处舵机朝前    
                
              
                //35步进电机控制夹爪距离顶部4圈
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[4], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
                APP_SERVO_Jaw('S');            //夹爪996舵机转30度，松开物块
                     
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0);            //35步进电机控制夹爪距离顶部1圈    
                vTaskDelay(500);
                APP_MOTOR_ZDT_Move_P(MOTOR_X, MOTOR_X_Direction_FORWARD, MOTOR_X_Vel, MOTOR_X_Acc, MOTOR_X_Pos[_thirdIndex], 1, 0);                   //X导轨伸出到指定色环
            }
            //第三个
            {
                APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
                
                APP_SERVO_Plate(_thirdIndex);        //转盘舵机转到对应物块的放置角度

                APP_SERVO_Wrist('R');            //腕处舵机朝右            
                APP_MOTOR_ZDT_WaitAck(MOTOR_R);            
                APP_MOTOR_ZDT_Move_P(MOTOR_R, MOTOR_R_Direction_CR, MOTOR_R_Vel, MOTOR_R_Acc, MOTOR_R_Pos[_thirdIndex], 1, 0);     //控制底盘转盘转动到指定色环方向           
                
                
                //35步进电机控制夹爪距离顶部1圈
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[2], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
                ////printf("3-4\r\n");

                APP_SERVO_Jaw('Z');            //夹爪舵机夹紧物块
             
                //35步进电机控制夹爪距离顶部0.5圈
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
                ////printf("3-5\r\n");

                APP_SERVO_Wrist('F');            //腕处舵机朝前    
                
              
                //35步进电机控制夹爪距离顶部4圈
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[4], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
                APP_SERVO_Jaw('S');            //夹爪996舵机转30度，松开物块
                     
                //35步进电机控制夹爪距离顶部1圈    
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[6], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

                APP_MOTOR_ZDT_WaitAck(MOTOR_R);            
            }
        }
        else    //第二个是右边的
        {
            //第二个
            {
                APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
                
                APP_SERVO_Plate(_secondIndex);        //转盘舵机转到对应物块的放置角度

                APP_SERVO_Wrist('R');            //腕处舵机朝右
                APP_MOTOR_ZDT_Move_P(MOTOR_X, MOTOR_X_Direction_FORWARD, MOTOR_X_Vel, MOTOR_X_Acc, MOTOR_X_Pos[_secondIndex], 1, 0);                   //X导轨伸出到指定色环
                
                
                //35步进电机控制夹爪距离顶部1圈
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[2], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

                APP_SERVO_Jaw('Z');            //夹爪舵机夹紧物块
             
                //35步进电机控制夹爪距离顶部0.5圈
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
                ////printf("3-5\r\n");

                APP_SERVO_Wrist('F');            //腕处舵机朝前    
                
              
                //35步进电机控制夹爪距离顶部4圈
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[4], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
                APP_SERVO_Jaw('S');            //夹爪996舵机转30度，松开物块
                     
                //35步进电机控制夹爪距离顶部1圈
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0);                
                vTaskDelay(Delay_up);

                APP_MOTOR_ZDT_WaitAck(MOTOR_R);            
                APP_MOTOR_ZDT_Move_P(MOTOR_R, MOTOR_R_Direction_CR, MOTOR_R_Vel, MOTOR_R_Acc, MOTOR_R_Pos[_thirdIndex], 1, 0);     //控制底盘转盘转动到指定色环方向           
            }
            //第三个
            {
                APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
                
                APP_SERVO_Plate(_thirdIndex);        //转盘舵机转到对应物块的放置角度

                APP_SERVO_Wrist('R');            //腕处舵机朝右
                APP_MOTOR_ZDT_Move_P(MOTOR_X, MOTOR_X_Direction_FORWARD, MOTOR_X_Vel, MOTOR_X_Acc, MOTOR_X_Pos[_thirdIndex], 1, 0);                   //X导轨伸出到指定色环
                
                
                //35步进电机控制夹爪距离顶部1圈
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[2], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

                APP_SERVO_Jaw('Z');            //夹爪舵机夹紧物块
             
                //35步进电机控制夹爪距离顶部0.5圈
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

                APP_SERVO_Wrist('F');            //腕处舵机朝前    
                
              
                //35步进电机控制夹爪距离顶部4圈
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[4], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
                APP_SERVO_Jaw('S');            //夹爪996舵机转30度，松开物块
                     
                //35步进电机控制夹爪距离顶部1圈
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[6], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
                APP_MOTOR_ZDT_WaitAck(MOTOR_R);            
            }
        }
    }
    if (_firstIndex == 2)
    {
        //第一个
        {
            APP_SERVO_Jaw('S');            //夹爪舵机松开物块
            APP_SERVO_Plate(_firstIndex);        //转盘舵机转到对应物块的放置角度
            
            //35步进电机控制夹爪距离顶部0.5圈    
            APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
            APP_SERVO_Wrist('R');            //腕处舵机朝右
            
            
            //35步进电机控制夹爪距离顶部1圈
            APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[2], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

            APP_SERVO_Jaw('Z');            //夹爪舵机夹紧物块
         
            //35步进电机控制夹爪距离顶部0.5圈
            APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

            APP_SERVO_Wrist('F');            //腕处舵机朝前    
            
          
            //35步进电机控制夹爪距离顶部4圈
            APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[4], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
            APP_SERVO_Jaw('S');            //夹爪996舵机转30度，松开物块
                 
            //35步进电机控制夹爪距离顶部1圈    
            APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0);            
            vTaskDelay(Delay_up);
            APP_MOTOR_ZDT_Move_P(MOTOR_X, MOTOR_X_Direction_FORWARD, MOTOR_X_Vel, MOTOR_X_Acc, MOTOR_X_Pos[_secondIndex], 1, 0);                   //X导轨伸出到指定色环
        }
//        if (_secondIndex == 2) //第二个是中间的
        {
            //第二个
            {
                APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
                
                APP_SERVO_Plate(_secondIndex);        //转盘舵机转到对应物块的放置角度
                APP_SERVO_Wrist('R');            //腕处舵机朝右
                APP_MOTOR_ZDT_Move_P(MOTOR_R, MOTOR_R_Direction_CR, MOTOR_R_Vel, MOTOR_R_Acc, MOTOR_R_Pos[_secondIndex], 1, 0);     //控制底盘转盘转动到指定色环方向           
                
                
                //35步进电机控制夹爪距离顶部1圈
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[2], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

                APP_SERVO_Jaw('Z');            //夹爪舵机夹紧物块
             
                //35步进电机控制夹爪距离顶部0.5圈
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

                APP_SERVO_Wrist('F');            //腕处舵机朝前    
                
              
                //35步进电机控制夹爪距离顶部4圈
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[4], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
                APP_SERVO_Jaw('S');            //夹爪996舵机转30度，松开物块
                     
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0);            //35步进电机控制夹爪距离顶部1圈    
                vTaskDelay(Delay_up);

                APP_MOTOR_ZDT_WaitAck(MOTOR_R);
                APP_MOTOR_ZDT_Move_P(MOTOR_R, MOTOR_R_Direction_CR, MOTOR_R_Vel, MOTOR_R_Acc, MOTOR_R_Pos[_thirdIndex], 1, 0);     //控制底盘转盘转动到指定色环方向       
                vTaskDelay(500);
            }
            //第三个
            {
                APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
                
                APP_SERVO_Plate(_thirdIndex);        //转盘舵机转到对应物块的放置角度
                APP_SERVO_Wrist('R');            //腕处舵机朝右            
                APP_MOTOR_ZDT_Move_P(MOTOR_X, MOTOR_X_Direction_FORWARD, MOTOR_X_Vel, MOTOR_X_Acc, MOTOR_X_Pos[_thirdIndex], 1, 0);                   //X导轨伸出到指定色环
                
                
                //35步进电机控制夹爪距离顶部1圈
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[2], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

                APP_SERVO_Jaw('Z');            //夹爪舵机夹紧物块
             
                //35步进电机控制夹爪距离顶部0.5圈
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

                APP_SERVO_Wrist('F');            //腕处舵机朝前    
                
              
                //35步进电机控制夹爪距离顶部4圈
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[4], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
                APP_SERVO_Jaw('S');            //夹爪996舵机转30度，松开物块
                     
                //35步进电机控制夹爪距离顶部1圈
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[6], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

                APP_MOTOR_ZDT_WaitAck(MOTOR_R);            
            }
        }       
    }
}

void APP_Handle_Pick_All(uint8_t _firstIndex, uint8_t _secondIndex, uint8_t _thirdIndex)
{
    uint32_t Delay_up = 400;
    uint32_t Delay_turn = 500;
    printf("Seq %d %d %d \r\n", _firstIndex, _secondIndex, _thirdIndex);
    if (_firstIndex == 1 || _firstIndex == 3)
    {
        //第一个
        {
            if (_thirdIndex == 2)   //如果最后放置的是中间的
            {
                APP_MOTOR_ZDT_Move_P(MOTOR_X, MOTOR_X_Direction_FORWARD, MOTOR_X_Vel, MOTOR_X_Acc, MOTOR_X_Pos[_firstIndex], 1, 0);                   //X导轨伸出到指定色环
                vTaskDelay(Delay_turn);
                APP_MOTOR_ZDT_Move_P(MOTOR_R, MOTOR_R_Direction_CR, MOTOR_R_Vel, MOTOR_R_Acc, MOTOR_R_Pos[_firstIndex], 1, 0);     //控制底盘转盘转动到指定色环方向
                APP_MOTOR_ZDT_WaitAck(MOTOR_R);
            }
            else
            {
                APP_MOTOR_ZDT_Move_P(MOTOR_R, MOTOR_R_Direction_CR, MOTOR_R_Vel, MOTOR_R_Acc, MOTOR_R_Pos[_firstIndex], 1, 0);     //控制底盘转盘转动到指定色环方向
                APP_MOTOR_ZDT_WaitAck(MOTOR_R);
                APP_MOTOR_ZDT_Move_P(MOTOR_X, MOTOR_X_Direction_FORWARD, MOTOR_X_Vel, MOTOR_X_Acc, MOTOR_X_Pos[_firstIndex], 1, 0);                   //X导轨伸出到指定色环
            }
            
            //35步进电机控制夹爪距离顶部4圈
            APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[4], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
            APP_SERVO_Jaw('Z');            //夹爪996舵机转30度，松开物块
                 
            //35步进电机控制夹爪距离顶部1圈
            APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0);                
            vTaskDelay(Delay_up);

            APP_MOTOR_ZDT_Move_P(MOTOR_R, MOTOR_R_Direction_CR, MOTOR_R_Vel, MOTOR_R_Acc, MOTOR_R_Pos[_secondIndex], 1, 0);     //控制底盘转盘转动到指定色环方向           
        }
        if (_secondIndex == 2) //第二个是中间的
        {
            //第二个
            {
                APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
                
                APP_SERVO_Plate(_firstIndex);        //转盘舵机转到对应物块的放置角度
                APP_SERVO_Wrist('R');            //腕处舵机朝右
                APP_MOTOR_ZDT_Move_P(MOTOR_X, MOTOR_X_Direction_FORWARD, MOTOR_X_Vel, MOTOR_X_Acc, MOTOR_X_Pos[_secondIndex], 1, 0);                   //X导轨伸出到指定色环
                
                
                //35步进电机控制夹爪距离顶部1圈
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[2], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

                APP_SERVO_Jaw('S');            //夹爪舵机夹紧物块
             
                //35步进电机控制夹爪距离顶部0.5圈
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

                APP_SERVO_Wrist('F');            //腕处舵机朝前    
                
              
                //35步进电机控制夹爪距离顶部4圈
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[4], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
                APP_SERVO_Jaw('Z');            //夹爪996舵机转30度，松开物块
                     
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0);            //35步进电机控制夹爪距离顶部1圈    
                vTaskDelay(Delay_up);
                APP_MOTOR_ZDT_Move_P(MOTOR_X, MOTOR_X_Direction_FORWARD, MOTOR_X_Vel, MOTOR_X_Acc, MOTOR_X_Pos[_thirdIndex], 1, 0);                   //X导轨伸出到指定色环
            }
            //第三个
            {
                APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
                
                APP_SERVO_Plate(_secondIndex);        //转盘舵机转到对应物块的放置角度

                APP_SERVO_Wrist('R');            //腕处舵机朝右            
                APP_MOTOR_ZDT_WaitAck(MOTOR_R);            
                APP_MOTOR_ZDT_Move_P(MOTOR_R, MOTOR_R_Direction_CR, MOTOR_R_Vel, MOTOR_R_Acc, MOTOR_R_Pos[_thirdIndex], 1, 0);     //控制底盘转盘转动到指定色环方向           
                
                
                //35步进电机控制夹爪距离顶部1圈
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[2], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

                APP_SERVO_Jaw('S');            //夹爪舵机夹紧物块
             
                //35步进电机控制夹爪距离顶部0.5圈
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

                APP_SERVO_Wrist('F');            //腕处舵机朝前    
                
             
                //35步进电机控制夹爪距离顶部4圈
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[4], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

                APP_SERVO_Jaw('Z');            //夹爪舵机夹紧物块
             
                //35步进电机控制夹爪距离顶部0.5圈
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0);        
                vTaskDelay(Delay_up);

                APP_MOTOR_ZDT_WaitAck(MOTOR_R);            
                //控制底盘转盘转动到指定色环方向           
                APP_MOTOR_ZDT_Move_P(MOTOR_R, MOTOR_R_Direction_CR, MOTOR_R_Vel, MOTOR_R_Acc, MOTOR_R_Pos[_secondIndex], 1, 0);     //控制底盘转盘转动到指定色环方向           
                
                APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
                
                APP_SERVO_Plate(_thirdIndex);        //转盘舵机转到对应物块的放置角度
                APP_SERVO_Wrist('R');            //腕处舵机朝右    
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[2], 1, 0);
                
                
                //35步进电机控制夹爪距离顶部1圈
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[2], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

                APP_SERVO_Jaw('S');            //夹爪舵机松开物块
             
                //35步进电机控制夹爪距离顶部0.5圈
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

//                APP_SERVO_Wrist('F');            //腕处舵机朝前    
//                

                APP_MOTOR_ZDT_WaitAck(MOTOR_R);            
            }
        }           
        else    //第二个是右边的
        {
            //第二个
            {
                APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
                
                APP_SERVO_Plate(_firstIndex);        //转盘舵机转到对应物块的放置角度

                APP_SERVO_Wrist('R');            //腕处舵机朝右
                APP_MOTOR_ZDT_Move_P(MOTOR_X, MOTOR_X_Direction_FORWARD, MOTOR_X_Vel, MOTOR_X_Acc, MOTOR_X_Pos[_secondIndex], 1, 0);                   //X导轨伸出到指定色环
                
                
                //35步进电机控制夹爪距离顶部1圈
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[2], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

                APP_SERVO_Jaw('S');            //夹爪舵机夹紧物块
             
                //35步进电机控制夹爪距离顶部0.5圈
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

                APP_SERVO_Wrist('F');            //腕处舵机朝前    
                
              
                //35步进电机控制夹爪距离顶部4圈
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[4], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
                APP_SERVO_Jaw('Z');            //夹爪996舵机转30度，松开物块
                     
                //35步进电机控制夹爪距离顶部0.5圈
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

                APP_MOTOR_ZDT_WaitAck(MOTOR_R);            
                APP_MOTOR_ZDT_Move_P(MOTOR_R, MOTOR_R_Direction_CR, MOTOR_R_Vel, MOTOR_R_Acc, MOTOR_R_Pos[_thirdIndex], 1, 0);     //控制底盘转盘转动到指定色环方向           
            }
            //第三个
            {
                APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
                
                APP_SERVO_Plate(_secondIndex);        //转盘舵机转到对应物块的放置角度

                APP_SERVO_Wrist('R');            //腕处舵机朝右
                APP_MOTOR_ZDT_Move_P(MOTOR_X, MOTOR_X_Direction_FORWARD, MOTOR_X_Vel, MOTOR_X_Acc, MOTOR_X_Pos[_thirdIndex], 1, 0);                   //X导轨伸出到指定色环
                
                
                //35步进电机控制夹爪距离顶部1圈
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[2], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

                APP_SERVO_Jaw('S');            //夹爪舵机夹紧物块
             
                //35步进电机控制夹爪距离顶部0.5圈
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

                APP_SERVO_Wrist('F');            //腕处舵机朝前    
                
              
                //35步进电机控制夹爪距离顶部4圈
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[4], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
                APP_SERVO_Jaw('Z');            //夹爪996舵机转30度，松开物块
                     
                //35步进电机控制夹爪距离顶部0.5圈
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
                
                APP_SERVO_Plate(_thirdIndex);        //转盘舵机转到对应物块的放置角度
                APP_SERVO_Wrist('R');            //腕处舵机朝右
                
                
                //35步进电机控制夹爪距离顶部1圈
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[2], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

                APP_SERVO_Jaw('S');            //夹爪舵机夹紧物块
             
                //35步进电机控制夹爪距离顶部0.5圈
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

//                APP_SERVO_Wrist('F');            //腕处舵机朝前    
//                
               
                APP_MOTOR_ZDT_WaitAck(MOTOR_R);            
            }
        }
    }
    else if (_firstIndex == 2)
    {
        //第一个
        {
            {
                APP_MOTOR_ZDT_Move_P(MOTOR_R, MOTOR_R_Direction_CR, MOTOR_R_Vel, MOTOR_R_Acc, MOTOR_R_Pos[_firstIndex], 1, 0);     //控制底盘转盘转动到指定色环方向
                APP_MOTOR_ZDT_WaitAck(MOTOR_R);            
                APP_MOTOR_ZDT_Move_P(MOTOR_X, MOTOR_X_Direction_FORWARD, MOTOR_X_Vel, MOTOR_X_Acc, MOTOR_X_Pos[_firstIndex], 1, 0);                   //X导轨伸出到指定色环
                vTaskDelay(500);
            }
            //35步进电机控制夹爪距离顶部4圈
            APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[4], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
            APP_SERVO_Jaw('Z');            //夹爪996舵机转30度，松开物块
                 
            //35步进电机控制夹爪距离顶部1圈    
            APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0);            
            vTaskDelay(Delay_up);
            APP_MOTOR_ZDT_Move_P(MOTOR_X, MOTOR_X_Direction_FORWARD, MOTOR_X_Vel, MOTOR_X_Acc, MOTOR_X_Pos[_secondIndex], 1, 0);                   //X导轨伸出到指定色环

            //第二个
            {
                APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
                
                APP_SERVO_Plate(_firstIndex);        //转盘舵机转到对应物块的放置角度

                APP_SERVO_Wrist('R');            //腕处舵机朝右            
                APP_MOTOR_ZDT_Move_P(MOTOR_R, MOTOR_R_Direction_CR, MOTOR_R_Vel, MOTOR_R_Acc, MOTOR_R_Pos[_secondIndex], 1, 0);     //控制底盘转盘转动到指定色环方向           
                
                
                //35步进电机控制夹爪距离顶部1圈
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[2], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

                APP_SERVO_Jaw('S');            //夹爪舵机夹紧物块
             
                //35步进电机控制夹爪距离顶部0.5圈
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
                ////printf("3-5\r\n");

                APP_SERVO_Wrist('F');            //腕处舵机朝前    
                
             
                //35步进电机控制夹爪距离顶部0.5圈
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[4], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

                APP_SERVO_Jaw('Z');            //夹爪舵机夹紧物块
             
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0);        //35步进电机控制夹爪距离顶部0.5圈
                vTaskDelay(Delay_up);

                APP_MOTOR_ZDT_WaitAck(MOTOR_R);            
                APP_MOTOR_ZDT_Move_P(MOTOR_R, MOTOR_R_Direction_CR, MOTOR_R_Vel, MOTOR_R_Acc, MOTOR_R_Pos[_thirdIndex], 1, 0);     //控制底盘转盘转动到指定色环方向           
                
                APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
                
                APP_SERVO_Plate(_secondIndex);        //转盘舵机转到对应物块的放置角度
                APP_SERVO_Wrist('R');            //腕处舵机朝右    
                APP_MOTOR_ZDT_Move_P(MOTOR_X, MOTOR_X_Direction_FORWARD, MOTOR_X_Vel, MOTOR_X_Acc, MOTOR_X_Pos[_thirdIndex], 1, 0);
                
                
                //35步进电机控制夹爪距离顶部1圈
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[2], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

                APP_SERVO_Jaw('S');            //夹爪舵机松开物块
             
                //35步进电机控制夹爪距离顶部0.5圈
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

                APP_SERVO_Wrist('F');            //腕处舵机朝前    
                
                
                //35步进电机控制夹爪距离顶部0.5圈
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[4], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

                APP_SERVO_Jaw('Z');            //夹爪舵机夹紧物块
             
                //35步进电机控制夹爪距离顶部0.5圈
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0);        
                vTaskDelay(Delay_up);

                APP_MOTOR_ZDT_WaitAck(MOTOR_R);            
                //控制底盘转盘转动到指定色环方向           
                APP_MOTOR_ZDT_Move_P(MOTOR_R, MOTOR_R_Direction_CR, MOTOR_R_Vel, MOTOR_R_Acc, MOTOR_R_Pos[_secondIndex], 1, 0);     //控制底盘转盘转动到指定色环方向           
                
                APP_MOTOR_ZDT_WaitAck(MOTOR_Y);
                
                APP_SERVO_Plate(_thirdIndex);        //转盘舵机转到对应物块的放置角度
                APP_SERVO_Wrist('R');            //腕处舵机朝右    
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[2], 1, 0);
                
                
                //35步进电机控制夹爪距离顶部1圈
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[2], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

                APP_SERVO_Jaw('S');            //夹爪舵机松开物块
                
                //35步进电机控制夹爪距离顶部0.5圈
                APP_MOTOR_ZDT_Move_P(MOTOR_Y, MOTOR_Y_Direction_DOWN, MOTOR_Y_Vel, MOTOR_Y_Acc, MOTOR_Y_Pos[1], 1, 0); APP_MOTOR_ZDT_WaitAck(MOTOR_Y);

//                APP_SERVO_Wrist('F');            //腕处舵机朝前    
//                

                APP_MOTOR_ZDT_WaitAck(MOTOR_R);            
            }
        }
    }
}
