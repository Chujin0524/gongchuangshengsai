F407 
串口1             蓝牙调试
串口2             陀螺仪
串口3             摄像头
串口4             屏幕
串口5             二维码
CAN               各个步进电机
三个PWM输出口     舵机

名词解释
Raw    : 原料区
Pro    : 加工区
ProFir : 粗加工区
ProSec : 细加工区

沿用前407的 STATUS_E 枚举量，进行程序运行状态的转移

--------------------------------------------------------------------------------------------------
|                                          通信类任务                                            |
--------------------------------------------------------------------------------------------------
|Task_UART_BlueTooth   |   与蓝牙通信调试。                                                      |
|                      |   使用消息缓冲区，在串口中断中，收到数据后，存入 MessageBuffer ，       | 
|                      |   在 Task_UART_BlueTooth 里读取 MessageBuffer 并解析，                  |
|                      |   使用任务通知 Task_Main ，用于调试各 Status                            |
--------------------------------------------------------------------------------------------------
|Task_MaixCAM          |   与摄像头 MaixCAM 串口通信，得到 x y，                                 |
|                      |   使用流缓冲区，将计算后的坐标差数据传递给 Task_HandleCalculate         | 
--------------------------------------------------------------------------------------------------
|Task_HWT101           |   与陀螺仪 HWT101 串口通信，得到 theta ，                               |
|                      |   使用流缓冲区，将数据传递给 Task_PID                                   | 
--------------------------------------------------------------------------------------------------
|Task_CAN_MOTOR        |   与底部步进电机 CAN 通信，控制电机（速度、方向等）                     |
|                      |   使用流缓冲区接收控制命令，                                            |
|                      |   使用任务通知 Task_HandleStraight / Task_HandleCalculate 电机控制完成  |
--------------------------------------------------------------------------------------------------


---------------------------------------------------------------------------------------------------
|                                         逻辑算法类任务                                          |
---------------------------------------------------------------------------------------------------
|Task_Main             |   主任务，负责各处理的挂起/恢复                                          |
---------------------------------------------------------------------------------------------------
|Task_PID              |   通过流缓冲区得到角度或坐标theta/xy，进行pid处理，                      |
|                      |                                                                          |
|                      |   根据theta ， 只用来计算出能让车 走直线的数据   f1(theta)，             |
|                      |   使用流缓冲区将数据传递给 Task_HandleStraight ;                         |
|                      | [或根据坐标xy， 只用来计算出能让车 对准中心的数据 f2(x,y)                |
|                      |   使用流缓冲区将数据传递给 Task_HandleCalculate]                         |
---------------------------------------------------------------------------------------------------
|Task_HandleMove       |   结合 Task_MaixCAM、Task_HWT101、Task_PID 和 Task_CAN_MOTOR ，执行让车走直线的处理，  |
|                      |   与 Task_UART_HWT101 进行任务通知，负责陀螺仪任务的挂起/恢复            |
|                      |   通过流缓冲区得到经PID计算后的角度 theta，                              |
|                      |   使用流缓冲区发送控制命令给 Task_CAN_MOTOR                              |
---------------------------------------------------------------------------------------------------
|Task_HandlePick       |   传入 Color 、原料区和加工区俩地方的抓取高度(Raw / Pro)，               |
|                      |   结合 Task_PWM_Servo 和 Task_CAN_MOTOR ，执行抓取物块的处理             |
|                      |   使用流缓冲区接收抓取命令（如颜色、位置等），                           |
|                      |   使用任务通知 Task_Main 抓取完成。                                      |
---------------------------------------------------------------------------------------------------
|Task_HandlePlace      |   传入 Color 、码垛高度(ProFir / ProSec)，                               |
|                      |   结合 Task_PWM_Servo 和 Task_CAN_MOTOR ，执行放置物块的处理             |
|                      |   使用流缓冲区接收放置命令（如颜色、码垛高度等），                       |
|                      |   使用任务通知 Task_Main 抓取完成。                                      |
---------------------------------------------------------------------------------------------------