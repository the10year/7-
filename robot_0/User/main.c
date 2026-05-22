#include "stm32f10x.h"      
#include "Delay.h"        
#include "OLED.h"          
#include "LED.h"           
#include "PWM.h"          
#include "BOO.h"           
#include "Input.h"          
#include "Servo.h"         
#include "Serial.h"         
#include "WS2812B.h"        
#include <string.h>         
#include <stdlib.h>        
#include <stdio.h>        
#include <math.h>           

//基于STM32四足机器人(更新版)
//行走步态:前进，后退，原地左转，原地右转
//固定动作:招手，抬腿，展开，复位，摇摆，坐下
//外设:红外线，电磁炮，舵机云台，WS2812B彩色灯光，蜂鸣器，指示灯
//功能:离地检测，锂电池充放电
//实时状态机 + 舵机时序控制 + 非阻塞延时 + 标志位管理

//程序基于江协科技OLED模板程序上编写
//程序开源，仅供交流学习

//!!!!!!注意程序默认串口波特率9600,因为新买的HC05蓝牙模块就是9600，无需配置就能与手机app直接连接。
//!!!!!!如果你之前使用了遥控器版本，或者设置了蓝牙模块波特率，那么在Serial.c程序里修改为对应波特率!


//ID：我的STM32又烧啦   抖音/B站/小红书/微信视频号    ***账号同名***


extern char Serial_RxPacket[64];    // 串口接收数据缓冲区
extern uint8_t Serial_RxFlag;      // 串口接收完成标志位

volatile uint8_t MAGE_State = 0;               // 模式状态标志
extern const unsigned char BiLi[];             // 外部图片数据
extern const unsigned char DOUYIN[];
extern const unsigned char LiC[];

// 9路舵机角度变量
volatile float Angle1 = 90.0f;
volatile float Angle2 = 90.0f;
volatile float Angle3 = 90.0f;
volatile float Angle4 = 90.0f;
volatile float Angle5 = 0.0f;
volatile float Angle6 = 180.0f;
volatile float Angle7 = 0.0f;
volatile float Angle8 = 180.0f;
volatile float Angle9 = 100.0f;	// 云台舵机

// 云台舵机控制参数
volatile float ReceivedAngle9 = 100.0f;  // 目标角度
const float MinServoStep = 0.5f;         // 最小调节步长
const float MaxServoStep = 8.0f;         // 最大调节步长
const float FastThreshold = 20.0f;       // 快速调节阈值
const float SlowThreshold = 2.0f;        // 慢速调节阈值

// 功能动作状态标志
volatile uint8_t A4_State = 0;     
volatile uint8_t A5_State = 0;     
volatile uint8_t XNLED_State = 0;  
volatile uint8_t ZMLED_State = 0;  
volatile uint8_t HW_State = 0;     

// 左右摇摆动作参数
volatile uint8_t ActionState = 0;          // 动作状态机
volatile uint16_t ActionDelayCnt = 0;       // 动作延时计数器
const uint16_t ACTION_DELAY_TIME = 12;     // 动作延时时间
const float ServoSwingStep = 2.0f;         // 舵机摆动步长

// 前进动作参数
volatile uint8_t Forward_State = 0;        // 前进状态
volatile uint16_t Forward_DelayCnt = 0;    // 前进延时计数器
const uint16_t FORWARD_DELAY_TIME = 12;    // 前进延时时间
volatile uint8_t Forward_Step = 0;         // 前进步骤

// 后退动作参数
volatile uint8_t Back_State = 0;          
volatile uint16_t Back_DelayCnt = 0;       
const uint16_t BACK_DELAY_TIME = 12;      
volatile uint8_t Back_Step = 0;            

// 左转动作参数
volatile uint8_t Left_State = 0;           
volatile uint16_t Left_DelayCnt = 0;       
const uint16_t LEFT_DELAY_TIME = 12;       
volatile uint8_t Left_Step = 0;            

// 右转动作参数
volatile uint8_t Right_State = 0;          
volatile uint16_t Right_DelayCnt = 0;      
const uint16_t RIGHT_DELAY_TIME = 12;      
volatile uint8_t Right_Step = 0;           

// OLED刷新参数
uint8_t oledRefreshCnt = 0;                // OLED刷新计数器
const uint8_t OLED_REFRESH_INTERVAL = 8;   // OLED刷新间隔

// LED闪烁参数
volatile uint8_t LED_State = 0;            // LED状态
volatile uint16_t LED_DelayCnt = 0;         // LED延时计数器
const uint16_t LED_DELAY_TIME = 12;        // LED闪烁间隔

// 滑动动作参数
volatile uint8_t Huadong_State = 0;        // 滑动状态
volatile uint8_t Huadong_Step = 0;         // 滑动步骤
const uint8_t MAX_STEP = 15;               // 滑动最大步数
const uint8_t STEP_INTERVAL = 1;           // 滑动步间间隔
volatile uint8_t step_delay = 0;           // 滑动延时
volatile uint8_t huadong_release_flag = 0; // 滑动释放标志

uint16_t booOffCnt = 0;    

void ResetAll(void)
{
    // 舵机角度复位
    Angle1 = 100.0f; Angle2 = 80.0f; Angle3 = 90.0f; Angle4 = 90.0f;
    Angle5 = 0.0f;  Angle6 = 180.0f; Angle7 = 0.0f; Angle8 = 180.0f;
    Angle9 = 100.0f;
    ReceivedAngle9 = 100.0f;

    // 所有动作状态复位
    ActionState = 0;
    ActionDelayCnt = 0;
    A4_State = 0;
    A5_State = 0;
    XNLED_State = 0;
    MAGE_State = 0;
    Forward_State = 0;
    Forward_Step = 0;
    Back_State = 0;
    Back_Step = 0;
    Left_State = 0;
    Left_Step = 0;
    Right_State = 0;
    Right_Step = 0;

    // 复位后显示原图片
    OLED_Clear();
    OLED_ShowImage(0, 0, 128, 64, BiLi);
    OLED_Update();
}

void Huadong_Smooth(void)
{
    // 上部舵机保持固定姿态
    Angle1 = 135.0f; Angle2 = 45.0f; Angle3 = 45.0f; Angle4 = 135.0f;
    Servo_SetAngle1(Angle1); Servo_SetAngle2(Angle2);
    Servo_SetAngle3(Angle3); Servo_SetAngle4(Angle4);

    // 滑动动作延时控制
    step_delay++;
    if (step_delay < STEP_INTERVAL) return;
    step_delay = 0;

    // 滑动状态机：正滑/反滑切换
    switch(Huadong_State)
    {
        case 1:
            Huadong_Step++;
            if (Huadong_Step >= MAX_STEP)
            {
                Huadong_Step = MAX_STEP;
                Huadong_State = 2;
            }
            break;
        case 2:
            Huadong_Step--;
            if (Huadong_Step <= 0)
            {
                Huadong_Step = 0;
                Huadong_State = 1;
            }
            break;
        default:
            return;
    }

    // 计算下部舵机角度
    Angle5 = Huadong_Step * 2;
    Angle7 = Huadong_Step * 2;
    Angle6 = 180 - (Huadong_Step * 2);
    Angle8 = 180 - (Huadong_Step * 2);

    // 更新下部舵机
    Servo_SetAngle5(Angle5); Servo_SetAngle6(Angle6);
    Servo_SetAngle7(Angle7); Servo_SetAngle8(Angle8);
}

int main(void)
{
    // 外设初始化
    Servo_Init();
    Serial_Init();
    Input_Init();
    WS2812B_Init();
    BOO_Init();
    OLED_Init();
    LED_Init();
    LED_StartBlink();

    // 系统初始复位
    ResetAll();

    while (1)
    {
        // 实时刷新所有舵机角度
        Servo_SetAngle1(Angle1);
        Servo_SetAngle2(Angle2);
        Servo_SetAngle3(Angle3);
        Servo_SetAngle4(Angle4);
        Servo_SetAngle5(Angle5);
        Servo_SetAngle6(Angle6);
        Servo_SetAngle7(Angle7);
        Servo_SetAngle8(Angle8);
        Servo_SetAngle9(Angle9);

        /************************ 串口指令解析处理 ************************/
        if (Serial_RxFlag == 1)
        {
            char* endptr = NULL;
            float num = strtof(Serial_RxPacket, &endptr);

            // 云台角度指令（0-180数字）
            if (endptr != Serial_RxPacket && *endptr == '\0' && num >= 0.0f && num <= 180.0f)
            {
                ReceivedAngle9 = num;
            }
            // 复位指令
            else if (strncasecmp(Serial_RxPacket, "REST", 4) == 0)
            {
                ResetAll();
                Forward_State = 0;
                Back_State = 0;
                Left_State = 0;
                Right_State = 0;
              	Serial_SendString("OK\r\n");
            }
            // 展开指令
            else if (strncasecmp(Serial_RxPacket, "ZK", 2) == 0)
            {
                ActionState = 0;
                Forward_State = 0;
                Back_State = 0;
                Left_State = 0;
                Right_State = 0;
                Angle1 = 135.0f; Angle2 = 45.0f; Angle3 = 45.0f; Angle4 = 135.0f;
                Angle5 = 0.0f;   Angle6 = 180.0f; Angle7 = 0.0f; Angle8 = 180.0f;
                A4_State = 0; A5_State = 0;
            }
            // 坐下指令
            else if (strncasecmp(Serial_RxPacket, "ZX", 2) == 0)
            {
                ActionState = 0;
                Forward_State = 0;
                Back_State = 0;
                Left_State = 0;
                Right_State = 0;
                Angle1 = 135.0f; Angle2 = 45.0f; Angle3 = 45.0f; Angle4 = 135.0f;
                Angle5 = 90.0f;  Angle6 = 90.0f;  Angle7 = 90.0f; Angle8 = 90.0f;
                A4_State = 0; A5_State = 0;
            }
            // 左右摇摆指令
            else if (strncasecmp(Serial_RxPacket, "YB", 2) == 0)
            {
                ActionState = 1;
                Forward_State = 0;
                Back_State = 0;
                Left_State = 0;
                Right_State = 0;
                ActionDelayCnt = 0;
                A4_State = 0; A5_State = 0;
            }
            // 招手动作切换
            else if (strncasecmp(Serial_RxPacket, "ZS", 2) == 0)
            {
                ActionState = 0;
                Forward_State = 0;
                Back_State = 0;
                Left_State = 0;
                Right_State = 0;
                if (A4_State == 0)
                {
                    Angle1 = 160.0f; Angle2 = 45.0f; Angle3 = 110.0f; Angle4 = 70.0f;
                    Angle5 = 20.0f;  Angle6 = 35.0f;  Angle7 = 20.0f; Angle8 = 160.0f;
                    A4_State = 1;
                }
                else
                {
                    Angle1 = 135.0f; Angle2 = 45.0f; Angle3 = 45.0f; Angle4 = 135.0f;
                    Angle5 = 0.0f;   Angle6 = 180.0f; Angle7 = 0.0f; Angle8 = 180.0f;
                    A4_State = 0;
                }
            }
            // 抬脚动作切换
            else if (strncasecmp(Serial_RxPacket, "TJ", 2) == 0)
            {
                ActionState = 0;
                Forward_State = 0;
                Back_State = 0;
                Left_State = 0;
                Right_State = 0;
                if (A5_State == 0)
                {
                    Angle1 = 90.0f; Angle2 = 90.0f; Angle3 = 45.0f; Angle4 = 135.0f;
                    Angle5 = 0.0f;  Angle6 = 180.0f; Angle7 = 135.0f; Angle8 = 180.0f;
                    A5_State = 1;
                }
                else
                {
                    Angle1 = 135.0f; Angle2 = 45.0f; Angle3 = 45.0f; Angle4 = 135.0f;
                    Angle5 = 0.0f;   Angle6 = 180.0f; Angle7 = 0.0f; Angle8 = 180.0f;
                    A5_State = 0;
                }
            }
            // 前进指令
            else if (strncasecmp(Serial_RxPacket, "UP", 2) == 0)
            {
                ActionState = 0;
                Back_State = 0;
                Left_State = 0;
                Right_State = 0;
                Forward_State = 1;
                Forward_Step = 1;
                Forward_DelayCnt = 0;
                A4_State = 0;
                A5_State = 0;
            }
            // 后退指令
            else if (strncasecmp(Serial_RxPacket, "BACK", 4) == 0)
            {
                ActionState = 0;
                Forward_State = 0;
                Left_State = 0;
                Right_State = 0;
                Back_State = 1;
                Back_Step = 1;
                Back_DelayCnt = 0;
                A4_State = 0;
                A5_State = 0;
            }
            // 左转指令
            else if (strncasecmp(Serial_RxPacket, "LEFT", 4) == 0)
            {
                ActionState = 0;
                Forward_State = 0;
                Back_State = 0;
                Right_State = 0;
                Left_State = 1;
                Left_Step = 1;
                Left_DelayCnt = 0;
                A4_State = 0;
                A5_State = 0;
            }
            // 右转指令
            else if (strncasecmp(Serial_RxPacket, "RIGHT", 5) == 0)
            {
                ActionState = 0;
                Forward_State = 0;
                Back_State = 0;
                Left_State = 0;
                Right_State = 1;
                Right_Step = 1;
                Right_DelayCnt = 0;
                A4_State = 0;
                A5_State = 0;
            }
            // 红外控制
            else if (strncasecmp(Serial_RxPacket, "HW", 2) == 0)
            {
                if (HW_State == 0)
                {
                    HW_ON();
                    HW_State = 1;
                }
                else
                {
                    HW_State = 0;
                    HW_OFF();
                }
            }
            // 炫彩灯模式控制
            else if (strncasecmp(Serial_RxPacket, "XNLED", 5) == 0)
            {
                if (XNLED_State == 0)
                {
                    FlowFrom10To9_AllOn(255, 20, 0, 60000);
                    WS2812_Update();
                    XNLED_State = 1;
                }
                else
                {
                    ColorFlowFrom10To9_LargeChange(80000);
                    WS2812_Update();
                    XNLED_State = 0;
                }
            }
            // 照明灯控制
            else if (strncasecmp(Serial_RxPacket, "ZMLED", 5) == 0)
            {
                if (ZMLED_State == 0)
                {
                    All_LED_On(200, 200, 200);
                    WS2812_Update();
                    ZMLED_State = 1;
                }
                else
                {
                    All_LED_On(0, 0, 0);
                    WS2812_Update();
                    ZMLED_State = 0;
                }
            }
	        else if (strncasecmp(Serial_RxPacket, "BOO", 3) == 0) 
            {
                Color_Wipe(255, 20, 0, 50000);
                WS2812_Update();
                BOO_ON();
                booOffCnt = 16;
            }

            // 清空串口缓冲区，清除接收标志
            memset(Serial_RxPacket, 0, sizeof(Serial_RxPacket));
            Serial_RxFlag = 0;
        }

        /************************ 云台舵机平滑调节 ************************/
        float diff = ReceivedAngle9 - Angle9;
        float absDiff = fabs(diff);
        float servoStep = MinServoStep;

        if (absDiff >= FastThreshold) servoStep = MaxServoStep;
        else if (absDiff > SlowThreshold)
        {
            servoStep = MinServoStep + (MaxServoStep - MinServoStep) *
                       (absDiff - SlowThreshold) / (FastThreshold - SlowThreshold);
        }

        if (diff > servoStep)
        {
            Angle9 += servoStep;
            if (Angle9 > 180.0f) Angle9 = 180.0f;
        }
        else if (diff < -servoStep)
        {
            Angle9 -= servoStep;
            if (Angle9 < 0.0f) Angle9 = 0.0f;
        }
        else if (absDiff > 0.1f)
        {
            Angle9 = ReceivedAngle9;
        }

        /************************ 左右摇摆动作执行 ************************/
        if (ActionState != 0)
        {
            ActionDelayCnt++;
            switch(ActionState)
            {
                case 1:
                    if (Angle5 > 0.0f) Angle5 -= ServoSwingStep;
                    if (Angle7 > 0.0f) Angle7 -= ServoSwingStep;
                    if (Angle6 < 180.0f) Angle6 += ServoSwingStep;
                    if (Angle8 < 180.0f) Angle8 += ServoSwingStep;
                    if (ActionDelayCnt >= ACTION_DELAY_TIME)
                    {
                        ActionState = 2; ActionDelayCnt = 0;
                    }
                    break;
                case 2:
                    if (Angle5 < 40.0f) Angle5 += ServoSwingStep;
                    if (Angle7 < 40.0f) Angle7 += ServoSwingStep;
                    if (ActionDelayCnt >= ACTION_DELAY_TIME)
                    {
                        ActionState = 3; ActionDelayCnt = 0;
                    }
                    break;
                case 3:
                    if (Angle5 > 0.0f) Angle5 -= ServoSwingStep;
                    if (Angle7 > 0.0f) Angle7 -= ServoSwingStep;
                    if (ActionDelayCnt >= ACTION_DELAY_TIME)
                    {
                        ActionState = 4; ActionDelayCnt = 0;
                    }
                    break;
                case 4:
                    if (Angle6 > 140.0f) Angle6 -= ServoSwingStep;
                    if (Angle8 > 140.0f) Angle8 -= ServoSwingStep;
                    if (ActionDelayCnt >= ACTION_DELAY_TIME)
                    {
                        ActionState = 1; ActionDelayCnt = 0;
                    }
                    break;
                default:
                    ActionState = 0; ActionDelayCnt = 0;
                    break;
            }
        }

        /************************ 前进动作执行 ************************/
        if (Forward_State != 0)
        {
            Forward_DelayCnt++;
            if (Forward_DelayCnt >= FORWARD_DELAY_TIME)
            {
                Forward_DelayCnt = 0;
                Forward_Step++;
                if (Forward_Step > 4) Forward_Step = 1;
            }

            switch(Forward_Step)
            {
                case 1:
                    Angle1 = 135.0f; Angle2 = 45.0f;  Angle3 = 45.0f;  Angle4 = 135.0f;
                    Angle5 = 0.0f;   Angle6 = 150.0f; Angle7 = 30.0f; Angle8 = 180.0f;
                    break;
                case 2:
                    Angle1 = 135.0f; Angle2 = 15.0f;  Angle3 = 75.0f;  Angle4 = 135.0f;
                    Angle5 = 0.0f;   Angle6 = 180.0f; Angle7 = 0.0f;  Angle8 = 180.0f;
                    break;
                case 3:
                    Angle1 = 135.0f; Angle2 = 15.0f;  Angle3 = 75.0f;  Angle4 = 135.0f;
                    Angle5 = 30.0f;  Angle6 = 180.0f; Angle7 = 0.0f;  Angle8 = 150.0f;
                    break;
                case 4:
                    Angle1 = 165.0f; Angle2 = 45.0f;  Angle3 = 45.0f;  Angle4 = 105.0f;
                    Angle5 = 0.0f;   Angle6 = 180.0f; Angle7 = 0.0f;  Angle8 = 180.0f;
                    break;
            }
        }

        /************************ 后退动作执行 ************************/
        if (Back_State != 0)
        {
            Back_DelayCnt++;
            if (Back_DelayCnt >= BACK_DELAY_TIME)
            {
                Back_DelayCnt = 0;
                Back_Step++;
                if (Back_Step > 4) Back_Step = 1;
            }

            switch(Back_Step)
            {
                case 1:
                    Angle1 = 135.0f; Angle2 = 45.0f;  Angle3 = 45.0f;  Angle4 = 135.0f;
                    Angle5 = 0.0f;   Angle6 = 150.0f; Angle7 = 30.0f; Angle8 = 180.0f;
                    break;
                case 2:
                    Angle1 = 135.0f; Angle2 = 75.0f;  Angle3 = 15.0f;  Angle4 = 135.0f;
                    Angle5 = 0.0f;   Angle6 = 180.0f; Angle7 = 0.0f;  Angle8 = 180.0f;
                    break;
                case 3:
                    Angle1 = 135.0f; Angle2 = 75.0f;  Angle3 = 15.0f;  Angle4 = 135.0f;
                    Angle5 = 30.0f;  Angle6 = 180.0f; Angle7 = 0.0f;  Angle8 = 150.0f;
                    break;
                case 4:
                    Angle1 = 105.0f; Angle2 = 45.0f;  Angle3 = 45.0f;  Angle4 = 165.0f;
                    Angle5 = 0.0f;   Angle6 = 180.0f; Angle7 = 0.0f;  Angle8 = 180.0f;
                    break;
            }
        }

/************************ 左转动作执行************************/
if (Left_State != 0)
{
    Left_DelayCnt++;
    if (Left_DelayCnt >= LEFT_DELAY_TIME)
    {
        Left_DelayCnt = 0;
        Left_Step++;
        if (Left_Step > 4) Left_Step = 1;
    }

    switch(Left_Step)
    {
        case 1:
            Angle1 = 150.0f; Angle2 = 30.0f; Angle3 = 30.0f; Angle4 = 150.0f;
            Angle5 = 30.0f;  Angle6 = 180.0f; Angle7 = 0.0f;  Angle8 = 150.0f;
            break;
        case 2:
           
            Angle1 = 165.0f; Angle2 = 15.0f; Angle3 = 15.0f; Angle4 = 165.0f;
            Angle5 = 0.0f;  Angle6 = 180.0f; Angle7 = 0.0f;  Angle8 = 180.0f;
            break;
        case 3:
            Angle1 = 165.0f; Angle2 = 15.0f; Angle3 = 15.0f; Angle4 = 165.0f;
            Angle5 = 0.0f;  Angle6 = 150.0f; Angle7 = 30.0f; Angle8 = 180.0f;
            break;
        case 4:
            Angle1 = 135.0f; Angle2 = 45.0f; Angle3 = 45.0f; Angle4 = 135.0f;
            Angle5 = 0.0f;  Angle6 = 180.0f; Angle7 = 0.0f;  Angle8 = 180.0f;
            break;
    }
}

 /************************ 右转动作执行************************/
if (Right_State != 0)
{
    Right_DelayCnt++;
    if (Right_DelayCnt >= RIGHT_DELAY_TIME)
    {
        Right_DelayCnt = 0;
        Right_Step++;
        if (Right_Step > 4) Right_Step = 1;
    }

    switch(Right_Step)
    {
        case 1:
       
            Angle1 = 150.0f; Angle2 = 30.0f; Angle3 = 30.0f; Angle4 = 150.0f;
            Angle5 = 0.0f;  Angle6 = 150.0f; Angle7 = 30.0f; Angle8 = 180.0f;
            break;
        case 2:
         
            Angle1 = 165.0f; Angle2 = 15.0f; Angle3 = 15.0f; Angle4 = 165.0f;
            Angle5 = 0.0f;  Angle6 = 180.0f; Angle7 = 0.0f;  Angle8 = 180.0f;
            break;
        case 3:
         
            Angle1 = 165.0f; Angle2 = 15.0f; Angle3 = 15.0f; Angle4 = 165.0f;
            Angle5 = 30.0f; Angle6 = 180.0f; Angle7 = 0.0f;  Angle8 = 150.0f;
            break;
        case 4:
           
            Angle1 = 135.0f; Angle2 = 45.0f; Angle3 = 45.0f; Angle4 = 135.0f;
            Angle5 = 0.0f;  Angle6 = 180.0f; Angle7 = 0.0f;  Angle8 = 180.0f;
            break;
    }
}

        /************************ OLED定时刷新 ************************/
        if (oledRefreshCnt >= OLED_REFRESH_INTERVAL)
        {
            OLED_Update();
            oledRefreshCnt = 0;
        }
        else
            oledRefreshCnt++;

        /************************ LED闪烁控制 ************************/
        if (LED_State != 0)
        {
            LED_DelayCnt++;
            if (LED_DelayCnt >= LED_DELAY_TIME)
            {
                LED_DelayCnt = 0;
                if (LED_State == 1)
                {
                    GPIO_ResetBits(GPIOA, GPIO_Pin_12);
                    GPIO_SetBits(GPIOA, GPIO_Pin_15);
                    LED_State = 2;
                }
                else
                {
                    GPIO_SetBits(GPIOA, GPIO_Pin_12);
                    GPIO_ResetBits(GPIOA, GPIO_Pin_15);
                    LED_State = 1;
                }
            }
        }

        /************************ (离地检测)PA5滑动动作控制 ************************/
        if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5) == 1)
        {
            if (Huadong_State == 0)
            {
                Forward_State = 0;
                Back_State = 0;
                Left_State = 0;
                Right_State = 0;
                Huadong_State = 1;
                Huadong_Step = 0;
                step_delay = 0;
            }
            huadong_release_flag = 1;
        }
        else
        {
            if (huadong_release_flag == 1)
            {
                // 恢复站立姿态
                Angle1 = 135.0f; Angle2 = 45.0f; Angle3 = 45.0f; Angle4 = 135.0f;
                Angle5 = 0.0f;   Angle6 = 180.0f; Angle7 = 0.0f; Angle8 = 180.0f;

                Servo_SetAngle1(Angle1); Servo_SetAngle2(Angle2);
                Servo_SetAngle3(Angle3); Servo_SetAngle4(Angle4);
                Servo_SetAngle5(Angle5); Servo_SetAngle6(Angle6);
                Servo_SetAngle7(Angle7); Servo_SetAngle8(Angle8);

                huadong_release_flag = 0;
            }
            Huadong_State = 0;
        }

        // 执行滑动动作
        if (Huadong_State != 0)
        {
            Huadong_Smooth();
        }
	  
        if (booOffCnt > 0)
        {
            booOffCnt--;
            if (booOffCnt == 0) BOO_OFF();
        }

        // 主循环延时25ms
        Delay_ms(25);
    }
}
