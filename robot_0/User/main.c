#include "stm32f10x.h"
#include "Delay.h"
#include "OLED.h"
#include "LED.h"
#include "PWM.h"
#include "Input.h"
#include "Servo.h"
#include "Serial.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "bh1750.h"

extern char Serial_RxPacket[64];
extern uint8_t Serial_RxFlag;

volatile uint8_t MAGE_State = 0;
extern const unsigned char BiLi[];
extern const unsigned char DOUYIN[];
extern const unsigned char LiC[];

volatile float Angle1 = 90.0f;
volatile float Angle2 = 90.0f;
volatile float Angle3 = 90.0f;
volatile float Angle4 = 90.0f;
volatile float Angle5 = 0.0f;
volatile float Angle6 = 180.0f;
volatile float Angle7 = 0.0f;
volatile float Angle8 = 180.0f;

volatile uint8_t A4_State = 0;
volatile uint8_t A5_State = 0;
volatile uint8_t XNLED_State = 0;
volatile uint8_t ZMLED_State = 0;
volatile uint8_t HW_State = 0;

volatile uint8_t ActionState = 0;
volatile uint16_t ActionDelayCnt = 0;
const uint16_t ACTION_DELAY_TIME = 12;
const float ServoSwingStep = 2.0f;

volatile uint8_t Forward_State = 0;
volatile uint16_t Forward_DelayCnt = 0;
const uint16_t FORWARD_DELAY_TIME = 12;
volatile uint8_t Forward_Step = 0;

volatile uint8_t Back_State = 0;
volatile uint16_t Back_DelayCnt = 0;
const uint16_t BACK_DELAY_TIME = 12;
volatile uint8_t Back_Step = 0;

volatile uint8_t Left_State = 0;
volatile uint16_t Left_DelayCnt = 0;
const uint16_t LEFT_DELAY_TIME = 12;
volatile uint8_t Left_Step = 0;

volatile uint8_t Right_State = 0;
volatile uint16_t Right_DelayCnt = 0;
const uint16_t RIGHT_DELAY_TIME = 12;
volatile uint8_t Right_Step = 0;

volatile uint16_t bh1750_cnt = 0;
const uint16_t BH1750_READ_INTERVAL = 40;
float lux_val = 0;

volatile uint8_t LED_State = 0;
volatile uint16_t LED_DelayCnt = 0;
const uint16_t LED_DELAY_TIME = 12;

volatile uint8_t Huadong_State = 0;
volatile uint8_t Huadong_Step = 0;
const uint8_t MAX_STEP = 15;
const uint8_t STEP_INTERVAL = 1;
volatile uint8_t step_delay = 0;
volatile uint8_t huadong_release_flag = 0;

uint16_t booOffCnt = 0;

void ResetAll(void)
{
    Angle1 = 100.0f; Angle2 = 80.0f; Angle3 = 90.0f; Angle4 = 90.0f;
    Angle5 = 0.0f;  Angle6 = 180.0f; Angle7 = 0.0f; Angle8 = 180.0f;
    ActionState = 0; ActionDelayCnt = 0;
    A4_State = 0; A5_State = 0; XNLED_State = 0; MAGE_State = 0;
    Forward_State = 0; Forward_Step = 0;
    Back_State = 0; Back_Step = 0;
    Left_State = 0; Left_Step = 0;
    Right_State = 0; Right_Step = 0;
}

void Huadong_Smooth(void)
{
    Angle1 = 135.0f; Angle2 = 45.0f; Angle3 = 45.0f; Angle4 = 135.0f;
    Servo_SetAngle1(Angle1); Servo_SetAngle2(Angle2);
    Servo_SetAngle3(Angle3); Servo_SetAngle4(Angle4);
    step_delay++;
    if (step_delay < STEP_INTERVAL) return;
    step_delay = 0;
    switch(Huadong_State)
    {
        case 1:
            Huadong_Step++;
            if (Huadong_Step >= MAX_STEP) { Huadong_Step = MAX_STEP; Huadong_State = 2; }
            break;
        case 2:
            Huadong_Step--;
            if (Huadong_Step <= 0) { Huadong_Step = 0; Huadong_State = 1; }
            break;
        default: return;
    }
    Angle5 = Huadong_Step * 2; Angle7 = Huadong_Step * 2;
    Angle6 = 180 - (Huadong_Step * 2); Angle8 = 180 - (Huadong_Step * 2);
    Servo_SetAngle5(Angle5); Servo_SetAngle6(Angle6);
    Servo_SetAngle7(Angle7); Servo_SetAngle8(Angle8);
}

int main(void)
{
    Servo_Init();
    Serial_Init();
    Input_Init();
    OLED_Init();
    LED_Init();
    LED_StartBlink();
    bh1750_init();
    ResetAll();

    while (1)
    {
        Servo_SetAngle1(Angle1);
        Servo_SetAngle2(Angle2);
        Servo_SetAngle3(Angle3);
        Servo_SetAngle4(Angle4);
        Servo_SetAngle5(Angle5);
        Servo_SetAngle6(Angle6);
        Servo_SetAngle7(Angle7);
        Servo_SetAngle8(Angle8);

        /************************ 串口指令解析处理 ************************/
        if (Serial_RxFlag == 1)
        {
            if (strncasecmp(Serial_RxPacket, "REST", 4) == 0)
            {
                ResetAll(); Forward_State=0; Back_State=0; Left_State=0; Right_State=0;
                Serial_SendString("OK\r\n");
            }
            else if (strncasecmp(Serial_RxPacket, "ZK", 2) == 0)
            {
                ActionState=0; Forward_State=0; Back_State=0; Left_State=0; Right_State=0;
                Angle1=135;Angle2=45;Angle3=45;Angle4=135;Angle5=0;Angle6=180;Angle7=0;Angle8=180;
                A4_State=0;A5_State=0;
            }
            else if (strncasecmp(Serial_RxPacket, "ZX", 2) == 0)
            {
                ActionState=0; Forward_State=0; Back_State=0; Left_State=0; Right_State=0;
                Angle1=135;Angle2=45;Angle3=45;Angle4=135;Angle5=90;Angle6=90;Angle7=90;Angle8=90;
                A4_State=0;A5_State=0;
            }
            else if (strncasecmp(Serial_RxPacket, "YB", 2) == 0)
            {
                ActionState=1; Forward_State=0; Back_State=0; Left_State=0; Right_State=0;
                ActionDelayCnt=0; A4_State=0;A5_State=0;
            }
            else if (strncasecmp(Serial_RxPacket, "ZS", 2) == 0)
            {
                ActionState=0; Forward_State=0; Back_State=0; Left_State=0; Right_State=0;
                if(A4_State==0)
                { Angle1=160;Angle2=45;Angle3=110;Angle4=70;Angle5=20;Angle6=35;Angle7=20;Angle8=160;A4_State=1; }
                else
                { Angle1=135;Angle2=45;Angle3=45;Angle4=135;Angle5=0;Angle6=180;Angle7=0;Angle8=180;A4_State=0; }
            }
            else if (strncasecmp(Serial_RxPacket, "TJ", 2) == 0)
            {
                ActionState=0; Forward_State=0; Back_State=0; Left_State=0; Right_State=0;
                if(A5_State==0)
                { Angle1=90;Angle2=90;Angle3=45;Angle4=135;Angle5=0;Angle6=180;Angle7=135;Angle8=180;A5_State=1; }
                else
                { Angle1=135;Angle2=45;Angle3=45;Angle4=135;Angle5=0;Angle6=180;Angle7=0;Angle8=180;A5_State=0; }
            }
            else if (strncasecmp(Serial_RxPacket, "UP", 2) == 0)
            {
                ActionState=0;Back_State=0;Left_State=0;Right_State=0;
                Forward_State=1;Forward_Step=1;Forward_DelayCnt=0;A4_State=0;A5_State=0;
            }
            else if (strncasecmp(Serial_RxPacket, "BACK", 4) == 0)
            {
                ActionState=0;Forward_State=0;Left_State=0;Right_State=0;
                Back_State=1;Back_Step=1;Back_DelayCnt=0;A4_State=0;A5_State=0;
            }
            else if (strncasecmp(Serial_RxPacket, "LEFT", 4) == 0)
            {
                ActionState=0;Forward_State=0;Back_State=0;Right_State=0;
                Left_State=1;Left_Step=1;Left_DelayCnt=0;A4_State=0;A5_State=0;
            }
            else if (strncasecmp(Serial_RxPacket, "RIGHT", 5) == 0)
            {
                ActionState=0;Forward_State=0;Back_State=0;Left_State=0;
                Right_State=1;Right_Step=1;Right_DelayCnt=0;A4_State=0;A5_State=0;
            }
            memset(Serial_RxPacket, 0, sizeof(Serial_RxPacket));
            Serial_RxFlag = 0;
        }

        /************************ 左右摇摆动作执行 ************************/
        if (ActionState != 0)
        {
            ActionDelayCnt++;
            switch(ActionState)
            {
                case 1:
                    if(Angle5>0)Angle5-=ServoSwingStep;
                    if(Angle7>0)Angle7-=ServoSwingStep;
                    if(Angle6<180)Angle6+=ServoSwingStep;
                    if(Angle8<180)Angle8+=ServoSwingStep;
                    if(ActionDelayCnt>=ACTION_DELAY_TIME){ActionState=2;ActionDelayCnt=0;}
                    break;
                case 2:
                    if(Angle5<40)Angle5+=ServoSwingStep;
                    if(Angle7<40)Angle7+=ServoSwingStep;
                    if(ActionDelayCnt>=ACTION_DELAY_TIME){ActionState=3;ActionDelayCnt=0;}
                    break;
                case 3:
                    if(Angle5>0)Angle5-=ServoSwingStep;
                    if(Angle7>0)Angle7-=ServoSwingStep;
                    if(ActionDelayCnt>=ACTION_DELAY_TIME){ActionState=4;ActionDelayCnt=0;}
                    break;
                case 4:
                    if(Angle6>140)Angle6-=ServoSwingStep;
                    if(Angle8>140)Angle8-=ServoSwingStep;
                    if(ActionDelayCnt>=ACTION_DELAY_TIME){ActionState=1;ActionDelayCnt=0;}
                    break;
                default:
                    ActionState=0;ActionDelayCnt=0;
                    break;
            }
        }

        /************************ 前进动作执行 ************************/
        if (Forward_State != 0)
        {
            Forward_DelayCnt++;
            if(Forward_DelayCnt>=FORWARD_DELAY_TIME){Forward_DelayCnt=0;Forward_Step++;if(Forward_Step>4)Forward_Step=1;}
            switch(Forward_Step)
            {
                case 1:Angle1=135;Angle2=45;Angle3=45;Angle4=135;Angle5=0;Angle6=150;Angle7=30;Angle8=180;break;
                case 2:Angle1=135;Angle2=15;Angle3=75;Angle4=135;Angle5=0;Angle6=180;Angle7=0;Angle8=180;break;
                case 3:Angle1=135;Angle2=15;Angle3=75;Angle4=135;Angle5=30;Angle6=180;Angle7=0;Angle8=150;break;
                case 4:Angle1=165;Angle2=45;Angle3=45;Angle4=105;Angle5=0;Angle6=180;Angle7=0;Angle8=180;break;
            }
        }

        /************************ 后退动作执行 ************************/
        if (Back_State != 0)
        {
            Back_DelayCnt++;
            if(Back_DelayCnt>=BACK_DELAY_TIME){Back_DelayCnt=0;Back_Step++;if(Back_Step>4)Back_Step=1;}
            switch(Back_Step)
            {
                case 1:Angle1=135;Angle2=45;Angle3=45;Angle4=135;Angle5=0;Angle6=150;Angle7=30;Angle8=180;break;
                case 2:Angle1=135;Angle2=75;Angle3=15;Angle4=135;Angle5=0;Angle6=180;Angle7=0;Angle8=180;break;
                case 3:Angle1=135;Angle2=75;Angle3=15;Angle4=135;Angle5=30;Angle6=180;Angle7=0;Angle8=150;break;
                case 4:Angle1=105;Angle2=45;Angle3=45;Angle4=165;Angle5=0;Angle6=180;Angle7=0;Angle8=180;break;
            }
        }

        /************************ 左转动作执行 ************************/
        if (Left_State != 0)
        {
            Left_DelayCnt++;
            if(Left_DelayCnt>=LEFT_DELAY_TIME){Left_DelayCnt=0;Left_Step++;if(Left_Step>4)Left_Step=1;}
            switch(Left_Step)
            {
                case 1:Angle1=150;Angle2=30;Angle3=30;Angle4=150;Angle5=30;Angle6=180;Angle7=0;Angle8=150;break;
                case 2:Angle1=165;Angle2=15;Angle3=15;Angle4=165;Angle5=0;Angle6=180;Angle7=0;Angle8=180;break;
                case 3:Angle1=165;Angle2=15;Angle3=15;Angle4=165;Angle5=0;Angle6=150;Angle7=30;Angle8=180;break;
                case 4:Angle1=135;Angle2=45;Angle3=45;Angle4=135;Angle5=0;Angle6=180;Angle7=0;Angle8=180;break;
            }
        }

        /************************ 右转动作执行 ************************/
        if (Right_State != 0)
        {
            Right_DelayCnt++;
            if(Right_DelayCnt>=RIGHT_DELAY_TIME){Right_DelayCnt=0;Right_Step++;if(Right_Step>4)Right_Step=1;}
            switch(Right_Step)
            {
                case 1:Angle1=150;Angle2=30;Angle3=30;Angle4=150;Angle5=0;Angle6=150;Angle7=30;Angle8=180;break;
                case 2:Angle1=165;Angle2=15;Angle3=15;Angle4=165;Angle5=0;Angle6=180;Angle7=0;Angle8=180;break;
                case 3:Angle1=165;Angle2=15;Angle3=15;Angle4=165;Angle5=30;Angle6=180;Angle7=0;Angle8=150;break;
                case 4:Angle1=135;Angle2=45;Angle3=45;Angle4=135;Angle5=0;Angle6=180;Angle7=0;Angle8=180;break;
            }
        }

        /************************ BH1750 光照读取 + OLED显示 ************************/
        bh1750_cnt++;
        if (bh1750_cnt >= BH1750_READ_INTERVAL)
        {
            bh1750_cnt = 0;
            lux_val = bh1750_get_lux();
            OLED_Clear();
            OLED_Printf(0, 0, 8, "Lux:%.0f", lux_val);
            OLED_Update();
        }

        /************************ LED闪烁控制 ************************/
        if (LED_State != 0)
        {
            LED_DelayCnt++;
            if(LED_DelayCnt>=LED_DELAY_TIME)
            {
                LED_DelayCnt=0;
                if(LED_State==1)
                { GPIO_ResetBits(GPIOA,GPIO_Pin_12);GPIO_SetBits(GPIOA,GPIO_Pin_15);LED_State=2; }
                else
                { GPIO_SetBits(GPIOA,GPIO_Pin_12);GPIO_ResetBits(GPIOA,GPIO_Pin_15);LED_State=1; }
            }
        }

        /************************ 离地检测滑动动作控制 ************************/
        if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7) == 1)
        {
            if(Huadong_State==0)
            { Forward_State=0;Back_State=0;Left_State=0;Right_State=0;Huadong_State=1;Huadong_Step=0;step_delay=0; }
            huadong_release_flag=1;
        }
        else
        {
            if(huadong_release_flag==1)
            {
                Angle1=135;Angle2=45;Angle3=45;Angle4=135;Angle5=0;Angle6=180;Angle7=0;Angle8=180;
                Servo_SetAngle1(Angle1);Servo_SetAngle2(Angle2);
                Servo_SetAngle3(Angle3);Servo_SetAngle4(Angle4);
                Servo_SetAngle5(Angle5);Servo_SetAngle6(Angle6);
                Servo_SetAngle7(Angle7);Servo_SetAngle8(Angle8);
                huadong_release_flag=0;
            }
            Huadong_State=0;
        }

        if(Huadong_State!=0) Huadong_Smooth();

        Delay_ms(25);
    }
}
