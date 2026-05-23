/**
 * ============================================================================
 * 主程序 - 四足机器人控制系统
 * ============================================================================
 * 功能概述：
 *   1. 8路舵机驱动（4条腿，每条腿2个舵机）
 *   2. 串口指令控制（前进/后退/左转/右转/各种动作）
 *   3. BH1750 光照传感器读取
 *   4. LM2904 噪声传感器读取
 *   5. HC-SR04 超声波测距 + 自动避障
 *   6. 红外传感器离地检测
 *   7. OLED 显示传感器数据和运行状态
 *   8. LED 状态指示
 *
 * 舵机分配：
 *   舵机1/2 - 左前腿（抬腿/摆腿）
 *   舵机3/4 - 右前腿（抬腿/摆腿）
 *   舵机5/6 - 左后腿（抬腿/摆腿）
 *   舵机7/8 - 右后腿（抬腿/摆腿）
 *
 * 硬件引脚：
 *   PB12/PB13/PB14 - BH1750 光照传感器 (I2C)
 *   PA5            - LM2904 噪声传感器 (ADC)
 *   PB11           - HC-SR04 超声波 TRIG
 *   PB10           - HC-SR04 超声波 ECHO
 *   PB7            - 红外避障传感器 (GPIO输入)
 *   PA12/PA15      - LED 指示灯
 * ============================================================================
 */

/* ======================== 头文件包含 ======================== */
#include "stm32f10x.h"       // STM32F10x 标准外设库
#include "Delay.h"            // 延时函数（Delay_us/Delay_ms/Delay_s）
#include "OLED.h"             // OLED 显示屏驱动
#include "LED.h"              // LED 指示灯驱动
#include "PWM.h"              // PWM 输出驱动
#include "Input.h"            // 输入检测驱动
#include "Servo.h"            // 舵机控制驱动（8路舵机）
#include "Serial.h"           // 串口通信驱动
#include <string.h>           // 字符串操作（memset/strncasecmp）
#include <stdlib.h>           // 标准库
#include <stdio.h>            // 标准输入输出
#include <math.h>             // 数学函数
#include "bh1750.h"           // BH1750 光照传感器驱动
#include "noise_sensor.h"     // LM2904 噪声传感器驱动
#include "hcsr04.h"           // HC-SR04 超声波传感器驱动

/* ======================== 外部变量声明 ======================== */
extern char Serial_RxPacket[64];    // 串口接收数据缓冲区
extern uint8_t Serial_RxFlag;       // 串口接收完成标志（1=收到完整数据包）

/* 图片资源声明（用于OLED显示图片） */
volatile uint8_t MAGE_State = 0;    // 图片显示状态
extern const unsigned char BiLi[];  // 图片数据数组
extern const unsigned char DOUYIN[];// 图片数据数组
extern const unsigned char LiC[];   // 图片数据数组

/* ======================== 舵机角度变量 ======================== */
/*
 * 每条腿有2个舵机：
 *   - 腿1（左前）：舵机1（抬腿）+ 舵机5（摆腿）
 *   - 腿2（右前）：舵机3（抬腿）+ 舵机7（摆腿）
 *   - 腿3（左后）：舵机2（抬腿）+ 舵机6（摆腿）
 *   - 腿4（右后）：舵机4（抬腿）+ 舵机8（摆腿）
 *
 * 角度范围：0 ~ 180 度
 * 90度为中间位置
 */
volatile float Angle1 = 90.0f;     // 舵机1 - 左前抬腿
volatile float Angle2 = 90.0f;     // 舵机2 - 左后抬腿
volatile float Angle3 = 90.0f;     // 舵机3 - 右前抬腿
volatile float Angle4 = 90.0f;     // 舵机4 - 右后抬腿
volatile float Angle5 = 0.0f;      // 舵机5 - 左前摆腿
volatile float Angle6 = 180.0f;    // 舵机6 - 左后摆腿
volatile float Angle7 = 0.0f;      // 舵机7 - 右前摆腿
volatile float Angle8 = 180.0f;    // 舵机8 - 右后摆腿

/* ======================== 动作状态标志 ======================== */
volatile uint8_t A4_State = 0;     // 招手动作切换状态（0=初始 1=招手中）
volatile uint8_t A5_State = 0;     // 抬脚动作切换状态（0=初始 1=抬脚中）
volatile uint8_t XNLED_State = 0;  // 虚拟LED状态
volatile uint8_t ZMLED_State = 0;  // 照明LED状态
volatile uint8_t HW_State = 0;     // 红外状态

/* ======================== 左右摇摆动作控制 ======================== */
/*
 * 摇摆动作通过4个状态循环执行：
 *   状态1 → 左右腿同时向内收
 *   状态2 → 左右腿向外展
 *   状态3 → 左右腿再向内收
 *   状态4 → 左右腿再向外展
 *   然后回到状态1，形成循环摇摆
 */
volatile uint8_t ActionState = 0;              // 摇摆动作当前状态（0=未运行, 1~4=执行中）
volatile uint16_t ActionDelayCnt = 0;          // 摇摆动作延时计数器
const uint16_t ACTION_DELAY_TIME = 12;         // 摇摆动作每次执行间隔（单位：主循环周期）
const float ServoSwingStep = 2.0f;             // 每次舵机角度变化步长（度）

/* ======================== 前进动作控制 ======================== */
/*
 * 前进动作通过4步循环实现四足步态：
 *   步骤1 → 初始姿态，准备迈步
 *   步骤2 → 抬起前腿
 *   步骤3 → 前腿前摆
 *   步骤4 → 放下前腿，身体前移
 *   回到步骤1，循环前进
 */
volatile uint8_t Forward_State = 0;            // 前进状态（0=停止, 1=前进中）
volatile uint16_t Forward_DelayCnt = 0;        // 前进延时计数器
const uint16_t FORWARD_DELAY_TIME = 12;        // 前进步骤间隔
volatile uint8_t Forward_Step = 0;             // 前进当前步骤（1~4）

/* ======================== 后退动作控制 ======================== */
volatile uint8_t Back_State = 0;               // 后退状态（0=停止, 1=后退中）
volatile uint16_t Back_DelayCnt = 0;           // 后退延时计数器
const uint16_t BACK_DELAY_TIME = 12;           // 后退步骤间隔
volatile uint8_t Back_Step = 0;                // 后退当前步骤（1~4）

/* ======================== 左转动作控制 ======================== */
volatile uint8_t Left_State = 0;               // 左转状态（0=停止, 1=左转中）
volatile uint16_t Left_DelayCnt = 0;           // 左转延时计数器
const uint16_t LEFT_DELAY_TIME = 12;           // 左转步骤间隔
volatile uint8_t Left_Step = 0;                // 左转当前步骤（1~4）

/* ======================== 右转动作控制 ======================== */
volatile uint8_t Right_State = 0;              // 右转状态（0=停止, 1=右转中）
volatile uint16_t Right_DelayCnt = 0;          // 右转延时计数器
const uint16_t RIGHT_DELAY_TIME = 12;          // 右转步骤间隔
volatile uint8_t Right_Step = 0;               // 右转当前步骤（1~4）

/* ======================== BH1750 光照传感器 ======================== */
/*
 * BH1750 通过 I2C 协议通信（软件模拟I2C）
 * 接口：PB12(SCL) / PB13(SDA) / PB14(ADDR)
 * 测量范围：1 ~ 65535 lux
 * 读取间隔：每40个主循环周期读一次（约1秒）
 */
volatile uint16_t bh1750_cnt = 0;              // BH1750 读取计数器
const uint16_t BH1750_READ_INTERVAL = 40;      // 读取间隔（单位：主循环周期）
float lux_val = 0;                             // 当前光照值（单位：lux）

/* ======================== LM2904 噪声传感器 ======================== */
/*
 * LM2904 通过 ADC 采集模拟信号
 * 接口：PA5 (ADC通道5)
 * 采集方式：DMA连续采集 + 去极值滤波
 * 读取间隔：每40个主循环周期读一次
 */
volatile uint16_t noise_cnt = 0;               // 噪声读取计数器
const uint16_t NOISE_READ_INTERVAL = 40;       // 读取间隔
NoiseData_TypeDef noise_data;                  // 噪声数据结构体（含ADC值、电压、分贝、等级）

/* ======================== HC-SR04 超声波传感器 ======================== */
/*
 * HC-SR04 工作原理：
 *   1. 向 TRIG 引脚发送 >=10us 的高电平脉冲
 *   2. 模块自动发送8个40kHz超声波脉冲
 *   3. ECHO 引脚返回高电平，持续时间 = 超声波往返时间
 *   4. 距离 = 时间 * 声速 / 2
 *
 * 接口：PB11(TRIG输出) / PB10(ECHO输入)
 * 有效范围：2 ~ 400 cm
 * 读取间隔：每40个主循环周期读一次
 */
volatile uint16_t hcsr04_cnt = 0;              // 超声波读取计数器
const uint16_t HCSR04_READ_INTERVAL = 40;      // 读取间隔
float dist_val = 0;                            // 当前距离值（单位：cm）

/* ======================== 自动避障模式 ======================== */
/*
 * 自动避障逻辑：
 *   - 开机默认进入自动模式（auto_mode = 1）
 *   - 距离 < 10cm → 停止前进，自动左转避障
 *   - 距离 >= 10cm → 停止左转，恢复前进
 *   - 串口发送 "MANU" 可切换为手动模式
 *   - 串口发送 "AUTO" 可切换回自动模式
 */
volatile uint8_t auto_mode = 1;                // 0=手动模式  1=自动避障模式
#define DIST_THRESHOLD  10.0f                   // 避障距离阈值（单位：cm）

/* ======================== LED 闪烁控制 ======================== */
volatile uint8_t LED_State = 0;                // LED状态（0=不闪烁, 1/2=交替闪烁）
volatile uint16_t LED_DelayCnt = 0;            // LED闪烁延时计数器
const uint16_t LED_DELAY_TIME = 12;            // LED闪烁间隔

/* ======================== 离地检测滑动控制 ======================== */
/*
 * 离地检测使用红外传感器（PB7）
 * 当机器人被拿离地面时，红外传感器检测到信号
 * 自动进入滑动动作模式（腿部展开收回循环）
 * 放下后自动恢复初始姿态
 */
volatile uint8_t Huadong_State = 0;            // 离地滑动状态（0=未触发, 1=展开, 2=收回）
volatile uint8_t Huadong_Step = 0;             // 滑动当前步数（0~MAX_STEP）
const uint8_t MAX_STEP = 15;                   // 滑动最大步数（腿展开的最大幅度）
const uint8_t STEP_INTERVAL = 1;               // 步进间隔
volatile uint8_t step_delay = 0;               // 步进延时计数
volatile uint8_t huadong_release_flag = 0;     // 离地释放标志（检测到放下瞬间）

uint16_t booOffCnt = 0;                        // 蜂鸣器关闭计数

/**
 * ============================================================================
 * 函数名：ResetAll
 * 功能：  重置所有状态到初始值
 * 说明：  恢复舵机初始角度，清除所有动作状态，退出自动模式
 * ============================================================================
 */
void ResetAll(void)
{
    /* 舵机回到初始站立姿态 */
    Angle1 = 100.0f;   // 左前抬腿 - 略微抬起
    Angle2 = 80.0f;    // 左后抬腿 - 略微放下
    Angle3 = 90.0f;    // 右前抬腿 - 中间
    Angle4 = 90.0f;    // 右后抬腿 - 中间
    Angle5 = 0.0f;     // 左前摆腿 - 最左
    Angle6 = 180.0f;   // 左后摆腿 - 最右
    Angle7 = 0.0f;     // 右前摆腿 - 最左
    Angle8 = 180.0f;   // 右后摆腿 - 最右

    /* 清除所有动作状态 */
    ActionState = 0;          // 清除摇摆状态
    ActionDelayCnt = 0;       // 清除摇摆计数
    A4_State = 0;             // 清除招手状态
    A5_State = 0;             // 清除抬脚状态
    XNLED_State = 0;          // 清除虚拟LED状态
    MAGE_State = 0;           // 清除图片显示状态

    /* 清除移动状态 */
    Forward_State = 0;        // 停止前进
    Forward_Step = 0;         // 清除前进步骤
    Back_State = 0;           // 停止后退
    Back_Step = 0;            // 清除后退步骤
    Left_State = 0;           // 停止左转
    Left_Step = 0;            // 清除左转步骤
    Right_State = 0;          // 停止右转
    Right_Step = 0;           // 清除右转步骤

    /* 保持自动避障模式（不重置） */
    auto_mode = 1;            // 保持自动模式
}

/**
 * ============================================================================
 * 函数名：Huadong_Smooth
 * 功能：  离地滑动动作执行（平滑动画）
 * 说明：  当机器人被拿离地面时，自动执行腿部展开/收回的循环动作
 *         展开过程：步数从0递增到MAX_STEP，腿部逐渐展开
 *         收回过程：步数从MAX_STEP递减到0，腿部逐渐收回
 *         两腿交替往复，形成滑动效果
 * ============================================================================
 */
void Huadong_Smooth(void)
{
    /* 首先将身体舵机固定为标准站立姿态 */
    Angle1 = 135.0f;  // 左前抬腿
    Angle2 = 45.0f;   // 左后抬腿
    Angle3 = 45.0f;   // 右前抬腿
    Angle4 = 135.0f;  // 右后抬腿
    Servo_SetAngle1(Angle1);
    Servo_SetAngle2(Angle2);
    Servo_SetAngle3(Angle3);
    Servo_SetAngle4(Angle4);

    /* 步进延时控制（控制动画速度） */
    step_delay++;
    if (step_delay < STEP_INTERVAL) return;  // 还没到执行时间，直接返回
    step_delay = 0;                          // 到达执行时间，重置计数器

    /* 根据当前状态执行展开或收回 */
    switch(Huadong_State)
    {
        case 1:  /* 展开状态：步数递增，腿部逐渐展开 */
            Huadong_Step++;
            if (Huadong_Step >= MAX_STEP)
            {
                Huadong_Step = MAX_STEP;     // 到达最大展开
                Huadong_State = 2;           // 切换到收回状态
            }
            break;

        case 2:  /* 收回状态：步数递减，腿部逐渐收回 */
            Huadong_Step--;
            if (Huadong_Step <= 0)
            {
                Huadong_Step = 0;            // 完全收回
                Huadong_State = 1;           // 切换到展开状态
            }
            break;

        default:
            return;  // 状态异常，不执行
    }

    /* 根据当前步数计算4个摆腿舵机的角度 */
    /* 步数越大，展开越大（摆腿角度偏离中心越多） */
    Angle5 = Huadong_Step * 2;               // 左前摆腿：0 ~ 30度
    Angle7 = Huadong_Step * 2;               // 右前摆腿：0 ~ 30度
    Angle6 = 180 - (Huadong_Step * 2);       // 左后摆腿：180 ~ 150度
    Angle8 = 180 - (Huadong_Step * 2);       // 右后摆腿：180 ~ 150度

    /* 设置舵机角度 */
    Servo_SetAngle5(Angle5);
    Servo_SetAngle6(Angle6);
    Servo_SetAngle7(Angle7);
    Servo_SetAngle8(Angle8);
}

/**
 * ============================================================================
 * 函数名：main
 * 功能：  主函数，程序入口
 * 说明：  初始化所有外设，进入主循环
 *         主循环中依次处理：
 *           1. 舵机输出
 *           2. 串口指令解析
 *           3. 超声波避障逻辑
 *           4. 各动作执行（摇摆/前进/后退/左转/右转）
 *           5. 传感器数据采集（光照/噪声/超声波）
 *           6. OLED 显示更新
 *           7. LED 闪烁控制
 *           8. 离地检测处理
 *         主循环周期约 25ms
 * ============================================================================
 */
int main(void)
{
    /* ==================== 系统初始化 ==================== */
    Servo_Init();              // 初始化8路舵机（PWM输出）
    Serial_Init();             // 初始化串口通信（UART）
    Input_Init();              // 初始化输入检测（红外传感器等）
    OLED_Init();               // 初始化OLED显示屏（I2C/SPI）
    LED_Init();                // 初始化LED指示灯（PA12/PA15）
    LED_StartBlink();          // 启动LED闪烁（表示系统运行中）
    bh1750_init();             // 初始化BH1750光照传感器（PB12/PB13/PB14）
    Noise_Sensor_Init();       // 初始化LM2904噪声传感器（PA5 ADC）
    hcsr04_init();             // 初始化HC-SR04超声波传感器（PB11 TRIG / PB10 ECHO）
    ResetAll();                // 重置所有状态到初始值

    /* ==================== 主循环 ==================== */
    while (1)
    {
        /**
         * ============================================================
         * 1. 舵机实时输出
         * ============================================================
         * 每次循环都将当前角度值写入舵机
         * 确保动作变化能实时反映到舵机上
         */
        Servo_SetAngle1(Angle1);   // 左前抬腿
        Servo_SetAngle2(Angle2);   // 左后抬腿
        Servo_SetAngle3(Angle3);   // 右前抬腿
        Servo_SetAngle4(Angle4);   // 右后抬腿
        Servo_SetAngle5(Angle5);   // 左前摆腿
        Servo_SetAngle6(Angle6);   // 左后摆腿
        Servo_SetAngle7(Angle7);   // 右前摆腿
        Servo_SetAngle8(Angle8);   // 右后摆腿

        /**
         * ============================================================
         * 2. 串口指令解析处理
         * ============================================================
         * 串口数据包格式：字符串指令（如 "UP", "BACK", "LEFT" 等）
         * Serial_RxFlag == 1 表示收到完整数据包
         * 解析完成后清空缓冲区和标志位
         *
         * 指令列表：
         *   "REST"  - 重置所有状态
         *   "AUTO"  - 切换到自动避障模式
         *   "MANU"  - 切换到手动控制模式
         *   "ZK"    - 展开（站立姿态）
         *   "ZX"    - 坐下
         *   "YB"    - 左右摇摆
         *   "ZS"    - 招手
         *   "TJ"    - 抬脚
         *   "UP"    - 前进
         *   "BACK"  - 后退
         *   "LEFT"  - 左转
         *   "RIGHT" - 右转
         */
        if (Serial_RxFlag == 1)
        {
            /* --- 重置指令 --- */
            if (strncasecmp(Serial_RxPacket, "REST", 4) == 0)
            {
                ResetAll();                           // 恢复初始状态
                Serial_SendString("OK\r\n");          // 回复确认
            }
            /* --- 自动避障模式开启 --- */
            else if (strncasecmp(Serial_RxPacket, "AUTO", 4) == 0)
            {
                auto_mode = 1;                        // 开启自动模式
                ActionState = 0;                      // 停止摇摆
                Back_State = 0;                       // 停止后退
                Right_State = 0;                      // 停止右转
                A4_State = 0;                         // 清除招手
                A5_State = 0;                         // 清除抬脚
                Forward_State = 1;                    // 自动模式下默认前进
                Forward_Step = 1;                     // 从第一步开始
                Forward_DelayCnt = 0;                 // 重置延时计数
                Serial_SendString("AUTO ON\r\n");
            }
            /* --- 手动控制模式开启 --- */
            else if (strncasecmp(Serial_RxPacket, "MANU", 4) == 0)
            {
                auto_mode = 0;                        // 关闭自动模式
                ResetAll();                           // 重置所有动作
                Serial_SendString("MANUAL ON\r\n");
            }
            /* --- 展开指令（站立姿态）--- */
            else if (strncasecmp(Serial_RxPacket, "ZK", 2) == 0)
            {
                if (auto_mode == 0)                   // 仅手动模式下有效
                {
                    ActionState = 0;                  // 停止摇摆
                    Forward_State = 0;                // 停止前进
                    Back_State = 0;                   // 停止后退
                    Left_State = 0;                   // 停止左转
                    Right_State = 0;                  // 停止右转
                    /* 设置标准站立姿态 */
                    Angle1 = 135; Angle2 = 45;        // 抬腿舵机：标准站立
                    Angle3 = 45;  Angle4 = 135;
                    Angle5 = 0;   Angle6 = 180;       // 摆腿舵机：展开
                    Angle7 = 0;   Angle8 = 180;
                    A4_State = 0; A5_State = 0;       // 清除动作状态
                }
            }
            /* --- 坐下指令 --- */
            else if (strncasecmp(Serial_RxPacket, "ZX", 2) == 0)
            {
                if (auto_mode == 0)
                {
                    ActionState = 0;
                    Forward_State = 0;
                    Back_State = 0;
                    Left_State = 0;
                    Right_State = 0;
                    /* 设置坐下姿态：摆腿舵机收回到90度 */
                    Angle1 = 135; Angle2 = 45;
                    Angle3 = 45;  Angle4 = 135;
                    Angle5 = 90;  Angle6 = 90;       // 摆腿收回（坐下）
                    Angle7 = 90;  Angle8 = 90;
                    A4_State = 0; A5_State = 0;
                }
            }
            /* --- 摇摆指令（来回摇摆身体）--- */
            else if (strncasecmp(Serial_RxPacket, "YB", 2) == 0)
            {
                if (auto_mode == 0)
                {
                    ActionState = 1;                  // 启动摇摆动作（从状态1开始）
                    Forward_State = 0;                // 停止前进
                    Back_State = 0;                   // 停止后退
                    Left_State = 0;                   // 停止左转
                    Right_State = 0;                  // 停止右转
                    ActionDelayCnt = 0;               // 重置摇摆计时
                    A4_State = 0; A5_State = 0;
                }
            }
            /* --- 招手指令（抬起前腿招手）--- */
            else if (strncasecmp(Serial_RxPacket, "ZS", 2) == 0)
            {
                if (auto_mode == 0)
                {
                    ActionState = 0;
                    Forward_State = 0;
                    Back_State = 0;
                    Left_State = 0;
                    Right_State = 0;
                    if (A4_State == 0)
                    {
                        /* 招手姿态：右前腿抬起，身体侧倾 */
                        Angle1 = 160; Angle2 = 45;
                        Angle3 = 110; Angle4 = 70;
                        Angle5 = 20;  Angle6 = 35;
                        Angle7 = 20;  Angle8 = 160;
                        A4_State = 1;                 // 标记为已招手
                    }
                    else
                    {
                        /* 恢复站立 */
                        Angle1 = 135; Angle2 = 45;
                        Angle3 = 45;  Angle4 = 135;
                        Angle5 = 0;   Angle6 = 180;
                        Angle7 = 0;   Angle8 = 180;
                        A4_State = 0;                 // 清除招手状态
                    }
                }
            }
            /* --- 抬脚指令（抬起一条前腿）--- */
            else if (strncasecmp(Serial_RxPacket, "TJ", 2) == 0)
            {
                if (auto_mode == 0)
                {
                    ActionState = 0;
                    Forward_State = 0;
                    Back_State = 0;
                    Left_State = 0;
                    Right_State = 0;
                    if (A5_State == 0)
                    {
                        /* 抬脚姿态：左前腿抬起，右后腿抬起 */
                        Angle1 = 90;  Angle2 = 90;
                        Angle3 = 45;  Angle4 = 135;
                        Angle5 = 0;   Angle6 = 180;
                        Angle7 = 135; Angle8 = 180;
                        A5_State = 1;                 // 标记为已抬脚
                    }
                    else
                    {
                        /* 恢复站立 */
                        Angle1 = 135; Angle2 = 45;
                        Angle3 = 45;  Angle4 = 135;
                        Angle5 = 0;   Angle6 = 180;
                        Angle7 = 0;   Angle8 = 180;
                        A5_State = 0;                 // 清除抬脚状态
                    }
                }
            }
            /* --- 前进指令 --- */
            else if (strncasecmp(Serial_RxPacket, "UP", 2) == 0)
            {
                if (auto_mode == 0)
                {
                    ActionState = 0;                  // 停止摇摆
                    Back_State = 0;                   // 停止后退
                    Left_State = 0;                   // 停止左转
                    Right_State = 0;                  // 停止右转
                    Forward_State = 1;                // 启动前进
                    Forward_Step = 1;                 // 从第一步开始
                    Forward_DelayCnt = 0;             // 重置计时
                    A4_State = 0; A5_State = 0;       // 清除其他动作
                }
            }
            /* --- 后退指令 --- */
            else if (strncasecmp(Serial_RxPacket, "BACK", 4) == 0)
            {
                if (auto_mode == 0)
                {
                    ActionState = 0;
                    Forward_State = 0;
                    Left_State = 0;
                    Right_State = 0;
                    Back_State = 1;                   // 启动后退
                    Back_Step = 1;
                    Back_DelayCnt = 0;
                    A4_State = 0; A5_State = 0;
                }
            }
            /* --- 左转指令 --- */
            else if (strncasecmp(Serial_RxPacket, "LEFT", 4) == 0)
            {
                if (auto_mode == 0)
                {
                    ActionState = 0;
                    Forward_State = 0;
                    Back_State = 0;
                    Right_State = 0;
                    Left_State = 1;                   // 启动左转
                    Left_Step = 1;
                    Left_DelayCnt = 0;
                    A4_State = 0; A5_State = 0;
                }
            }
            /* --- 右转指令 --- */
            else if (strncasecmp(Serial_RxPacket, "RIGHT", 5) == 0)
            {
                if (auto_mode == 0)
                {
                    ActionState = 0;
                    Forward_State = 0;
                    Back_State = 0;
                    Left_State = 0;
                    Right_State = 1;                  // 启动右转
                    Right_Step = 1;
                    Right_DelayCnt = 0;
                    A4_State = 0; A5_State = 0;
                }
            }

            /* 清空串口接收缓冲区和标志位，准备接收下一条指令 */
            memset(Serial_RxPacket, 0, sizeof(Serial_RxPacket));
            Serial_RxFlag = 0;
        }

        /**
         * ============================================================
         * 3. 超声波避障自动控制逻辑
         * ============================================================
         * 仅在自动模式（auto_mode == 1）下生效
         * 判断逻辑：
         *   距离有效（> 0）且 < 10cm → 有障碍物 → 自动左转
         *   距离 >= 10cm 或传感器异常 → 无障碍 → 恢复前进
         */
        if (auto_mode == 1)
        {
            if (dist_val > 0 && dist_val < DIST_THRESHOLD)
            {
                /*
                 * 距离 < 10cm：前方有障碍物
                 * 停止前进和所有其他动作，切换到左转
                 */
                if (Left_State == 0)                  // 当前不在左转状态才切换
                {
                    Forward_State = 0;                // 停止前进
                    Back_State = 0;                   // 停止后退
                    Right_State = 0;                  // 停止右转
                    ActionState = 0;                  // 停止摇摆
                    Left_State = 1;                   // 启动左转
                    Left_Step = 1;                    // 从第一步开始
                    Left_DelayCnt = 0;                // 重置计时
                }
            }
            else if (dist_val >= DIST_THRESHOLD || dist_val < 0)
            {
                /*
                 * 距离 >= 10cm：前方无障碍
                 * 停止左转和其他动作，恢复前进
                 * dist_val < 0 表示传感器异常（超时），也恢复前进
                 */
                if (Forward_State == 0)               // 当前不在前进状态才切换
                {
                    Left_State = 0;                   // 停止左转
                    Back_State = 0;                   // 停止后退
                    Right_State = 0;                  // 停止右转
                    ActionState = 0;                  // 停止摇摆
                    Forward_State = 1;                // 恢复前进
                    Forward_Step = 1;                 // 从第一步开始
                    Forward_DelayCnt = 0;             // 重置计时
                }
            }
        }

        /**
         * ============================================================
         * 4. 左右摇摆动作执行
         * ============================================================
         * ActionState != 0 表示摇摆动作正在运行
         * 4个状态循环执行，控制摆腿舵机往复运动
         */
        if (ActionState != 0)
        {
            ActionDelayCnt++;                         // 延时计数递增

            switch(ActionState)
            {
                case 1:
                    /* 状态1：左侧腿向内收，右侧腿向内收 */
                    if (Angle5 > 0)   Angle5 -= ServoSwingStep;   // 左前摆腿向内
                    if (Angle7 > 0)   Angle7 -= ServoSwingStep;   // 右前摆腿向内
                    if (Angle6 < 180) Angle6 += ServoSwingStep;   // 左后摆腿向内
                    if (Angle8 < 180) Angle8 += ServoSwingStep;   // 右后摆腿向内
                    if (ActionDelayCnt >= ACTION_DELAY_TIME)
                    { ActionState = 2; ActionDelayCnt = 0; }      // 切换到状态2
                    break;

                case 2:
                    /* 状态2：所有腿向外展开 */
                    if (Angle5 < 40)  Angle5 += ServoSwingStep;   // 左前摆腿向外
                    if (Angle7 < 40)  Angle7 += ServoSwingStep;   // 右前摆腿向外
                    if (ActionDelayCnt >= ACTION_DELAY_TIME)
                    { ActionState = 3; ActionDelayCnt = 0; }      // 切换到状态3
                    break;

                case 3:
                    /* 状态3：所有腿向内收回 */
                    if (Angle5 > 0)   Angle5 -= ServoSwingStep;   // 左前摆腿向内
                    if (Angle7 > 0)   Angle7 -= ServoSwingStep;   // 右前摆腿向内
                    if (ActionDelayCnt >= ACTION_DELAY_TIME)
                    { ActionState = 4; ActionDelayCnt = 0; }      // 切换到状态4
                    break;

                case 4:
                    /* 状态4：后腿向外展开 */
                    if (Angle6 > 140) Angle6 -= ServoSwingStep;   // 左后摆腿向外
                    if (Angle8 > 140) Angle8 -= ServoSwingStep;   // 右后摆腿向外
                    if (ActionDelayCnt >= ACTION_DELAY_TIME)
                    { ActionState = 1; ActionDelayCnt = 0; }      // 回到状态1，循环
                    break;

                default:
                    /* 异常状态：清除摇摆 */
                    ActionState = 0;
                    ActionDelayCnt = 0;
                    break;
            }
        }

        /**
         * ============================================================
         * 5. 前进动作执行
         * ============================================================
         * 四足步态前进，4步一个循环：
         *   步骤1：准备姿态，后腿摆动
         *   步骤2：右前腿抬起
         *   步骤3：右前腿前摆
         *   步骤4：身体前移，回到准备姿态
         */
        if (Forward_State != 0)
        {
            Forward_DelayCnt++;
            /* 到达时间间隔，切换到下一步 */
            if (Forward_DelayCnt >= FORWARD_DELAY_TIME)
            {
                Forward_DelayCnt = 0;
                Forward_Step++;
                if (Forward_Step > 4) Forward_Step = 1;  // 循环到第一步
            }
            /* 根据当前步骤设置舵机角度 */
            switch(Forward_Step)
            {
                case 1:  /* 步骤1：准备姿态 */
                    Angle1=135; Angle2=45;  // 抬腿舵机
                    Angle3=45;  Angle4=135;
                    Angle5=0;   Angle6=150;  // 左后腿后摆
                    Angle7=30;  Angle8=180;  // 右前腿前移
                    break;
                case 2:  /* 步骤2：抬起右前腿 */
                    Angle1=135; Angle2=15;  // 左后腿抬起
                    Angle3=75;  Angle4=135;
                    Angle5=0;   Angle6=180;
                    Angle7=0;   Angle8=180;
                    break;
                case 3:  /* 步骤3：右前腿前摆 */
                    Angle1=135; Angle2=15;
                    Angle3=75;  Angle4=135;
                    Angle5=30;  Angle6=180;  // 左前腿前摆
                    Angle7=0;   Angle8=150;
                    break;
                case 4:  /* 步骤4：身体前移 */
                    Angle1=165; Angle2=45;  // 左前腿前移
                    Angle3=45;  Angle4=105;  // 右后腿前移
                    Angle5=0;   Angle6=180;
                    Angle7=0;   Angle8=180;
                    break;
            }
        }

        /**
         * ============================================================
         * 6. 后退动作执行
         * ============================================================
         * 与前进动作类似，但方向相反
         */
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
                    Angle1=135; Angle2=45;  Angle3=45;  Angle4=135;
                    Angle5=0;   Angle6=150;  Angle7=30;  Angle8=180;
                    break;
                case 2:  /* 与前进相反：左后腿抬起 */
                    Angle1=135; Angle2=75;  Angle3=15;  Angle4=135;
                    Angle5=0;   Angle6=180;  Angle7=0;   Angle8=180;
                    break;
                case 3:
                    Angle1=135; Angle2=75;  Angle3=15;  Angle4=135;
                    Angle5=30;  Angle6=180;  Angle7=0;   Angle8=150;
                    break;
                case 4:  /* 身体后移 */
                    Angle1=105; Angle2=45;  Angle3=45;  Angle4=165;
                    Angle5=0;   Angle6=180;  Angle7=0;   Angle8=180;
                    break;
            }
        }

        /**
         * ============================================================
         * 7. 左转动作执行
         * ============================================================
         * 左转通过不对称步态实现：左侧腿步幅大，右侧腿步幅小
         */
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
                case 1:  /* 左侧腿大幅展开 */
                    Angle1=150; Angle2=30;  // 左侧腿幅度大
                    Angle3=30;  Angle4=150;
                    Angle5=30;  Angle6=180;
                    Angle7=0;   Angle8=150;
                    break;
                case 2:  /* 收回准备 */
                    Angle1=165; Angle2=15;
                    Angle3=15;  Angle4=165;
                    Angle5=0;   Angle6=180;
                    Angle7=0;   Angle8=180;
                    break;
                case 3:  /* 右侧腿小幅摆动 */
                    Angle1=165; Angle2=15;
                    Angle3=15;  Angle4=165;
                    Angle5=0;   Angle6=150;
                    Angle7=30;  Angle8=180;
                    break;
                case 4:  /* 回到标准站立 */
                    Angle1=135; Angle2=45;
                    Angle3=45;  Angle4=135;
                    Angle5=0;   Angle6=180;
                    Angle7=0;   Angle8=180;
                    break;
            }
        }

        /**
         * ============================================================
         * 8. 右转动作执行
         * ============================================================
         * 与左转对称，右侧腿步幅大
         */
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
                case 1:  /* 右侧腿大幅展开 */
                    Angle1=150; Angle2=30;
                    Angle3=30;  Angle4=150;
                    Angle5=0;   Angle6=150;
                    Angle7=30;  Angle8=180;
                    break;
                case 2:
                    Angle1=165; Angle2=15;
                    Angle3=15;  Angle4=165;
                    Angle5=0;   Angle6=180;
                    Angle7=0;   Angle8=180;
                    break;
                case 3:  /* 左侧腿小幅摆动 */
                    Angle1=165; Angle2=15;
                    Angle3=15;  Angle4=165;
                    Angle5=30;  Angle6=180;
                    Angle7=0;   Angle8=150;
                    break;
                case 4:
                    Angle1=135; Angle2=45;
                    Angle3=45;  Angle4=135;
                    Angle5=0;   Angle6=180;
                    Angle7=0;   Angle8=180;
                    break;
            }
        }

        /**
         * ============================================================
         * 9. 传感器数据采集
         * ============================================================
         * 三个传感器各自独立定时采集，互不干扰
         * 采集间隔由各自的 INTERVAL 常量控制
         */

        /* --- BH1750 光照传感器采集 --- */
        bh1750_cnt++;
        if (bh1750_cnt >= BH1750_READ_INTERVAL)
        {
            bh1750_cnt = 0;                           // 重置计数器
            lux_val = bh1750_get_lux();               // 通过I2C读取光照值
        }

        /* --- LM2904 噪声传感器采集 --- */
        noise_cnt++;
        if (noise_cnt >= NOISE_READ_INTERVAL)
        {
            noise_cnt = 0;                            // 重置计数器
            Noise_Sensor_GetData(&noise_data);        // 通过ADC读取噪声数据
        }

        /* --- HC-SR04 超声波传感器采集 --- */
        hcsr04_cnt++;
        if (hcsr04_cnt >= HCSR04_READ_INTERVAL)
        {
            hcsr04_cnt = 0;                           // 重置计数器
            dist_val = hcsr04_get_distance();         // 发送超声波脉冲并计算距离
        }

        /**
         * ============================================================
         * 10. OLED 显示更新
         * ============================================================
         * 每4个主循环周期（约100ms）刷新一次OLED显示
         * 显示内容：
         *   第一行：光照值 + 距离
         *   第二行：噪声分贝
         *   第三行：当前模式和状态
         */
        {
            static uint8_t oled_cnt = 0;              // OLED刷新计数器（静态变量，保持值）
            oled_cnt++;
            if (oled_cnt >= 4)                        // 每4个周期刷新一次
            {
                oled_cnt = 0;

                OLED_Clear();                         // 清屏

                /* 第一行：光照值和超声波距离 */
                OLED_Printf(0, 0, 8, "Lux:%.0f D:%.0fcm",
                    lux_val,
                    dist_val > 0 ? dist_val : 0);     // 传感器异常时显示0

                /* 第二行：噪声分贝值 */
                OLED_Printf(0, 16, 8, "dB:%.1f", noise_data.db_value);

                /* 第三行：运行模式和当前状态 */
                if (auto_mode)
                {
                    /* 自动模式下显示避障状态 */
                    if (Left_State != 0)
                        OLED_Printf(0, 32, 8, "AUTO: Turn Left");   // 正在避障左转
                    else
                        OLED_Printf(0, 32, 8, "AUTO: Forward");     // 正在前进
                }
                else
                {
                    /* 手动模式 */
                    OLED_Printf(0, 32, 8, "Mode: Manual");
                }

                OLED_Update();                        // 将显示缓冲区数据写入OLED
            }
        }

        /**
         * ============================================================
         * 11. LED 闪烁控制
         * ============================================================
         * LED_State != 0 时，PA12 和 PA15 交替闪烁
         * 状态1：PA12 亮 / PA15 灭 → 切换到状态2
         * 状态2：PA12 灭 / PA15 亮 → 切换到状态1
         */
        if (LED_State != 0)
        {
            LED_DelayCnt++;
            if (LED_DelayCnt >= LED_DELAY_TIME)
            {
                LED_DelayCnt = 0;
                if (LED_State == 1)
                {
                    GPIO_ResetBits(GPIOA, GPIO_Pin_12);   // PA12 亮
                    GPIO_SetBits(GPIOA, GPIO_Pin_15);     // PA15 灭
                    LED_State = 2;                         // 切换到状态2
                }
                else
                {
                    GPIO_SetBits(GPIOA, GPIO_Pin_12);     // PA12 灭
                    GPIO_ResetBits(GPIOA, GPIO_Pin_15);   // PA15 亮
                    LED_State = 1;                         // 切换到状态1
                }
            }
        }

        /**
         * ============================================================
         * 12. 离地检测 + 滑动动作控制
         * ============================================================
         * 红外传感器（PB7）检测机器人是否离开地面：
         *   PB7 == 1 → 机器人被拿起（离地）
         *     → 停止所有动作，启动滑动模式
         *   PB7 == 0 → 机器人放在地面上
         *     → 恢复站立姿态，停止滑动模式
         *
         * huadong_release_flag 用于检测"从离地到落地"的瞬间
         * 在落地瞬间一次性恢复站立姿态
         */
        if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7) == 1)
        {
            /*
             * 机器人离开地面
             * 首次离地时停止所有动作，启动滑动模式
             */
            if (Huadong_State == 0)
            {
                Forward_State = 0;                    // 停止前进
                Back_State = 0;                       // 停止后退
                Left_State = 0;                       // 停止左转
                Right_State = 0;                      // 停止右转
                Huadong_State = 1;                    // 启动滑动模式（展开）
                Huadong_Step = 0;                     // 从0步开始
                step_delay = 0;                       // 重置步进延时
            }
            huadong_release_flag = 1;                 // 标记当前为离地状态
        }
        else
        {
            /*
             * 机器人放在地面上
             * 检测到落地瞬间（flag从1变0），恢复站立姿态
             */
            if (huadong_release_flag == 1)
            {
                /* 落地瞬间：立即恢复标准站立姿态 */
                Angle1 = 135; Angle2 = 45;
                Angle3 = 45;  Angle4 = 135;
                Angle5 = 0;   Angle6 = 180;
                Angle7 = 0;   Angle8 = 180;
                Servo_SetAngle1(Angle1); Servo_SetAngle2(Angle2);
                Servo_SetAngle3(Angle3); Servo_SetAngle4(Angle4);
                Servo_SetAngle5(Angle5); Servo_SetAngle6(Angle6);
                Servo_SetAngle7(Angle7); Servo_SetAngle8(Angle8);
                huadong_release_flag = 0;             // 清除释放标志
            }
            Huadong_State = 0;                        // 停止滑动模式
        }

        /* 如果滑动模式正在运行，执行滑动动画 */
        if (Huadong_State != 0)
        {
            Huadong_Smooth();                         // 执行展开/收回循环动画
        }

        /**
         * ============================================================
         * 主循环延时
         * ============================================================
         * 25ms 延时，控制主循环频率约为 40Hz
         * 所有动作的速度、传感器采集频率都基于这个周期计算
         */
        Delay_ms(25);
    }
}