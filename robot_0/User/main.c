/**
 * ============================================================================
 * 主程序 - 四足机器人控制系统 (优化修复版)
 * ============================================================================
 * 功能概述：
 *   1. 8路舵机驱动（4条腿，每条腿2个舵机）
 *   2. 串口指令控制（前进/后退/左转/右转/各种动作）
 *   3. BH1750 光照传感器读取
 *   4. LM2904 噪声传感器读取
 *   5. HC-SR04 超声波测距 + 自动避障（仅前进时启用）
 *   6. 红外传感器离地检测
 *   7. OLED 显示传感器数据和运行状态
 *   8. LED 状态指示
 *
 * 舵机分配：
 *   舵机1 - servo_angle[0] - 左前抬腿
 *   舵机2 - servo_angle[1] - 左后抬腿
 *   舵机3 - servo_angle[2] - 右前抬腿
 *   舵机4 - servo_angle[3] - 右后抬腿
 *   舵机5 - servo_angle[4] - 左前摆腿
 *   舵机6 - servo_angle[5] - 左后摆腿
 *   舵机7 - servo_angle[6] - 右前摆腿
 *   舵机8 - servo_angle[7] - 右后摆腿
 *
 * 硬件引脚：
 *   PB12/PB13/PB14 - BH1750 光照传感器 (I2C)
 *   PA5            - LM2904 噪声传感器 (ADC)
 *   PB11           - HC-SR04 超声波 TRIG
 *   PB10           - HC-SR04 超声波 ECHO
 *   PB7            - 红外避障传感器 (GPIO输入)
 *   PA12           - LED 指示灯
 *   PA15           - LED 指示灯
 *   PA9/PA10       - 蓝牙 + 串口 (USART1)
 *
 * 软件架构：
 *   采用时间片轮询架构，通过 system_tick 系统时钟计数器
 *   实现不同任务的不同频率执行：
 *     - 5ms   检查串口指令
 *     - 100ms 传感器采集 + OLED 刷新
 *     - 250ms 舵机动作更新（步态切换）
 *     - 500ms LED 闪烁
 * ============================================================================
 */

/* ======================== 头文件包含 ======================== */
#include "stm32f10x.h"       // STM32F10x 标准外设库
#include "Serial.h"           // 串口通信驱动（USART1，同时用于蓝牙）
#include "Servo.h"            // 舵机控制驱动（8路舵机）
#include "Input.h"            // 输入检测驱动（红外传感器）
#include "bh1750.h"           // BH1750 光照传感器驱动（I2C）
#include "hcsr04.h"           // HC-SR04 超声波传感器驱动
#include "OLED.h"             // OLED 显示屏驱动
#include "noise_sensor.h"     // LM2904 噪声传感器驱动（ADC）
#include <string.h>           // 字符串操作

/* ======================== 函数声明 ======================== */
void SystemClock_Config(void);         // 系统时钟配置（72MHz HSE + PLL）
void GPIO_Init_Robot(void);            // GPIO 初始化（LED 引脚）
void ProcessCommand(char *cmd);        // 串口指令解析处理
void UpdateServo(void);                // 舵机动作状态机（核心动作逻辑）
void ReadSensors(void);                // 传感器数据采集
void UpdateDisplay(void);              // OLED 显示刷新
void Tick_Delay(uint32_t ms);          // 软件延时 + 系统时钟计数

/* ======================== 系统时钟 ======================== */
volatile uint32_t system_tick = 0;     // 系统时钟计数器（单位：ms）

/* ======================== 动作控制状态机 ======================== */
volatile uint8_t action_state = 0;     // 当前动作状态
/*
 * action_state 取值：
 *   0 - 静止（站立）
 *   1 - 前进
 *   2 - 后退
 *   3 - 左转
 *   4 - 右转
 *   5 - 摇摆
 *   6 - 坐下
 *   7 - 展开（站立姿态）
 *   8 - 抬脚
 *   9 - 招手
 */
volatile uint16_t step_counter = 0;    // 步态计数器（递增，取模实现循环）

/* ======================== 自动避障相关 ======================== */
volatile uint8_t auto_mode = 0;        // 0=手动模式  1=自动避障模式
volatile uint8_t target_action = 0;    // 动作切换目标（用于平滑过渡）
volatile uint8_t transition_state = 0; // 过渡状态（0=无过渡, 1=收腿, 2=展开）
volatile uint8_t obstacle_avoiding = 0;// 0=无避障  1=正在避障左转

/* ======================== 特殊动作状态 ======================== */
volatile uint8_t wave_state = 0;       // 招手状态（0=未招手  1=已招手，用于切换）
volatile uint8_t lift_state = 0;       // 抬脚状态（0=未抬脚  1=已抬脚，用于切换）
volatile uint8_t infrared_state = 0;   // 红外传感器状态（0=落地  1=离地）
volatile uint8_t swing_state = 0;      // 离地摇摆状态（0=未摇摆  1=摇摆中）

/* ======================== 传感器数据 ======================== */
float lux_value = 0;                   // 光照值（单位：lux）
float distance_value = 0;              // 超声波距离（单位：cm）
float noise_value = 0;                 // 噪声分贝值

/* ======================== 舵机角度数组 ======================== */
/*
 * 8 个舵机角度，范围 0~180 度
 * 索引对应关系：
 *   [0]=舵机1(左前抬) [1]=舵机2(左后抬) [2]=舵机3(右前抬) [3]=舵机4(右后抬)
 *   [4]=舵机5(左前摆) [5]=舵机6(左后摆) [6]=舵机7(右前摆) [7]=舵机8(右后摆)
 *
 * 初始值：抬腿舵机 90度（中间），摆腿舵机 0/180（展开）
 */
float servo_angle[8] = {90, 90, 90, 90, 0, 180, 0, 180};

/**
 * ============================================================================
 * 函数名：main
 * 功能：  主函数，程序入口
 * 说明：  初始化所有外设后进入主循环
 *         采用时间片轮询架构，通过 system_tick 实现不同任务不同执行频率
 *         system_tick 由 Tick_Delay() 在每次延时后递增
 * ============================================================================
 */
int main(void)
{
    /* ==================== 系统初始化 ==================== */
    SystemClock_Config();        // 配置系统时钟为 72MHz（HSE + PLL x9）
    GPIO_Init_Robot();           // 初始化 LED 引脚（PA12/PA15 推挽输出）
    Servo_Init();                // 初始化 8 路舵机（PWM 输出）
    Serial_Init();               // 初始化串口通信（USART1，115200 波特率）
    Input_Init();                // 初始化输入检测（红外传感器 PB7）
    bh1750_init();               // 初始化 BH1750 光照传感器（I2C，PB12/PB13/PB14）
    hcsr04_init();               // 初始化 HC-SR04 超声波传感器（PB11 TRIG / PB10 ECHO）
    Noise_Sensor_Init();         // 初始化 LM2904 噪声传感器（ADC，PA5）

    /* OLED 初始化并显示启动信息 */
    OLED_Init();
    OLED_Clear();
    OLED_Printf(0, 0, OLED_8X16, "Init...");   // 显示"初始化中"
    OLED_Update();
    Tick_Delay(1000);            // 延时 1 秒等待传感器稳定

    /* ==================== 主循环 ==================== */
    while (1)
    {
        /* 基础延时 1ms，同时递增 system_tick */
        Tick_Delay(1);

        /**
         * ============================================================
         * 时间片1：LED 闪烁（每 500ms 周期，250ms 切换一次）
         * ============================================================
         * PA15 引脚控制 LED 闪烁
         * system_tick % 500 < 250 → LED 亮
         * system_tick % 500 >= 250 → LED 灭
         * 闪烁周期 500ms，占空比 50%
         */
        if (system_tick % 500 < 250)
        {
            GPIO_ResetBits(GPIOA, GPIO_Pin_15);   // PA15 低电平 → LED 亮
        }
        else
        {
            GPIO_SetBits(GPIOA, GPIO_Pin_15);     // PA15 高电平 → LED 灭
        }

        /**
         * ============================================================
         * 时间片2：串口指令检查（每 5ms 检查一次）
         * ============================================================
         * 每 5ms 检查一次是否有新的串口数据包
         * Serial_RxFlag == 1 表示收到完整指令（以 @ 开头，#数据* 结尾）
         * 解析指令后清空缓冲区和标志位
         *
         * 指令列表：
         *   "UP"    - 前进（action_state = 1）
         *   "BACK"  - 后退（action_state = 2）
         *   "LEFT"  - 左转（action_state = 3）
         *   "RIGHT" - 右转（action_state = 4）
         *   "YB"    - 摇摆（action_state = 5）
         *   "ZX"    - 坐下（action_state = 6）
         *   "ZK"    - 展开/站立（action_state = 7）
         *   "TJ"    - 抬脚（action_state = 8）
         *   "ZS"    - 招手（action_state = 9）
         *   "AUTO"  - 自动避障模式
         *   "MANU"  - 手动控制模式
         *   "REST"  - 重置所有状态
         *   "STAT"  - 查询当前传感器数据
         */
        if (system_tick % 5 == 0 && Serial_RxFlag == 1)
        {
            ProcessCommand(Serial_RxPacket);       // 解析并执行指令
            Serial_RxFlag = 0;                     // 清除接收标志
            Serial_RxPacket[0] = '\0';             // 清空接收缓冲区
        }

        /**
         * ============================================================
         * 时间片3：传感器采集 + OLED 刷新（每 100ms 一次）
         * ============================================================
         * 三个传感器数据采集：
         *   BH1750 光照（I2C 通信，约 5ms）
         *   HC-SR04 超声波（需等待回波，约 30ms）
         *   LM2904 噪声（ADC 采集，约 1ms）
         * 采集完成后立即刷新 OLED 显示
         */
        if (system_tick % 100 == 0)
        {
            ReadSensors();                         // 采集所有传感器数据
            UpdateDisplay();                       // 刷新 OLED 显示
        }

        /**
         * ============================================================
         * 时间片4：舵机动作更新（每 250ms 一次）
         * ============================================================
         * 这是动作控制的核心
         * 每 250ms 递增步态计数器并更新舵机角度
         * step_counter 递增后在 UpdateServo() 内部取模实现循环步态
         * 不同动作的步态周期：
         *   前进/后退/左转/右转：step_counter % 4（4步循环，1秒一个完整周期）
         *   摇摆：step_counter % 24（24步循环，6秒一个完整周期）
         *   离地摇摆：step_counter % 8（8步循环，2秒一个完整周期）
         */
        if (system_tick % 250 == 0)
        {
            step_counter++;                        // 步态计数器递增
            UpdateServo();                         // 根据当前动作状态更新舵机角度
        }
    }
}

/**
 * ============================================================================
 * 函数名：ReadSensors
 * 功能：  读取所有传感器数据
 * 说明：  依次读取 BH1750 光照、HC-SR04 超声波、LM2904 噪声、红外状态
 *         每 100ms 调用一次
 * ============================================================================
 */
void ReadSensors(void)
{
    float temp_lux, temp_distance;
    NoiseData_TypeDef noise_data;

    /* 读取 BH1750 光照值（I2C 通信） */
    temp_lux = bh1750_get_lux();
    if (temp_lux >= 0) lux_value = temp_lux;       // 有效值才更新

    /* 读取 HC-SR04 超声波距离 */
    temp_distance = hcsr04_get_distance();
    if (temp_distance > 0) distance_value = temp_distance;  // 有效值才更新

    /* 读取 LM2904 噪声数据（ADC 采集 + 去极值滤波） */
    Noise_Sensor_GetData(&noise_data);
    noise_value = noise_data.db_value;              // 提取分贝值

    /* 读取红外传感器状态（PB7 GPIO 输入） */
    infrared_state = Input_GetInfrared();           // 0=落地  1=离地
}

/**
 * ============================================================================
 * 函数名：UpdateDisplay
 * 功能：  刷新 OLED 显示屏内容
 * 说明：  每 100ms 调用一次，显示4行数据：
 *   第一行：光照值
 *   第二行：超声波距离
 *   第三行：噪声分贝
 *   第四行：当前模式（AUTO/MANU）
 * ============================================================================
 */
void UpdateDisplay(void)
{
    OLED_Clear();                                      // 清屏

    /* 第一行：光照传感器数据 */
    OLED_Printf(0, 0, OLED_8X16, "Lux:%.1f lx", lux_value);

    /* 第二行：超声波距离数据 */
    OLED_Printf(0, 16, OLED_8X16, "Dist:%.1f cm", distance_value);

    /* 第三行：噪声传感器数据 */
    OLED_Printf(0, 32, OLED_8X16, "Noise:%.1f dB", noise_value);

    /* 第四行：当前运行模式 */
    OLED_Printf(0, 48, OLED_8X16, "Mode:%s", auto_mode ? "AUTO" : "MANU");

    OLED_Update();                                     // 将缓冲区写入 OLED
}

/**
 * ============================================================================
 * 函数名：GPIO_Init_Robot
 * 功能：  初始化机器人 GPIO 引脚
 * 说明：  配置 PA12 和 PA15 为推挽输出（LED 指示灯）
 *         同时禁用 JTAG 以释放 PA15 引脚给 LED 使用
 *         （PA15 默认是 JTDI 功能，需要重映射后才能作为普通 GPIO）
 * ============================================================================
 */
void GPIO_Init_Robot(void)
{
    /* 开启 GPIOA、GPIOB 和 AFIO 时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

    /* 禁用 JTAG 但保留 SWD（释放 PA15/PB3/PB4 给普通 GPIO 使用） */
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

    /* 配置 PA12 和 PA15 为推挽输出（LED） */
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;         // 推挽输出模式
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12 | GPIO_Pin_15; // PA12 + PA15
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;          // 50MHz 输出速度
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* 初始状态：LED 全部熄灭（高电平） */
    GPIO_SetBits(GPIOA, GPIO_Pin_12 | GPIO_Pin_15);
}

/**
 * ============================================================================
 * 函数名：ProcessCommand
 * 功能：  解析串口指令并执行对应操作
 * 参数：  cmd - 指令字符串（不含 @ # * 帧头帧尾）
 * 说明：  根据指令首字符快速判断指令类型
 *         动作切换时会经过平滑过渡（先收腿再展开到新动作）
 *         招手和抬脚是单次动作（触发后自动回到静止状态）
 * ============================================================================
 */
void ProcessCommand(char *cmd)
{
    uint8_t new_action = 0;    // 新动作编号，0 表示无动作

    /* ==================== 指令解析 ==================== */
    /*
     * 通过首字符快速匹配指令
     * 优先匹配短指令（UP 2字符），避免被长指令前缀误匹配
     */
    if (cmd[0] == 'U' && cmd[1] == 'P')
        new_action = 1;                               // 前进
    else if (cmd[0] == 'B' && cmd[1] == 'A' && cmd[2] == 'C' && cmd[3] == 'K')
        new_action = 2;                               // 后退
    else if (cmd[0] == 'L' && cmd[1] == 'E' && cmd[2] == 'F' && cmd[3] == 'T')
        new_action = 3;                               // 左转
    else if (cmd[0] == 'R' && cmd[1] == 'I' && cmd[2] == 'G' && cmd[3] == 'H' && cmd[4] == 'T')
        new_action = 4;                               // 右转
    else if (cmd[0] == 'Y' && cmd[1] == 'B')
        new_action = 5;                               // 摇摆
    else if (cmd[0] == 'Z' && cmd[1] == 'K')
        new_action = 7;                               // 展开/站立
    else if (cmd[0] == 'Z' && cmd[1] == 'X')
        new_action = 6;                               // 坐下
    else if (cmd[0] == 'T' && cmd[1] == 'J')
        new_action = 8;                               // 抬脚
    else if (cmd[0] == 'Z' && cmd[1] == 'S')
        new_action = 9;                               // 招手

    /* ==================== 特殊指令 ==================== */

    /* --- AUTO：切换到自动避障模式 --- */
    else if (cmd[0] == 'A' && cmd[1] == 'U' && cmd[2] == 'T' && cmd[3] == 'O')
    {
        auto_mode = 1;                                // 开启自动模式
        target_action = 1;                            // 目标动作为前进
        transition_state = 1;                         // 启动过渡动画
        step_counter = 0;                             // 重置步态计数器
        return;
    }
    /* --- MANU：切换到手动控制模式 --- */
    else if (cmd[0] == 'M' && cmd[1] == 'A' && cmd[2] == 'N' && cmd[3] == 'U')
    {
        auto_mode = 0;                                // 关闭自动模式
        target_action = 0;                            // 清除目标动作
        transition_state = 0;                         // 清除过渡状态
        step_counter = 0;                             // 重置步态计数器
        obstacle_avoiding = 0;                        // 清除避障状态
        action_state = 7;                             // 切换到展开/站立姿态
        return;
    }
    /* --- REST：重置所有状态到初始值 --- */
    else if (cmd[0] == 'R' && cmd[1] == 'E' && cmd[2] == 'S' && cmd[3] == 'T')
    {
        /* 清除所有状态变量 */
        auto_mode = 0;
        target_action = 0;
        transition_state = 0;
        step_counter = 0;
        obstacle_avoiding = 0;
        wave_state = 0;
        lift_state = 0;
        action_state = 0;

        /* 舵机回到标准站立姿态 */
        servo_angle[0] = 135;   // 左前抬腿
        servo_angle[1] = 45;    // 左后抬腿
        servo_angle[2] = 45;    // 右前抬腿
        servo_angle[3] = 135;   // 右后抬腿
        servo_angle[4] = 0;     // 左前摆腿
        servo_angle[5] = 180;   // 左后摆腿
        servo_angle[6] = 0;     // 右前摆腿
        servo_angle[7] = 180;   // 右后摆腿
        return;
    }
    /* --- STAT：查询当前传感器数据（蓝牙回传） --- */
    else if (cmd[0] == 'S' && cmd[1] == 'T' && cmd[2] == 'A' && cmd[3] == 'T')
    {
        /* 通过串口（蓝牙）回传传感器数据 */
        Serial_Printf("Lux:%.1f, Dist:%.1f, Noise:%.1f\r\n",
                       lux_value, distance_value, noise_value);
        return;
    }

    /* ==================== 动作指令执行 ==================== */
    if (new_action > 0)
    {
        auto_mode = 0;                                // 手动指令退出自动模式
        obstacle_avoiding = 0;                        // 清除避障状态

        if (action_state != 0 &&                      // 当前正在执行某个动作
            new_action != action_state &&              // 新动作与当前不同
            new_action != 7)                           // 展开指令除外（展开是直接切换）
        {
            /*
             * 需要平滑过渡：
             * 先记录目标动作，启动过渡动画
             * transition_state = 1 → 先收腿到中间位置
             * transition_state = 2 → 再展开到标准站立
             * 过渡完成后自动切换到目标动作
             */
            target_action = new_action;
            transition_state = 1;
        }
        else
        {
            /*
             * 无需过渡，直接切换动作
             * 当前无动作、与当前相同、或者是展开指令时走这里
             */
            action_state = new_action;
            step_counter = 0;                         // 重置步态计数器
        }
    }
}

/**
 * ============================================================================
 * 函数名：UpdateServo
 * 功能：  舵机动作状态机（核心函数）
 * 说明：  每 250ms 调用一次
 *         处理优先级（从高到低）：
 *           1. 离地检测（红外传感器）→ 离地摇摆动画
 *           2. 动作过渡动画 → 平滑切换动作
 *           3. 避障逻辑 → 前进时检测障碍物自动左转
 *           4. 常规动作 → 根据 action_state 执行步态
 *         所有动作最终都通过 Servo_SetAngle1~8 写入舵机
 * ============================================================================
 */
void UpdateServo(void)
{
    uint8_t current_action = action_state;     // 当前要执行的动作（可能被避障覆盖）

    /* ==================== 1. 离地检测处理 ==================== */
    /*
     * 红外传感器（PB7）检测机器人是否被拿起
     * 离地时（infrared_state == 1）：
     *   → 停止所有动作，进入离地摇摆动画
     *   → 腿部左右交替展开/收回，模拟滑行动作
     * 落地时（infrared_state == 0）：
     *   → 停止摇摆动画，恢复到当前动作状态
     */
    if (infrared_state == 1)
    {
        if (swing_state == 0)
        {
            swing_state = 1;                             // 启动摇摆模式
        }
    }
    else
    {
        if (swing_state == 1)
        {
            swing_state = 0;                             // 落地，停止摇摆
        }
    }

    /* 如果处于离地摇摆状态，执行摇摆动画并直接返回 */
    if (swing_state == 1)
    {
        /*
         * 离地摇摆动画（8步循环）
         * 腿部交替展开/收回，形成左右摇摆效果
         * step_counter % 8 实现 8 步循环
         */
        switch (step_counter % 8)
        {
            case 0:  /* 左前腿展开 */
                servo_angle[0] = 135.0f; servo_angle[1] = 45.0f;
                servo_angle[2] = 45.0f;  servo_angle[3] = 135.0f;
                servo_angle[4] = 30.0f;  servo_angle[5] = 150.0f;
                servo_angle[6] = 0.0f;   servo_angle[7] = 180.0f;
                break;
            case 2:  /* 右前腿展开 */
                servo_angle[0] = 150.0f; servo_angle[1] = 30.0f;
                servo_angle[2] = 30.0f;  servo_angle[3] = 150.0f;
                servo_angle[4] = 0.0f;   servo_angle[5] = 180.0f;
                servo_angle[6] = 30.0f;  servo_angle[7] = 150.0f;
                break;
            case 4:  /* 左后腿展开 */
                servo_angle[0] = 135.0f; servo_angle[1] = 45.0f;
                servo_angle[2] = 45.0f;  servo_angle[3] = 135.0f;
                servo_angle[4] = 0.0f;   servo_angle[5] = 180.0f;
                servo_angle[6] = 30.0f;  servo_angle[7] = 150.0f;
                break;
            case 6:  /* 右后腿展开 */
                servo_angle[0] = 150.0f; servo_angle[1] = 30.0f;
                servo_angle[2] = 30.0f;  servo_angle[3] = 150.0f;
                servo_angle[4] = 30.0f;  servo_angle[5] = 150.0f;
                servo_angle[6] = 0.0f;   servo_angle[7] = 180.0f;
                break;
            /* case 1/3/5/7 保持上一帧角度，形成停顿效果 */
        }

        /* 直接写入舵机并返回（不执行后续动作逻辑） */
        Servo_SetAngle1(servo_angle[0]);
        Servo_SetAngle2(servo_angle[1]);
        Servo_SetAngle3(servo_angle[2]);
        Servo_SetAngle4(servo_angle[3]);
        Servo_SetAngle5(servo_angle[4]);
        Servo_SetAngle6(servo_angle[5]);
        Servo_SetAngle7(servo_angle[6]);
        Servo_SetAngle8(servo_angle[7]);
        return;                                          // 离地状态不执行其他动作
    }

    /* ==================== 2. 动作过渡动画 ==================== */
    /*
     * 当切换动作时，先执行平滑过渡：
     *   transition_state = 1 → 收腿到中间位置（90度）
     *   transition_state = 2 → 展开到标准站立姿态
     * 过渡完成后自动切换到目标动作
     *
     * 这样避免从一个极端姿态直接跳到另一个极端
     * 保护舵机和机械结构
     */
    if (transition_state > 0)
    {
        switch (transition_state)
        {
            case 1:  /* 第一步：收腿到中间位置 */
                servo_angle[0] = 90.0f;  servo_angle[1] = 90.0f;   // 抬腿全部收到90度
                servo_angle[2] = 90.0f;  servo_angle[3] = 90.0f;
                servo_angle[4] = 0.0f;   servo_angle[5] = 180.0f;  // 摆腿保持展开
                servo_angle[6] = 0.0f;   servo_angle[7] = 180.0f;
                transition_state = 2;                             // 下一步展开
                break;

            case 2:  /* 第二步：展开到标准站立姿态 */
                servo_angle[0] = 135.0f; servo_angle[1] = 45.0f;  // 标准站立
                servo_angle[2] = 45.0f;  servo_angle[3] = 135.0f;
                servo_angle[4] = 0.0f;   servo_angle[5] = 180.0f;
                servo_angle[6] = 0.0f;   servo_angle[7] = 180.0f;
                transition_state = 0;                             // 过渡完成
                action_state = target_action;                     // 切换到目标动作
                step_counter = 0;                                 // 重置步态计数器
                break;
        }
    }
    /* ==================== 3. 避障逻辑 ==================== */
    /*
     * 避障仅在 action_state == 1（前进）时生效
     * 超声波距离 < 15cm → 触发避障
     *   → 覆盖 current_action 为 3（左转）
     *   → 设置 obstacle_avoiding = 1
     * 超声波距离 >= 15cm → 取消避障
     *   → obstacle_avoiding = 0
     *   → 继续执行前进动作
     *
     * 避障状态只影响当前这一次 UpdateServo 调用
     * 不改变 action_state，所以下次调用仍会检查距离
     */
    else if (action_state == 1)
    {
        if (distance_value > 0 && distance_value < 15)  // 距离有效且 < 15cm
        {
            obstacle_avoiding = 1;                       // 标记正在避障
            current_action = 3;                          // 覆盖为左转动作
        }
        else
        {
            obstacle_avoiding = 0;                       // 无障碍，继续前进
        }
    }
    else
    {
        obstacle_avoiding = 0;                           // 非前进状态，清除避障
    }

    /* ==================== 4. 常规动作执行 ==================== */
    /*
     * 根据 current_action 执行对应的步态
     * step_counter 在主循环中每 250ms 递增一次
     * step_counter % N 实现 N 步循环
     *
     * 步态说明：
     *   case 0 表示步态循环的最后一帧
     *   case 1/2/3 表示中间帧
     *   奇数帧（1/3）：准备姿态
     *   偶数帧（2/0）：抬腿/摆腿执行
     */
    switch (current_action)
    {
        /**
         * -------------------------------------------------------
         * case 1：前进动作
         * -------------------------------------------------------
         * 4步循环（step_counter % 4）
         * 步骤1 → 准备姿态，后腿摆动
         * 步骤2 → 抬起前腿
         * 步骤3 → 前腿前摆
         * 步骤4 → 放下前腿，身体前移
         */
        case 1:
            switch (step_counter % 4)
            {
                case 1:  /* 步骤1：准备姿态 */
                    servo_angle[0]=135.0f; servo_angle[1]=45.0f;
                    servo_angle[2]=45.0f;  servo_angle[3]=135.0f;
                    servo_angle[4]=0.0f;   servo_angle[5]=150.0f;  // 左后腿后摆
                    servo_angle[6]=30.0f;  servo_angle[7]=180.0f;  // 右前腿前移
                    break;
                case 2:  /* 步骤2：抬起前腿 */
                    servo_angle[0]=135.0f; servo_angle[1]=15.0f;  // 左后腿抬起
                    servo_angle[2]=75.0f;  servo_angle[3]=135.0f;  // 右前腿抬起
                    servo_angle[4]=0.0f;   servo_angle[5]=180.0f;
                    servo_angle[6]=0.0f;   servo_angle[7]=180.0f;
                    break;
                case 3:  /* 步骤3：前腿前摆 */
                    servo_angle[0]=135.0f; servo_angle[1]=15.0f;
                    servo_angle[2]=75.0f;  servo_angle[3]=135.0f;
                    servo_angle[4]=30.0f;  servo_angle[5]=180.0f;  // 左前腿前摆
                    servo_angle[6]=0.0f;   servo_angle[7]=150.0f;
                    break;
                case 0:  /* 步骤4：身体前移 */
                    servo_angle[0]=165.0f; servo_angle[1]=45.0f;  // 左前腿前移
                    servo_angle[2]=45.0f;  servo_angle[3]=105.0f;  // 右后腿前移
                    servo_angle[4]=0.0f;   servo_angle[5]=180.0f;
                    servo_angle[6]=0.0f;   servo_angle[7]=180.0f;
                    break;
            }
            break;

        /**
         * -------------------------------------------------------
         * case 2：后退动作
         * -------------------------------------------------------
         * 与前进对称，方向相反
         */
        case 2:
            switch (step_counter % 4)
            {
                case 1:
                    servo_angle[0]=135.0f; servo_angle[1]=45.0f;
                    servo_angle[2]=45.0f;  servo_angle[3]=135.0f;
                    servo_angle[4]=0.0f;   servo_angle[5]=150.0f;
                    servo_angle[6]=30.0f;  servo_angle[7]=180.0f;
                    break;
                case 2:  /* 左后腿抬起（与前进相反） */
                    servo_angle[0]=135.0f; servo_angle[1]=75.0f;
                    servo_angle[2]=15.0f;  servo_angle[3]=135.0f;
                    servo_angle[4]=0.0f;   servo_angle[5]=180.0f;
                    servo_angle[6]=0.0f;   servo_angle[7]=180.0f;
                    break;
                case 3:
                    servo_angle[0]=135.0f; servo_angle[1]=75.0f;
                    servo_angle[2]=15.0f;  servo_angle[3]=135.0f;
                    servo_angle[4]=30.0f;  servo_angle[5]=180.0f;
                    servo_angle[6]=0.0f;   servo_angle[7]=150.0f;
                    break;
                case 0:  /* 身体后移 */
                    servo_angle[0]=105.0f; servo_angle[1]=45.0f;
                    servo_angle[2]=45.0f;  servo_angle[3]=165.0f;
                    servo_angle[4]=0.0f;   servo_angle[5]=180.0f;
                    servo_angle[6]=0.0f;   servo_angle[7]=180.0f;
                    break;
            }
            break;

        /**
         * -------------------------------------------------------
         * case 3：左转动作
         * -------------------------------------------------------
         * 4步循环，不对称步态
         * 左侧腿步幅大，右侧腿步幅小 → 身体向左偏转
         */
        case 3:
            switch (step_counter % 4)
            {
                case 1:  /* 左侧腿大幅展开 */
                    servo_angle[0]=150.0f; servo_angle[1]=30.0f;  // 左侧幅度大
                    servo_angle[2]=30.0f;  servo_angle[3]=150.0f;
                    servo_angle[4]=30.0f;  servo_angle[5]=180.0f;
                    servo_angle[6]=0.0f;   servo_angle[7]=150.0f;
                    break;
                case 2:  /* 收回 */
                    servo_angle[0]=165.0f; servo_angle[1]=15.0f;
                    servo_angle[2]=15.0f;  servo_angle[3]=165.0f;
                    servo_angle[4]=0.0f;   servo_angle[5]=180.0f;
                    servo_angle[6]=0.0f;   servo_angle[7]=180.0f;
                    break;
                case 3:  /* 右侧腿小幅摆动 */
                    servo_angle[0]=165.0f; servo_angle[1]=15.0f;
                    servo_angle[2]=15.0f;  servo_angle[3]=165.0f;
                    servo_angle[4]=0.0f;   servo_angle[5]=150.0f;
                    servo_angle[6]=30.0f;  servo_angle[7]=180.0f;
                    break;
                case 0:  /* 回到标准站立 */
                    servo_angle[0]=135.0f; servo_angle[1]=45.0f;
                    servo_angle[2]=45.0f;  servo_angle[3]=135.0f;
                    servo_angle[4]=0.0f;   servo_angle[5]=180.0f;
                    servo_angle[6]=0.0f;   servo_angle[7]=180.0f;
                    break;
            }
            break;

        /**
         * -------------------------------------------------------
         * case 4：右转动作
         * -------------------------------------------------------
         * 与左转对称，右侧腿步幅大
         */
        case 4:
            switch (step_counter % 4)
            {
                case 1:  /* 右侧腿大幅展开 */
                    servo_angle[0]=150.0f; servo_angle[1]=30.0f;
                    servo_angle[2]=30.0f;  servo_angle[3]=150.0f;
                    servo_angle[4]=0.0f;   servo_angle[5]=150.0f;
                    servo_angle[6]=30.0f;  servo_angle[7]=180.0f;
                    break;
                case 2:
                    servo_angle[0]=165.0f; servo_angle[1]=15.0f;
                    servo_angle[2]=15.0f;  servo_angle[3]=165.0f;
                    servo_angle[4]=0.0f;   servo_angle[5]=180.0f;
                    servo_angle[6]=0.0f;   servo_angle[7]=180.0f;
                    break;
                case 3:  /* 左侧腿小幅摆动 */
                    servo_angle[0]=165.0f; servo_angle[1]=15.0f;
                    servo_angle[2]=15.0f;  servo_angle[3]=165.0f;
                    servo_angle[4]=30.0f;  servo_angle[5]=180.0f;
                    servo_angle[6]=0.0f;   servo_angle[7]=150.0f;
                    break;
                case 0:
                    servo_angle[0]=135.0f; servo_angle[1]=45.0f;
                    servo_angle[2]=45.0f;  servo_angle[3]=135.0f;
                    servo_angle[4]=0.0f;   servo_angle[5]=180.0f;
                    servo_angle[6]=0.0f;   servo_angle[7]=180.0f;
                    break;
            }
            break;

        /**
         * -------------------------------------------------------
         * case 5：摇摆动作
         * -------------------------------------------------------
         * 24步循环（step_counter % 24）
         * 前12步：摆腿角度从0递增到60（展开）
         * 后12步：摆腿角度从60递减到0（收回）
         * 4个摆腿同步运动，形成身体左右摇摆效果
         */
        case 5:
            servo_angle[0] = 135; servo_angle[1] = 45;    // 抬腿固定站立
            servo_angle[2] = 45;  servo_angle[3] = 135;

            /* 计算摆腿角度（三角波：0→60→0） */
            servo_angle[4] = (step_counter % 24 < 12 ?
                              (step_counter % 12) * 5 :       // 前半段：0→55
                              (24 - step_counter % 24) * 5);  // 后半段：55→0
            servo_angle[5] = 180 - servo_angle[4];           // 左后摆腿对称
            servo_angle[6] = servo_angle[4];                  // 右前摆腿同步
            servo_angle[7] = 180 - servo_angle[4];           // 右后摆腿对称
            break;

        /**
         * -------------------------------------------------------
         * case 6：坐下动作（单次）
         * -------------------------------------------------------
         * 抬腿舵机保持站立，摆腿舵机全部收到 90 度（收腿坐下）
         * 执行一次后 action_state 归零
         */
        case 6:
            servo_angle[0] = 135; servo_angle[1] = 45;
            servo_angle[2] = 45;  servo_angle[3] = 135;
            servo_angle[4] = 90;  servo_angle[5] = 90;       // 摆腿收到中间
            servo_angle[6] = 90;  servo_angle[7] = 90;
            break;

        /**
         * -------------------------------------------------------
         * case 7：展开/站立动作
         * -------------------------------------------------------
         * 抬腿舵机全部收到 90 度（中间），摆腿全部展开
         * 用于切换姿态或初始站立
         */
        case 7:
            servo_angle[0] = 90;  servo_angle[1] = 90;       // 抬腿中间
            servo_angle[2] = 90;  servo_angle[3] = 90;
            servo_angle[4] = 0;   servo_angle[5] = 180;       // 摆腿展开
            servo_angle[6] = 0;   servo_angle[7] = 180;
            break;

        /**
         * -------------------------------------------------------
         * case 8：抬脚动作（单次切换）
         * -------------------------------------------------------
         * lift_state 用于在两个姿态之间切换
         * lift_state == 0 → 切换到抬脚姿态
         * lift_state == 1 → 恢复站立姿态
         * 执行一次后 action_state 归零（等待下次触发）
         */
        case 8:
            if (lift_state == 0)
            {
                /* 抬脚姿态：左前腿抬起，右后腿抬起 */
                servo_angle[0] = 90.0f;  servo_angle[1] = 90.0f;
                servo_angle[2] = 45.0f;  servo_angle[3] = 135.0f;
                servo_angle[4] = 0.0f;   servo_angle[5] = 180.0f;
                servo_angle[6] = 135.0f; servo_angle[7] = 180.0f;
                lift_state = 1;                               // 标记为已抬脚
            }
            else
            {
                /* 恢复站立 */
                servo_angle[0] = 135.0f; servo_angle[1] = 45.0f;
                servo_angle[2] = 45.0f;  servo_angle[3] = 135.0f;
                servo_angle[4] = 0.0f;   servo_angle[5] = 180.0f;
                servo_angle[6] = 0.0f;   servo_angle[7] = 180.0f;
                lift_state = 0;                               // 标记为未抬脚
            }
            action_state = 0;                                 // 单次动作，执行后归零
            break;

        /**
         * -------------------------------------------------------
         * case 9：招手动作（单次切换）
         * -------------------------------------------------------
         * wave_state 用于在两个姿态之间切换
         * wave_state == 0 → 切换到招手姿态（右前腿抬起）
         * wave_state == 1 → 恢复站立姿态
         * 执行一次后 action_state 归零
         */
        case 9:
            if (wave_state == 0)
            {
                /* 招手姿态：右前腿抬起，身体侧倾 */
                servo_angle[0] = 160.0f; servo_angle[1] = 45.0f;
                servo_angle[2] = 110.0f; servo_angle[3] = 70.0f;
                servo_angle[4] = 20.0f;  servo_angle[5] = 35.0f;
                servo_angle[6] = 20.0f;  servo_angle[7] = 160.0f;
                wave_state = 1;                               // 标记为已招手
            }
            else
            {
                /* 恢复站立 */
                servo_angle[0] = 135.0f; servo_angle[1] = 45.0f;
                servo_angle[2] = 45.0f;  servo_angle[3] = 135.0f;
                servo_angle[4] = 0.0f;   servo_angle[5] = 180.0f;
                servo_angle[6] = 0.0f;   servo_angle[7] = 180.0f;
                wave_state = 0;                               // 标记为未招手
            }
            action_state = 0;                                 // 单次动作，执行后归零
            break;
    }

    /* ==================== 舵机角度输出 ==================== */
    /*
     * 将计算好的角度值写入 8 路舵机
     * 每次 UpdateServo 调用都会执行
     * 确保舵机实时响应角度变化
     */
    Servo_SetAngle1(servo_angle[0]);  // 左前抬腿
    Servo_SetAngle2(servo_angle[1]);  // 左后抬腿
    Servo_SetAngle3(servo_angle[2]);  // 右前抬腿
    Servo_SetAngle4(servo_angle[3]);  // 右后抬腿
    Servo_SetAngle5(servo_angle[4]);  // 左前摆腿
    Servo_SetAngle6(servo_angle[5]);  // 左后摆腿
    Servo_SetAngle7(servo_angle[6]);  // 右前摆腿
    Servo_SetAngle8(servo_angle[7]);  // 右后摆腿
}

/**
 * ============================================================================
 * 函数名：Tick_Delay
 * 功能：  软件延时 + 系统时钟递增
 * 参数：  ms - 延时毫秒数
 * 说明：  通过空循环实现近似延时
 *         每次调用后 system_tick 递增对应的毫秒数
 *         72MHz 主频下，1ms ≈ 7200 次循环
 *         精度不高，但足够满足本项目的时间片需求
 * ============================================================================
 */
void Tick_Delay(uint32_t ms)
{
    volatile uint32_t i;
    for (i = 0; i < ms * 7200; i++);     // 空循环实现延时
    system_tick += ms;                    // 递增系统时钟
}

/**
 * ============================================================================
 * 函数名：SystemClock_Config
 * 功能：  配置系统时钟为 72MHz
 * 说明：  使用外部高速晶振（HSE）作为时钟源
 *         HSE → PLL（×9倍频）→ SYSCLK = 8MHz × 9 = 72MHz
 *         AHB 总线 = 72MHz（不分频）
 *         APB1 总线 = 36MHz（2分频，低速外设）
 *         APB2 总线 = 72MHz（不分频，高速外设）
 *         Flash 等待 2 个周期（72MHz 需要）
 * ============================================================================
 */
void SystemClock_Config(void)
{
    /* 复位 RCC 寄存器到默认值 */
    RCC_DeInit();

    /* 开启外部高速晶振（HSE，通常 8MHz） */
    RCC_HSEConfig(RCC_HSE_ON);
    while (RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET);  // 等待 HSE 就绪

    /* 配置总线分频 */
    RCC_HCLKConfig(RCC_SYSCLK_Div1);        // AHB = SYSCLK / 1 = 72MHz
    RCC_PCLK1Config(RCC_HCLK_Div2);         // APB1 = HCLK / 2 = 36MHz
    RCC_PCLK2Config(RCC_HCLK_Div1);         // APB2 = HCLK / 1 = 72MHz

    /* 配置 Flash 等待周期（72MHz 需要 2 个等待周期） */
    FLASH_SetLatency(FLASH_Latency_2);

    /* 配置 PLL：HSE 不分频，9 倍频 → 8MHz × 9 = 72MHz */
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
    RCC_PLLCmd(ENABLE);
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);  // 等待 PLL 就绪

    /* 切换系统时钟源为 PLL */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    while (RCC_GetSYSCLKSource() != 0x08);   // 确认已切换到 PLL（0x08 = PLL 输出）
}
