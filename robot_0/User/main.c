/**
 * ============================================================================
 * 主程序 - 四足机器人控制系统 (优化修复版)
 * ============================================================================
 * @brief 基于STM32F103C8的四足机器人主控程序
 * @details 实现多传感器数据采集、舵机运动控制、串口通信、自动避障等功能
 * @author 机器人控制项目组
 * @date 2026-05-25
 * @version v1.2
 */

#include "stm32f10x.h"      /* STM32标准库 */
#include "Serial.h"         /* 串口通信模块 */
#include "Servo.h"          /* 舵机控制模块 */
#include "Input.h"          /* 输入按键模块 */
#include "bh1750.h"         /* BH1750光照传感器驱动 */
#include "hcsr04.h"         /* HC-SR04超声波传感器驱动 */
#include "OLED.h"           /* OLED显示模块 */
#include "noise_sensor.h"   /* 噪声传感器驱动 */
#include "LED.h"            /* LED控制模块 */
#include <string.h>         /* 字符串处理函数 */

/** @brief 系统时钟配置函数声明 */
void SystemClock_Config(void);

/* ==================== 全局变量定义 ==================== */

/** @brief 系统时间计数器(ms)，用于任务调度 */
volatile uint32_t system_tick = 0;

/** @brief 当前动作状态：0-站立 1-前进 2-后退 3-左转 4-右转 5-摇摆 6-深蹲 7-平躺 8-抬腿 9-挥手 */
volatile uint8_t action_state = 0;

/** @brief 动作步数计数器，用于控制步态循环 */
volatile uint16_t step_counter = 0;

/** @brief 自动/手动模式标志：0-手动 1-自动 */
volatile uint8_t auto_mode = 0;

/** @brief 挥手动作状态标志 */
volatile uint8_t wave_state = 0;

/** @brief 抬腿动作状态标志 */
volatile uint8_t lift_state = 0;

/** @brief 目标动作状态（用于动作切换过渡） */
volatile uint8_t target_action = 0;

/** @brief 过渡状态：0-无过渡 1-过渡中(回中位) 2-过渡中(站立) */
volatile uint8_t transition_state = 0;

/** @brief 避障状态标志 */
volatile uint8_t obstacle_avoiding = 0;

/** @brief 离地检测状态：0-着地 1-离地 */
volatile uint8_t air_detect_state = 0;

/** @brief 数据采集模式标志：0-正常 1-采集模式(定时发送传感器数据) */
volatile uint8_t collect_mode = 0;

/** @brief 8个舵机角度数组：[0-3]髋关节 [4-7]膝关节 */
float servo_angle[8] = {90, 90, 90, 90, 0, 180, 0, 180};

/** @brief 光照强度值(lux) */
float lux_value = 0;

/** @brief 超声波距离值(cm) */
float distance_value = 0;

/** @brief 噪声分贝值(dB) */
float noise_value = 0;

/* ==================== 函数声明 ==================== */

/**
 * @brief 机器人GPIO初始化函数
 * @details 配置LED输出引脚、红外传感器输入引脚等
 */
void GPIO_Init_Robot(void);

/**
 * @brief 串口命令处理函数
 * @param cmd 接收到的命令字符串
 */
void ProcessCommand(char *cmd);

/**
 * @brief 舵机更新函数
 * @details 根据当前动作状态计算各舵机角度并输出
 */
void UpdateServo(void);

/**
 * @brief 传感器数据读取函数
 * @details 读取光照、超声波、噪声传感器数据
 */
void ReadSensors(void);

/**
 * @brief 显示更新函数
 * @details 更新OLED屏幕显示内容
 */
void UpdateDisplay(void);

/**
 * @brief 延时函数（带系统tick计数）
 * @param ms 延时毫秒数
 */
void Tick_Delay(uint32_t ms);

/* ==================== 主函数 ==================== */

/**
 * @brief 主函数 - 四足机器人控制系统入口
 * @return 无
 * @details 完成系统初始化后进入主循环，执行多任务调度
 */
int main(void) {
    /* 系统时钟配置：72MHz */
    SystemClock_Config();
    
    /* GPIO初始化：LED、红外传感器引脚 */
    GPIO_Init_Robot();
    
    /* 舵机控制初始化：8路PWM输出 */
    Servo_Init();
    
    /* 串口初始化：115200波特率 */
    Serial_Init();
    
    /* 输入按键初始化 */
    Input_Init();
    
    /* 光照传感器BH1750初始化 */
    bh1750_init();
    
    /* 超声波传感器HC-SR04初始化 */
    hcsr04_init();
    
    /* 噪声传感器初始化 */
    Noise_Sensor_Init();
    
    /* LED指示灯初始化 */
    LED_Init();
    
    /* OLED显示初始化 */
    OLED_Init();
    OLED_Clear();
    OLED_Printf(0, 0, OLED_8X16, "Init...");
    OLED_Update();
    
    /* 等待1秒让传感器稳定 */
    Tick_Delay(1000);
    
    /* ==================== 主循环 ==================== */
    while (1) {
        /* 基础延时1ms，增加系统tick */
        Tick_Delay(1);
        
        /* LED2心跳指示：500ms周期闪烁 */
        if (system_tick % 500 < 250) {
            GPIO_ResetBits(GPIOA, GPIO_Pin_15);  /* LED2亮 */
        } else {
            GPIO_SetBits(GPIOA, GPIO_Pin_15);    /* LED2灭 */
        }
        
        /* 串口命令处理：每5ms检查一次 */
        if (system_tick % 5 == 0 && Serial_RxFlag == 1) {
            ProcessCommand(Serial_RxPacket);     /* 解析并执行命令 */
            Serial_RxFlag = 0;                   /* 清除接收标志 */
            Serial_RxPacket[0] = '\0';           /* 清空接收缓冲区 */
        }
        
        /* 传感器读取与显示更新：每100ms执行一次 */
        if (system_tick % 100 == 0) {
            ReadSensors();                       /* 读取所有传感器 */
            UpdateDisplay();                     /* 更新OLED显示 */
        }
        
        /* 数据采集模式：每1秒发送一次传感器数据 */
        if (system_tick % 1000 == 0 && collect_mode == 1) {
            Serial_Printf("CJ_LUX:%.1f_CJ_NOISE:%.1f\r\n", lux_value, noise_value);
        }
        
        /* 舵机动作更新：每250ms执行一次 */
        if (system_tick % 250 == 0) {
            step_counter++;                      /* 步数计数器递增 */
            UpdateServo();                       /* 更新舵机角度 */
        }
        
        /* 离地检测：每50ms检测一次 */
        if (system_tick % 50 == 0) {
            /* 读取PB7引脚状态（红外传感器输出） */
            uint8_t ir_state = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7);
            
            if (ir_state == 1) {
                /* 高电平：检测到离地，触发摇摆动作 */
                if (air_detect_state == 0 && action_state != 5) {
                    air_detect_state = 1;        /* 设置离地标志 */
                    action_state = 5;            /* 切换到摇摆动作 */
                    step_counter = 0;            /* 重置步数 */
                    auto_mode = 0;               /* 退出自动模式 */
                }
            } else {
                /* 低电平：检测到着地，恢复站立动作 */
                if (air_detect_state == 1) {
                    air_detect_state = 0;        /* 清除离地标志 */
                    action_state = 7;            /* 切换到平躺/站立 */
                    step_counter = 0;            /* 重置步数 */
                }
            }
        }
    }
}

/* ==================== 传感器读取函数 ==================== */

/**
 * @brief 读取所有传感器数据
 * @details 依次读取光照传感器、超声波传感器、噪声传感器数据
 *          并根据光照值控制LED1状态
 */
void ReadSensors(void) {
    float temp_lux, temp_distance;      /* 临时变量存储传感器值 */
    NoiseData_TypeDef noise_data;       /* 噪声传感器数据结构体 */
    
    /* 读取光照传感器数据 */
    temp_lux = bh1750_get_lux();
    if (temp_lux >= 0) {               /* 数据有效时更新 */
        lux_value = temp_lux;
    }
    
    /* 低光指示：光照<10 lux时LED1常亮 */
    if (lux_value < 10) {
        LED1_ON();
    } else {
        LED1_OFF();
    }
    
    /* 读取超声波传感器数据 */
    temp_distance = hcsr04_get_distance();
    if (temp_distance > 0) {            /* 数据有效时更新 */
        distance_value = temp_distance;
    }
    
    /* 读取噪声传感器数据 */
    Noise_Sensor_GetData(&noise_data);
    noise_value = noise_data.db_value;  /* 获取分贝值 */
}

/* ==================== 显示更新函数 ==================== */

/**
 * @brief 更新OLED屏幕显示内容
 * @details 显示光照、距离、噪声数据及运行模式
 */
void UpdateDisplay(void) {
    OLED_Clear();                        /* 清屏 */
    
    /* 显示光照强度 */
    OLED_Printf(0, 0, OLED_8X16, "Lux:%.1f lx", lux_value);
    
    /* 显示超声波距离 */
    OLED_Printf(0, 16, OLED_8X16, "Dist:%.1f cm", distance_value);
    
    /* 显示噪声分贝 */
    OLED_Printf(0, 32, OLED_8X16, "Noise:%.1f dB", noise_value);
    
    /* 显示运行模式 */
    OLED_Printf(0, 48, OLED_8X16, "Mode:%s", auto_mode ? "AUTO" : "MANU");
    
    OLED_Update();                       /* 更新显示缓冲区 */
}

/* ==================== GPIO初始化函数 ==================== */

/**
 * @brief 机器人GPIO引脚初始化
 * @details 配置LED输出引脚(PA12, PA15)和红外传感器输入引脚(PB7)
 */
void GPIO_Init_Robot(void) {
    /* 使能GPIOA、GPIOB、AFIO时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    
    /* 禁用JTAG，释放PB3/PB4用于普通GPIO */
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
    
    GPIO_InitTypeDef GPIO_InitStructure;
    
    /* 配置PA12、PA15为推挽输出（LED控制） */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    /* 初始状态：LED熄灭 */
    GPIO_SetBits(GPIOA, GPIO_Pin_12 | GPIO_Pin_15);
    
    /* 配置PB7为上拉输入（红外离地检测传感器） */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/* ==================== 命令处理函数 ==================== */

/**
 * @brief 串口命令解析与执行
 * @param cmd 接收到的命令字符串
 * @details 支持的命令：
 *          UP      - 前进
 *          BACK    - 后退
 *          LEFT    - 左转
 *          RIGHT   - 右转
 *          YB      - 摇摆
 *          ZK      - 深蹲
 *          ZX      - 平躺
 *          TJ      - 抬腿
 *          ZS      - 挥手
 *          AUTO    - 自动模式
 *          MANU    - 手动模式
 *          REST    - 复位
 *          STAT    - 状态查询
 *          CJ      - 数据采集模式
 */
void ProcessCommand(char *cmd) {
    uint8_t new_action = 0;              /* 解析出的新动作 */
    
    /* 解析命令字符串 */
    if (cmd[0] == 'U' && cmd[1] == 'P') new_action = 1;           /* UP - 前进 */
    else if (cmd[0] == 'B' && cmd[1] == 'A' && cmd[2] == 'C' && cmd[3] == 'K') new_action = 2;  /* BACK - 后退 */
    else if (cmd[0] == 'L' && cmd[1] == 'E' && cmd[2] == 'F' && cmd[3] == 'T') new_action = 3;  /* LEFT - 左转 */
    else if (cmd[0] == 'R' && cmd[1] == 'I' && cmd[2] == 'G' && cmd[3] == 'H' && cmd[4] == 'T') new_action = 4;  /* RIGHT - 右转 */
    else if (cmd[0] == 'Y' && cmd[1] == 'B') new_action = 5;       /* YB - 摇摆 */
    else if (cmd[0] == 'Z' && cmd[1] == 'K') new_action = 7;       /* ZK - 深蹲 */
    else if (cmd[0] == 'Z' && cmd[1] == 'X') new_action = 6;       /* ZX - 平躺 */
    else if (cmd[0] == 'T' && cmd[1] == 'J') new_action = 8;       /* TJ - 抬腿 */
    else if (cmd[0] == 'Z' && cmd[1] == 'S') new_action = 9;       /* ZS - 挥手 */
    else if (cmd[0] == 'C' && cmd[1] == 'J') {
        /* CJ - 数据采集模式 */
        auto_mode = 0; obstacle_avoiding = 0;
        collect_mode = 1;                /* 开启采集模式 */
        action_state = 7;                /* 切换到平躺状态 */
        step_counter = 0;
        return;
    } else if (cmd[0] == 'A' && cmd[1] == 'U' && cmd[2] == 'T' && cmd[3] == 'O') {
        /* AUTO - 自动巡航模式 */
        auto_mode = 1; target_action = 1; transition_state = 1; step_counter = 0;
        collect_mode = 0;                /* 关闭采集模式 */
        return;
    } else if (cmd[0] == 'M' && cmd[1] == 'A' && cmd[2] == 'N' && cmd[3] == 'U') {
        /* MANU - 手动控制模式 */
        auto_mode = 0; target_action = 0; transition_state = 0; step_counter = 0; obstacle_avoiding = 0;
        action_state = 7;                /* 切换到平躺状态 */
        collect_mode = 0;                /* 关闭采集模式 */
        return;
    } else if (cmd[0] == 'R' && cmd[1] == 'E' && cmd[2] == 'S' && cmd[3] == 'T') {
        /* REST - 系统复位 */
        auto_mode = 0; target_action = 0; transition_state = 0; step_counter = 0; obstacle_avoiding = 0;
        wave_state = 0; lift_state = 0; action_state = 0;
        collect_mode = 0;                /* 关闭采集模式 */
        /* 复位舵机到初始站立角度 */
        servo_angle[0] = 135; servo_angle[1] = 45;
        servo_angle[2] = 45; servo_angle[3] = 135;
        servo_angle[4] = 0; servo_angle[5] = 180;
        servo_angle[6] = 0; servo_angle[7] = 180;
        return;
    } else if (cmd[0] == 'S' && cmd[1] == 'T' && cmd[2] == 'A' && cmd[3] == 'T') {
        /* STAT - 状态查询 */
        Serial_Printf("Lux:%.1f, Dist:%.1f, Noise:%.1f\r\n", lux_value, distance_value, noise_value);
        return;
    }
    
    /* 处理动作切换 */
    if (new_action > 0) {
        auto_mode = 0; obstacle_avoiding = 0;
        
        /* 非平躺动作时关闭采集模式 */
        if (new_action != 7) {
            collect_mode = 0;
        }
        
        /* 需要过渡的情况：当前有动作且新动作不同且非平躺 */
        if (action_state != 0 && new_action != action_state && new_action != 7) {
            target_action = new_action;
            transition_state = 1;         /* 进入过渡状态 */
        } else {
            action_state = new_action;    /* 直接切换动作 */
            step_counter = 0;
        }
    }
}

/* ==================== 舵机更新函数 ==================== */

/**
 * @brief 舵机角度计算与输出
 * @details 根据当前动作状态计算8个舵机的目标角度，
 *          实现平滑的步态控制和动作切换
 */
void UpdateServo(void) {
    uint8_t current_action = action_state;  /* 当前执行的动作 */
    
    /* 动作过渡处理 */
    if (transition_state > 0) {
        switch (transition_state) {
            case 1:
                /* 过渡状态1：所有关节回中位(90°) */
                servo_angle[0] = 90.0f; servo_angle[1] = 90.0f;
                servo_angle[2] = 90.0f; servo_angle[3] = 90.0f;
                servo_angle[4] = 0.0f; servo_angle[5] = 180.0f;
                servo_angle[6] = 0.0f; servo_angle[7] = 180.0f;
                transition_state = 2;      /* 进入下一过渡状态 */
                break;
            case 2:
                /* 过渡状态2：调整到站立姿态 */
                servo_angle[0] = 135.0f; servo_angle[1] = 45.0f;
                servo_angle[2] = 45.0f; servo_angle[3] = 135.0f;
                servo_angle[4] = 0.0f; servo_angle[5] = 180.0f;
                servo_angle[6] = 0.0f; servo_angle[7] = 180.0f;
                transition_state = 0;      /* 完成过渡 */
                action_state = target_action;  /* 切换到目标动作 */
                step_counter = 0;
                break;
        }
    } else if (action_state == 1) {
        /* 自动避障逻辑：前进模式下检测障碍物 */
        if (distance_value > 0 && distance_value < 15) {
            obstacle_avoiding = 1;
            current_action = 3;           /* 强制左转 */
        } else {
            obstacle_avoiding = 0;
        }
    } else {
        obstacle_avoiding = 0;
    }
    
    /* 根据当前动作计算舵机角度 */
    switch (current_action) {
        case 1:  /* 前进动作 */
            switch (step_counter % 4) {
                case 1:  /* 对角腿抬起 */
                    servo_angle[0] = 135.0f; servo_angle[1] = 45.0f; servo_angle[2] = 45.0f; servo_angle[3] = 135.0f;
                    servo_angle[4] = 0.0f; servo_angle[5] = 150.0f; servo_angle[6] = 30.0f; servo_angle[7] = 180.0f;
                    break;
                case 2:  /* 左前腿前移 */
                    servo_angle[0] = 135.0f; servo_angle[1] = 15.0f; servo_angle[2] = 75.0f; servo_angle[3] = 135.0f;
                    servo_angle[4] = 0.0f; servo_angle[5] = 180.0f; servo_angle[6] = 0.0f; servo_angle[7] = 180.0f;
                    break;
                case 3:  /* 对角腿抬起 */
                    servo_angle[0] = 135.0f; servo_angle[1] = 15.0f; servo_angle[2] = 75.0f; servo_angle[3] = 135.0f;
                    servo_angle[4] = 30.0f; servo_angle[5] = 180.0f; servo_angle[6] = 0.0f; servo_angle[7] = 150.0f;
                    break;
                case 0:  /* 右前腿前移 */
                    servo_angle[0] = 165.0f; servo_angle[1] = 45.0f; servo_angle[2] = 45.0f; servo_angle[3] = 105.0f;
                    servo_angle[4] = 0.0f; servo_angle[5] = 180.0f; servo_angle[6] = 0.0f; servo_angle[7] = 180.0f;
                    break;
            }
            break;
        case 2:  /* 后退动作 */
            switch (step_counter % 4) {
                case 1:
                    servo_angle[0] = 135.0f; servo_angle[1] = 45.0f; servo_angle[2] = 45.0f; servo_angle[3] = 135.0f;
                    servo_angle[4] = 0.0f; servo_angle[5] = 150.0f; servo_angle[6] = 30.0f; servo_angle[7] = 180.0f;
                    break;
                case 2:
                    servo_angle[0] = 135.0f; servo_angle[1] = 75.0f; servo_angle[2] = 15.0f; servo_angle[3] = 135.0f;
                    servo_angle[4] = 0.0f; servo_angle[5] = 180.0f; servo_angle[6] = 0.0f; servo_angle[7] = 180.0f;
                    break;
                case 3:
                    servo_angle[0] = 135.0f; servo_angle[1] = 75.0f; servo_angle[2] = 15.0f; servo_angle[3] = 135.0f;
                    servo_angle[4] = 30.0f; servo_angle[5] = 180.0f; servo_angle[6] = 0.0f; servo_angle[7] = 150.0f;
                    break;
                case 0:
                    servo_angle[0] = 105.0f; servo_angle[1] = 45.0f; servo_angle[2] = 45.0f; servo_angle[3] = 165.0f;
                    servo_angle[4] = 0.0f; servo_angle[5] = 180.0f; servo_angle[6] = 0.0f; servo_angle[7] = 180.0f;
                    break;
            }
            break;
        case 3:  /* 左转动作 */
            switch (step_counter % 4) {
                case 1:
                    servo_angle[0] = 150.0f; servo_angle[1] = 30.0f; servo_angle[2] = 30.0f; servo_angle[3] = 150.0f;
                    servo_angle[4] = 30.0f; servo_angle[5] = 180.0f; servo_angle[6] = 0.0f; servo_angle[7] = 150.0f;
                    break;
                case 2:
                    servo_angle[0] = 165.0f; servo_angle[1] = 15.0f; servo_angle[2] = 15.0f; servo_angle[3] = 165.0f;
                    servo_angle[4] = 0.0f; servo_angle[5] = 180.0f; servo_angle[6] = 0.0f; servo_angle[7] = 180.0f;
                    break;
                case 3:
                    servo_angle[0] = 165.0f; servo_angle[1] = 15.0f; servo_angle[2] = 15.0f; servo_angle[3] = 165.0f;
                    servo_angle[4] = 0.0f; servo_angle[5] = 150.0f; servo_angle[6] = 30.0f; servo_angle[7] = 180.0f;
                    break;
                case 0:
                    servo_angle[0] = 135.0f; servo_angle[1] = 45.0f; servo_angle[2] = 45.0f; servo_angle[3] = 135.0f;
                    servo_angle[4] = 0.0f; servo_angle[5] = 180.0f; servo_angle[6] = 0.0f; servo_angle[7] = 180.0f;
                    break;
            }
            break;
        case 4:  /* 右转动作 */
            switch (step_counter % 4) {
                case 1:
                    servo_angle[0] = 150.0f; servo_angle[1] = 30.0f; servo_angle[2] = 30.0f; servo_angle[3] = 150.0f;
                    servo_angle[4] = 0.0f; servo_angle[5] = 150.0f; servo_angle[6] = 30.0f; servo_angle[7] = 180.0f;
                    break;
                case 2:
                    servo_angle[0] = 165.0f; servo_angle[1] = 15.0f; servo_angle[2] = 15.0f; servo_angle[3] = 165.0f;
                    servo_angle[4] = 0.0f; servo_angle[5] = 180.0f; servo_angle[6] = 0.0f; servo_angle[7] = 180.0f;
                    break;
                case 3:
                    servo_angle[0] = 165.0f; servo_angle[1] = 15.0f; servo_angle[2] = 15.0f; servo_angle[3] = 165.0f;
                    servo_angle[4] = 30.0f; servo_angle[5] = 180.0f; servo_angle[6] = 0.0f; servo_angle[7] = 150.0f;
                    break;
                case 0:
                    servo_angle[0] = 135.0f; servo_angle[1] = 45.0f; servo_angle[2] = 45.0f; servo_angle[3] = 135.0f;
                    servo_angle[4] = 0.0f; servo_angle[5] = 180.0f; servo_angle[6] = 0.0f; servo_angle[7] = 180.0f;
                    break;
            }
            break;
        case 5:  /* 摇摆动作 */
            servo_angle[0] = 135; servo_angle[1] = 45;
            servo_angle[2] = 45; servo_angle[3] = 135;
            /* 膝关节角度周期性变化，产生摇摆效果 */
            servo_angle[4] = (step_counter % 24 < 12 ? (step_counter % 12) * 5 : (24 - step_counter % 24) * 5);
            servo_angle[5] = 180 - servo_angle[4];
            servo_angle[6] = servo_angle[4];
            servo_angle[7] = 180 - servo_angle[4];
            break;
        case 6:  /* 深蹲动作 */
            servo_angle[0] = 135; servo_angle[1] = 45;
            servo_angle[2] = 45; servo_angle[3] = 135;
            servo_angle[4] = 90; servo_angle[5] = 90;
            servo_angle[6] = 90; servo_angle[7] = 90;
            break;
        case 7:  /* 平躺动作 */
            servo_angle[0] = 90; servo_angle[1] = 90;
            servo_angle[2] = 90; servo_angle[3] = 90;
            servo_angle[4] = 0; servo_angle[5] = 180;
            servo_angle[6] = 0; servo_angle[7] = 180;
            break;
        case 8:  /* 抬腿动作 */
            if (lift_state == 0) {
                servo_angle[0] = 90.0f; servo_angle[1] = 90.0f; servo_angle[2] = 45.0f; servo_angle[3] = 135.0f;
                servo_angle[4] = 0.0f; servo_angle[5] = 180.0f; servo_angle[6] = 135.0f; servo_angle[7] = 180.0f;
                lift_state = 1;
            } else {
                servo_angle[0] = 135.0f; servo_angle[1] = 45.0f; servo_angle[2] = 45.0f; servo_angle[3] = 135.0f;
                servo_angle[4] = 0.0f; servo_angle[5] = 180.0f; servo_angle[6] = 0.0f; servo_angle[7] = 180.0f;
                lift_state = 0;
            }
            action_state = 0;  /* 完成后返回站立状态 */
            break;
        case 9:  /* 挥手动作 */
            if (wave_state == 0) {
                servo_angle[0] = 160.0f; servo_angle[1] = 45.0f; servo_angle[2] = 110.0f; servo_angle[3] = 70.0f;
                servo_angle[4] = 20.0f; servo_angle[5] = 35.0f; servo_angle[6] = 20.0f; servo_angle[7] = 160.0f;
                wave_state = 1;
            } else {
                servo_angle[0] = 135.0f; servo_angle[1] = 45.0f; servo_angle[2] = 45.0f; servo_angle[3] = 135.0f;
                servo_angle[4] = 0.0f; servo_angle[5] = 180.0f; servo_angle[6] = 0.0f; servo_angle[7] = 180.0f;
                wave_state = 0;
            }
            action_state = 0;  /* 完成后返回站立状态 */
            break;
    }
    
    /* 将计算好的角度输出到各舵机 */
    Servo_SetAngle1(servo_angle[0]);
    Servo_SetAngle2(servo_angle[1]);
    Servo_SetAngle3(servo_angle[2]);
    Servo_SetAngle4(servo_angle[3]);
    Servo_SetAngle5(servo_angle[4]);
    Servo_SetAngle6(servo_angle[5]);
    Servo_SetAngle7(servo_angle[6]);
    Servo_SetAngle8(servo_angle[7]);
}

/* ==================== 延时函数 ==================== */

/**
 * @brief 简单延时函数（带系统tick计数）
 * @param ms 延时毫秒数
 * @details 基于CPU时钟的软件延时，同时更新系统tick计数器
 */
void Tick_Delay(uint32_t ms) {
    volatile uint32_t i;
    for (i = 0; i < ms * 7200; i++);  /* 72MHz时钟下约1ms延时 */
    system_tick += ms;                 /* 更新系统时间 */
}

/* ==================== 系统时钟配置函数 ==================== */

/**
 * @brief 系统时钟配置
 * @details 配置系统时钟为72MHz（使用外部晶振+HSE）
 */
void SystemClock_Config(void) {
    RCC_DeInit();                           /* 复位RCC配置 */
    RCC_HSEConfig(RCC_HSE_ON);              /* 使能外部晶振 */
    while (RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET);  /* 等待HSE稳定 */
    
    RCC_HCLKConfig(RCC_SYSCLK_Div1);        /* AHB时钟 = 系统时钟 */
    RCC_PCLK1Config(RCC_HCLK_Div2);         /* APB1时钟 = AHB/2 */
    RCC_PCLK2Config(RCC_HCLK_Div1);         /* APB2时钟 = AHB */
    
    FLASH_SetLatency(FLASH_Latency_2);      /* Flash等待周期2 */
    
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);  /* PLL倍频9倍 */
    RCC_PLLCmd(ENABLE);                     /* 使能PLL */
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);  /* 等待PLL稳定 */
    
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);  /* 系统时钟源设为PLL */
    while (RCC_GetSYSCLKSource() != 0x08);       /* 等待时钟切换完成 */
}
