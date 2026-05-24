/**
 * ============================================================================
 * 主程序 - 四足机器人控制系统 (优化修复版)
 * ============================================================================
 */

#include "stm32f10x.h"
#include "Serial.h"
#include "Servo.h"
#include "Input.h"
#include "bh1750.h"
#include "hcsr04.h"
#include "OLED.h"
#include "noise_sensor.h"
#include <string.h>

void SystemClock_Config(void);

volatile uint32_t system_tick = 0;
volatile uint8_t action_state = 0;
volatile uint16_t step_counter = 0;
volatile uint8_t auto_mode = 0;
volatile uint8_t wave_state = 0;
volatile uint8_t lift_state = 0;
volatile uint8_t target_action = 0;
volatile uint8_t transition_state = 0;
volatile uint8_t obstacle_avoiding = 0;
volatile uint8_t infrared_state = 0;
volatile uint8_t swing_state = 0;

float servo_angle[8] = {90, 90, 90, 90, 0, 180, 0, 180};
float lux_value = 0;
float distance_value = 0;
float noise_value = 0;

void GPIO_Init_Robot(void);
void ProcessCommand(char *cmd);
void UpdateServo(void);
void ReadSensors(void);
void UpdateDisplay(void);
void Tick_Delay(uint32_t ms);

int main(void) {
    SystemClock_Config();
    GPIO_Init_Robot();
    Servo_Init();
    Serial_Init();
    Input_Init();
    bh1750_init();
    hcsr04_init();
    Noise_Sensor_Init();
    
    OLED_Init();
    OLED_Clear();
    OLED_Printf(0, 0, OLED_8X16, "Init...");
    OLED_Update();
    Tick_Delay(1000);
    
    while (1) {
        Tick_Delay(1);
        
        if (system_tick % 500 < 250) {
            GPIO_ResetBits(GPIOA, GPIO_Pin_15);
        } else {
            GPIO_SetBits(GPIOA, GPIO_Pin_15);
        }
        
        if (system_tick % 5 == 0 && Serial_RxFlag == 1) {
            ProcessCommand(Serial_RxPacket);
            Serial_RxFlag = 0;
            Serial_RxPacket[0] = '\0';
        }
        
        if (system_tick % 100 == 0) {
            ReadSensors();
            UpdateDisplay();
        }
        
        if (system_tick % 250 == 0) {
            step_counter++;
            UpdateServo();
        }
    }
}

void ReadSensors(void) {
    float temp_lux, temp_distance;
    NoiseData_TypeDef noise_data;
    
    temp_lux = bh1750_get_lux();
    if (temp_lux >= 0) lux_value = temp_lux;
    
    temp_distance = hcsr04_get_distance();
    if (temp_distance > 0) distance_value = temp_distance;
    
    Noise_Sensor_GetData(&noise_data);
    noise_value = noise_data.db_value;
    
    infrared_state = Input_GetInfrared();
}

void UpdateDisplay(void) {
    OLED_Clear();
    OLED_Printf(0, 0, OLED_8X16, "Lux:%.1f lx", lux_value);
    OLED_Printf(0, 16, OLED_8X16, "Dist:%.1f cm", distance_value);
    OLED_Printf(0, 32, OLED_8X16, "Noise:%.1f dB", noise_value);
    OLED_Printf(0, 48, OLED_8X16, "Mode:%s", auto_mode ? "AUTO" : "MANU");
    OLED_Update();
}

void GPIO_Init_Robot(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
    
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_SetBits(GPIOA, GPIO_Pin_12 | GPIO_Pin_15);
}

void ProcessCommand(char *cmd) {
    uint8_t new_action = 0;
    
    if (cmd[0] == 'U' && cmd[1] == 'P') new_action = 1;
    else if (cmd[0] == 'B' && cmd[1] == 'A' && cmd[2] == 'C' && cmd[3] == 'K') new_action = 2;
    else if (cmd[0] == 'L' && cmd[1] == 'E' && cmd[2] == 'F' && cmd[3] == 'T') new_action = 3;
    else if (cmd[0] == 'R' && cmd[1] == 'I' && cmd[2] == 'G' && cmd[3] == 'H' && cmd[4] == 'T') new_action = 4;
    else if (cmd[0] == 'Y' && cmd[1] == 'B') new_action = 5;
    else if (cmd[0] == 'Z' && cmd[1] == 'K') new_action = 7;
    else if (cmd[0] == 'Z' && cmd[1] == 'X') new_action = 6;
    else if (cmd[0] == 'T' && cmd[1] == 'J') new_action = 8;
    else if (cmd[0] == 'Z' && cmd[1] == 'S') new_action = 9;
    else if (cmd[0] == 'A' && cmd[1] == 'U' && cmd[2] == 'T' && cmd[3] == 'O') {
        auto_mode = 1; target_action = 1; transition_state = 1; step_counter = 0;
        return;
    } else if (cmd[0] == 'M' && cmd[1] == 'A' && cmd[2] == 'N' && cmd[3] == 'U') {
        auto_mode = 0; target_action = 0; transition_state = 0; step_counter = 0; obstacle_avoiding = 0;
        action_state = 7;
        return;
    } else if (cmd[0] == 'R' && cmd[1] == 'E' && cmd[2] == 'S' && cmd[3] == 'T') {
        auto_mode = 0; target_action = 0; transition_state = 0; step_counter = 0; obstacle_avoiding = 0;
        wave_state = 0; lift_state = 0; action_state = 0;
        servo_angle[0] = 135; servo_angle[1] = 45;
        servo_angle[2] = 45; servo_angle[3] = 135;
        servo_angle[4] = 0; servo_angle[5] = 180;
        servo_angle[6] = 0; servo_angle[7] = 180;
        return;
    } else if (cmd[0] == 'S' && cmd[1] == 'T' && cmd[2] == 'A' && cmd[3] == 'T') {
        Serial_Printf("Lux:%.1f, Dist:%.1f, Noise:%.1f\r\n", lux_value, distance_value, noise_value);
        return;
    }
    
    if (new_action > 0) {
        auto_mode = 0; obstacle_avoiding = 0;
        if (action_state != 0 && new_action != action_state && new_action != 7) {
            target_action = new_action;
            transition_state = 1;
        } else {
            action_state = new_action;
            step_counter = 0;
        }
    }
}

void UpdateServo(void) {
    uint8_t current_action = action_state;
    
    if (infrared_state == 1) {
        if (swing_state == 0) {
            swing_state = 1;
        }
    } else {
        if (swing_state == 1) {
            swing_state = 0;
        }
    }
    
    if (swing_state == 1) {
        switch (step_counter % 8) {
            case 0:
                servo_angle[0] = 135.0f; servo_angle[1] = 45.0f;
                servo_angle[2] = 45.0f; servo_angle[3] = 135.0f;
                servo_angle[4] = 30.0f; servo_angle[5] = 150.0f;
                servo_angle[6] = 0.0f; servo_angle[7] = 180.0f;
                break;
            case 2:
                servo_angle[0] = 150.0f; servo_angle[1] = 30.0f;
                servo_angle[2] = 30.0f; servo_angle[3] = 150.0f;
                servo_angle[4] = 0.0f; servo_angle[5] = 180.0f;
                servo_angle[6] = 30.0f; servo_angle[7] = 150.0f;
                break;
            case 4:
                servo_angle[0] = 135.0f; servo_angle[1] = 45.0f;
                servo_angle[2] = 45.0f; servo_angle[3] = 135.0f;
                servo_angle[4] = 0.0f; servo_angle[5] = 180.0f;
                servo_angle[6] = 30.0f; servo_angle[7] = 150.0f;
                break;
            case 6:
                servo_angle[0] = 150.0f; servo_angle[1] = 30.0f;
                servo_angle[2] = 30.0f; servo_angle[3] = 150.0f;
                servo_angle[4] = 30.0f; servo_angle[5] = 150.0f;
                servo_angle[6] = 0.0f; servo_angle[7] = 180.0f;
                break;
        }
        Servo_SetAngle1(servo_angle[0]);
        Servo_SetAngle2(servo_angle[1]);
        Servo_SetAngle3(servo_angle[2]);
        Servo_SetAngle4(servo_angle[3]);
        Servo_SetAngle5(servo_angle[4]);
        Servo_SetAngle6(servo_angle[5]);
        Servo_SetAngle7(servo_angle[6]);
        Servo_SetAngle8(servo_angle[7]);
        return;
    }
    
    if (transition_state > 0) {
        switch (transition_state) {
            case 1:
                servo_angle[0] = 90.0f; servo_angle[1] = 90.0f;
                servo_angle[2] = 90.0f; servo_angle[3] = 90.0f;
                servo_angle[4] = 0.0f; servo_angle[5] = 180.0f;
                servo_angle[6] = 0.0f; servo_angle[7] = 180.0f;
                transition_state = 2;
                break;
            case 2:
                servo_angle[0] = 135.0f; servo_angle[1] = 45.0f;
                servo_angle[2] = 45.0f; servo_angle[3] = 135.0f;
                servo_angle[4] = 0.0f; servo_angle[5] = 180.0f;
                servo_angle[6] = 0.0f; servo_angle[7] = 180.0f;
                transition_state = 0;
                action_state = target_action;
                step_counter = 0;
                break;
        }
    } else if (action_state == 1) {
        if (distance_value > 0 && distance_value < 15) {
            obstacle_avoiding = 1;
            current_action = 3;
        } else {
            obstacle_avoiding = 0;
        }
    } else {
        obstacle_avoiding = 0;
    }
    
    switch (current_action) {
        case 1:
            switch (step_counter % 4) {
                case 1:
                    servo_angle[0] = 135.0f; servo_angle[1] = 45.0f; servo_angle[2] = 45.0f; servo_angle[3] = 135.0f;
                    servo_angle[4] = 0.0f; servo_angle[5] = 150.0f; servo_angle[6] = 30.0f; servo_angle[7] = 180.0f;
                    break;
                case 2:
                    servo_angle[0] = 135.0f; servo_angle[1] = 15.0f; servo_angle[2] = 75.0f; servo_angle[3] = 135.0f;
                    servo_angle[4] = 0.0f; servo_angle[5] = 180.0f; servo_angle[6] = 0.0f; servo_angle[7] = 180.0f;
                    break;
                case 3:
                    servo_angle[0] = 135.0f; servo_angle[1] = 15.0f; servo_angle[2] = 75.0f; servo_angle[3] = 135.0f;
                    servo_angle[4] = 30.0f; servo_angle[5] = 180.0f; servo_angle[6] = 0.0f; servo_angle[7] = 150.0f;
                    break;
                case 0:
                    servo_angle[0] = 165.0f; servo_angle[1] = 45.0f; servo_angle[2] = 45.0f; servo_angle[3] = 105.0f;
                    servo_angle[4] = 0.0f; servo_angle[5] = 180.0f; servo_angle[6] = 0.0f; servo_angle[7] = 180.0f;
                    break;
            }
            break;
        case 2:
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
        case 3:
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
        case 4:
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
        case 5:
            servo_angle[0] = 135; servo_angle[1] = 45;
            servo_angle[2] = 45; servo_angle[3] = 135;
            servo_angle[4] = (step_counter % 24 < 12 ? (step_counter % 12) * 5 : (24 - step_counter % 24) * 5);
            servo_angle[5] = 180 - servo_angle[4];
            servo_angle[6] = servo_angle[4];
            servo_angle[7] = 180 - servo_angle[4];
            break;
        case 6:
            servo_angle[0] = 135; servo_angle[1] = 45;
            servo_angle[2] = 45; servo_angle[3] = 135;
            servo_angle[4] = 90; servo_angle[5] = 90;
            servo_angle[6] = 90; servo_angle[7] = 90;
            break;
        case 7:
            servo_angle[0] = 90; servo_angle[1] = 90;
            servo_angle[2] = 90; servo_angle[3] = 90;
            servo_angle[4] = 0; servo_angle[5] = 180;
            servo_angle[6] = 0; servo_angle[7] = 180;
            break;
        case 8:
            if (lift_state == 0) {
                servo_angle[0] = 90.0f; servo_angle[1] = 90.0f; servo_angle[2] = 45.0f; servo_angle[3] = 135.0f;
                servo_angle[4] = 0.0f; servo_angle[5] = 180.0f; servo_angle[6] = 135.0f; servo_angle[7] = 180.0f;
                lift_state = 1;
            } else {
                servo_angle[0] = 135.0f; servo_angle[1] = 45.0f; servo_angle[2] = 45.0f; servo_angle[3] = 135.0f;
                servo_angle[4] = 0.0f; servo_angle[5] = 180.0f; servo_angle[6] = 0.0f; servo_angle[7] = 180.0f;
                lift_state = 0;
            }
            action_state = 0;
            break;
        case 9:
            if (wave_state == 0) {
                servo_angle[0] = 160.0f; servo_angle[1] = 45.0f; servo_angle[2] = 110.0f; servo_angle[3] = 70.0f;
                servo_angle[4] = 20.0f; servo_angle[5] = 35.0f; servo_angle[6] = 20.0f; servo_angle[7] = 160.0f;
                wave_state = 1;
            } else {
                servo_angle[0] = 135.0f; servo_angle[1] = 45.0f; servo_angle[2] = 45.0f; servo_angle[3] = 135.0f;
                servo_angle[4] = 0.0f; servo_angle[5] = 180.0f; servo_angle[6] = 0.0f; servo_angle[7] = 180.0f;
                wave_state = 0;
            }
            action_state = 0;
            break;
    }
    
    Servo_SetAngle1(servo_angle[0]);
    Servo_SetAngle2(servo_angle[1]);
    Servo_SetAngle3(servo_angle[2]);
    Servo_SetAngle4(servo_angle[3]);
    Servo_SetAngle5(servo_angle[4]);
    Servo_SetAngle6(servo_angle[5]);
    Servo_SetAngle7(servo_angle[6]);
    Servo_SetAngle8(servo_angle[7]);
}

void Tick_Delay(uint32_t ms) {
    volatile uint32_t i;
    for (i = 0; i < ms * 7200; i++);
    system_tick += ms;
}

void SystemClock_Config(void) {
    RCC_DeInit();
    RCC_HSEConfig(RCC_HSE_ON);
    while (RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET);
    RCC_HCLKConfig(RCC_SYSCLK_Div1);
    RCC_PCLK1Config(RCC_HCLK_Div2);
    RCC_PCLK2Config(RCC_HCLK_Div1);
    FLASH_SetLatency(FLASH_Latency_2);
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
    RCC_PLLCmd(ENABLE);
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    while (RCC_GetSYSCLKSource() != 0x08);
}
