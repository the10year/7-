#include "stm32f10x.h"                  // Device header
#include "Delay.h"

// 按键状态枚举
typedef enum {
    KEY_STATE_IDLE,         // 空闲状态
    KEY_STATE_PRESS_DETECT, // 检测到按下
    KEY_STATE_CONFIRMED,    // 确认按下
    KEY_STATE_RELEASE       // 释放状态
} KeyState;

// 按键结构体（每个按键独立状态）
typedef struct {
    GPIO_TypeDef* GPIOx;
    uint16_t GPIO_Pin;
    KeyState state;
    uint8_t keyNum;
    uint8_t pressedFlag;    // 按下标志（仅触发一次）
    uint16_t pressTime;     // 按下计时
} Key_t;

// 按键数组初始化（对应8个按键）
Key_t keys[] = {
    {GPIOA, GPIO_Pin_3,  KEY_STATE_IDLE, 1, 0, 0},
    {GPIOA, GPIO_Pin_4,  KEY_STATE_IDLE, 2, 0, 0},
    {GPIOA, GPIO_Pin_5,  KEY_STATE_IDLE, 3, 0, 0},
    {GPIOA, GPIO_Pin_6,  KEY_STATE_IDLE, 4, 0, 0},
    {GPIOB, GPIO_Pin_0,  KEY_STATE_IDLE, 5, 0, 0},
    {GPIOB, GPIO_Pin_1,  KEY_STATE_IDLE, 6, 0, 0},
    {GPIOB, GPIO_Pin_10, KEY_STATE_IDLE, 7, 0, 0},
    {GPIOB, GPIO_Pin_11, KEY_STATE_IDLE, 8, 0, 0}
};

#define KEY_COUNT (sizeof(keys)/sizeof(Key_t))

void Key_Init(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    
    // 初始化A口按键
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // 初始化B口按键
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

// 按键扫描函数（10ms调用一次，非阻塞）
void Key_Scan(void) {
    for (uint8_t i = 0; i < KEY_COUNT; i++) {
        uint8_t pinState = GPIO_ReadInputDataBit(keys[i].GPIOx, keys[i].GPIO_Pin);
        
        switch (keys[i].state) {
            case KEY_STATE_IDLE:
                if (pinState == 0) {  // 检测到按键按下
                    keys[i].state = KEY_STATE_PRESS_DETECT;
                    keys[i].pressTime = 0;
                }
                break;
                
            case KEY_STATE_PRESS_DETECT:
                if (pinState == 0) {  // 持续按下
                    keys[i].pressTime++;
                    // 20ms防抖确认（10ms扫描一次，所以计数到2）
                    if (keys[i].pressTime >= 2) {
                        keys[i].state = KEY_STATE_CONFIRMED;
                        keys[i].pressedFlag = 1;  // 置按下标志
                    }
                } else {  // 误触释放
                    keys[i].state = KEY_STATE_IDLE;
                }
                break;
                
            case KEY_STATE_CONFIRMED:
                if (pinState == 1) {  // 按键释放
                    keys[i].state = KEY_STATE_RELEASE;
                }
                break;
                
            case KEY_STATE_RELEASE:
                keys[i].state = KEY_STATE_IDLE;  // 回到空闲状态
                break;
        }
    }
}

// 获取按键值
uint8_t Key_GetNum(void) {
    for (uint8_t i = 0; i < KEY_COUNT; i++) {
        if (keys[i].pressedFlag) {
            keys[i].pressedFlag = 0;  // 清除标志
            return keys[i].keyNum;
        }
    }
    return 0;
}
    
