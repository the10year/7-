#ifndef WS2812B_H
#define WS2812B_H

#include "stm32f10x.h"

// 硬件定义（PB7引脚配置）
#define WS2812_PIN        GPIO_Pin_7      // WS2812B 数据引脚连接到 PB7
#define WS2812_PORT       GPIOB           // 端口为GPIOB
#define WS2812_TIM        TIM4             // 使用定时器TIM4
#define WS2812_TIM_CH     TIM_Channel_2    // 使用TIM4的通道2
#define WS2812_DMA        DMA1_Channel4    // 使用DMA1通道4
#define LED_NUM           16                // 9位LED

// PWM时序参数（72MHz系统时钟）
#define PWM_PERIOD        90               // 72MHz/90=800kHz
#define T0H               30               // 0码高电平时间（≈0.417μs）
#define T1H               59               // 1码高电平时间（≈0.819μs）

// 函数声明
void WS2812B_Init(void);                     // 初始化WS2812B相关硬件
void Set_LED_Color(uint8_t led_num, uint8_t red, uint8_t green, uint8_t blue); // 设置LED颜色
void WS2812_Update(void);                   // 更新LED数据
void Color_Wipe(uint8_t red, uint8_t green, uint8_t blue, uint32_t delay);    // 流水灯效果
void All_LED_On(uint8_t red, uint8_t green, uint8_t blue); 
void FlowFrom10To9_AllOn(uint8_t red, uint8_t green, uint8_t blue, uint32_t delay);
void ColorFlowFrom10To9_LargeChange(uint32_t delay);
#endif
