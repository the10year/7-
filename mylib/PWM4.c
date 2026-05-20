#include "stm32f10x.h"                  // Device header
//第九个舵机PWM

void PWM4_Init(void)
{
    /* 1. 使能时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);    // TIM1属于APB2总线，用APB2使能函数
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);   // PA8属于GPIOA，使能GPIOA时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);    // 复用功能需要AFIO时钟

    /* 2. GPIO初始化（PA8 复用推挽输出） */
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;         // 复用推挽输出（由TIM1控制）
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;               // PA8引脚（TIM1_CH1）
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);                  // 初始化GPIOA

    /* 3. 时基单元初始化（保持原ARR和PSC参数，50Hz PWM） */
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 20000 - 1;       // ARR = 19999（周期20ms）
    TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;       // PSC = 71（72MHz/72=1MHz计数频率）
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;    // 高级定时器重复计数器，此处不用
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);     // 初始化TIM1

    /* 4. 输出比较初始化（通道1） */
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure);                 // 结构体默认值初始化
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;       // PWM模式1
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;// 高电平有效
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // 输出使能
    TIM_OCInitStructure.TIM_Pulse = 0;                      // 初始CCR值（占空比0）
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);                // 初始化TIM1通道1

    /* 5. 高级定时器特有：使能主输出（MOE） */
    // TIM1是高级定时器，PWM输出需额外使能主输出，否则无波形
    TIM_CtrlPWMOutputs(TIM1, ENABLE);

    /* 6. 使能定时器 */
    TIM_Cmd(TIM1, ENABLE);                                  // 启动TIM1
}

/**
  * @brief  设置PWM占空比（修改TIM1通道1的CCR值）
  * @param  Compare: 期望的CCR值（范围0~19999，因ARR=19999）
  * @retval 无
  */
void PWM_SetCompare1_TIM1(uint16_t Compare)
{
    TIM_SetCompare1(TIM1, Compare);                         // 设置TIM1通道1的比较寄存器
}
