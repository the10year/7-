#include "WS2812B.h"
#include "Delay.h"

// DMA 缓冲区，用于存储每个 LED 的 24 位数据
static uint16_t dma_buffer[LED_NUM * 24] = {0};  // 初始化清零，避免残留数据

// 初始化定时器PWM（PB7对应TIM4_CH2）
static void TIM4_PWM_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
    TIM_OCInitTypeDef TIM_OCInitStruct;

    // 使能 GPIOB、AFIO和TIM4时钟（TIM4属于APB1）
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

   

    // 配置 PB7 为复用推挽输出（PWM 输出）
    GPIO_InitStruct.GPIO_Pin = WS2812_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(WS2812_PORT, &GPIO_InitStruct);

    // 定时器基础配置
    TIM_TimeBaseStruct.TIM_Prescaler = 0;  // 不分频，72MHz
    TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStruct.TIM_Period = PWM_PERIOD - 1;  // 89
    TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(WS2812_TIM, &TIM_TimeBaseStruct);

    // PWM 通道2配置（TIM4_CH2）
    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_Pulse = 0;
    TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC2Init(WS2812_TIM, &TIM_OCInitStruct);  // 配置通道2
    TIM_OC2PreloadConfig(WS2812_TIM, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(WS2812_TIM, ENABLE);
    TIM_Cmd(WS2812_TIM, ENABLE);
}

// 初始化DMA（TIM4_CH2对应DMA1_Channel4）
static void WS2812_DMA_Init(void) {
    DMA_InitTypeDef DMA_InitStruct;

    // 启用 DMA1 时钟
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    // 复位DMA1_Channel4
    DMA_DeInit(WS2812_DMA);

    // 配置 DMA
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&WS2812_TIM->CCR2;  // TIM4_CH2比较寄存器
    DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)dma_buffer;
    DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStruct.DMA_BufferSize = LED_NUM * 24;
    DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStruct.DMA_Priority = DMA_Priority_High;
    DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(WS2812_DMA, &DMA_InitStruct);

    // 使能TIM4_CH2的DMA请求
    TIM_DMACmd(WS2812_TIM, TIM_DMA_CC2, ENABLE);
}

// 初始化WS2812相关硬件
void WS2812B_Init(void) {
    TIM4_PWM_Init();
    WS2812_DMA_Init();
}

// 设置LED颜色（GRB顺序）
void Set_LED_Color(uint8_t led_num, uint8_t red, uint8_t green, uint8_t blue) {
    if (led_num >= LED_NUM) return;
    
    uint32_t color = (green << 16) | (red << 8) | blue;  // WS2812B要求GRB格式
    for (int i = 0; i < 24; i++) {
        if (color & (1 << (23 - i))) {  // 高位先发
            dma_buffer[led_num * 24 + i] = T1H;
        } else {
            dma_buffer[led_num * 24 + i] = T0H;
        }
    }
}

// 更新LED数据
void WS2812_Update(void) {
    DMA_Cmd(WS2812_DMA, DISABLE);
    DMA_SetCurrDataCounter(WS2812_DMA, LED_NUM * 24);
    DMA_ClearFlag(DMA1_FLAG_TC4);  // 清除DMA1_Channel4完成标志
    DMA_Cmd(WS2812_DMA, ENABLE);

    // 等待DMA传输完成（带超时机制）
    uint32_t timeout = 0xFFFF;
    while (DMA_GetFlagStatus(DMA1_FLAG_TC4) == RESET && timeout--);
    DMA_ClearFlag(DMA1_FLAG_TC4);

    // 发送复位信号（延长至200μs确保识别）
    TIM_Cmd(WS2812_TIM, DISABLE);
    GPIO_ResetBits(WS2812_PORT, WS2812_PIN);
    Delay_us(200);
    TIM_SetCompare2(WS2812_TIM, 0);  // 对应通道2
    TIM_Cmd(WS2812_TIM, ENABLE);
}

// 流水灯效果
void Color_Wipe(uint8_t red, uint8_t green, uint8_t blue, uint32_t delay) {
    for(uint8_t i = 0; i < LED_NUM; i++) {
        Set_LED_Color(i, red, green, blue);
        WS2812_Update();
        Delay_us(delay);
    }
    // 清空所有LED
    for(uint8_t i = 0; i < LED_NUM; i++) {
        Set_LED_Color(i, 0, 0, 0);
    }
    WS2812_Update();
    Delay_us(delay);
}
// 从第10位开始的流水灯，顺序为10→11→...→24→0→1→...→9，所有点亮的LED保持常亮
void FlowFrom10To9_AllOn(uint8_t red, uint8_t green, uint8_t blue, uint32_t delay) {
    // 确保LED数量为25个(0-24)
    if (LED_NUM != 16) return;
    
    // 先关闭所有LED
    for(uint8_t i = 0; i < LED_NUM; i++) {
        Set_LED_Color(i, 0, 0, 0);
    }
    WS2812_Update();
    Delay_us(delay);
    
    // 从第10位(索引9)开始，到第24位(索引24)
    for(uint8_t i = 0; i < LED_NUM; i++) {
        // 点亮当前LED（之前的LED保持点亮）
        Set_LED_Color(i, red, green, blue);
        WS2812_Update();
        Delay_us(delay);
    }
    
    // 从第0位到第8位
    for(uint8_t i = 0; i < 9; i++) {
        // 点亮当前LED（之前的LED保持点亮）
        Set_LED_Color(i, red, green, blue);
        WS2812_Update();
        Delay_us(delay);
    }
    
    // 全部点亮后可以保持一段时间
    Delay_us(delay * 2);
    
    // 最后关闭所有LED，准备下一次循环
    for(uint8_t i = 0; i < LED_NUM; i++) {
        Set_LED_Color(i, 0, 0, 0);
    }
    WS2812_Update();
}

void ColorFlowFrom10To9_LargeChange(uint32_t delay) {
    // 确保LED数量为25个（索引0-24）
    if (LED_NUM != 16) return;
    
    // 定义25种差异明显的颜色（红、橙、黄、绿、青、蓝、紫等间隔分布）
    const uint8_t colors[25][3] = {
        {255, 0, 0},    // 红
        {255, 128, 0},  // 橙红
        {255, 255, 0},  // 黄
        {128, 255, 0},  // 黄绿
        {0, 255, 0},    // 绿
        {0, 255, 128},  // 青绿
        {0, 255, 255},  // 青
        {0, 128, 255},  // 蓝绿
        {0, 0, 255},    // 蓝
        {128, 0, 255},  // 靛蓝
        {255, 0, 255},  // 紫
        {255, 0, 128},  // 紫红
        {255, 100, 100}, // 粉红
        {100, 255, 100}, // 淡绿
        {100, 100, 255}, // 淡蓝
        {255, 255, 100}, // 淡金黄
      
    };
    
    // 先关闭所有LED
    for (uint8_t i = 0; i < LED_NUM; i++) {
        Set_LED_Color(i, 0, 0, 0);
    }
    WS2812_Update();
    Delay_us(delay);
    
    uint8_t color_idx = 0;  // 颜色索引计数器
    
    // 第一阶段：从第10位(索引9)开始，到第24位(索引24)
    for (uint8_t i = 0; i < LED_NUM; i++) {
        // 为当前LED设置差异明显的颜色
        Set_LED_Color(i, colors[color_idx][0], colors[color_idx][1], colors[color_idx][2]);
        WS2812_Update();
        Delay_us(delay);
        
        color_idx++;  // 切换到下一个差异明显的颜色
    }
    
    // 第二阶段：从第0位到第8位
    for (uint8_t i = 0; i < 9; i++) {
        // 继续为当前LED设置差异明显的颜色
        Set_LED_Color(i, colors[color_idx][0], colors[color_idx][1], colors[color_idx][2]);
        WS2812_Update();
        Delay_us(delay);
        
        color_idx++;  // 切换到下一个差异明显的颜色
    }
    
    // 全部点亮后保持一段时间
    Delay_us(delay * 2);
    
    // 最后关闭所有LED
    for (uint8_t i = 0; i < LED_NUM; i++) {
        Set_LED_Color(i, 0, 0, 0);
    }
    WS2812_Update();
}


void All_LED_On(uint8_t red, uint8_t green, uint8_t blue) {
    // 遍历所有LED，设置为目标颜色
    for(uint8_t i = 0; i < LED_NUM; i++) {
        Set_LED_Color(i, red, green, blue);
    }
    WS2812_Update();  // 一次性更新所有LED
}
