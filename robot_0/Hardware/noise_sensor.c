/**
 * ============================================================================
 * 噪声传感器驱动
 * ============================================================================
 * @brief 基于ADC+DMA的噪声传感器驱动
 * @details 使用MAX9814麦克风放大器模块，通过STM32的ADC采集模拟信号，
 *          采用DMA自动传输方式实现连续采样，配合去极值平均滤波算法
 *          将ADC值转换为分贝(dB)值
 * @author 机器人控制项目组
 * @date 2026-05-25
 * @version v1.1
 */

#include "noise_sensor.h"
#include "stm32f10x_adc.h"
#include "stm32f10x.h"
#include "Delay.h"

/* ==================== 滤波参数配置 ==================== */

/** @brief 总采样数 */
#define FILTER_SAMPLES       20
/** @brief 去掉最小值数量 */
#define FILTER_REMOVE_MIN    5
/** @brief 去掉最大值数量 */
#define FILTER_REMOVE_MAX    5

/* ==================== 全局变量 ==================== */

/** @brief ADC转换结果（DMA自动更新） */
static vu16 ADC_ConvertedValue;

/* ==================== 公共API函数 ==================== */

/**
 * @brief 噪声传感器初始化
 * @details 配置ADC+DMA实现连续采样模式：
 *          1. 使能时钟（DMA1、ADC1、GPIOA）
 *          2. 配置PA5为模拟输入
 *          3. 配置DMA循环模式传输
 *          4. 配置ADC连续转换模式
 *          5. 启动ADC校准和转换
 */
void Noise_Sensor_Init(void) {
    ADC_InitTypeDef ADC_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    
    /* 使能DMA1时钟 */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    
    /* 使能ADC1和GPIOA时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    
    /* 配置PA5为模拟输入模式 */
    GPIO_InitStructure.GPIO_Pin = NOISE_ADC_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    /* 配置DMA通道1 */
    DMA_DeInit(DMA1_Channel1);                              /* 复位DMA配置 */
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)0x4001244C;  /* ADC_DR寄存器地址 */
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADC_ConvertedValue; /* 内存地址 */
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;       /* 外设作为数据源 */
    DMA_InitStructure.DMA_BufferSize = 1;                    /* 缓冲区大小 */
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  /* 外设地址不递增 */
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;          /* 内存地址不递增 */
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; /* 16位数据 */
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;       /* 16位数据 */
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;          /* 循环模式 */
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;      /* 高优先级 */
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;             /* 非内存到内存模式 */
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);             /* 初始化DMA */
    
    /* 使能DMA通道 */
    DMA_Cmd(DMA1_Channel1, ENABLE);
    
    /* 配置ADC1 */
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;       /* 独立模式 */
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;            /* 禁用扫描模式 */
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;       /* 连续转换模式 */
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; /* 软件触发 */
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;   /* 数据右对齐 */
    ADC_InitStructure.ADC_NbrOfChannel = 1;                  /* 转换通道数 */
    ADC_Init(ADC1, &ADC_InitStructure);                      /* 初始化ADC */
    
    /* 配置ADC通道5（PA5） */
    ADC_RegularChannelConfig(ADC1, NOISE_ADC_CHANNEL, 1, ADC_SampleTime_239Cycles5);
    
    /* 使能ADC DMA */
    ADC_DMACmd(ADC1, ENABLE);
    
    /* 使能ADC */
    ADC_Cmd(ADC1, ENABLE);
    
    /* ADC校准 */
    ADC_ResetCalibration(ADC1);                             /* 复位校准 */
    while(ADC_GetResetCalibrationStatus(ADC1));              /* 等待复位完成 */
    
    ADC_StartCalibration(ADC1);                             /* 开始校准 */
    while(ADC_GetCalibrationStatus(ADC1));                   /* 等待校准完成 */
    
    /* 启动软件触发转换 */
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

/* ==================== 内部函数 ==================== */

/**
 * @brief 获取滤波后的ADC值
 * @return 滤波后的ADC值
 * @details 采用去极值平均滤波算法：
 *          1. 采集20个ADC样本
 *          2. 冒泡排序
 *          3. 去掉前5个最小值和后5个最大值
 *          4. 返回中间10个值的平均值
 */
static unsigned int Noise_Sensor_GetFilteredADC(void) {
    unsigned int samples[FILTER_SAMPLES];  /* 采样数组 */
    unsigned int sum = 0;                  /* 总和 */
    unsigned int temp;                     /* 临时变量（用于排序） */
    unsigned char i, j;                    /* 循环变量 */
    
    /* 1. 采集FILTER_SAMPLES个样本 */
    for(i = 0; i < FILTER_SAMPLES; i++) {
        samples[i] = ADC_ConvertedValue;   /* 读取当前ADC值 */
        sum += samples[i];                 /* 累加 */
        Delay_us(1);                       /* 短暂延时 */
    }
    
    /* 2. 冒泡排序（从小到大） */
    for(i = 0; i < FILTER_SAMPLES - 1; i++) {
        for(j = 0; j < FILTER_SAMPLES - 1 - i; j++) {
            if(samples[j] > samples[j + 1]) {
                /* 交换 */
                temp = samples[j];
                samples[j] = samples[j + 1];
                samples[j + 1] = temp;
            }
        }
    }
    
    /* 3. 去掉最小值和最大值 */
    for(i = 0; i < FILTER_REMOVE_MIN; i++) {
        sum -= samples[i];                 /* 减去最小值 */
    }
    for(i = FILTER_SAMPLES - FILTER_REMOVE_MAX; i < FILTER_SAMPLES; i++) {
        sum -= samples[i];                 /* 减去最大值 */
    }
    
    /* 4. 返回中间值的平均值 */
    return sum / (FILTER_SAMPLES - FILTER_REMOVE_MIN - FILTER_REMOVE_MAX);
}

/* ==================== 公共API函数 ==================== */

/**
 * @brief 获取噪声分贝值
 * @return 噪声分贝值(dB)
 * @details 将滤波后的ADC值转换为分贝值，包含低音量补偿校准
 */
float Noise_Sensor_GetDB(void) {
    unsigned int adc_val = Noise_Sensor_GetFilteredADC();  /* 获取滤波后的ADC值 */
    
    /* ADC值转换为分贝值 */
    /* 公式说明：adc_val(0-4095) → 电压(0-3.3V) → 幅度 → 分贝 */
    float db_value = (adc_val * 120.0 / 4096.0 * 2.5);
    
    /* 低音量补偿：当测量值低于50dB时，增加40dB偏移 */
    if(db_value < 50) {
        db_value += 40;
    }
    
    return db_value;
}

/**
 * @brief 根据分贝值获取噪声等级
 * @param db_value 分贝值
 * @return 噪声等级枚举值
 * @details 噪声等级划分：
 *          - NOISE_LEVEL_QUIET:    < 40dB 安静
 *          - NOISE_LEVEL_NORMAL:   40-60dB 正常
 *          - NOISE_LEVEL_LOUD:     60-80dB 大声
 *          - NOISE_LEVEL_VERY_LOUD: >= 80dB 非常大声
 */
NoiseLevel_TypeDef Noise_Sensor_GetLevel(float db_value) {
    if(db_value < 40) {
        return NOISE_LEVEL_QUIET;
    } else if(db_value < 60) {
        return NOISE_LEVEL_NORMAL;
    } else if(db_value < 80) {
        return NOISE_LEVEL_LOUD;
    } else {
        return NOISE_LEVEL_VERY_LOUD;
    }
}

/**
 * @brief 获取完整的噪声传感器数据
 * @param data 指向NoiseData_TypeDef结构体的指针
 * @details 填充数据结构体，包含原始ADC值、电压值、分贝值和噪声等级
 */
void Noise_Sensor_GetData(NoiseData_TypeDef *data) {
    unsigned int adc_val = Noise_Sensor_GetFilteredADC();  /* 获取滤波后的ADC值 */
    
    data->adc_value = adc_val;                             /* 原始ADC值 */
    data->voltage = (adc_val / 4096.0) * 3.3;              /* 转换为电压值(0-3.3V) */
    data->db_value = Noise_Sensor_GetDB();                  /* 分贝值 */
    data->level = Noise_Sensor_GetLevel(data->db_value);    /* 噪声等级 */
}
