#include "noise_sensor.h"
#include "stm32f10x_adc.h"
#include "stm32f10x.h"
#include "Delay.h"
#define FILTER_SAMPLES       20
#define FILTER_REMOVE_MIN    5
#define FILTER_REMOVE_MAX    5

static vu16 ADC_ConvertedValue;

void Noise_Sensor_Init(void) {
    ADC_InitTypeDef ADC_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = NOISE_ADC_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    DMA_DeInit(DMA1_Channel1);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)0x4001244C;
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADC_ConvertedValue;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = 1;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);
    
    DMA_Cmd(DMA1_Channel1, ENABLE);
    
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);
    
    ADC_RegularChannelConfig(ADC1, NOISE_ADC_CHANNEL, 1, ADC_SampleTime_239Cycles5);
    
    ADC_DMACmd(ADC1, ENABLE);
    ADC_Cmd(ADC1, ENABLE);
    
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
    
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

static unsigned int Noise_Sensor_GetFilteredADC(void) {
    unsigned int samples[FILTER_SAMPLES];
    unsigned int sum = 0;
    unsigned int temp;
    unsigned char i, j;
    
    for(i = 0; i < FILTER_SAMPLES; i++) {
        samples[i] = ADC_ConvertedValue;
        sum += samples[i];
        Delay_us(1);
    }
    
    for(i = 0; i < FILTER_SAMPLES - 1; i++) {
        for(j = 0; j < FILTER_SAMPLES - 1 - i; j++) {
            if(samples[j] > samples[j + 1]) {
                temp = samples[j];
                samples[j] = samples[j + 1];
                samples[j + 1] = temp;
            }
        }
    }
    
    for(i = 0; i < FILTER_REMOVE_MIN; i++) {
        sum -= samples[i];
    }
    for(i = FILTER_SAMPLES - FILTER_REMOVE_MAX; i < FILTER_SAMPLES; i++) {
        sum -= samples[i];
    }
    
    return sum / (FILTER_SAMPLES - FILTER_REMOVE_MIN - FILTER_REMOVE_MAX);
}

float Noise_Sensor_GetDB(void) {
    unsigned int adc_val = Noise_Sensor_GetFilteredADC();
    float db_value = (adc_val * 120.0 / 4096.0 * 2.5);
    
    if(db_value < 50) {
        db_value += 40;
    }
    
    return db_value;
}

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

void Noise_Sensor_GetData(NoiseData_TypeDef *data) {
    unsigned int adc_val = Noise_Sensor_GetFilteredADC();
    
    data->adc_value = adc_val;
    data->voltage = (adc_val / 4096.0) * 3.3;
    data->db_value = Noise_Sensor_GetDB();
    data->level = Noise_Sensor_GetLevel(data->db_value);
}
