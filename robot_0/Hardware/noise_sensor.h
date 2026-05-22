#ifndef __NOISE_SENSOR_H
#define __NOISE_SENSOR_H


#define NOISE_ADC_CHANNEL    ADC_Channel_5      // ? PA5
#define NOISE_ADC_PIN        GPIO_Pin_5         // ? PA5

typedef enum {
    NOISE_LEVEL_QUIET = 0,
    NOISE_LEVEL_NORMAL,
    NOISE_LEVEL_LOUD,
    NOISE_LEVEL_VERY_LOUD
} NoiseLevel_TypeDef;

typedef struct {
    float adc_value;
    float voltage;
    float db_value;
    NoiseLevel_TypeDef level;
} NoiseData_TypeDef;

void Noise_Sensor_Init(void);
void Noise_Sensor_GetData(NoiseData_TypeDef *data);
float Noise_Sensor_GetDB(void);
NoiseLevel_TypeDef Noise_Sensor_GetLevel(float db_value);

#endif
