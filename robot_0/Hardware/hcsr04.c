#include "stm32f10x.h"
#include "hcsr04.h"
#include "Delay.h"


#define TRIG_PORT   GPIOB
#define TRIG_PIN    GPIO_Pin_11

#define ECHO_PORT   GPIOB
#define ECHO_PIN    GPIO_Pin_10

void hcsr04_init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    
    gpio.GPIO_Pin   = TRIG_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(TRIG_PORT, &gpio);

    GPIO_ResetBits(TRIG_PORT, TRIG_PIN);

    gpio.GPIO_Pin  = ECHO_PIN;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(ECHO_PORT, &gpio);
}

float hcsr04_get_distance(void)
{
    u32 timeout;
    u32 pulse_us;
    float distance;

    
    GPIO_ResetBits(TRIG_PORT, TRIG_PIN);
    Delay_us(2);
    GPIO_SetBits(TRIG_PORT, TRIG_PIN);
    Delay_us(15);
    GPIO_ResetBits(TRIG_PORT, TRIG_PIN);

   
    timeout = 0;
    while (GPIO_ReadInputDataBit(ECHO_PORT, ECHO_PIN) == 0)
    {
        timeout++;
        Delay_us(1);
        if (timeout > 10000) return -1;   // ??,?? -1
    }

    /* 3. ?? ECHO ??????? */
    pulse_us = 0;
    while (GPIO_ReadInputDataBit(ECHO_PORT, ECHO_PIN) == 1)
    {
        pulse_us++;
        Delay_us(1);
        if (pulse_us > 60000) return -2;  // ??,?? -2
    }

    /* 4. ????
     *    ?? = ?? * ?? / 2
     *    ?? = 340 m/s = 0.034 cm/us
     *    ??(cm) = pulse_us * 0.034 / 2 = pulse_us / 58.8
     */
    distance = (float)pulse_us / 58.0f;

    /* ?????? */
    if (distance > 400.0f) distance = 400.0f;
    if (distance < 2.0f)   distance = 0.0f;

    return distance;
}
