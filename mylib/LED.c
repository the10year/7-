#include "stm32f10x.h"                  // Device header
#include "Delay.h"
extern volatile uint8_t LED_State;
extern volatile uint16_t LED_DelayCnt;
void LED_Init(void)
{

	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	 GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOA, GPIO_Pin_12 | GPIO_Pin_15);
}

void LED1_ON(void)
{
	GPIO_ResetBits(GPIOA, GPIO_Pin_12);
}

void LED1_OFF(void)
{
	GPIO_SetBits(GPIOA, GPIO_Pin_12);
}

\

void LED2_ON(void)
{
	GPIO_ResetBits(GPIOA, GPIO_Pin_15);
}

void LED2_OFF(void)
{
	GPIO_SetBits(GPIOA, GPIO_Pin_15);
}


void LED_StartBlink(void)
{
    LED_State = 1;  
    LED_DelayCnt = 0;
}


void LED_StopBlink(void)
{
    LED_State = 0;

    GPIO_SetBits(GPIOA, GPIO_Pin_12 | GPIO_Pin_15);
}
