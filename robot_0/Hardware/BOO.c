#include "stm32f10x.h"

void BOO_Init(void)
{
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	
	
	GPIO_InitTypeDef GPIO_InitStructure;				
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5| GPIO_Pin_14| GPIO_Pin_11;				
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		
	
	GPIO_Init(GPIOB, &GPIO_InitStructure);		
	
	
	GPIO_SetBits(GPIOB,GPIO_Pin_14);
	GPIO_SetBits(GPIOB,GPIO_Pin_5);
	GPIO_ResetBits(GPIOB,GPIO_Pin_11);
}

void HW_ON(void)
{
	
GPIO_ResetBits(GPIOB,GPIO_Pin_5);
	
}

void HW_OFF(void)
{
	
GPIO_SetBits(GPIOB,GPIO_Pin_5);

}

void BOO_ON(void)
{
	
GPIO_ResetBits(GPIOB,GPIO_Pin_14);
	
}

void BOO_OFF(void)
{
	
GPIO_SetBits(GPIOB,GPIO_Pin_14);

}


