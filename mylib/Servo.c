#include "stm32f10x.h"                  // Device header
#include "PWM.h"


void Servo_Init(void)
{
	PWM_Init();					//TIM2
	PWM_Init_TIM3(); //TIM3
	PWM4_Init();	//TIM4
}


void Servo_SetAngle1(float Angle1)
{
	PWM_SetCompare1(Angle1 / 180 * 2000 + 500);	//设置占空比
	
}

void Servo_SetAngle2(float Angle2)
{
	
	PWM_SetCompare2(Angle2 / 180 * 2000 + 500);	
	
}
void Servo_SetAngle3(float Angle3)
{
	PWM_SetCompare3(Angle3 / 180 * 2000 + 500);
}

void Servo_SetAngle4(float Angle4)
{
	PWM_SetCompare4(Angle4 / 180 * 2000 + 500);
}

void Servo_SetAngle5(float Angle5)
{
	PWM_SetCompare1_TIM3(Angle5 / 180 * 2000 + 500);
}
void Servo_SetAngle6(float Angle6)
{
	PWM_SetCompare2_TIM3(Angle6 / 180 * 2000 + 500);
}
void Servo_SetAngle7(float Angle7)
{
	PWM_SetCompare3_TIM3(Angle7 / 180 * 2000 + 500);
}
void Servo_SetAngle8(float Angle8)
{
	PWM_SetCompare4_TIM3(Angle8 / 180 * 2000 + 500);
}
void Servo_SetAngle9(float Angle9)
{
	PWM_SetCompare1_TIM1(Angle9 / 180 * 2000 + 500);
}
