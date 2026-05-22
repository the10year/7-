#ifndef __PWM_H
#define __PWM_H

void PWM_Init(void);
void PWM_SetCompare1(uint16_t Compare);
void PWM_SetCompare2(uint16_t Compare);
void PWM_SetCompare3(uint16_t Compare);
void PWM_SetCompare4(uint16_t Compare);
void PWM_SetCompare1_TIM3(uint16_t Compare);
void PWM_SetCompare2_TIM3(uint16_t Compare);
void PWM_SetCompare3_TIM3(uint16_t Compare);
void PWM_SetCompare4_TIM3(uint16_t Compare);
void PWM_Init_TIM3(void);
void PWM_SetCompare1_TIM1(uint16_t Compare);
#endif 

