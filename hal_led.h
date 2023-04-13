#ifndef _HAL_LED_H
#define _HAL_LED_H

#define LED1_PORT			GPIOA
#define LED1_PIN			GPIO_Pin_8

#define LED2_PORT			GPIOB
#define LED2_PIN			GPIO_Pin_15


typedef enum
{
	LED1,             //beep
	LED2,             //led
	LED_SUM
}LED_TYPEDEF;


typedef enum
{
	LED_DARK,
	LED_LIGHT,
}LED_EFFECT_TEPEDEF;

void hal_LedInit(void);
void hal_LedContorl(LED_TYPEDEF LedIdx,LED_EFFECT_TEPEDEF State);
void hal_LedProc(void);

 

 
#endif

