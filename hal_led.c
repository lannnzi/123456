#include "stm32F10x.h"
#include "hal_timer.h"
#include "hal_led.h"

unsigned short 	Led_Dark=0;
unsigned short 	Led_Light_Gear1=99;
unsigned short 	Led_Light_Gear2=1999;
unsigned short 	Led_Light_Gear3=9999;




static void hal_Led1Drive(unsigned char sta);
static void hal_Led2Drive(unsigned char sta);


void (*hal_LedDrive[])(unsigned char) = {	hal_Led1Drive,
        hal_Led2Drive,

                                        };





unsigned short LedTimer[LED_SUM];
unsigned short *pLed[LED_SUM];



static void hal_LedConfig(void);
//static void hal_Led_PWMConfig(u16 arr,u16 psc);






void hal_LedInit(void)
{

    hal_LedConfig();
//    hal_Led_PWMConfig(9999,SystemCoreClock/2000000 - 1);//arr=9999,psc=23,f=200Hz

//	TIM_SetCompare2(TIM4,99);
    hal_LedContorl(LED1,LED_DARK);
    hal_LedContorl(LED2,LED_DARK);

}


void hal_LedContorl(LED_TYPEDEF LedIdx,LED_EFFECT_TEPEDEF State)
{
    if(LedIdx) hal_Led2Drive(State);
    else hal_Led1Drive(State);

}



void hal_LedProc(void)
{


}

static void hal_LedConfig(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB , ENABLE);


	GPIO_InitStructure.GPIO_Pin = LED1_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; ;
	GPIO_Init(LED1_PORT, &GPIO_InitStructure);
	GPIO_ResetBits(LED1_PORT,LED1_PIN);

    GPIO_InitStructure.GPIO_Pin = LED2_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
    GPIO_Init(LED2_PORT, &GPIO_InitStructure);
    GPIO_ResetBits(LED2_PORT,LED2_PIN);

	
	

}










static void hal_Led1Drive(unsigned char sta)
{
    if(sta)
    {
        GPIO_ResetBits(LED1_PORT,LED1_PIN);
    } else
    {
        GPIO_SetBits(LED1_PORT,LED1_PIN);
    }





}

static void hal_Led2Drive(unsigned char sta)
{
    if(sta)
    {
        GPIO_SetBits(LED2_PORT,LED2_PIN);
    } else
    {
        GPIO_ResetBits(LED2_PORT,LED2_PIN);
    }




}




