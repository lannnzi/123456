#ifndef _HAL_KEY_H
#define _HAL_KEY_H

#define KEY1_PORT	GPIOB
#define KEY1_PIN	GPIO_Pin_15

#define KEY2_PORT	GPIOB
#define KEY2_PIN	GPIO_Pin_14

#define KEY3_PORT	GPIOB
#define KEY3_PIN	GPIO_Pin_12

#define KEY4_PORT	GPIOB
#define KEY4_PIN	GPIO_Pin_13

//#define S5_PORT	GPIOB
//#define S5_PIN	GPIO_Pin_8

typedef enum
{
	KEY1,
	KEY2,
	KEY3,
	KEY4,
	KEYNUM
}KEY_TYPEDEF;			//按键定义

 

// 按键检测过程
typedef enum
{
	KEY_STEP_WAIT,			//等待按键
	KEY_STEP_CLICK,				//按键按下
	KEY_STEP_LONG_PRESS,				//长按
	KEY_STEP_CONTINUOUS_PRESS,  			//持续按下
}KEY_STEP_TYPEDEF;




typedef enum
{	
	KEY_IDLE,       	 		 							//按键空闲
	KEY_CLICK,          								//单击确认
	KEY_CLICK_RELEASE,            			//单击释放
	KEY_LONG_PRESS,			   						 	//长按确认
	KEY_LONG_PRESS_CONTINUOUS,							//长按持续
	KEY_LONG_PRESS_RELEASE								//长按释放
	 
}KEY_EVENT_TYPEDEF;
typedef enum
{
	KEY_IDLE_VAL,
	KEY1_CLICK,                          //1
	KEY1_CLICK_RELEASE,
	KEY1_LONG_PRESS,
	KEY1_LONG_PRESS_CONTINUOUS,
	KEY1_LONG_PRESS_RELEASE,		//5
	
	KEY2_CLICK,								//6
	KEY2_CLICK_RELEASE,
	KEY2_LONG_PRESS,
	KEY2_LONG_PRESS_CONTINUOUS,
	KEY2_LONG_PRESS_RELEASE,
	
	KEY3_CLICK,							//11
	KEY3_CLICK_RELEASE,
	KEY3_LONG_PRESS,
	KEY3_LONG_PRESS_CONTINUOUS,
	KEY3_LONG_PRESS_RELEASE,
	
	KEY4_CLICK,						//16
	KEY4_CLICK_RELEASE,
	KEY4_LONG_PRESS,
	KEY4_LONG_PRESS_CONTINUOUS,
	KEY4_LONG_PRESS_RELEASE,
	
	
}KEY_VALUE_TYPEDEF;

typedef void (*KeyEvent_CallBack_t)(KEY_VALUE_TYPEDEF keys);

//扫描按键的定时器Tick,以系统Tick(1ms)为单位,10=10ms
#define KEY_SCANT_TICK		10		//10ms

//按键消抖时间,以10ms为Tick,2=20ms
#define KEY_SCANTIME	2		//20ms

//连续长按时间
#define	KEY_PRESS_LONG_TIME	200	//2s

//持续长按间隔时间
#define KEY_PRESS_CONTINUE_TIME	15	//150ms 

void hal_KeyInit(void);
void hal_KeyProc(void);
void hal_KeyScanCBSRegister(KeyEvent_CallBack_t pCBS);


#endif 
