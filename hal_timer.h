#ifndef _HAL_TIMER_H_
#define _HAL_TIMER_H_

typedef enum
{
	T_LED,
	T_AT_ATKCLDSTA,
	T_SUM,
}TIMER_ID_TYPEDEF;
	
typedef enum
{
	T_SUCCESS,
	T_FAIL,
}TIMER_RESULT_TYPEDEF;


typedef enum
{
	T_STA_STOP,					//定时器停止
	T_STA_START,				//定时器启动
}TIMER_STATE_TYPEDEF;

typedef struct
{
	TIMER_STATE_TYPEDEF state;		//0-定时器未启动 1-定时器已启动
	//unsigned char CompleteFlag;	//定时完成标志 0-未完成  1-定时完成
	unsigned short CurrentCount;	//当前计时值
	unsigned short Period;			//定时周期
	void (*func)(void);					//函数指针
	
}Stu_TimerTypedef;

void hal_timerInit(void);
void hal_CreatTimer(TIMER_ID_TYPEDEF id,void (*proc)(void), unsigned short Period,TIMER_STATE_TYPEDEF state);
TIMER_RESULT_TYPEDEF hal_CtrlTimerAction(TIMER_ID_TYPEDEF id,TIMER_STATE_TYPEDEF sta);
TIMER_RESULT_TYPEDEF hal_ResetTimer(TIMER_ID_TYPEDEF id,TIMER_STATE_TYPEDEF sta);


void Delay( unsigned int nTime);
#endif
