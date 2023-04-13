#include "stm32F10x.h"
#include "hal_timer.h"
#include "hal_led.h"
volatile Stu_TimerTypedef Stu_Timer[T_SUM];

static void Hal_TimerHandle(void);



/*******************************************************************************
* Function Name  : static void hal_timer3Config(void)
* Description    : 定时器硬件配置函数
* Input          : None
* Output         : None
* Return         : None
* Attention		 	 : None
*******************************************************************************/
static void hal_timer3Config(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    TIM_DeInit(TIM3);
//    TIM_TimeBaseStructure.TIM_Period = 10000; 			//10ms
//	TIM_TimeBaseStructure.TIM_Period = 1000; 			//1ms
		TIM_TimeBaseStructure.TIM_Period = 200; 			//200us
    TIM_TimeBaseStructure.TIM_Prescaler = SystemCoreClock/1000000 - 1;

    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);


//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);


    TIM_ClearFlag(TIM3, TIM_FLAG_Update);
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

    TIM_Cmd(TIM3, ENABLE);




//	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
//	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //向上计数
//	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);
//
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
//	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
//
//	//允许更新中断
//	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

//	//打开定时器
//	TIM_Cmd(TIM3,ENABLE);


}

/*******************************************************************************
* Function Name  : void hal_timerInit(void)
* Description    : 定时器初始化
* Input          : None
* Output         : None
* Return         : None
* Attention		 	 : None
*******************************************************************************/
void hal_timerInit(void)
{
    unsigned char i;
    hal_timer3Config();
    for(i=0; i<T_SUM; i++)
    {
        Stu_Timer[i].state = T_STA_STOP;
        Stu_Timer[i].CurrentCount = 0;
        Stu_Timer[i].Period = 0;
        Stu_Timer[i].func = 0;
    }
}


/*******************************************************************************
* Function Name  : hal_CreatTimer(TIMER_ID_TYPEDEF id,void (*proc)(void), unsigned short Period,unsigned char state)
* Description    : 创建定时器
* Input          : - id：定时器ID
*									- (*proc)() 函数指针
*									- Period 定时周期，单位10ms
* 								- state 定时器初始状态
* Output         : None
* Return         : None
* Attention		 	 : None
*******************************************************************************/
void hal_CreatTimer(TIMER_ID_TYPEDEF id,void (*proc)(void), unsigned short Period,TIMER_STATE_TYPEDEF state)
{
    Stu_Timer[id].state = state;

    Stu_Timer[id].CurrentCount = 0;
    Stu_Timer[id].Period = Period;
    Stu_Timer[id].func = proc;
}


/*******************************************************************************
* Function Name  : unsigned char hal_CtrlTimerAction(TIMER_ID_TYPEDEF id,TIMER_STATE_TYPEDEF sta)
* Description    : 控制定时器动作
* Input          : - id：定时器ID
*								 	 - sta 定时器状态
* Output         : None
* Return         : None
* Attention		 	 : None
*******************************************************************************/
TIMER_RESULT_TYPEDEF hal_CtrlTimerAction(TIMER_ID_TYPEDEF id,TIMER_STATE_TYPEDEF sta)
{
    if(Stu_Timer[id].func)		//判断定时器是否存在
    {
        Stu_Timer[id].state = sta;
        return T_SUCCESS;
    } else
    {
        return T_FAIL;
    }
}




/*******************************************************************************
* Function Name  : hal_DeleteTimer(TIMER_ID_TYPEDEF id)
* Description    : 删除定时器
* Input          : - id：定时器ID
*
* Output         : None
* Return         : None
* Attention		 	 : None
*******************************************************************************/
TIMER_RESULT_TYPEDEF hal_DeleteTimer(TIMER_ID_TYPEDEF id)
{
    if(Stu_Timer[id].func)
    {
        Stu_Timer[id].state = T_STA_STOP;

        Stu_Timer[id].CurrentCount = 0;
        //Stu_Timer[id].Period = 0;
        Stu_Timer[id].func = 0;
        return T_SUCCESS;
    } else
    {
        return T_FAIL;
    }
}

/*******************************************************************************
* Function Name  : hal_ResetTimer(TIMER_ID_TYPEDEF id,TIMER_STATE_TYPEDEF sta)
* Description    : 复位定时器状态和计时时间
* Input          : - id：定时器ID
*								 	 - sta 定时器状态
* Output         : None
* Return         : None
* Attention		 	 : None
*******************************************************************************/
TIMER_RESULT_TYPEDEF hal_ResetTimer(TIMER_ID_TYPEDEF id,TIMER_STATE_TYPEDEF sta)
{
    if(Stu_Timer[id].func)		//判断定时器是否存在
    {
        Stu_Timer[id].state = sta;
        Stu_Timer[id].CurrentCount = 0;

        return T_SUCCESS;
    } else
    {
        return T_FAIL;
    }
}
/*******************************************************************************
* Function Name  : static void Hal_TimerHandle(void)
* Description    : 定时器中断计时函数
* Input          : None
* Output         : None
* Return         : None
* Attention		 	 : None
*******************************************************************************/
static void Hal_TimerHandle(void)
{
    unsigned char i;

    for(i=0; i<T_SUM; i++)
    {
        if((Stu_Timer[i].func) && (Stu_Timer[i].state==T_STA_START))
        {
            //if(Stu_Timer[i].CurrentCount >= Stu_Timer[i].Period)


            Stu_Timer[i].CurrentCount++;
            if(Stu_Timer[i].CurrentCount >= Stu_Timer[i].Period)
            {
                Stu_Timer[i].state = T_STA_STOP;
                Stu_Timer[i].CurrentCount = Stu_Timer[i].CurrentCount;
                Stu_Timer[i].func();
            }

        }

    }
}



volatile unsigned int TimingDelay;	//定义进入中断服务程序次数

void Delay(unsigned int nTime)
{
    //也就是延时时间=进入定时器的次数*定时器每次进入中断的时间
    TimingDelay = nTime;
    while(TimingDelay != 0);
}



/*******************************************************************************
* Function Name  : void TIM3_IRQHandler(void)
* Description    : 定时器中断回调函数
* Input          : None
* Output         : None
* Return         : None
* Attention		 	 : None
*******************************************************************************/
void TIM3_IRQHandler(void)
{



    if ( TIM_GetITStatus(TIM3 , TIM_IT_Update) != RESET )
    {
        //这里的TimingDelay相当于计时次数，计算进入定时器的次数
        if(TimingDelay!=0x00) {
            TimingDelay--;
        }
    }


    Hal_TimerHandle();
    TIM_ClearFlag(TIM3, TIM_FLAG_Update);


}
