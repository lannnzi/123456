#include "stm32F10x.h"
#include "hal_timer.h"
#include "hal_led.h"
volatile Stu_TimerTypedef Stu_Timer[T_SUM];

static void Hal_TimerHandle(void);



/*******************************************************************************
* Function Name  : static void hal_timer3Config(void)
* Description    : ��ʱ��Ӳ�����ú���
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
//	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //���ϼ���
//	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);
//
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
//	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
//
//	//��������ж�
//	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

//	//�򿪶�ʱ��
//	TIM_Cmd(TIM3,ENABLE);


}

/*******************************************************************************
* Function Name  : void hal_timerInit(void)
* Description    : ��ʱ����ʼ��
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
* Description    : ������ʱ��
* Input          : - id����ʱ��ID
*									- (*proc)() ����ָ��
*									- Period ��ʱ���ڣ���λ10ms
* 								- state ��ʱ����ʼ״̬
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
* Description    : ���ƶ�ʱ������
* Input          : - id����ʱ��ID
*								 	 - sta ��ʱ��״̬
* Output         : None
* Return         : None
* Attention		 	 : None
*******************************************************************************/
TIMER_RESULT_TYPEDEF hal_CtrlTimerAction(TIMER_ID_TYPEDEF id,TIMER_STATE_TYPEDEF sta)
{
    if(Stu_Timer[id].func)		//�ж϶�ʱ���Ƿ����
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
* Description    : ɾ����ʱ��
* Input          : - id����ʱ��ID
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
* Description    : ��λ��ʱ��״̬�ͼ�ʱʱ��
* Input          : - id����ʱ��ID
*								 	 - sta ��ʱ��״̬
* Output         : None
* Return         : None
* Attention		 	 : None
*******************************************************************************/
TIMER_RESULT_TYPEDEF hal_ResetTimer(TIMER_ID_TYPEDEF id,TIMER_STATE_TYPEDEF sta)
{
    if(Stu_Timer[id].func)		//�ж϶�ʱ���Ƿ����
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
* Description    : ��ʱ���жϼ�ʱ����
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



volatile unsigned int TimingDelay;	//��������жϷ���������

void Delay(unsigned int nTime)
{
    //Ҳ������ʱʱ��=���붨ʱ���Ĵ���*��ʱ��ÿ�ν����жϵ�ʱ��
    TimingDelay = nTime;
    while(TimingDelay != 0);
}



/*******************************************************************************
* Function Name  : void TIM3_IRQHandler(void)
* Description    : ��ʱ���жϻص�����
* Input          : None
* Output         : None
* Return         : None
* Attention		 	 : None
*******************************************************************************/
void TIM3_IRQHandler(void)
{



    if ( TIM_GetITStatus(TIM3 , TIM_IT_Update) != RESET )
    {
        //�����TimingDelay�൱�ڼ�ʱ������������붨ʱ���Ĵ���
        if(TimingDelay!=0x00) {
            TimingDelay--;
        }
    }


    Hal_TimerHandle();
    TIM_ClearFlag(TIM3, TIM_FLAG_Update);


}
