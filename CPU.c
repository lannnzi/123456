/*--------------------------------------------------------------------------------------
*  @file     Device.c
*  @author   oypl
*  @version  MiniOS V1.0.1
*  @date     2014.01.14
*  @brief    硬件驱动层源文件,所有驱动函数开头统一以Driver_xxx()格式命名.
---------------------------------------------------------------------------------------*/


#include "stm32F10x.h"
#include "OS_System.h"

#include "CPU.h"




static void hal_CoreClockInit(void);
static unsigned char hal_getprimask(void);
void hal_CPU_Critical_Control(CPU_EA_TYPEDEF cmd,unsigned char *pSta);
/********************************************************************************************************
*  @函数名   hal_CPUInit
*  @描述     CPU系统时钟相关初始化
*  @参数     无
*  @返回值   0-总中断关闭 1-总中断打开
*  @注意     无
********************************************************************************************************/
void hal_CPUInit(void)
{
    hal_CoreClockInit();
    OS_CPUInterruptCBSRegister(hal_CPU_Critical_Control);
}

/********************************************************************************************************
*  @函数名   hal_getprimask
*  @描述     获取CPU总中断状态
*  @参数     无
*  @返回值   0-总中断关闭 1-总中断打开
*  @注意     无
********************************************************************************************************/
static unsigned char hal_getprimask(void)
{
    return (!__get_PRIMASK());		//0是中断打开，1是中断关闭，所以要取反
}


/********************************************************************************************************
*  @函数名   hal_CPU_Critical_Control
*  @描述     CPU临界处理控制
*  @参数     cmd-控制命令  *pSta-总中断状态
*  @返回值   无
*  @注意     无
********************************************************************************************************/
void hal_CPU_Critical_Control(CPU_EA_TYPEDEF cmd,unsigned char *pSta)
{
    if(cmd == CPU_ENTER_CRITICAL)

    {
        *pSta = hal_getprimask();	//保存中断状态
        __disable_irq();		//关CPU总中断
    } else if(cmd == CPU_EXIT_CRITICAL)
    {
        if(*pSta)
        {
            __enable_irq();		//打开中断
        } else
        {
            __disable_irq();	//关闭中断
        }
    }
}

/********************************************************************************************************
*  @函数名   hal_CoreClockInit
*  @描述     CPU系统时钟初始化
*  @参数     无
*  @返回值   无
*  @注意     这个时钟绝对任务调度的Tick值,为了保证实时性，一般设置为10ms
********************************************************************************************************/
static void hal_CoreClockInit(void)
{
    SysTick_Config(SystemCoreClock / 1000);			//使用48M作为系统时钟，那么计数器减1等于1/48M(ms), (1/48000000hz)*(48000000/1000)=0.001S=1ms
}


volatile unsigned int SysDelayTime;	//定义进入中断服务程序次数

void SysDelay1ms(unsigned int nTime)
{
    //也就是延时时间=进入定时器的次数*定时器每次进入中断的时间
    SysDelayTime = nTime;
    while(SysDelayTime != 0);
}

/********************************************************************************************************
*  @函数名   SysTick_Handler
*  @描述     CPU系统时钟中断
*  @参数     无
*  @返回值   无
*  @注意     内核时钟10ms定时中断回调函数,一定要把系统时钟处理函数放进去
********************************************************************************************************/
void SysTick_Handler(void)
{
    if(SysDelayTime!=0x00) {
        SysDelayTime--;
    }
    OS_ClockInterruptHandle();
}








