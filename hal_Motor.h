#ifndef _HAL_MOTOR_H
#define _HAL_MOTOR_H

#include "stm32f10x.h"

//位带操作,实现51类似的GPIO控制功能
//具体实现思想,参考<<CM3权威指南>>第五章(87页~92页).
//IO口操作宏定义
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//IO口地址映射
#define GPIOA_ODR_Addr    (GPIOA_BASE+12) //0x4001080C 
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C 
#define GPIOC_ODR_Addr    (GPIOC_BASE+12) //0x4001100C 
#define GPIOD_ODR_Addr    (GPIOD_BASE+12) //0x4001140C 
#define GPIOE_ODR_Addr    (GPIOE_BASE+12) //0x4001180C 
#define GPIOF_ODR_Addr    (GPIOF_BASE+12) //0x40011A0C    
#define GPIOG_ODR_Addr    (GPIOG_BASE+12) //0x40011E0C    

#define GPIOA_IDR_Addr    (GPIOA_BASE+8) //0x40010808 
#define GPIOB_IDR_Addr    (GPIOB_BASE+8) //0x40010C08 
#define GPIOC_IDR_Addr    (GPIOC_BASE+8) //0x40011008 
#define GPIOD_IDR_Addr    (GPIOD_BASE+8) //0x40011408 
#define GPIOE_IDR_Addr    (GPIOE_BASE+8) //0x40011808 
#define GPIOF_IDR_Addr    (GPIOF_BASE+8) //0x40011A08 
#define GPIOG_IDR_Addr    (GPIOG_BASE+8) //0x40011E08 
 
//IO口操作,只对单一的IO口!
//确保n的值小于16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //输出 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //输入 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //输出 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //输入 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //输出 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //输入 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //输出 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //输入 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //输出 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //输入

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //输出 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //输入

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //输出 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //输入



//#define Encoder2_RCC_APB1PeriphClockCmd	   RCC_APB1Periph_TIM4


//右电机A+     PB6
//右电机A-      PB7
//右电机B+     P8
//右电机B-      PB9
//左电机A+     PA15
//左电机A-      PB3
//左电机B+    PB4
//左电机B-     PB5



#define L_POSI_A_1()  GPIO_SetBits(GPIOA,GPIO_Pin_15)
#define L_POSI_A_0()  GPIO_ResetBits(GPIOA,GPIO_Pin_15)

#define L_NEG_A_1()  GPIO_SetBits(GPIOB,GPIO_Pin_3)
#define L_NEG_A_0()  GPIO_ResetBits(GPIOB,GPIO_Pin_3)

#define L_POSI_B_1()  GPIO_SetBits(GPIOB,GPIO_Pin_4)
#define L_POSI_B_0()  GPIO_ResetBits(GPIOB,GPIO_Pin_4)

#define L_NEG_B_1()  GPIO_SetBits(GPIOB,GPIO_Pin_5)
#define L_NEG_B_0()  GPIO_ResetBits(GPIOB,GPIO_Pin_5)



#define R_POSI_A_1()  GPIO_SetBits(GPIOB,GPIO_Pin_6)
#define R_POSI_A_0()  GPIO_ResetBits(GPIOB,GPIO_Pin_6)

#define R_NEG_A_1()  GPIO_SetBits(GPIOB,GPIO_Pin_7)
#define R_NEG_A_0()  GPIO_ResetBits(GPIOB,GPIO_Pin_7)

#define R_POSI_B_1()  GPIO_SetBits(GPIOB,GPIO_Pin_8)
#define R_POSI_B_0()  GPIO_ResetBits(GPIOB,GPIO_Pin_8)

#define R_NEG_B_1()  GPIO_SetBits(GPIOB,GPIO_Pin_9)
#define R_NEG_B_0()  GPIO_ResetBits(GPIOB,GPIO_Pin_9)

#define DATA_S PBin(12)
#define PL  PBout(13)                     //165并行输入数据锁存线
#define CP  PBout(14)                     //165数据时钟线 


#define S_sensorCount_Value    1    //避障传感器检测持续判断阈值

#define Out_Track_Count_Value    40    //循迹传感器检测丢线持续判断阈值

#define Track_TurnBack_Value      65        //循迹丢线掉头角度

typedef enum
{
	Avoiding_Step_1,
	Avoiding_Step_2,
	Avoiding_Step_3,
	Avoiding_Step_4,	
} Avoiding_Step_TYPEDEF;
typedef enum
{
	Empty,	
	Turn_Left,
	Turn_Right,	
} Avoiding_Turning_LastTime_TYPEDEF;


void hal_MotorInit(void);
void hal_MotorProc(void);

void Get_TSX(void);//获得循迹感器数据
void Get_SX(void);//获得避障感器数据

void hal_Tracking_Proc(void);//循迹
void hal_Avoiding_Proc(void);//避障
	
void Motor_Drive(unsigned char left_step,unsigned char right_step,unsigned char Step,unsigned char delay_time);
void Motor_Drive1(unsigned char left_Run,unsigned char right_Run,unsigned char step,unsigned char delay_time);//循迹前行
void Motor_Drive2(unsigned char left_Run,unsigned char right_Run,unsigned char step,unsigned char delay_time);//避障前行
void Motor_Drive3(unsigned char left_Run,unsigned char right_Run,unsigned char step,unsigned char delay_time);//避障右转
void Motor_Drive4(unsigned char left_Run,unsigned char right_Run,unsigned char step,unsigned char delay_time);//避障左转

void L_Motor_Stop(void);//左电机停转
void R_Motor_Stop(void);//右电机停转

unsigned short int Read165(void);

#endif
