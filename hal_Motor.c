/*DM542������������������ļ�*/
#include "stm32F10x.h"
#include "OS_System.h"
#include "hal_Motor.h"
#include "hal_usart.h"
#include "hal_timer.h"
#include "oled.h"
#include "CPU.h"
#include "RFID_RC522.h"
#include "hal_led.h"

unsigned char S2_LastTime=1,S3_LastTime=1,S4_LastTime=1,S5_LastTime=1;

unsigned char S1,S2,S3,S4,S5;

unsigned char TS1,TS2,TS3,TS4,TS5;

unsigned char TS3_LastTime,TS4_LastTime;

unsigned char Tracking_Flag=1,Avoiding_Flag;//ѭ�����б�־λ���������б�־λ

unsigned int Out_Track_Count;//���ѭ���Ƿ�ͷ

unsigned int S_sensorCount;//������⵽�ϰ����ʱ��

unsigned char Body_Sensor_Flag=0;

unsigned char Body_Sensor;

unsigned char Avoiding_Step=Avoiding_Step_1;//���ϲ���

unsigned char Avoiding_Turning_count;//����ת����ת����������

unsigned char Avoiding_Turning_LastTime;//��¼�ϴα��ϵ�ת��

unsigned char Arlm_Flag;//������־λ

unsigned short int Read165(void)//���⴫�������ݲɼ�����Ƭ����ת��������оƬ��74HC165����������16�����⴫�����ĸߵ͵�ƽ
{

    unsigned char i;

    unsigned short int read165data;
    CP=0;
    PL=0;
//    Delay(1);
    __nop();
    PL=1;
    if(DATA_S) read165data=read165data|0x01;
    for(i=0; i<15; i++)                //15��ѭ��,�������紫����
    {
        CP=0;
//        Delay(1);
        __nop();
        __nop();
        CP=1;
        read165data=read165data<<1;

        if(DATA_S) read165data=read165data|0x01;
    }


    return read165data;

}
void Get_TSX(void)//���ѭ����������
{
    unsigned short int temp,temp_S ;
    temp= Read165();

    temp_S=	temp;
    TS1=(unsigned char)(temp_S>>8)&0x01;
    temp_S=	temp;
    TS2=(unsigned char)(temp_S>>9)&0x01;
    temp_S=	temp;
    TS3=(unsigned char)(temp_S>>10)&0x01;
    temp_S=	temp;
    TS4=(unsigned char)(temp_S>>11)&0x01;
    temp_S=	temp;
    TS5=(unsigned char)(temp_S>>12)&0x01;

}
void Get_SX(void)//��ñ��ϸ�������
{
    unsigned short int temp,temp_S ;
    temp= Read165();

    temp_S=	temp;
    S1=(unsigned char)(temp_S)&0x01;
    temp_S=	temp;
    S2=(unsigned char)(temp_S>>1)&0x01;
    temp_S=	temp;
    S3=(unsigned char)(temp_S>>2)&0x01;
    temp_S=	temp;
    S4=(unsigned char)(temp_S>>3)&0x01;
    temp_S=	temp;
    S5=(unsigned char)(temp_S>>4)&0x01;
}
static void Motor_Config(void)//����������������������ú���
{

    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);  //AFIOʱ��
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); //����JTAG����,������������Ϊ��������

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_SetBits(GPIOB,GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_SetBits(GPIOA,GPIO_Pin_15);


}
static void S_165_Config(void)//74HC165D�������ú���
{

    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);


    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_SetBits(GPIOB,GPIO_Pin_14);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

//    GPIO_SetBits(GPIOB,GPIO_Pin_12);


}
static void HC_SR501_Config(void)//74HC165D�������ú���
{

    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);


    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_ResetBits(GPIOA,GPIO_Pin_9);


}
static unsigned char hal_getHC_SR501_Sta(void)
{
    return (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_9));
}
void L_Motor_Stop(void)
{
    L_POSI_A_1();//A+=1
    L_POSI_B_1();//B+=1
    L_NEG_A_1();//A-=1
    L_NEG_B_1();//B-=1
}
void R_Motor_Stop(void)
{
    R_POSI_A_1();//A+=1
    R_POSI_B_1();//B+=1
    R_NEG_A_1();//A-=1
    R_NEG_B_1();//B-=1
}
void Motor_Drive(unsigned char left_step,unsigned char right_step,unsigned char step,unsigned char delay_time)
{

    unsigned char i;
//    unsigned char L_i,R_i;

    for(i=0; i<=step; i++)
    {

//˫����

//Step 1
        if(i<=left_step)
        {
            L_POSI_A_1();//A+=1
            L_POSI_B_1();//B+=1
            L_NEG_A_0();//A-=1
            L_NEG_B_0();//B-=1
        }

        if(i<=right_step)
        {
            R_POSI_A_1();//A+=1
            R_POSI_B_1();//B+=1
            R_NEG_A_0();//A-=1
            R_NEG_B_0();//B-=1
        }
        Delay(delay_time);//��ʱ
//Step 2
        if(i<=left_step)
        {
            L_POSI_A_0();
            L_POSI_B_1();
            L_NEG_A_1();
            L_NEG_B_0();
        }

        if(i<=right_step)
        {
            R_POSI_A_0();
            R_POSI_B_1();
            R_NEG_A_1();
            R_NEG_B_0();
        }

        Delay(delay_time);//��ʱ
//Step 3
        if(i<=left_step)
        {
            L_POSI_A_0();
            L_POSI_B_0();
            L_NEG_A_1();
            L_NEG_B_1();
        }

        if(i<=right_step)
        {
            R_POSI_A_0();
            R_POSI_B_0();
            R_NEG_A_1();
            R_NEG_B_1();
        }

        Delay(delay_time);//��ʱ
//Step 4
        if(i<=left_step)
        {
            L_POSI_A_1();
            L_POSI_B_0();
            L_NEG_A_0();
            L_NEG_B_1();
        }

        if(i<=right_step)
        {
            R_POSI_A_1();
            R_POSI_B_0();
            R_NEG_A_0();
            R_NEG_B_1();

        }

        Delay(delay_time);//��ʱ
//            L_i++;
//            R_i++;

//SysDelay1ms(2);

    }


}
void Motor_Drive1(unsigned char left_Run,unsigned char right_Run,unsigned char step,unsigned char delay_time)
{

    unsigned char i;
//    unsigned char L_i,R_i;

    for(i=0; i<=step; i++)
    {

//˫����

//Step 1
        if(left_Run)
        {
            L_POSI_A_1();//A+=1
            L_POSI_B_1();//B+=1
            L_NEG_A_0();//A-=1
            L_NEG_B_0();//B-=1
        }
        else L_Motor_Stop();

        if(right_Run)
        {
            R_POSI_A_1();//A+=1
            R_POSI_B_1();//B+=1
            R_NEG_A_0();//A-=1
            R_NEG_B_0();//B-=1
        }
        else R_Motor_Stop();
        SysDelay1ms(delay_time);//��ʱ
//Step 2
        if(left_Run)
        {
            L_POSI_A_0();
            L_POSI_B_1();
            L_NEG_A_1();
            L_NEG_B_0();
        }

        if(right_Run)
        {
            R_POSI_A_0();
            R_POSI_B_1();
            R_NEG_A_1();
            R_NEG_B_0();
        }

        SysDelay1ms(delay_time);//��ʱ
//Step 3
        if(left_Run)
        {
            L_POSI_A_0();
            L_POSI_B_0();
            L_NEG_A_1();
            L_NEG_B_1();
        }

        if(right_Run)
        {
            R_POSI_A_0();
            R_POSI_B_0();
            R_NEG_A_1();
            R_NEG_B_1();
        }

        SysDelay1ms(delay_time);//��ʱ
//Step 4
        if(left_Run)
        {
            L_POSI_A_1();
            L_POSI_B_0();
            L_NEG_A_0();
            L_NEG_B_1();
        }

        if(right_Run)
        {
            R_POSI_A_1();
            R_POSI_B_0();
            R_NEG_A_0();
            R_NEG_B_1();

        }

        SysDelay1ms(delay_time);//��ʱ
//            L_i++;
//            R_i++;
//SysDelay1ms(2);
    }


}

//������ת(���ַ�ת��������ת)
void Motor_Drive3(unsigned char left_Run,unsigned char right_Run,unsigned char step,unsigned char delay_time)
{

    unsigned char i;
//    unsigned char L_i,R_i;

    for(i=0; i<=step; i++)
    {

//˫����

//Step 1
        if(left_Run)
        {
            L_POSI_A_1();//A+=1
            L_POSI_B_1();//B+=1
            L_NEG_A_0();//A-=1
            L_NEG_B_0();//B-=1
        }
        else L_Motor_Stop();

        if(right_Run)
        {
//            R_POSI_A_1();//A+=1
//            R_POSI_B_1();//B+=1
//            R_NEG_A_0();//A-=1
//            R_NEG_B_0();//B-=1

            R_POSI_A_1();
            R_POSI_B_0();
            R_NEG_A_0();
            R_NEG_B_1();

        }
        else R_Motor_Stop();
        SysDelay1ms(delay_time);//��ʱ
//Step 2
        if(left_Run)
        {
            L_POSI_A_0();
            L_POSI_B_1();
            L_NEG_A_1();
            L_NEG_B_0();
        }

        if(right_Run)
        {
//            R_POSI_A_0();
//            R_POSI_B_1();
//            R_NEG_A_1();
//            R_NEG_B_0();
            R_POSI_A_0();
            R_POSI_B_0();
            R_NEG_A_1();
            R_NEG_B_1();
        }

        SysDelay1ms(delay_time);//��ʱ
//Step 3
        if(left_Run)
        {
            L_POSI_A_0();
            L_POSI_B_0();
            L_NEG_A_1();
            L_NEG_B_1();
        }

        if(right_Run)
        {
//            R_POSI_A_0();
//            R_POSI_B_0();
//            R_NEG_A_1();
//            R_NEG_B_1();
            //            R_POSI_A_0();
            R_POSI_B_1();
            R_NEG_A_1();
            R_NEG_B_0();
        }

        SysDelay1ms(delay_time);//��ʱ
//Step 4
        if(left_Run)
        {
            L_POSI_A_1();
            L_POSI_B_0();
            L_NEG_A_0();
            L_NEG_B_1();
        }

        if(right_Run)
        {
//            R_POSI_A_1();
//            R_POSI_B_0();
//            R_NEG_A_0();
//            R_NEG_B_1();
            R_POSI_A_1();//A+=1
            R_POSI_B_1();//B+=1
            R_NEG_A_0();//A-=1
            R_NEG_B_0();//B-=1

        }

        SysDelay1ms(delay_time);//��ʱ
//            L_i++;
//            R_i++;
//SysDelay1ms(2);
    }

}
void Motor_Drive4(unsigned char left_Run,unsigned char right_Run,unsigned char step,unsigned char delay_time)
{

    unsigned char i;
//    unsigned char L_i,R_i;

    for(i=0; i<=step; i++)
    {

//˫����

//Step 1
        if(left_Run)
        {
//            L_POSI_A_1();//A+=1
//            L_POSI_B_1();//B+=1
//            L_NEG_A_0();//A-=1
//            L_NEG_B_0();//B-=1
            L_POSI_A_1();
            L_POSI_B_0();
            L_NEG_A_0();
            L_NEG_B_1();
        }
        else L_Motor_Stop();

        if(right_Run)
        {
            R_POSI_A_1();//A+=1
            R_POSI_B_1();//B+=1
            R_NEG_A_0();//A-=1
            R_NEG_B_0();//B-=1
        }
        else R_Motor_Stop();
        SysDelay1ms(delay_time);//��ʱ
//Step 2
        if(left_Run)
        {
//            L_POSI_A_0();
//            L_POSI_B_1();
//            L_NEG_A_1();
//            L_NEG_B_0();
            L_POSI_A_0();
            L_POSI_B_0();
            L_NEG_A_1();
            L_NEG_B_1();
        }

        if(right_Run)
        {
            R_POSI_A_0();
            R_POSI_B_1();
            R_NEG_A_1();
            R_NEG_B_0();
        }

        SysDelay1ms(delay_time);//��ʱ
//Step 3
        if(left_Run)
        {
//            L_POSI_A_0();
//            L_POSI_B_0();
//            L_NEG_A_1();
//            L_NEG_B_1();
            L_POSI_A_0();
            L_POSI_B_1();
            L_NEG_A_1();
            L_NEG_B_0();
        }

        if(right_Run)
        {
            R_POSI_A_0();
            R_POSI_B_0();
            R_NEG_A_1();
            R_NEG_B_1();
        }

        SysDelay1ms(delay_time);//��ʱ
//Step 4
        if(left_Run)
        {
//            L_POSI_A_1();
//            L_POSI_B_0();
//            L_NEG_A_0();
//            L_NEG_B_1();
            L_POSI_A_1();//A+=1
            L_POSI_B_1();//B+=1
            L_NEG_A_0();//A-=1
            L_NEG_B_0();//B-=1
        }

        if(right_Run)
        {
            R_POSI_A_1();
            R_POSI_B_0();
            R_NEG_A_0();
            R_NEG_B_1();

        }

        SysDelay1ms(delay_time);//��ʱ
//            L_i++;
//            R_i++;
//SysDelay1ms(2);
    }


}
void hal_MotorInit(void)
{

    Motor_Config();//DM542�����������������ú���
    S_165_Config();
    HC_SR501_Config();

//    hal_KeyScanCBSRegister(KeyEventHandle);
//    QueueEmpty(KeyMsg);

//    printf("���� ���ʾID��");
//    printf("���ʶ��ɹ�");
//    printf("���ʶ��ʧ��");

//    printf("��⵽����");



//     Motor_Drive1(1,1,100,4);//ǰ��

// L_Motor_Stop();

// R_Motor_Stop();
}

void hal_Tracking_Proc(void)
{
    Get_TSX();
    if(TS1==1&&TS2==1&&TS3==1&&TS4==1&&TS5==1)
    {

        Out_Track_Count++;
        Motor_Drive1(1,1,1,4);
        if(Out_Track_Count>=Out_Track_Count_Value)//�ж϶���
        {

            Motor_Drive3(0,0,20,4);//ԭ����ת
            SysDelay1ms(1000);//��ʱ
            Motor_Drive3(1,1,Track_TurnBack_Value,5);//ԭ����ת
            SysDelay1ms(1000);//��ʱ
            Motor_Drive3(0,0,20,4);//ԭ����ת
            Out_Track_Count=0;

        }


    }
    else Out_Track_Count=0;


    if(TS1==1&&TS2==1&&TS3==1&&TS4==1&&TS5==0)
    {
//        if(TS3_LastTime==0&&TS4_LastTime==1)
//        {
//            Motor_Drive1(0,1,10,2);
//            Motor_Drive1(0,1,10,2);
//        }
//        else if(TS4_LastTime==0&&TS3_LastTime==1)
//        {
//            Motor_Drive1(1,0,10,2);
//            Motor_Drive1(1,0,10,2);
//        }
//        else if(TS3_LastTime==0&&TS4_LastTime==0) Motor_Drive1(1,1,10,4);
        Motor_Drive1(1,1,10,4);
    }
    else
    {
        if(TS1==0&&TS2==0&&TS3==0&&TS4==0) Motor_Drive1(0,0,10,4);
        else
        {
            if(TS3&&TS4)
            {
                if(TS1==0&&TS2==1)
                {
                    Motor_Drive1(0,1,10,4);
//                    TS3_LastTime=1;
                }
                else if(TS2==0&&TS1==1)
                {
                    Motor_Drive1(1,0,10,4);
//                    TS4_LastTime=1;
                }
            }
            else if(TS3==0&&TS4==1)
            {
//                Motor_Drive1(0,1,10,4);
                if(TS1==0)
                {
                    Motor_Drive1(0,1,10,3);
                }
                else
                {
                    Motor_Drive1(0,1,10,2);
//                    TS3_LastTime=0;
                }
            }
            else if(TS4==0&&TS3==1)
            {
//                Motor_Drive1(1,0,10,4);
                if(TS2==0)
                {
                    Motor_Drive1(1,0,10,3);
                }
                else
                {
                    Motor_Drive1(1,0,10,2);
//                    TS4_LastTime=0;
                }
            }
        }
//Motor_Drive(1,1,10,4);
    }

}
void hal_Avoiding_Proc(void)
{

    Get_SX();


    switch (Avoiding_Step)
    {
//���ϲ���1
    case Avoiding_Step_1:
        if(S1==0&&S2==1&&S3==1)//���Ҷ�û�ϰ���Ĭ������ת
        {
            Avoiding_Turning_LastTime	=Turn_Right;
            while(!S1)
            {
                Get_SX();
                Motor_Drive3(1,1,1,4);//ԭ����ת
                Avoiding_Turning_count++;
            }
//            while(S3_LastTime==0&&S3==1)
            while(!S3)
            {
                Get_SX();
                S3_LastTime=S3;

                Motor_Drive3(1,1,1,4);//ԭ����ת
                Avoiding_Turning_count++;
            }
            Motor_Drive1(1,1,70,4);//ǰ��
//            while(S5_LastTime==0&&S5==1)
            while(!S5)
            {
                Get_SX();
                S5_LastTime=S5;
                Motor_Drive1(1,1,10,4);//ǰ��
            }

//            Motor_Drive1(1,1,70,4);//ǰ��


        }
        else if(S1==0&&S2==0&&S3==0)//���Ҷ����ϰ���Ĭ������ת
        {
            Avoiding_Turning_LastTime	=Turn_Right;
            while(!S1)
            {
                Get_SX();
                Motor_Drive3(1,1,1,4);//ԭ����ת
                Avoiding_Turning_count++;
            }
            while(!S3)
            {
                Get_SX();
//                S3_LastTime=S3;

                Motor_Drive3(1,1,1,4);//ԭ����ת
                Avoiding_Turning_count++;
            }
            Motor_Drive1(1,1,70,4);//ǰ��
//            while(S5_LastTime==0&&S5==1)
            while(!S5)
            {
                Get_SX();
                S5_LastTime=S5;
                Motor_Drive1(1,1,10,4);//ǰ��
            }

//            Motor_Drive1(1,1,70,4);//ǰ��

        }
        else if(S1==0&&S2==1&&S3==0)//�����ϰ�����ת
        {
            Avoiding_Turning_LastTime	=Turn_Right;
            while(!S1)
            {
                Get_SX();
                Motor_Drive3(1,1,1,4);//ԭ����ת
                Avoiding_Turning_count++;
            }
            while(!S3)
            {

                Get_SX();
//                S3_LastTime=S3;

                Motor_Drive3(1,1,1,4);//ԭ����ת
                Avoiding_Turning_count++;

            }
            Motor_Drive1(1,1,70,4);//ǰ��
//            while(S5_LastTime==0&&S5==1)
            while(!S5)
            {
                Get_SX();
                S5_LastTime=S5;

                Motor_Drive1(1,1,10,4);//ǰ��
            }

//            Motor_Drive1(1,1,70,4);//ǰ��


        }
        else if(S1==1&&S2==1&&S3==0)//�����ϰ�����ת
        {
            Avoiding_Turning_LastTime	=Turn_Right;
            while(!S1)
            {
                Get_SX();
                Motor_Drive3(1,1,1,4);//ԭ����ת
                Avoiding_Turning_count++;
            }
            while(!S3)
            {

                Get_SX();
//                S3_LastTime=S3;

                Motor_Drive3(1,1,1,4);//ԭ����ת
                Avoiding_Turning_count++;

            }
            Motor_Drive1(1,1,70,4);//ǰ��
//            while(S5_LastTime==0&&S5==1)
            while(!S5)
            {
                Get_SX();
                S5_LastTime=S5;

                Motor_Drive1(1,1,10,4);//ǰ��
            }

//            Motor_Drive1(1,1,70,4);//ǰ��


        }
        else if(S1==0&&S2==0&&S3==1)//�����ϰ�����ת
        {
            Avoiding_Turning_LastTime	=Turn_Left;
            while(!S1)
            {
                Get_SX();
                Motor_Drive4(1,1,1,4);//ԭ����ת
                Avoiding_Turning_count++;
            }
            while(!S2)
            {

                Get_SX();
//                S3_LastTime=S3;

                Motor_Drive4(1,1,1,4);//ԭ����ת
                Avoiding_Turning_count++;

            }
            Motor_Drive1(1,1,70,4);//ǰ��
//            while(S4_LastTime==0&&S4==1)
            while(!S4)
            {
                Get_SX();
                S4_LastTime=S4;

                Motor_Drive1(1,1,10,4);//ǰ��
            }

//            Motor_Drive1(1,1,70,4);//ǰ��


        }
        else if(S1==1&&S2==0&&S3==1)//�����ϰ�����ת
        {
            Avoiding_Turning_LastTime	=Turn_Left;
            while(!S1)
            {
                Get_SX();
                Motor_Drive4(1,1,1,4);//ԭ����ת
                Avoiding_Turning_count++;
            }
            while(!S2)
            {

                Get_SX();
//                S3_LastTime=S3;

                Motor_Drive4(1,1,1,4);//ԭ����ת
                Avoiding_Turning_count++;

            }
            Motor_Drive1(1,1,70,4);//ǰ��
//            while(S4_LastTime==0&&S4==1)
            while(!S4)
            {
                Get_SX();
                S4_LastTime=S4;

                Motor_Drive1(1,1,10,4);//ǰ��
            }

//            Motor_Drive1(1,1,70,4);//ǰ��


        }

//        Motor_Drive1(1,1,70,4);//ǰ��

        Motor_Drive1(0,0,1,4);//ֹͣ
        Get_SX();
        Avoiding_Step=Avoiding_Step_2;

        break ;
//���ϲ���2
    case Avoiding_Step_2:

//        Motor_Drive1(0,0,1,4);//ֹͣ

        if( Avoiding_Turning_LastTime	==Turn_Left)
        {

            Motor_Drive3(1,1,Avoiding_Turning_count*1.8,4);//ԭ����ת

            Motor_Drive1(1,1,80,4);//ǰ��
            while(!S4)
            {
                Get_SX();
                S4_LastTime=S4;

                Motor_Drive1(1,1,10,4);//ǰ��
            }
        }
        else if(Avoiding_Turning_LastTime	==Turn_Right)
        {
            Motor_Drive4(1,1,Avoiding_Turning_count*1.8,4);//ԭ����ת

            Motor_Drive1(1,1,80,4);//ǰ��
            while(!S5)
            {
                Get_SX();
                S5_LastTime=S5;

                Motor_Drive1(1,1,10,4);//ǰ��
            }
        }

//        Motor_Drive1(1,1,80,4);//ǰ��
        Get_SX();
        Avoiding_Step=Avoiding_Step_3;
        break ;
//���ϲ���3
    case Avoiding_Step_3:
        if( Avoiding_Turning_LastTime	==Turn_Left)
        {

            Motor_Drive3(1,1,Avoiding_Turning_count*1.8,4);//ԭ����ת

        }
        else if(Avoiding_Turning_LastTime	==Turn_Right)
        {

            Motor_Drive4(1,1,Avoiding_Turning_count*1.8,4);//ԭ����ת

        }
        Motor_Drive1(1,1,5,4);//ǰ��
        Get_SX();
        Avoiding_Step=Avoiding_Step_4;
        break ;
//���ϲ���4
    case Avoiding_Step_4:

//        Motor_Drive1(0,0,1,4);//ֹͣ


        Motor_Drive1(1,1,1,4);//ǰ��

        Get_TSX();

        if(TS3==0&&TS4==1)
        {
//            Motor_Drive1(1,1,5,4);//ǰ��

            Motor_Drive1(1,0,30,4);//

            Avoiding_Turning_LastTime	=Empty;
            Avoiding_Turning_count=0;

            Tracking_Flag=1;
            Avoiding_Flag=0;

//            Motor_Drive1(0,0,20,4);//

        }
        else if(TS4==0&&TS3==1)
        {
//            Motor_Drive1(1,1,5,4);//ǰ��

            Motor_Drive1(0,1,30,4);//

            Avoiding_Turning_LastTime	=Empty;
            Avoiding_Turning_count=0;

            Tracking_Flag=1;
            Avoiding_Flag=0;

//            Motor_Drive1(0,0,30,4);//

        }
        else if(TS1==0&&TS2==1)
        {
//            Motor_Drive1(1,1,5,4);//ǰ��

            Motor_Drive1(1,0,30,4);//

            Avoiding_Turning_LastTime	=Empty;
            Avoiding_Turning_count=0;

            Tracking_Flag=1;
            Avoiding_Flag=0;

//            Motor_Drive1(0,0,30,4);//

        }
        else if(TS2==0&&TS1==1)
        {
//            Motor_Drive1(1,1,5,4);//ǰ��

            Motor_Drive1(0,1,30,4);//

            Avoiding_Turning_LastTime	=Empty;
            Avoiding_Turning_count=0;

            Tracking_Flag=1;
            Avoiding_Flag=0;

//            Motor_Drive1(0,0,30,4);//

        }


//        Avoiding_Turning_LastTime	=Empty;
//        Avoiding_Turning_count=0;

//        Tracking_Flag=1;
//        Avoiding_Flag=0;

        break ;

    }


}


void hal_IdentifyProc(void)
{

//        printf("���ʶ��ɹ�");
//        printf("���ʶ��ʧ��");
    Motor_Drive1(0,0,10,4);






//        printf("��⵽����");
//        SysDelay1ms(50);//��ʱ
    printf("���� ���ʾID��");
    SysDelay1ms(4000);//��ʱ


    hal_RC522Proc();


//        hal_RC522Proc();

    Body_Sensor_Flag=0;
    Tracking_Flag=1;







    SysDelay1ms(3000);//��ʱ
    SysDelay1ms(3000);//��ʱ
}
void hal_MotorProc(void)
{

//    Motor_Drive(10,10,10,4);
//    Motor_Drive1(0,0,10,4);
//    Motor_Drive1(S1,S2,10,4);


//    Motor_Drive3(1,1,2,4);//ԭ����ת
//    Motor_Drive4(1,1,2,4);//ԭ����ת


    if(Tracking_Flag==1&&Body_Sensor_Flag==0)
    {
        Body_Sensor=hal_getHC_SR501_Sta();
        if(Body_Sensor)
        {
            Body_Sensor_Flag=1;
            Tracking_Flag=0;
        }

    }



    if(Body_Sensor_Flag==0)
    {
        Get_SX();

        if(S1==0||S2==0||S3==0)
        {

            S_sensorCount++;

            if(S_sensorCount>=S_sensorCount_Value)
            {
                Avoiding_Flag=1;
                Tracking_Flag=0;
                S_sensorCount=0;
                Avoiding_Step=Avoiding_Step_1;
            }


        }
        else
        {
            S_sensorCount=0;
//        Avoiding_Flag=0;
        }


    }


    if(Tracking_Flag==1)
    {
        hal_Tracking_Proc();
    }
//    Avoiding_Flag=1;
    if(Avoiding_Flag==1)
    {
        hal_Avoiding_Proc();
    }
    if(Body_Sensor_Flag==1)
    {
        hal_IdentifyProc();
//       Body_Sensor_Flag=0;
    }



}

