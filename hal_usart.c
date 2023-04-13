#include <string.h>
#include "OS_System.h"
#include "stm32F10x.h"

#include "hal_usart.h"



Queue256	RxMsg;
Queue256 TxMsg;

RCV_SETP_TYPEDEF RcvPos;

UsartEvent_Callback UsartEventCBS;

unsigned char RxDataBuff[128];
unsigned char TxDataBuff[128];

unsigned int TxCounter = 0;//0表示没有发送数据 其他值表示发送数据的下标
unsigned int TxDataSize = 0;//串口需要发送的有效数据的长度

int fputc(int ch,FILE *p)  //函数默认的，在使用printf函数时自动调用
{
	USART_SendData(USART2,(u8)ch);	//串口号应注意修改
	while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==RESET);
	return ch;
}


static void hal_usart_Config(void);





//串口初始化
void hal_UsartInit()
{
	
    hal_usart_Config();
    QueueEmpty(RxMsg);
    QueueEmpty(TxMsg);
    RcvPos = USART_STEP_HEAD;
    UsartEventCBS = 0;
}

void hal_UsartCBSRegister(UsartEvent_Callback pCBS)
{
    if(UsartEventCBS == 0)
    {
        UsartEventCBS = pCBS;
    }
}

void hal_UsartProc()
{
    static unsigned char Lenght = 0;
    unsigned char i,temp;
    unsigned char data;
    switch(RcvPos)
    {
    case USART_STEP_HEAD:			//判断帧头
        if(QueueDataLen(RxMsg))
        {
            QueueDataOut(RxMsg,&data);
            if(data == 0xAA)
            {
                RcvPos = USART_STEP_LENGHT;
            }
//					else {UsartEventCBS(	USART_EVENT_LED_UNKNOWN_CMD);
////					RcvPos = USART_STEP_HEAD;
//					break;}
        }
        break;
    case USART_STEP_LENGHT:					//判断长度
        if(QueueDataLen(RxMsg))
        {
            QueueDataOut(RxMsg,&data);
            RcvPos = USART_STEP_DATA;
            Lenght = data;
        }
        break;
    case USART_STEP_DATA:						//开始判断数据字节
        if(QueueDataLen(RxMsg) >= Lenght)
        {
            for(i=0; i<Lenght; i++)
            {
                QueueDataOut(RxMsg,&RxDataBuff[i]);
            }
            //RxDataBuff[0] 命令  RxDataBuff[1] 校验码
            temp = Lenght^RxDataBuff[0];

            if(temp == RxDataBuff[1])
            {

                switch ((RxDataBuff[0]))
                {
                case 0:
                    UsartEventCBS(USART_EVENT_LED_DARK);
                    break ;
                case 1:
                    UsartEventCBS(USART_EVENT_LED_LIGHT_GEAR1);
                    break ;
                case 2:
                    UsartEventCBS(USART_EVENT_LED_LIGHT_GEAR2);
                    break ;
                case 3:
                    UsartEventCBS(USART_EVENT_LED_LIGHT_GEAR3);
                    break ;
                case 4:
                    UsartEventCBS(USART_EVENT_LED_CHECK);
                    break ;
                default:
                    UsartEventCBS(	USART_EVENT_LED_UNKNOWN_CMD);
                }

//					TxDataBuff[0] = 0xAA;
//					TxDataBuff[1] = RxDataBuff[1];
//					TxDataBuff[2] = RxDataBuff[2];
//					TxDataBuff[2] = temp;
//					TxDataBuff[4] = 0xFE;

//					TxDataBuff[0] = 0xAA;
//					TxDataBuff[1] = Lenght;
//					TxDataBuff[2] = RxDataBuff[0];
//					TxDataBuff[3] = temp;
//					TxDataBuff[4] = 0xFE;


//					hal_Usart_SendFrame(TxDataBuff,5);
                temp = 0;
                Lenght = 0;
                RcvPos = USART_STEP_HEAD;


            }
//				else {UsartEventCBS(	USART_EVENT_LED_UNKNOWN_CMD);
////				RcvPos = USART_STEP_HEAD;
//				break;}
        }
//				else {UsartEventCBS(	USART_EVENT_LED_UNKNOWN_CMD);
////				RcvPos = USART_STEP_HEAD;
//				break;}
        break;
    }

}




static void hal_usart_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA,ENABLE);
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_USART2,ENABLE);


    /*
    *  USART2_TX -> PA2 , USART1_RX ->	PA3
    */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART2, &USART_InitStructure);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USART2, USART_IT_TC, ENABLE);


    USART_Cmd(USART2, ENABLE);

    //USART1 INT Cofig
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);


}

void hal_Usart_SendByte(unsigned char data)
{
    USART_SendData(USART2,data);
}

void hal_Usart_SendFrame(unsigned char *pData,unsigned char len)
{
    unsigned char data;
    QueueDataIn(TxMsg,pData,len);
    QueueDataOut(TxMsg,&data);
    hal_Usart_SendByte(data);
}

void USART2_IRQHandler(void)
{
    unsigned char dat;
    if(USART_GetITStatus(USART2,USART_IT_RXNE) != RESET)
    {
        dat = USART_ReceiveData(USART2);

        QueueDataIn(RxMsg,&dat,1);
        USART_ClearITPendingBit(USART2,USART_IT_RXNE);

    }

    if(USART_GetITStatus(USART2,USART_IT_TC) != RESET)
    {
        USART_ClearITPendingBit(USART2, USART_IT_TC);
        if(QueueDataLen(TxMsg))
        {
            QueueDataOut(TxMsg,&dat);
            hal_Usart_SendByte(dat);
        }
    }
}

