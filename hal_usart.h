#ifndef _HAL_USART_H
#define _HAL_USART_H

#include "stdio.h"

typedef enum
{
	USART_STEP_HEAD,
	USART_STEP_LENGHT,
	USART_STEP_DATA,
	
}RCV_SETP_TYPEDEF;

typedef enum
{
	USART_EVENT_LED_DARK,
	USART_EVENT_LED_LIGHT_GEAR1,
	USART_EVENT_LED_LIGHT_GEAR2,
	USART_EVENT_LED_LIGHT_GEAR3,
	USART_EVENT_LED_CHECK,
	USART_EVENT_LED_UNKNOWN_CMD,
}USART_EVENT_TYPEDEF;

typedef	void (*UsartEvent_Callback)(USART_EVENT_TYPEDEF event);

void hal_UsartInit(void);
void hal_UsartCBSRegister(UsartEvent_Callback pCBS);
void hal_UsartProc(void);

void hal_Usart_SendByte(unsigned char data);
void hal_Usart_SendFrame(unsigned char *pData,unsigned char len);

#endif

