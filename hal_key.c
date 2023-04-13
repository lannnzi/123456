
#include "stm32F10x.h"

#include "hal_key.h"



KeyEvent_CallBack_t KeyScanCBS;


static void hal_keyConfig(void);

static unsigned char hal_getKeyS1Sta(void);
static unsigned char hal_getKeyS2Sta(void);
static unsigned char hal_getKeyS3Sta(void);
static unsigned char hal_getKeyS4Sta(void);

unsigned char (*getKeysState[KEYNUM])() = {   hal_getKeyS1Sta,hal_getKeyS2Sta,hal_getKeyS3Sta,hal_getKeyS4Sta
                                          };



unsigned char KeyStep[KEYNUM];								//按键检测流程
unsigned short KeyScanTime[KEYNUM];							//去抖延时
unsigned short KeyPressLongTimer[KEYNUM];						//长按延时
unsigned short KeyContPressTimer[KEYNUM];						//连续长按延时

void hal_KeyInit(void)
{
    unsigned char i;
    KeyScanCBS = 0;
    hal_keyConfig();

    for(i=0; i<KEYNUM; i++)
    {
        KeyStep[i] = KEY_STEP_WAIT;
        KeyScanTime[i] = KEY_SCANTIME;
        KeyPressLongTimer[i] = KEY_PRESS_LONG_TIME;
        KeyContPressTimer[i] = KEY_PRESS_CONTINUE_TIME;
    }

}

void hal_KeyScanCBSRegister(KeyEvent_CallBack_t pCBS)
{
    if(KeyScanCBS == 0)
    {
        KeyScanCBS = pCBS;
    }
}

void hal_KeyProc(void)
{
    unsigned char i,KeyState[KEYNUM];
    unsigned char  keys;

    for(i=0; i<KEYNUM; i++)
    {
        keys = 0;

        KeyState[i] = getKeysState[i]();
        switch(KeyStep[i])
        {
        case KEY_STEP_WAIT:		//等待按键
            if(KeyState[i])
            {
                KeyStep[i] = KEY_STEP_CLICK;
            }
            break;
        case KEY_STEP_CLICK:				//按键单击按下
            if(KeyState[i])
            {
                if(!(--KeyScanTime[i]))
                {
                    KeyScanTime[i] = KEY_SCANTIME;
                    KeyStep[i] = KEY_STEP_LONG_PRESS;
                    //keys = i+1;										//记录按键ID号
                    //state = KEY_CLICK;								//按键单击按下
                    keys = (i*5)+1;

                }
            } else
            {
                KeyScanTime[i] = KEY_SCANTIME;
                KeyStep[i] = KEY_STEP_WAIT;
            }
            break;
        case KEY_STEP_LONG_PRESS:			//按键长按
            if(KeyState[i])
            {
                if(!(--KeyPressLongTimer[i]))
                {
                    KeyPressLongTimer[i] = KEY_PRESS_LONG_TIME;
                    KeyStep[i] = KEY_STEP_CONTINUOUS_PRESS;

                    //keys = i+1;										//记录按键ID号
                    //state = KEY_LONG_PRESS;
                    keys = (i*5)+3;								//长按确认
                }
            } else
            {
                KeyPressLongTimer[i] = KEY_PRESS_LONG_TIME;
                KeyStep[i] = KEY_STEP_WAIT;
                //keys = i+1;										//记录按键ID号
                //state = KEY_CLICK_RELEASE;						//单击释放
                keys = (i*5)+2;										//单击释放
            }
            break;
        case KEY_STEP_CONTINUOUS_PRESS:
            if(KeyState[i])
            {
                if(!(--KeyContPressTimer[i]))
                {
                    KeyContPressTimer[i] = KEY_PRESS_CONTINUE_TIME;
                    //keys = i+1;							//持续长按
                    //state = KEY_LONG_PRESS_CONTINUOUS;
                    keys = (i*5)+4;					//持续长按
                }
            } else
            {
                KeyStep[i] = KEY_STEP_WAIT;
                KeyContPressTimer[i] = KEY_PRESS_CONTINUE_TIME;
                keys = i+1;								//记录按键ID号

                keys = (i*5)+5;								//长按释放
            }

            break;

        }
        if(keys)
        {
            if(KeyScanCBS)
            {
                KeyScanCBS((KEY_VALUE_TYPEDEF)keys);
            }
        }

    }


}


static void hal_keyConfig(void)//按键引脚配置函数
{
    GPIO_InitTypeDef GPIO_InitStructure;


    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    //S1
    GPIO_InitStructure.GPIO_Pin = KEY1_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(KEY1_PORT, &GPIO_InitStructure);

    //S2
    GPIO_InitStructure.GPIO_Pin = KEY2_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(KEY2_PORT, &GPIO_InitStructure);

    //S3
    GPIO_InitStructure.GPIO_Pin = KEY3_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(KEY3_PORT, &GPIO_InitStructure);

    //S4
    GPIO_InitStructure.GPIO_Pin = KEY4_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(KEY4_PORT, &GPIO_InitStructure);

}



static unsigned char hal_getKeyS1Sta(void)
{
    return (!GPIO_ReadInputDataBit(KEY1_PORT, KEY1_PIN));
}

static unsigned char hal_getKeyS2Sta(void)
{
    return (!GPIO_ReadInputDataBit(KEY2_PORT, KEY2_PIN));
}


static unsigned char hal_getKeyS3Sta(void)
{
    return (GPIO_ReadInputDataBit(KEY3_PORT, KEY3_PIN));
}

static unsigned char hal_getKeyS4Sta(void)
{
    return (!GPIO_ReadInputDataBit(KEY4_PORT, KEY4_PIN));
}
