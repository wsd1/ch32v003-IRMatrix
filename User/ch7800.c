#include "ch7800.h"

#define CH7800_FRAME_LEN 7

static volatile uint8_t ch7800_buf[CH7800_FRAME_LEN];
static volatile uint8_t ch7800_idx;
static volatile uint8_t ch7800_busy;

void ch7800_init(void)
{
    GPIO_InitTypeDef  gpio = {0};
    USART_InitTypeDef usart = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO, ENABLE);

    /* USART1 partial remap 1: TX -> PD0 (default is PD5) */
    GPIO_PinRemapConfig(GPIO_PartialRemap1_USART1, ENABLE);

    gpio.GPIO_Pin   = GPIO_Pin_0;
    gpio.GPIO_Speed = GPIO_Speed_30MHz;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOD, &gpio);

    usart.USART_BaudRate            = 9600;
    usart.USART_WordLength          = USART_WordLength_8b;
    usart.USART_StopBits            = USART_StopBits_1;
    usart.USART_Parity              = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode                = USART_Mode_Tx;
    USART_Init(USART1, &usart);

    USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
    NVIC_EnableIRQ(USART1_IRQn);
    USART_Cmd(USART1, ENABLE);
}

void ch7800_play(uint8_t n)
{
    if (ch7800_busy) return;    /* previous frame still sending: ignore */

    ch7800_buf[0] = 0x7E;
    ch7800_buf[1] = 0x13;
    ch7800_buf[2] = 0x00;
    ch7800_buf[3] = 0x02;
    ch7800_buf[4] = 0x00;
    ch7800_buf[5] = n;
    ch7800_buf[6] = 0xEF;

    ch7800_idx  = 0;
    ch7800_busy = 1;
    USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
}

void USART1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
    {
        if (ch7800_idx < CH7800_FRAME_LEN)
        {
            USART_SendData(USART1, ch7800_buf[ch7800_idx++]);
        }
        else
        {
            USART_ITConfig(USART1, USART_IT_TXE, DISABLE);   /* buffer done: turn off interrupt */
            ch7800_busy = 0;
        }
    }
}
