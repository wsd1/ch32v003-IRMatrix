/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/12/25
 * Description        : Main program body.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/*
 *@Note
 *Multiprocessor communication mode routine:
 *Master:USART1_Tx(PD5)\USART1_Rx(PD6).
 *This routine demonstrates that USART1 receives the data sent by CH341 and inverts
 *it and sends it (baud rate 115200).
 *
 *Hardware connection:PD5 -- Rx
 *                     PD6 -- Tx
 *
 */

#include "debug.h"
#include "ws2812b_driver.h"
#include "i2c_slave.h"
#include "ch7800.h"
#include <stdbool.h>


/* ================= 1ms 时基 (SysTick 中断累加) ================= */

volatile uint32_t now_msec = 0;

uint32_t now_ms(void)
{
    return now_msec;
}

bool isTimeout(uint32_t deadline)
{
    return (int32_t)(now_ms() - deadline) >= 0;
}

void SysTick_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void SysTick_Handler(void)
{
    SysTick->SR = 0;              /* 清 CNTIF */
    now_msec++;
}

static void timebase_init(void)
{
    NVIC_EnableIRQ(SysTick_IRQn);
    SysTick->SR   = 0;
    SysTick->CNT  = 0;
    SysTick->CMP  = SystemCoreClock / 1000 - 1;   /* 48MHz -> 1ms */
    SysTick->CTLR = 0xF;                          /* STE|STIE|STCLK|STRE, 自动重载周期中断 */
}


/* ================= 共享状态 ================= */

volatile uint16_t IRmatrix = 0;
volatile uint8_t  IRFlag   = 0;

/* 把 IRmatrix 发布到 I2C 从机寄存器: [0]=低字节, [1]=高字节 */
static void ir_publish(void)
{
    uint16_t v = IRmatrix;
    I2C_REG[0] = (uint8_t)(v & 0xFF);
    I2C_REG[1] = (uint8_t)(v >> 8);
}


/* ================= 红外 GPIO ================= */

static const uint16_t ir_row_pin[4] = { GPIO_Pin_3, GPIO_Pin_4, GPIO_Pin_5, GPIO_Pin_6 }; /* PD3..PD6 = IR_OUT0..3 */
static const uint16_t ir_col_pin[4] = { GPIO_Pin_3, GPIO_Pin_4, GPIO_Pin_5, GPIO_Pin_6 }; /* PC3..PC6 = IR_IN0..3  */
#define IR_TX_PIN   GPIO_Pin_0   /* PC0 */
#define IR_RX_PIN   GPIO_Pin_7   /* PC7 */

static void ir_gpio_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);

    /* IR_OUT0..3 推挽输出, 默认关闭 */
    gpio.GPIO_Pin   = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
    gpio.GPIO_Speed = GPIO_Speed_30MHz;
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOD, &gpio);
    GPIO_ResetBits(GPIOD, gpio.GPIO_Pin);

    /* IR_TX 推挽输出, 默认关闭 */
    gpio.GPIO_Pin  = IR_TX_PIN;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &gpio);
    GPIO_ResetBits(GPIOC, IR_TX_PIN);

    /* IR_IN0..3 / IR_RX 浮空输入 (外部 10K 上拉) */
    gpio.GPIO_Pin  = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | IR_RX_PIN;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &gpio);
}

static void matrixLightOnRow(int r)  { GPIO_SetBits(GPIOD, ir_row_pin[r]); }
static void matrixLightOffRow(int r) { GPIO_ResetBits(GPIOD, ir_row_pin[r]); }
static void IRLightOn(void)          { GPIO_SetBits(GPIOC, IR_TX_PIN); }
static void IRLightOff(void)         { GPIO_ResetBits(GPIOC, IR_TX_PIN); }

/* 读四列, 低电平=激活=1; IR_IN0->bit0 ... IR_IN3->bit3 */
static uint16_t ir_read_columns(void)
{
    uint16_t v = 0;
    int i;
    for (i = 0; i < 4; i++)
    {
        if (GPIO_ReadInputDataBit(GPIOC, ir_col_pin[i]) == RESET)
            v |= (uint16_t)(1u << i);
    }
    return v;
}

static uint8_t ir_read_rx(void)
{
    return (GPIO_ReadInputDataBit(GPIOC, IR_RX_PIN) == RESET) ? 1 : 0;
}

/* 最低非零位下标 (0..15) */
static int lowIndexOf(uint16_t v)
{
    int i = 0;
    while (i < 16 && !(v & (1u << i))) i++;
    return i;
}


/* ================= 红外反射矩阵任务 ================= */

#define MATRIX_LIGHTUP_TIME 2
#define MATRIX_DELAY_TIME   6

typedef enum {
    STEP_ROW0_LIGHTUP = 0,
    STEP_ROW0_READ,
    STEP_ROW1_LIGHTUP,
    STEP_ROW1_READ,
    STEP_ROW2_LIGHTUP,
    STEP_ROW2_READ,
    STEP_ROW3_LIGHTUP,
    STEP_ROW3_READ
} state_irMatrix_t;

typedef struct { state_irMatrix_t state; } irMatrix_ctx_t;
static irMatrix_ctx_t irMatrix_ctx = { STEP_ROW0_LIGHTUP };

static uint32_t task_matrix(irMatrix_ctx_t *ctx)
{
    switch (ctx->state)
    {
    case STEP_ROW0_LIGHTUP:
        matrixLightOnRow(0);
        ctx->state = STEP_ROW0_READ;
        return now_ms() + MATRIX_LIGHTUP_TIME;
    case STEP_ROW0_READ:
        IRmatrix = (IRmatrix & ~0x000Fu) | (ir_read_columns() & 0x000Fu);
        ir_publish();
        matrixLightOffRow(0);
        ctx->state = STEP_ROW1_LIGHTUP;
        return now_ms() + MATRIX_DELAY_TIME;

    case STEP_ROW1_LIGHTUP:
        matrixLightOnRow(1);
        ctx->state = STEP_ROW1_READ;
        return now_ms() + MATRIX_LIGHTUP_TIME;
    case STEP_ROW1_READ:
        IRmatrix = (IRmatrix & ~0x00F0u) | ((ir_read_columns() & 0x000Fu) << 4);
        ir_publish();
        matrixLightOffRow(1);
        ctx->state = STEP_ROW2_LIGHTUP;
        return now_ms() + MATRIX_DELAY_TIME;

    case STEP_ROW2_LIGHTUP:
        matrixLightOnRow(2);
        ctx->state = STEP_ROW2_READ;
        return now_ms() + MATRIX_LIGHTUP_TIME;
    case STEP_ROW2_READ:
        IRmatrix = (IRmatrix & ~0x0F00u) | ((ir_read_columns() & 0x000Fu) << 8);
        ir_publish();
        matrixLightOffRow(2);
        ctx->state = STEP_ROW3_LIGHTUP;
        return now_ms() + MATRIX_DELAY_TIME;

    case STEP_ROW3_LIGHTUP:
        matrixLightOnRow(3);
        ctx->state = STEP_ROW3_READ;
        return now_ms() + MATRIX_LIGHTUP_TIME;
    case STEP_ROW3_READ:
        IRmatrix = (IRmatrix & ~0xF000u) | ((ir_read_columns() & 0x000Fu) << 12);
        ir_publish();
        matrixLightOffRow(3);
        ctx->state = STEP_ROW0_LIGHTUP;
        return now_ms() + MATRIX_DELAY_TIME;
    }
    return now_ms();
}


/* ================= 红外遮挡任务 ================= */

#define IR_LIGHTUP_TIME 3
#define IR_DELAY_TIME   30

typedef enum {
    STEP_IR_LIGHTUP = 0,
    STEP_IR_READ
} state_irBreak_t;

typedef struct { state_irBreak_t state; } irBreak_ctx_t;
static irBreak_ctx_t irBreak_ctx = { STEP_IR_LIGHTUP };

static uint32_t task_ir(irBreak_ctx_t *ctx)
{
    switch (ctx->state)
    {
    case STEP_IR_LIGHTUP:
        IRLightOn();
        ctx->state = STEP_IR_READ;
        return now_ms() + IR_LIGHTUP_TIME;
    case STEP_IR_READ:
        IRFlag = ir_read_rx();
        IRLightOff();
        ctx->state = STEP_IR_LIGHTUP;
        return now_ms() + IR_DELAY_TIME;
    }
    return now_ms();
}


/* ================= ws2812 彩灯 ================= */

void ws2812_bitset(uint16_t bits)
{
    int i;
    for (i = 0; i < 16; i++)
    {
        if (bits & (uint16_t)(1u << i)) { leds[i].r = 0x0; leds[i].g = 0x03; leds[i].b = 0x03; }
        else                            { leds[i].r = 0x00; leds[i].g = 0x00; leds[i].b = 0x00; }
    }
    ws2812b_show();
}


/* ================= 红外事件展示任务 ================= */

#define PLAY_AUDIO_DELAY 50
static uint8_t lastPlay = 0xFF;

static uint32_t task_play_audio(void)
{
    if (IRFlag != 0)
    {
        /* 遮断: 播放 0 号节目, 全部点亮 */
        if (lastPlay != 1) { ch7800_play(1); lastPlay = 1; }
        ws2812_bitset(0x5A5A);
    }
    else if (IRmatrix != 0)
    {
        /* 按 IRmatrix 最低非零位播放 n+1 号节目, 灯串按 IRmatrix 点亮 */
        int n = lowIndexOf(IRmatrix) + 1;
        if (lastPlay != n) { ch7800_play((uint8_t)n); lastPlay = (uint8_t)n; }
        ws2812_bitset(IRmatrix);
    }
    else
    {
        lastPlay = 0xFF;
        ws2812_bitset(0x0000);   /* 熄灭 */
    }

    return now_ms() + PLAY_AUDIO_DELAY;
}


/* ================= 主程序 ================= */

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();

    ir_gpio_init();
    ch7800_init();
    ws2812b_init();
    I2C_init();

    /* 上电自检: 播一次 0 号节目 + 间隔点亮灯, 阻塞 3s */
    ch7800_play(1);
    ws2812_bitset(0xA5A5);
    Delay_Ms(3000);

    /* 启动 1ms 时基 (放在 Delay_Ms 之后: Delay_Ms 会重配 SysTick, 与周期时基冲突) */
    timebase_init();

    uint32_t t_matrix = now_ms();
    uint32_t t_ir     = now_ms();
    uint32_t t_play   = now_ms();

    while (1)
    {
        //if (isTimeout(t_matrix)) t_matrix = task_matrix(&irMatrix_ctx);
        if (isTimeout(t_ir))     t_ir     = task_ir(&irBreak_ctx);
        if (isTimeout(t_play))   t_play   = task_play_audio();
    }
}
