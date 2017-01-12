#include <irq.h>
#include "usart.h"
#include <stm32.h>
#include <gpio.h>

/* Buffer length (in bytes) */
#define BUFLEN 2048

/* Utility macros */
#define WRAP(x) ((x) & (BUFLEN - 1))
#define INC(x) WRAP((x)+1)
#define DEC(x) WRAP((x)+BUFLEN-1)

/* A DMA cyclic buffer.
 *
 * [begin, middle) is being currently transmitted using DMA, it should not be modified
 * [middle, end) is enqueued for transmission
 * [end, begin) is free
 */
struct buffer {
    int begin, middle, end;
    char data[BUFLEN];
};

/* DMA IRQs must be disabled */
inline int buffer_empty(struct buffer *buffer) {
    return buffer->begin == buffer->end;
}

/* DMA IRQs must be disabled */
inline int buffer_full(struct buffer *buffer) {
    return INC(buffer->end) == buffer->begin;
}

/* DMA IRQs must be disabled */
inline int buffer_size(struct buffer *buffer) {
    return WRAP(buffer->end - buffer->begin);
}

/* DMA IRQs must be disabled */
inline void buffer_write(struct buffer *buffer, char c) {
    buffer->data[buffer->end] = c;
    buffer->end = INC(buffer->end);
}

struct buffer out_buffer;

#define MAY_IN (USART2->SR & USART_SR_RXNE)
#define MAY_OUT (USART2->SR & USART_SR_TXE)

#define IN_LENGTH 8

static int transmitting;

/* DMA IRQs must be disabled */
static void push_out(void) {
    if (transmitting || buffer_empty(&out_buffer)) {
        return;
    }

    DMA1_Stream6->M0AR = (uint32_t)&out_buffer.data[out_buffer.begin];
    if (out_buffer.begin < out_buffer.end) {
        DMA1_Stream6->NDTR = out_buffer.end - out_buffer.begin;
        out_buffer.middle = out_buffer.end;
    }
    else {
        DMA1_Stream6->NDTR = BUFLEN - out_buffer.begin;
        out_buffer.middle = 0;
    }

    transmitting = 1;
    DMA1_Stream6->CR |= DMA_SxCR_EN;
}

void DMA1_Stream6_IRQHandler() {
    uint32_t isr = DMA1->HISR;

    if (isr & DMA_HISR_TCIF6) {
        DMA1->HIFCR = DMA_HIFCR_CTCIF6;

        out_buffer.begin = out_buffer.middle;
        transmitting = 0;
        push_out();
    }
}

/** Enqueues the given string for transmission. Interrupt-safe */
void output(const char *str) {
    irq_level_t irq_level = IRQprotect(1);

    while (*str && !buffer_full(&out_buffer)) {
        buffer_write(&out_buffer, *(str++));
    }

    push_out();

    IRQunprotect(irq_level);
}

/** Enqueues the given integer for transmission (as a decimal number). Interrupt-safe */
void output_int(int number) {
    char buf[11];
    int idx = 10;
    irq_level_t irq_level = IRQprotect(1);

    if (number == 0) {
        output("0");
    }
    else if (number < 0) {
        output("-");
        number = -number;
    }

    buf[10] = 0;
    while(number) {
        buf[--idx] = (number % 10) + '0';
        number /= 10;
    }

    output(&buf[idx]);
    IRQunprotect(irq_level);
}

#define USART_Mode_Rx_Tx (USART_CR1_RE | \
        USART_CR1_TE)
#define USART_Enable USART_CR1_UE

#define USART_StopBits_1 0x0000

#define USART_FlowControl_None 0x0000

#define USART_WordLength_8b 0x0000
#define USART_WordLength_9b USART_CR1_M

#define USART_Parity_No 0x0000
#define USART_Parity_Even USART_CR1_PCE
#define USART_Parity_Odd (USART_CR1_PCE | \
        USART_CR1_PS)


#define HSI_HZ 16000000U

#define PCLK1_HZ HSI_HZ

void init_usart(void) {
    uint32_t const baudrate = 9600U;

    /* Enable the clock */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_DMA1EN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    /* Configure the TXD line */
    GPIOafConfigure(GPIOA,
            2,
            GPIO_OType_PP,
            GPIO_Fast_Speed,
            GPIO_PuPd_NOPULL,
            GPIO_AF_USART2);

    /* Configure the RXD line */
    GPIOafConfigure(GPIOA,
            3,
            GPIO_OType_PP,
            GPIO_Fast_Speed,
            GPIO_PuPd_UP,
            GPIO_AF_USART2);

    /* Configure USART */
    USART2->CR1 = USART_Mode_Rx_Tx |
        USART_WordLength_8b |
        USART_Parity_No;

    USART2->CR2 = USART_StopBits_1;

    USART2->CR3 = USART_CR3_DMAT | USART_CR3_DMAR;

    USART2->BRR = (PCLK1_HZ + (baudrate / 2U)) /
        baudrate;

    /* Configure DMA */
    DMA1_Stream6->CR = 4U << 25 |
        DMA_SxCR_PL_1 |
        DMA_SxCR_MINC |
        DMA_SxCR_DIR_0 |
        DMA_SxCR_TCIE;
    DMA1_Stream6->PAR = (uint32_t)&USART2->DR;

    DMA1_Stream5->CR = 4U << 25 |
        DMA_SxCR_PL_1 |
        DMA_SxCR_MINC |
        DMA_SxCR_TCIE;
    DMA1_Stream5->PAR = (uint32_t)&USART2->DR;

    DMA1->HIFCR = DMA_HIFCR_CTCIF6 | DMA_HIFCR_CTCIF5;

    /* Enable interrupts */
    NVIC_SetPriority(DMA1_Stream6_IRQn, USART_IRQ_LEVEL);
    NVIC_SetPriority(DMA1_Stream5_IRQn, USART_IRQ_LEVEL);

    NVIC_EnableIRQ(DMA1_Stream6_IRQn);
    NVIC_EnableIRQ(DMA1_Stream5_IRQn);

    /* Start USART */
    USART2->CR1 |= USART_Enable;
}
