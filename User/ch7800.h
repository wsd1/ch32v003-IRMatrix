#ifndef CH7800_H
#define CH7800_H

#include "ch32v00x.h"

/* Init USART1 for CH7800 voice chip: 9600 8N1, TX on PD0 (USART1 partial remap 1). */
void ch7800_init(void);

/* Play program number n (0..255). Frame: 7E 13 00 02 00 n EF, sent by interrupt. */
void ch7800_play(uint8_t n);

#endif /* CH7800_H */
