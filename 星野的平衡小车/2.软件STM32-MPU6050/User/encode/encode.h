#ifndef _ENCODE_H
#define	_ENCODE_H

//#include <sys.h>	 
#include "init.h" 
#include "stm32f10x.h"

uint32_t read_cnt_TIM3(void);
uint32_t read_cnt_TIM4(void);

void encoder_Tim3_init(void);
void encoder_Tim4_init(void);

//void delay_ms(uint16_t delay_ms);
#endif
