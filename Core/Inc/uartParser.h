#ifndef UART_PARSER_H_
#define UART_PARSER_H_

#include <stdio.h>
#include <stdlib.h>
#include "stm32f0xx.h"
#include <string.h>

#define TMP_STR_LEN 20

void initUsart3(void);
void transmitOneChar(uint8_t ch);
void transmitCharArray (char *arr);
void USART3_4_IRQHandler(void);
void commandError(char *arr);

#endif /* UART_PARSER_H_ */
