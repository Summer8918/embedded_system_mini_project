#ifndef UART_PARSER_H_
#define UART_PARSER_H_

#include <stdio.h>
#include <stdlib.h>
#include "stm32f0xx.h"

#include "cmdQueue.h"

#define TMP_STR_LEN 20
#define CMD_QUEUE_CAPACITY 5

void initUsart3(void);

void transmitOneChar(uint8_t ch);

void transmitCharArray (char *arr);

void USART3_4_IRQHandler(void);

void commandError(char *arr);

void StartParseUartTask(void *argument);

void sendUint16BinToUart(uint16_t x);

uint8_t parseCmd(void);

uint8_t convertSpeed(char *ascii);

extern osThreadId_t UARTTaskHandle;

extern Cmd_Queue * cmdQueue;

extern osSemaphoreId_t binarySem02UartParserHandle;

extern const osThreadAttr_t UARTTask_attributes;
#endif /* UART_PARSER_H_ */
