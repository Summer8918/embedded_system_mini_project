#ifndef UART_PARSER_H_
#define UART_PARSER_H_

#include <stdio.h>
#include <stdlib.h>
#include "stm32f0xx.h"
#include "sync.h"
#include "cmdQueue.h"

#define TMP_STR_LEN 20
#define CMD_QUEUE_CAPACITY 5

/* -------------------------------------------------------------------------------------------------------------
 *  Global Variable and Type Declarations
 *  -------------------------------------------------------------------------------------------------------------
 */
extern osThreadId_t UARTTaskHandle;

extern Cmd_Queue * cmdQueue;

extern osSemaphoreId_t binarySem02UartParserHandle;

extern const osThreadAttr_t UARTTask_attributes;

/**
 * Initialze usart 3 and create a command queue
 */
void initUsart3(void);

/**
 * Send a char via UART
 * 
 * @param ch, the char to be sent
 */
void transmitOneChar(uint8_t ch);

/**
 * Send the string in char array via UART
 * 
 * @param arr The pointer of char array of string
 */
void transmitCharArray (char *arr);

/**
 * Interrupt handler of usart 3 and 4
 */
void USART3_4_IRQHandler(void);

/**
 * UART parser thread function
 * 
 * @param  argument: Not used
 */
void StartParseUartTask(void *argument);

/**
 * Convert a uint16_t integer to binary and send it via UART.
 * 
 * @param x The integer to be converted and sent.
 */
void sendUint16BinToUart(uint16_t x);

/**
 * Parse user command and store the command in global variable if it is valid
 * 
 * @return 1 if fail to parse, otherwise 0
 */
uint8_t parseCmd(void);

/**
 * Convert UART ascii speed into into uint8 to add.
 * 
 * @param x ascii, speed in ascii format.
 * @return the uint8 format of speed.
 */
// to command
uint8_t convertSpeed(char *ascii);

#endif /* UART_PARSER_H_ */
