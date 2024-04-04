#ifndef CMD_QUEUE_H_
#define CMD_QUEUE_H_

#include "stm32f072xb.h"
#include "cmsis_os2.h"

extern osSemaphoreId_t binarySem01CmdQueueHandle;

typedef struct Cmd_Queue_t {
    volatile int8_t front;
    volatile int8_t back;
    volatile int8_t capacity;
    volatile int8_t itemBytes;
    volatile int8_t itemNum;
    volatile uint16_t *data;
} Cmd_Queue;

Cmd_Queue* createQueue(uint8_t cap, uint8_t itemBytes);
uint8_t isQueueEmpty(Cmd_Queue *q);
uint8_t isQueueFull(Cmd_Queue *q);
int queuePush(Cmd_Queue *q, uint16_t item);
uint16_t queuePop(Cmd_Queue *q);
void freeQueue(Cmd_Queue* q);

#endif /* CMD_QUEUE_H_ */