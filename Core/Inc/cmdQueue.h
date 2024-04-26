#ifndef CMD_QUEUE_H_
#define CMD_QUEUE_H_

#include "stm32f072xb.h"
#include "cmsis_os2.h"

#define LED_COMMAND_OPCODE 0xA
#define MOTOR_COMMAND_OPCODE 0xB


/* -------------------------------------------------------------------------------------------------------------
 *  Global Variable and Type Declarations
 *  -------------------------------------------------------------------------------------------------------------
 */
extern osSemaphoreId_t binarySem01CmdQueueHandle;

typedef struct Cmd_Queue_t {
    volatile int8_t front;
    volatile int8_t back;
    volatile int8_t capacity;
    volatile int8_t itemBytes;
    volatile int8_t itemNum;
    volatile uint16_t *data;
} Cmd_Queue;

/** 
 * Create new command queue.
 *
 * @param cap the capacity of queue.
 * @param itemBytes the number of bytes of the item of command queue
 * @return  Return command queue pointer, NULL when failing to create.
 */
Cmd_Queue* createQueue(uint8_t cap, uint8_t itemBytes);

/**
 * Whether the queue is empty.
 * 
 * @param q Pointer of queue.
 * @return 1 When queue is empty, 0 when queue is not empty.
 */
uint8_t isQueueEmpty(Cmd_Queue *q);

/**
 * Whether the queue is full.
 *
 * @param q Pointer of queue.
 * @return 1 when queue is full, 0 when queue is not full.
 */
uint8_t isQueueFull(Cmd_Queue *q);

/**
 * Pushes an item onto the queue.
 * 
 * @param q Pointer to the queue.
 * @param item The item to be pushed onto the queue.
 * @return -1 if the operation fails, 0 if it succeeds.
 */
int queuePush(Cmd_Queue *q, uint16_t item);

/**
 * Pops an item from the queue.
 * 
 * @param q Pointer to the queue.
 * @return 0xffff if the operation fails, item if it succeeds.
 */
uint16_t queuePop(Cmd_Queue *q);

/**
 * Pops an item from the queue according to the opcode of item
 * 
 * @param q Pointer to the queue.
 * @param targetOpcode The opcode of item when want to get from the queue
 * @return 0xffff if the operation fails, item if it succeeds.
 */
uint16_t queuePopItemByOpcode(Cmd_Queue *q, uint8_t targetOpcode);

/**
 * Release the resource of queue
 * 
 * @param q Pointer to the queue.
 */
void freeQueue(Cmd_Queue* q);

#endif /* CMD_QUEUE_H_ */