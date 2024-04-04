#include "cmdQueue.h"

osSemaphoreId_t binarySem01CmdQueueHandle;

Cmd_Queue *createQueue(uint8_t cap, uint8_t itemBytes) {
    /* Create the semaphores(s) */
    binarySem01CmdQueueHandle = osSemaphoreNew(1, 1, NULL);
    Cmd_Queue *q = NULL;
    // Fail to create binary semaphore.
    if (binarySem01CmdQueueHandle == NULL) {
        return q;
    }
    uint16_t xQueueSizeInBytes = itemBytes * cap;
    q = (Cmd_Queue *)pvPortMalloc(xQueueSizeInBytes + sizeof(Cmd_Queue));
    if (q == NULL) {
        return q;
    }
    // storage area
    q->data = ( uint16_t * )q + sizeof(Cmd_Queue);
    q->capacity = cap;
    q->front = 0;
    q->back = -1;
    q->itemBytes = itemBytes;
    q->itemNum = 0;
    return q;
}

uint8_t isQueueEmpty(Cmd_Queue *q) {
    osSemaphoreAcquire(binarySem01CmdQueueHandle, osWaitForever);
    uint8_t res = (q->itemNum == 0);
    osSemaphoreRelease(binarySem01CmdQueueHandle);
    return res;
}

uint8_t isQueueFull(Cmd_Queue *q) {
    osSemaphoreAcquire(binarySem01CmdQueueHandle, osWaitForever);
    uint8_t res =(q->itemNum == q->capacity);
    osSemaphoreRelease(binarySem01CmdQueueHandle);
    return res;
}

int queuePush(Cmd_Queue *q, uint16_t item) {
    osSemaphoreAcquire(binarySem01CmdQueueHandle, osWaitForever);
    if (q->itemNum == q->capacity) {
        osSemaphoreRelease(binarySem01CmdQueueHandle);
        return -1;
    }
    int pos = (q->back + 1) % q->capacity;
    q->back = pos;
    q->data[pos] = item;
    q->itemNum += 1;
    osSemaphoreRelease(binarySem01CmdQueueHandle);
    return 0;
}

uint16_t queuePop(Cmd_Queue *q) {
    osSemaphoreAcquire(binarySem01CmdQueueHandle, osWaitForever);
    if (q->itemNum == 0) {
        osSemaphoreRelease(binarySem01CmdQueueHandle);
        return -1;
    }
    uint16_t item = q->data[q->front];
    q->front = ((q->front + 1) % q->capacity);
    q->itemNum--;
    osSemaphoreRelease(binarySem01CmdQueueHandle);
    return item;
}

void freeQueue(Cmd_Queue* q) {
    osSemaphoreAcquire(binarySem01CmdQueueHandle, osWaitForever);
	vPortFree(q);
    osSemaphoreRelease(binarySem01CmdQueueHandle);
}

/*

// A sample test of your program
// You can add as many unit tests as you like
// We will be adding our own to test your program.
void unitTest1() {
	printf("\n********Running unit test 1********");
	Cmd_Queue* test1 = createQueue(3, 2);
	printf("\nAttempting to add %d\n", 1);
	push(test1, 1);
	printf("Attempting to add %d\n", 2);
	push(test1, 2);
	printf("Attempting to add %d\n", 3);
	push(test1, 3);
	printf("Attempting to add %d\n", 4);
	push(test1, 4);
	//printf("Removing: %d\n", queue_dequeue(test1));
    printf("Removing: %d\n", pop(test1));
    printf("Removing: %d\n", pop(test1));
    printf("Removing: %d\n", pop(test1));
    printf("Removing: %d\n", pop(test1));
	freeQueue(test1);
}

void unitTest2() {
	printf("\n********Running unit test 2********");
	Cmd_Queue* test2 = createQueue(2,2);
	printf("\nAttempting to add %d\n", 4);
	push(test2, 4);
	printf("Removing: %d\n", pop(test2));
    push(test2, 2);
	printf("Removing: %d\n", pop(test2));
    push(test2, 1);
	printf("Removing: %d\n", pop(test2));
	freeQueue(test2);
}
void unitTest3() {

	printf("\n********Running unit test 3********");
	Cmd_Queue* test1 = createQueue(3,2);
	printf("\nAttempting to add %d\n", 1);
	push(test1, 1);
	freeQueue(test1);
}

*/