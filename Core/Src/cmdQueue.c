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
        return 0xffff;
    }
    uint16_t item = q->data[q->front];
    q->front = ((q->front + 1) % q->capacity);
    q->itemNum--;
    osSemaphoreRelease(binarySem01CmdQueueHandle);
    return item;
}

// When the return value is 0xffff, fail to find the item
// that contain the target opcode.
uint16_t queuePopItemByOpcode(Cmd_Queue *q, uint8_t targetOpcode) {
    osSemaphoreAcquire(binarySem01CmdQueueHandle, osWaitForever);
    uint16_t idx = q->front;
    if (q->itemNum == 0) {
        osSemaphoreRelease(binarySem01CmdQueueHandle);
        return 0xffff;
    }
    uint16_t targetItem = 0xffff;
    int8_t cnt = 0;
    while (cnt < q->itemNum) {
        uint8_t curOpcode = (q->data[(idx + cnt) % q->capacity] >> 12);
        if (curOpcode == targetOpcode) {
            targetItem = q->data[(idx + cnt) % q->capacity];
            break;
        }
        cnt++;
    }
    if (targetItem == 0xffff) {
        osSemaphoreRelease(binarySem01CmdQueueHandle);
        return 0xffff;
    }
    // Delete the item in the queue
    idx = ((idx + cnt) % q->capacity);
    int8_t remain = q->itemNum - cnt;  //remaining elements number
    while (remain > 0) {
        uint16_t nextIdx = ((idx + 1) % q->capacity);
        q->data[idx] = q->data[nextIdx];
        idx = ((idx + 1) % q->capacity);
        remain--;
    }
    q->itemNum--;
    q->back = ((q->back - 1 + q->capacity) % q->capacity);
    osSemaphoreRelease(binarySem01CmdQueueHandle);
    return targetItem;
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

// test queuePopItemByOpcode
void unitTest0() {
    printf("\n********Running unit test 1********");
	Cmd_Queue* test1 = createQueue(4, 2);
	printf("\nAttempting to add %d\n", 1);
	push(test1, 0xa001);
	printf("Attempting to add %d\n",0xa001);
	push(test1, 0xb002);
	printf("Attempting to add %d\n", 0xb002);
	push(test1, 0xa003);
	printf("Attempting to add %d\n", 0xa003);
	push(test1, 0xa004);
    printf("Attempting to add %d\n", 0xa004);

    auto res = queuePopItemByOpcode(test1, 0xb);
    printf("Removing: %x\n", res);
    res = queuePopItemByOpcode(test1, 0xa);
    printf("Removing: %x\n", res);
    push(test1, 0xb004);
	printf("Attempting to add %d\n", 0xb004);
    res = queuePopItemByOpcode(test1, 0xa);
    printf("Removing: %x\n", res);
    res = queuePopItemByOpcode(test1, 0xa);
    printf("Removing: %x\n", res);
    res = queuePopItemByOpcode(test1, 0xb);
    printf("Removing: %x\n", res);
    res = queuePopItemByOpcode(test1, 0xa);
    printf("Removing: %x\n", res);
	freeQueue(test1);
}

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