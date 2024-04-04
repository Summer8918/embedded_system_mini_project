#include "cmdQueue.h"

Cmd_Queue createQueue(uint8_t cap, uint8_t itemBytes) {
    Cmd_Queue *q = NULL;
    uint16_t xQueueSizeInBytes = itemBytes * cap;
    q = (Cmd_Queue *)pvPortMalloc(xQueueSizeInBytes + sizeof(Cmd_Queue_t));
    // storage area
    q->data = ( uint16_t * )q + sizeof(Cmd_Queue_t);
    q->capacity = cap;
    q->front = 0;
    q->back = -1;
    q->itemBytes = itemBytes;
    q->itemNum = 0;
}

uint8_t isQueueEmpty(Cmd_Queue *q) {
    return q->itemNum == 0;
}

uint8_t isQueueFull(Cmd_Queue *q) {
    return q->itemNum == q->capacity;
}

int queuePush(Cmd_Queue *q, uint16_t item) {
    if (isQueueFull(q)) {
        return -1;
    }
    int pos = (q->back + 1) % q->capacity;
    q->back = pos;
    q->data[pos] = item;
    q->itemNum += 1;
    return 0;
}

uint16_t queuePop(Cmd_Queue *q) {
    if (isQueueEmpty(q)) {
        return -1;
    }
    uint16_t item = q->data[q->front];
    q->front = ((q->front + 1) % q->capacity);
    q->itemNum--;
    return item;
}

void freeQueue(Cmd_Queue* q) {
	vPortFree(q);
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