#include <stdio.h>
#include <stdlib.h>
typedef __uint16_t uint16_t;
typedef __uint8_t uint8_t;

#include <chrono>
#include <iostream>
#include <semaphore>
#include <thread>
using namespace std;
 
// global binary semaphore instances
// object counts are set to zero
// objects are in non-signaled state
std::binary_semaphore smphSignalMainToThread{0};
std::binary_semaphore smphSignalThreadToMain{0};

void ThreadProc() {
    // wait for a signal from the main proc
    // by attempting to decrement the semaphore
    smphSignalMainToThread.acquire();
 
    // this call blocks until the semaphore's count
    // is increased from the main proc
 
    std::cout << "[thread] Got the signal\n"; // response message
 
    // wait for 3 seconds to imitate some work
    // being done by the thread
    using namespace std::literals;
    std::this_thread::sleep_for(3s);
 
    std::cout << "[thread] Send the signal\n"; // message
 
    // signal the main proc back
    smphSignalThreadToMain.release();
}

typedef struct Cmd_Queue_t {
    volatile int8_t front;
    volatile int8_t back;
    volatile int8_t capacity;
    volatile int8_t itemBytes;
    volatile int8_t itemNum;
    volatile uint16_t *data;
} Cmd_Queue;

Cmd_Queue* createQueue(uint8_t cap, uint8_t itemBytes) {
    Cmd_Queue *q = NULL;
    uint16_t xQueueSizeInBytes = itemBytes * cap;
    q = (Cmd_Queue *)malloc(xQueueSizeInBytes + sizeof(Cmd_Queue_t));
    // storage area
    q->data = ( uint16_t * )q + sizeof(Cmd_Queue_t);
    q->capacity = cap;
    q->front = 0;
    q->back = -1;
    q->itemBytes = itemBytes;
    q->itemNum = 0;
    return q;
}

uint8_t isEmpty(Cmd_Queue *q) {
    return q->itemNum == 0;
}

uint8_t isFull(Cmd_Queue *q) {
    return q->itemNum == q->capacity;
}

int push(Cmd_Queue *q, uint16_t item) {
    if (isFull(q)) {
        return -1;
    }
    int pos = (q->back + 1) % q->capacity;
    q->back = pos;
    q->data[pos] = item;
    q->itemNum += 1;
    return 0;
}

uint16_t pop(Cmd_Queue *q) {
    if (isEmpty(q)) {
        return -1;
    }
    uint16_t item = q->data[q->front];
    q->front = ((q->front + 1) % q->capacity);
    q->itemNum--;
    return item;
}

void freeQueue(Cmd_Queue* q) {
	free(q);
}


// When the return value is 0xffff, fail to find the item
// that contain the target opcode.
uint16_t queuePopItemInOpcode(Cmd_Queue *q, uint8_t targetOpcode) {
    //osSemaphoreAcquire(binarySem01CmdQueueHandle, osWaitForever);
    uint16_t idx = q->front;
    if (q->itemNum == 0) {
        //osSemaphoreRelease(binarySem01CmdQueueHandle);
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
        cnt += 1;
    }
    if (targetItem == 0xffff) {
        //osSemaphoreRelease(binarySem01CmdQueueHandle);
        return 0xffff;
    }
    // Delete the item in the queue
    idx = ((idx + cnt) % q->capacity);
    int8_t remain = q->itemNum - cnt;  //remaining elements number
    while (remain > 0) {
        uint16_t nextIdx = ((idx + 1) % q->capacity);
        q->data[idx] = q->data[nextIdx];
        idx = ((idx + 1) % q->capacity);
        remain -= 1;
    }
    q->itemNum--;
    q->back = ((q->back - 1 + q->capacity) % q->capacity);
    //osSemaphoreRelease(binarySem01CmdQueueHandle);
    //return targetItem;
}

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

    auto res = queuePopItemInOpcode(test1, 0xb);
    printf("Removing: %x\n", res);
    res = queuePopItemInOpcode(test1, 0xa);
    printf("Removing: %x\n", res);
    push(test1, 0xb004);
	printf("Attempting to add %d\n", 0xb004);
    res = queuePopItemInOpcode(test1, 0xa);
    printf("Removing: %x\n", res);
    res = queuePopItemInOpcode(test1, 0xa);
    printf("Removing: %x\n", res);
    res = queuePopItemInOpcode(test1, 0xb);
    printf("Removing: %x\n", res);
    res = queuePopItemInOpcode(test1, 0xa);
    printf("Removing: %x\n", res);
	freeQueue(test1);
}

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

int main() {
    // create some worker thread
    unitTest0();
    std::thread thrWorker(ThreadProc);
 
    std::cout << "[main] Send the signal\n"; // message
 
    // signal the worker thread to start working
    // by increasing the semaphore's count
    smphSignalMainToThread.release();
 
    // wait until the worker thread is done doing the work
    // by attempting to decrement the semaphore's count
    smphSignalThreadToMain.acquire();
 
    std::cout << "[main] Got the signal\n"; // response message
    thrWorker.join();
}