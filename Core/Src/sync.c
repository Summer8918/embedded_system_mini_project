#include "sync.h"

osSemaphoreId_t countSem01;
osSemaphoreId_t ledRouterSem01;
osSemaphoreId_t motorRouterSem01;
osSemaphoreId_t workerStatusMutex;
volatile uint8_t ledWorkerBusy = 0;
volatile uint8_t motorWorkerBusy = 0;

// Return -1, osSemaphoreNew == NULL
// Return 0, init success
int8_t syncInit(void) {
    int8_t status = 0;
    countSem01 = osSemaphoreNew(3, 0, NULL);
    if (countSem01 == NULL) {
        //transmitCharArray("Fail to create countSem01\n");
        return -1;
    }
    ledRouterSem01 = osSemaphoreNew(1, 0, NULL);
    if (ledRouterSem01 == NULL) {
        //transmitCharArray("Fail to create ledRouterSem01\n");
        return -1;
    }
    motorRouterSem01 = osSemaphoreNew(1, 0, NULL);
    if (motorRouterSem01 == NULL) {
        //transmitCharArray("Fail to create motorRouterSem01\n");
        return -1;
    }
    workerStatusMutex = osSemaphoreNew(1, 1, NULL);
    if (workerStatusMutex == NULL) {
        //transmitCharArray("Fail to create workerStatusMutex\n");
        return -1;
    }
    return 0;
}