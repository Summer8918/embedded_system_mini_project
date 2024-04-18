#include "sync.h"

osSemaphoreId_t countSem01;
osSemaphoreId_t ledRouterSem01;
osSemaphoreId_t motorRouterSem01;
osSemaphoreId_t ledStatusMutex;
osSemaphoreId_t motorStatusMutex;
volatile uint8_t ledWorkerBusy = 0;
volatile uint8_t motorWorkerBusy = 0;

// Return -1, osSemaphoreNew == NULL
// Return 0, init success
int8_t syncInit(void) {
    int8_t status = 0;
    countSem01 = osSemaphoreNew(3, 0, NULL);
    if (countSem01 == NULL) {
        return -1;
    }
    ledRouterSem01 = osSemaphoreNew(1, 0, NULL);
    if (ledRouterSem01 == NULL) {
        return -1;
    }
    motorRouterSem01 = osSemaphoreNew(1, 0, NULL);
    if (motorRouterSem01 == NULL) {
        return -1;
    }
    ledStatusMutex = osSemaphoreNew(1, 1, NULL);
    if (ledStatusMutex == NULL) {
        return -1;
    }
    motorStatusMutex = osSemaphoreNew(1, 1, NULL);
    if (motorStatusMutex == NULL) {
        return -1;
    }
    return 0;
}