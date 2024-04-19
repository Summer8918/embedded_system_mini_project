#ifndef SYNC_H_
#define SYNC_H_

#include "stm32f072xb.h"
#include "cmsis_os2.h"
#include "uartParser.h"

extern osSemaphoreId_t countSem01;
extern osSemaphoreId_t ledRouterSem01;
extern osSemaphoreId_t motorRouterSem01;
// extern osSemaphoreId_t ledStatusMutex;
// extern osSemaphoreId_t motorStatusMutex;
extern osSemaphoreId_t workerStatusMutex;
extern volatile uint8_t ledWorkerBusy;
extern volatile uint8_t motorWorkerBusy;

int8_t syncInit(void);

#endif // SYNC_H_