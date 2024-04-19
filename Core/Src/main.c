/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "stm32f072xb.h"
#include "motor.h"
#include "motor.c"

//Global variables
volatile uint16_t commandLED = 0;
volatile uint16_t commandMotor = 0;

// Semaphores
// osSemaphoreId_t binarySem03LEDWorkerHandle;
// osSemaphoreId_t binarySem04MotorWorkerHandle;

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi2;

TSC_HandleTypeDef htsc;
/* For motor */
volatile uint32_t debouncer; 
// The speed we are setting the motor to (3rd and 4th digits)
volatile int16_t target_rpm = 0;

PCD_HandleTypeDef hpcd_USB_FS;
/* Definitions for task router */
osThreadId_t routerTaskHandle;
const osThreadAttr_t routerTask_attributes = {
  .name = "routerTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for LED worker task */
osThreadId_t LEDTaskHandle;
const osThreadAttr_t LEDTask_attributes = {
  .name = "LEDTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for Motor worker task */
osThreadId_t motorTaskHandle;
const osThreadAttr_t motorTask_attributes = {
  .name = "motorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};


/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI2_Init(void);
static void MX_TSC_Init(void);
static void MX_USB_PCD_Init(void);
void StartRouterTask(void *argument);
void StartMotorTask(void *argument);
void StartLEDTask(void *argument);
void initLEDs(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  initUsart3();
  initLEDs();
  motor_init();

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  // comment thread temprariely.
  /* creation of routerTask */
  routerTaskHandle = osThreadNew(StartRouterTask, NULL, &routerTask_attributes);
  /* creation of LEDTask */
  LEDTaskHandle = osThreadNew(StartLEDTask, NULL, &LEDTask_attributes);
  /* creation of motorTask */
  motorTaskHandle = osThreadNew(StartMotorTask, NULL, &motorTask_attributes);
  /* creation of UARTTask */
  UARTTaskHandle = osThreadNew(StartParseUartTask, NULL, &UARTTask_attributes);

  // init the resources for the synchronization of router
  // uart parser, led and motor threads
  uint8_t status = syncInit();
  if (status != 0) {
    transmitCharArray("Fail to init sync resources!\n");
  }

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  while (1)
  {
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, NCS_MEMS_SPI_Pin|EXT_RESET_Pin|LD3_Pin|LD6_Pin
                          |LD4_Pin|LD5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : NCS_MEMS_SPI_Pin EXT_RESET_Pin LD3_Pin LD6_Pin
                           LD4_Pin LD5_Pin */
  GPIO_InitStruct.Pin = NCS_MEMS_SPI_Pin|EXT_RESET_Pin|LD3_Pin|LD6_Pin
                          |LD4_Pin|LD5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : MEMS_INT1_Pin MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT1_Pin|MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

//New branch

/* USER CODE BEGIN Header_StartRouterTask */
/**
  * @brief  Function implementing the routerTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartRouterTask */
void StartRouterTask(void *argument)
{ 
  //Global command queue
  extern Cmd_Queue * cmdQueue;
  //Global variables to pass command to worker threads
  extern volatile uint16_t commandLED;
  extern volatile uint16_t commandMotor;
  //Command popped from queue
  uint16_t commandIn = 0;
  /* Infinite loop */
  //uint16_t item = queuePop(cmdQueue);
  
  for(;;)
  {
    // wait until countSem01 >= 1
    osSemaphoreAcquire(countSem01, osWaitForever);
    transmitCharArray("Router: Get countSem01\n");

    //If queue is not empty
    if (!isQueueEmpty(cmdQueue)) {
      //Placeholder for task priorities
      //osDelay(1);

      // Acquire the mutex for led status
      osSemaphoreAcquire(workerStatusMutex, osWaitForever);

      // LED worker is not busy
      if (ledWorkerBusy == 0) {
        osSemaphoreRelease(workerStatusMutex);

        commandIn = queuePopItemByOpcode(cmdQueue, LED_COMMAND_OPCODE);
        // There is LED command in the queue
        if (commandIn != 0xFFFF) {
          commandLED = commandIn;
          transmitCharArray("Wake up led worker\n");
          //Wake up led worker thread
          osSemaphoreRelease(ledRouterSem01);
        } else {
          //transmitCharArray("LED thread not busy but no LED command in queue\n");
        }
      } else {
        osSemaphoreRelease(workerStatusMutex);
      }

      // Acquire the mutex for motor status
      osSemaphoreAcquire(workerStatusMutex, osWaitForever);

      if (motorWorkerBusy == 0) {
        osSemaphoreRelease(workerStatusMutex);
        commandIn = queuePopItemByOpcode(cmdQueue, MOTOR_COMMAND_OPCODE);
        // There is Motor command in the queue
        if (commandIn != 0xFFFF) {
          commandMotor = commandIn;
          transmitCharArray("Wake up motor worker\n");
          //Wake up motor worker thread
          osSemaphoreRelease(motorRouterSem01);
        } else {
          //transmitCharArray("Motor thread not busy but no motor command in queue\n");
        }
      } else {
        osSemaphoreRelease(workerStatusMutex);
      }

      // //Retrieve command from queue
      // commandIn = queuePop(cmdQueue);
      // //Determine which worker task corresponds to command
      //   switch (commandIn & 0xF000) {
      //   //LED command
      //   case 0xA000:
      //     commandLED = commandIn;
      //     break;
      //   //Motor command
      //   case 0xB000:
      //     commandMotor = commandIn;
      //     break;
      //   default:
      //   break;
      // }
    }
    //transmitCharArray("Router: go to sleep\n");
  }
}
  /* USER CODE END 5 */

void initLEDs(void) {
	// red LED PC6, blue LED (PC7), green LED PC9, orange LED PC8
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // Enable peripheral clock to PC
  // set the MODER, 01: General purpose output mode
	// init PC6 MODER
	GPIOC->MODER |= (1 << 12);
	GPIOC->MODER &= ~(1 << 13);
    // init PC7 MODER
	GPIOC->MODER |= (1 << 14);
	GPIOC->MODER &= ~(1 << 15);
	// init PC8 MODER
	GPIOC->MODER |= (1 << 16);
	GPIOC->MODER &= ~(1 << 17);
	// init PC9 MODER
	GPIOC->MODER |= (1 << 18);
	GPIOC->MODER &= ~(1 << 19);
  // Set the pins to low speed in the OSPEEDR register
	GPIOC->OSPEEDR &= ~((1 << 12) | (1 << 13));
	GPIOC->OSPEEDR &= ~((1 << 14) | (1 << 15));
	GPIOC->OSPEEDR &= ~((1 << 16) | (1 << 17));
	GPIOC->OSPEEDR &= ~((1 << 18) | (1 << 19));

	// Set LED to no pull-up/down resistors in the PUPDR register
	// 00: No pull-up, pull-down
	GPIOC->PUPDR &= ~((1 << 16) | (1 << 17) | (1 << 18) | (1 << 19));
	GPIOC->PUPDR &= ~((1 << 12) | (1 << 13) | (1 << 14) | (1 << 15));
	// set PC6-9 to 0
	GPIOC->ODR &= ~(1 << 6);
	GPIOC->ODR &= ~(1 << 7);
	GPIOC->ODR &= ~(1 << 8);
	GPIOC->ODR &= ~(1 << 9);
}

/* USER CODE BEGIN Header_StartLEDTask */
/**
  * @brief  Function implementing the LEDTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartLEDTask */
void StartLEDTask(void *argument)
{
  //binarySem03LEDWorkerHandle = osSemaphoreNew(1, 1, NULL);

  extern volatile uint16_t commandLED;
  volatile uint8_t LEDColor = 0, LEDAction = 0, LEDSpeed = 0;
  volatile uint32_t colorMask[5] = {GPIO_ODR_6, GPIO_ODR_9, GPIO_ODR_7, GPIO_ODR_8, GPIO_ODR_6 | GPIO_ODR_7 | GPIO_ODR_8 | GPIO_ODR_9};
  volatile uint8_t blink[4] = {0,0,0,0};
  volatile uint8_t blinkTime[4] = {0,0,0,0};
  volatile uint32_t startTime[4] = {0,0,0,0};
  volatile uint32_t currentTime = 0;

  /* Infinite loop */
  for(;;)
  {
    //osSemaphoreAcquire(binarySem03LEDWorkerHandle, osWaitForever);

    // wait to be woken up by router
    osSemaphoreAcquire(ledRouterSem01, osWaitForever);
    transmitCharArray("LED worker Got the signal\n");

    osSemaphoreAcquire(workerStatusMutex, osWaitForever);
    ledWorkerBusy = 1;
    osSemaphoreRelease(workerStatusMutex);

    LEDColor = (commandLED & 0x0F00) >> 8;
    LEDAction = (commandLED & 0x00F0) >> 4;

    //LED action
    switch (LEDAction) {
      //On
      case 1:
        GPIOC->ODR |= colorMask[LEDColor-1];
        if (LEDColor == 5) {
          blink[0] = 0;
          blink[1] = 0;
          blink[2] = 0;
          blink[3] = 0;
        }
        else
          blink[LEDColor-1] = 0;
        break;
      //Off
      case 2:
        GPIOC->ODR &= ~colorMask[LEDColor-1];
        if (LEDColor == 5) {
          blink[0] = 0;
          blink[1] = 0;
          blink[2] = 0;
          blink[3] = 0;
        }
        else
          blink[LEDColor-1] = 0;
        break;
      //Toggle
      case 3:
        GPIOC->ODR ^= colorMask[LEDColor-1];
        if (LEDColor == 5) {
          blink[0] = 0;
          blink[1] = 0;
          blink[2] = 0;
          blink[3] = 0;
        }
        else
          blink[LEDColor-1] = 0;
        break;
      //Blink
      case 4:
        if (LEDColor == 5) {
          blink[0] = 1;
          blinkTime[0] = commandLED & 0xF;
          blink[1] = 1;
          blinkTime[1] = commandLED & 0xF;
          blink[2] = 1;
          blinkTime[2] = commandLED & 0xF;
          blink[3] = 1;
          blinkTime[3] = commandLED & 0xF;
        }
        else
          blink[LEDColor-1] = 1;
          blinkTime[LEDColor-1] = commandLED & 0xF;
        break;
      default:
    }

    //Blinking
    currentTime = xTaskGetTickCount();
    for (int i = 0; i < 4; i++) {
      if (blink[i]) {
        if (currentTime - startTime[i] > blinkTime[i]*100) {
          GPIOC->ODR ^= colorMask[i];
          startTime[i] = currentTime;
        }
      }
      else
        startTime[i] = currentTime;
    }

    commandLED = 0;

    transmitCharArray("LED worker go to sleep\n");

    osSemaphoreAcquire(workerStatusMutex, osWaitForever);
    ledWorkerBusy = 0;
    osSemaphoreRelease(workerStatusMutex);

    //osSemaphoreRelease(binarySem03LEDWorkerHandle);
  }
}

/* USER CODE BEGIN Header_StartLEDTask */
/**
  * @brief  Function implementing the MotorTask thread.
  * @param  argument: Not used
  * @retval None
  */ 
/* USER CODE END Header_StartLEDTask */
void StartMotorTask(void *argument)
{
  extern volatile uint16_t commandMotor;
  int speedAdjust = 0; // boolean that says if speed needs to be adjusted: speed accounted for 3rd and 4th digit
  //command 0xB-[1/2/3/4]
  /* 2nd Digit
  *   - 1: Turn motor on (enable 3rd and 4th digit)
  *   - 2: Turn motor off
  *   - 3: Change motor speed (enable 3rd and 4th digit)
  * 3rd & 4th Digit: RPM of speed --> Clamped at < 100, done in motor.c 
  */

  /* Infinite loop */
  for(;;)
  {
    // wait to be woken up by router
    osSemaphoreAcquire(motorRouterSem01, osWaitForever);
    transmitCharArray("Motor worker Got the signal\n");
    osSemaphoreAcquire(workerStatusMutex, osWaitForever);
    motorWorkerBusy = 1;
    osSemaphoreRelease(workerStatusMutex);

    extern volatile uint16_t commandMotor;
    // 2nd character (turn motor on, off, or adjust speed)
    switch (commandMotor & 0x0F00) { 
      case 0x0100: 
        // turn motor on (will need to adjust speed)
        speedAdjust = 1;
        NVIC_EnableIRQ(TIM7_IRQn);          // Enable interrupt in NVIC
        NVIC_SetPriority(TIM7_IRQn,2);
        break;
      case 0x0200:
        // turn motor off
        target_rpm = 0;
        //NVIC_DisableIRQ(TIM7_IRQn);          // Enable interrupt in NVIC
        break;
      case 0x0300:
        // change motor speed 
        speedAdjust = 1;
        break;
      default:
        break;
    }
    if (speedAdjust == 1){
      // 3rd & 4th character
      target_rpm = (commandMotor & 0x00FF);
    }
    commandMotor = 0;
    speedAdjust = 0;
    
    //If motor is running
      //Read encoder in timer 3
      //osDelay known time
      //Read encoder in timer 3
      //Reset encoder value
      //do PI update

    //Placeholder for task priorities
    osDelay(1);
    osSemaphoreAcquire(workerStatusMutex, osWaitForever);
    motorWorkerBusy = 0;
    osSemaphoreRelease(workerStatusMutex);
    transmitCharArray("Motor worker go to sleep\n");
  }
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
