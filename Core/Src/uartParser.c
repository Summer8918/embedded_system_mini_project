#include "uartParser.h"

volatile uint8_t strIndex = 0;
volatile uint8_t cmdIndex = 0;
volatile uint8_t cmdStrIndex = 0;
volatile char tmpStr[TMP_STR_LEN];
volatile char cmd[4][TMP_STR_LEN];
volatile uint16_t commandOut = 0x0000;

//Initialize USART3 - PC4 TX, PC5 RX
void initUsart3(void) {
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // Enable peripheral clock to PC
  // set pc4 to AF mode, 0x10
  GPIOC->MODER |= (1 << 9);
  GPIOC->MODER &= ~(1 << 8);
  // set pc5 to AF mode, 0x10
  GPIOC->MODER |= (1 << 11);
  GPIOC->MODER &= ~(1 << 10);

  // set PC4 AFRL to 0001: AF1
  GPIOC->AFR[0] |= (0x1 << GPIO_AFRL_AFRL4_Pos);
  // set PC5 AFRL to 0001: AF1
  GPIOC->AFR[0] |= (0x1 << GPIO_AFRL_AFRL5_Pos);
  RCC->APB1ENR |= RCC_APB1ENR_USART3EN;

  uint32_t fClk = HAL_RCC_GetHCLKFreq();

  // set baud rate
  uint32_t baudRate = 115200;
  uint32_t usartBRR = fClk / baudRate;
  USART3->BRR = usartBRR;

  // enable the transmitter and receiver hardware of USART3
  USART3->CR1 |= USART_CR1_TE;
  USART3->CR1 |= USART_CR1_RE;

  // Enable USART peripheral.
  USART3->CR1 |= USART_CR1_UE;

  // Enable the receive register not empty interrupt.
  USART3->CR1 |= USART_CR1_RXNEIE;
  // Enable and set the USART interrupt priority in the NVIC.
  NVIC_EnableIRQ(USART3_4_IRQn);
  NVIC_SetPriority(USART3_4_IRQn, 3);
  transmitCharArray("UART enabled");
}

// Handle uart RX with interrupt
void USART3_4_IRQHandler(void) {
	while((USART3->ISR & USART_ISR_RXNE) == 0) {
	}
    //Pull character from UART
	  volatile uint8_t usartReceivedData = USART3->RDR;
    //Command string terminated with enter key
	  if (usartReceivedData == '\r') {
      //Parse words from received command string
      volatile uint8_t strLen = strIndex + 1;
      strIndex = 0;
      //transmitCharArray(tmpStr);
      cmdStrIndex = 0;
      cmdIndex = 0;
      while (strIndex < strLen) {
        if (tmpStr[strIndex] == ' ') {
          cmdIndex++;
          cmdStrIndex = 0;
        }
        else {
          cmd[cmdIndex][cmdStrIndex] = tmpStr[strIndex];
          cmdStrIndex++;
        }
        strIndex++;
      }
      //Convert words into command format
      //First digit
      if (strcmp(cmd[0], "led") == 0) {
        commandOut |= 0xA000;
        //Second digit - LED
        if (strcmp(cmd[1], "red") == 0) {
          commandOut |= 0x0100;
        }
        else if (strcmp(cmd[1], "green") == 0) {
          commandOut |= 0x0200;
        }
        else if (strcmp(cmd[1], "blue") == 0) {
          commandOut |= 0x0300;
        }
        else if (strcmp(cmd[1], "orange") == 0) {
          commandOut |= 0x0400;
        }
        else {
          commandError(tmpStr);
          return;
        }

        //Third digit - LED
        if (strcmp(cmd[2], "on") == 0) {
          commandOut |= 0x0010;
        }
        else if (strcmp(cmd[2], "off") == 0) {
          commandOut |= 0x0020;
        }
        else if (strcmp(cmd[2], "toggle") == 0) {
          commandOut |= 0x0030;
        }
        else {
          commandError(tmpStr);
          return;
        }
      }    
      else if (strcmp(cmd[0], "motor") == 0) {
        commandOut |= 0xB000;
      }
      else {
        commandError(tmpStr);
        return;
      }

      //Echo successful command
      transmitCharArray("Command:");
      transmitCharArray(tmpStr);

      //Reset command string and index
      strIndex = 0;
      for (uint8_t i = 0; i < TMP_STR_LEN; i++) {
        tmpStr[i] = '\0';
        cmd[0][i] = '\0';
        cmd[1][i] = '\0';
        cmd[2][i] = '\0';
        cmd[3][i] = '\0';
      }
      
      // Store the commandOut in queue
        /*
         * TODO:
         * if queue not full, push the command to the tail of queue, tell user command is stored successfully.
         * else tell the user that queue is full, wait and try again.
         * */

        //

      //Dummy queue-router-worker to demonstrate correct uart parsing
      //Enables UART commands for green LED
      if (commandOut == 0xA210)
        GPIOC->ODR |= GPIO_ODR_9;
      else if (commandOut == 0xA220)
        GPIOC->ODR &= ~GPIO_ODR_9;
      else if (commandOut == 0xA230)
        GPIOC->ODR ^= GPIO_ODR_9;

      //Reset commandOut before starting next command receive
      commandOut = 0;

	  } else {
      //Command too long
      if (strIndex == TMP_STR_LEN) {
      	transmitCharArray("Command is too long!");
        strIndex = 0;
        for (uint8_t i = 0; i < TMP_STR_LEN; i++)
          tmpStr[i] = '\0';
      	return;
      }
      //Move to next character in command
      else {
		    tmpStr[strIndex] = usartReceivedData;
		    strIndex++;
	    }
    }
	// Transmit the characters sent by the user back to the user.
	//transmitOneChar(usartReceivedData);
}

void transmitOneChar(uint8_t ch) {
  while ((USART3->ISR & USART_ISR_TXE) == 0) {
  }
  USART3->TDR = ch;
}

void transmitCharArray (char *arr) {
  while (*arr != '\0') {
		transmitOneChar(*arr);
		arr++;
	}
  transmitOneChar('\n');
  transmitOneChar('\r');
}

//Print error message and reset strings when command is bad
void commandError(char *arr) {
  transmitCharArray("Command not recognized");
  transmitCharArray(tmpStr);
  //Reset command string and index
  strIndex = 0;
  for (uint8_t i = 0; i < TMP_STR_LEN; i++) {
    tmpStr[i] = '\0';
    cmd[0][i] = '\0';
    cmd[1][i] = '\0';
    cmd[2][i] = '\0';
    cmd[3][i] = '\0';
  }
  commandOut = 0;
}    