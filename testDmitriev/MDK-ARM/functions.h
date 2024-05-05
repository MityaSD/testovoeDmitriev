#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include "type.h"
#include "main.h"

extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;

extern unionUART_t unionUART;
extern unionUART_t unionUARTtx;
extern unionReceipt_t unionReceipt;
	
extern CircBuf_t txMaster;
extern CircBuf_t rxMaster;
	
extern CircBuf_t txSlave;
extern CircBuf_t rxSlave;

extern int counter;

uint16_t dataProcessingSlave(void);
uint16_t dataProcessingMaster(void);
void SendMessageSlave(unionReceipt_t *message);
void SendMessageMaster(unionUART_t *message);
void creatingData(unionUART_t* unionUART, uint8_t length);
uint16_t readingLength(uint8_t *length, CircBuf_t *circBuf);
void readingData(uint8_t headPackage, CircBuf_t *circBuf, uint8_t *dataStruct, uint16_t sizeStruct);
void updateHead(void);

#endif
