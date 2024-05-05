#ifndef TYPE_H
#define TYPE_H

#include <stdint.h>

#define BUFFER_SIZE 1024
#define SIZE_DATA 20
#define SIZE_CRC 2

typedef struct __attribute__((packed))
{
    uint8_t length;
    uint16_t crc;
    uint8_t data[SIZE_DATA]; 
}package_t;

typedef union
{
    package_t package;
    uint8_t masUART[sizeof(package_t)];
}unionUART_t;

typedef struct __attribute__((packed))
{
	uint8_t length;
	uint16_t crc;
	uint8_t state;
}receipt_t;

typedef union{
	receipt_t receipt;
	uint8_t masReceipt[sizeof(receipt_t)];
}unionReceipt_t;

enum {
	STATE_WAITING_LENGTH,
	STATE_CRC_CHECK,
	STATE_RECEIVING_DATA,
	STATE_PROCESSING_DATA
};

enum{
		OK = 1,
		ERROR_RECEIPT
};

enum{
	MASTER,
	SLAVE
};

typedef struct
{
	uint16_t tail;
	uint16_t head;
	uint8_t buffer[BUFFER_SIZE];
}CircBuf_t;

extern const unsigned short Crc16Table[256];




#endif