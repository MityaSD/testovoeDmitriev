#ifndef COM_H
#define COM_H

#include <windows.h>
#include <stdio.h>
#include <stdint.h>
#include <time.h>

#define SizeReceiverBuffer 20

int InitCOMport(HANDLE *port, LPCSTR nameCom);
void DeInitCOM(HANDLE *port);
void TransmitDataCOMdelay(HANDLE *port, unsigned char *data, int size);
void TransmitDataCOM(HANDLE *port, unsigned char *data, int size);
void TransmitCharCOM(HANDLE *port,  unsigned char *data);
void ReceiveDataCOM(HANDLE *port, unsigned char *buffer, int size);
void TransmitDataWaitCOM(HANDLE *port, unsigned char *data, int size);
void WaitForData(HANDLE *port);
void delay(int microcseconds);

#endif
