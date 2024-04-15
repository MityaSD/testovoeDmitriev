#include "stdio.h"
#include <windows.h>
#include "COM.h"
#include "type.h"

#include <locale.h>
#include <conio.h>
#include <time.h>

HANDLE COM;
LPCSTR COMname = "COM7";

char test = 't';
char *pTest = &test;
uint32_t number = 0;

void FormirovaniePaketa(unionUART_t* unionUART, uint8_t length)
{
    unionUART->Paket.tag = TAG;
    unionUART->Paket.length = length;
    unionUART->Paket.data[0] = number++;
    unionUART->Paket.crc =Crc16(unionUART->Paket.data, length);
    
    
}

uint16_t Crc16(uint8_t * pcBlock, unsigned short len)
{
    uint16_t crc = 0xFFFF;

    while (len--)
        crc = (crc << 8) ^ Crc16Table[(crc >> 8) ^ *pcBlock++];

    return crc;
}

int main(void)
{

    char key;


    unionUART_t unionUART;

    unionUART.Paket.tag = TAG;
    unionUART.Paket.length = 1;
    unionUART.Paket.data[0]= number;
    

    if (InitCOMport(&COM, COMname))
    {
        printf("ne sudba COM\n");

        return 1;
    }

     while (1)
    {
        if (_kbhit())
        {
            key = _getch();
            if (key == 27)
            {
                break;
            }
            if(key == 't')
            {
                FormirovaniePaketa(&unionUART, 1);
                printf("CRC = %#x\n", unionUART.Paket.crc);
                printf("number = %d\n", unionUART.Paket.data[0]);
                TransmitDataCOM(&COM, unionUART.masUART, 5 );
                printf("transmit\n\n");
            }
            if(key == 'e')
            {
                FormirovaniePaketa(&unionUART, 1);
                unionUART.Paket.crc = 0xffff;
                TransmitDataCOM(&COM, unionUART.masUART, 5 );
                printf("CRC = %#x\n", unionUART.Paket.crc);
                printf("number = %d\n", unionUART.Paket.data[0]);
                printf("transmit\n\n");
            }
            if(key == 'n')
            {   
                printf("number = ");
                scanf("%d", &number);

                FormirovaniePaketa(&unionUART, 1);
                unionUART.Paket.crc = 0xffff;
                TransmitDataCOM(&COM, unionUART.masUART, 5 );
                printf("\nCRC = %#x\n", unionUART.Paket.crc);
                printf("number = %d\n", unionUART.Paket.data[0]);
                printf("transmit\n\n");
            }
        }
    }

    return 0;
}
