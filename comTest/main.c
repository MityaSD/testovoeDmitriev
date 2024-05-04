#include "stdio.h"
#include <windows.h>
#include "COM.h"
#include "type.h"

#include <locale.h>
#include <conio.h>
#include <time.h>

#define SIZE_DATA 20
#define SIZE_CRC 2
#define SIZE_LENGTH 1
#define SIZE_PAKET SIZE_DATA + SIZE_CRC +SIZE_LENGTH

HANDLE COM;
LPCSTR COMname = "COM7";

char test = 't';
char *pTest = &test;
uint32_t number = 0;

void FormirovaniePaketa(unionUART_t* unionUART, uint8_t length)
{
    //unionUART->Paket.tag = TAG;
    unionUART->Paket.length = length + SIZE_CRC;//sizeof(CRC) =2
    /*for(int i=0; i<length;i++)
    {
        unionUART->Paket.data[i] +=1;
    }*/
    memset(unionUART->Paket.data, number++, length);

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


    unionUART_t unionUART = {0};

    //unionUART.Paket.tag = TAG;

    unionUART.Paket.length = SIZE_PAKET;
    printf("\nmasUART = %#x %#x %#x %#x\n", unionUART.masUART[0],unionUART.masUART[1],unionUART.masUART[2],unionUART.masUART[3]);
    memset(unionUART.Paket.data, 0x0, SIZE_DATA);
    //unionUART.Paket.data[0]= number;
    

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
                FormirovaniePaketa(&unionUART, SIZE_DATA);
                printf("length = %d\n", unionUART.Paket.length);
                printf("CRC = %#x\n", unionUART.Paket.crc);
                printf("data[0] = %d\n", unionUART.Paket.data[0]);
                TransmitDataCOM(&COM, unionUART.masUART, SIZE_PAKET);
                printf("masUART = %#x %#x %#x %#x\n", unionUART.masUART[0],unionUART.masUART[1],unionUART.masUART[2],unionUART.masUART[3]);
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
            if(key == 'w')
            {
                while(1)
                {
                    if (_kbhit())
                    {
                        key = _getch();
                        if(key == 'q')
                        {
                              break;
                        }
                        TransmitDataCOM(&COM, unionUART.masUART, 5 );
                    }
        
                }
            }
        }
    }

    return 0;
}
