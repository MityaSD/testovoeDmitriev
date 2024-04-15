#ifndef COM_c
#define COM_c

#include "COM.h"

int InitCOMport(HANDLE *port, LPCSTR nameCom)
{

    DCB dcb;
    //DWORD dwCommEvent;

    *port = CreateFile(nameCom, GENERIC_WRITE | GENERIC_READ, 0, NULL, OPEN_EXISTING, 0, NULL);
    if (port == INVALID_HANDLE_VALUE)
    {
        printf("ERROR: error open COM port");
        return 1;
    }

    memset(&dcb, 0, sizeof(dcb));
    dcb.DCBlength = sizeof(DCB);

    if (!GetCommState(*port, &dcb))
    {
        printf("ERROR: Failed to get current port settings\n");
        CloseHandle(*port);
        return 1;
    }

    dcb.BaudRate = CBR_115200; 
    // dcb.BaudRate = BAUD_9600;

    dcb.fBinary = 1;                       // 1- Бинарный режим
    dcb.fParity = 0;                       // Отключение проверки четности
    dcb.fOutxCtsFlow = 0;                  // Отключение управлении потоком через CTS
    dcb.fOutxDsrFlow = 0;                  // Отключение управлении потоком через DSR
    dcb.fDtrControl = DTR_CONTROL_DISABLE; // Отключение управления потоком через DTR
    dcb.fDsrSensitivity = 0;               // Отключение чувствительности к DSR
    dcb.fTXContinueOnXoff = 0;             // Остановка передачи данных при получении символа Xoff
    dcb.fOutX = 0;                         // Отключение управления потоком через Xon/Xoff на линии выходных данных
    dcb.fInX = 0;                          // Отключение управления потоком через Xon/Xoff на линии входных данных
    dcb.fErrorChar = 0;                    // Отключение замещения символа при возникновении ошибок
    dcb.fNull = 0;                         // Отключение удаления символа NULL
    dcb.fRtsControl = RTS_CONTROL_DISABLE; // Отключение управления потоком через RTS
    dcb.fAbortOnError = 0;                 // Не прекращать операции чтения и записи при ошибке
    dcb.fDummy2 = 0;                       // Зарезервированные биты
    dcb.wReserved = 0;                     // Зарезервированное поле
    dcb.XonLim = 0;                        // Пороговое значение символа Xon
    dcb.XoffLim = 0;                       // Пороговое значение символа Xoff
    dcb.ByteSize = 8;                      // Количество бит данных в байте (8)
    dcb.Parity = NOPARITY;                 // Отключение бита четности
    dcb.StopBits = ONESTOPBIT;             // Один стоп-бит
    dcb.XonChar = 0;                       // Символ Xon не используется
    dcb.XoffChar = 0;                      // Символ Xoff не используется
    dcb.ErrorChar = 0;                     // Замена ошибочных символов отключена
    dcb.EofChar = 0;                       // Конец входных данных не используется
    dcb.EvtChar = 0;                       // Приема событий не используется
    dcb.wReserved1 = 0;                    // Зарезервированное поле

    if (!SetCommState(*port, &dcb))
    {
        printf("ya v SetCommState\n");
        DWORD lastError = GetLastError();
        LPSTR errorMessage;
        FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM |
                          FORMAT_MESSAGE_IGNORE_INSERTS,
                      NULL, lastError,
                      MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (LPSTR)&errorMessage, 0, NULL);
        MessageBox(NULL, errorMessage, "Error", MB_OK);
        // printf ("SetCommState() failed with error code: %ld\n", lastError);

        CloseHandle(*port);
        *port = INVALID_HANDLE_VALUE;
    }

    return 0;
}

void DeInitCOM(HANDLE *port)
{
    CloseHandle(*port);
}

void TransmitDataCOMdelay(HANDLE *port, unsigned char *data, int size)
{
    DWORD bytess;
    for (int i = 0; i < size; i++)
    {
        //printf("Send %#X\n", *(data+i));
        if (!WriteFile(*port, data + i , 1, &bytess, NULL))
        {
            DWORD errorCode = GetLastError();
            LPSTR errorMessage;

            FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM |
                              FORMAT_MESSAGE_IGNORE_INSERTS,
                          NULL, errorCode,
                          MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (LPSTR)&errorMessage, 0, NULL);

            // printf("ОШИБКА transmit: %s", errorMessage);
            MessageBox(NULL, errorMessage, "Error", MB_OK);

            LocalFree(errorMessage);
        }
        printf("data+%d = %#X\n", i, *(data+i));
        delay(8000);
    }
}

void TransmitDataCOM(HANDLE *port, unsigned char *data, int size)
{
    DWORD bytess;
    if (!WriteFile(*port, data, size, &bytess, NULL))
        {
            DWORD errorCode = GetLastError();
            LPSTR errorMessage;

            FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM |
                              FORMAT_MESSAGE_IGNORE_INSERTS,
                          NULL, errorCode,
                          MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (LPSTR)&errorMessage, 0, NULL);

            // printf("ОШИБКА transmit: %s", errorMessage);
            MessageBox(NULL, errorMessage, "Error", MB_OK);

            LocalFree(errorMessage);
        }
}


void TransmitCharCOM(HANDLE *port, unsigned char *data)
{
    DWORD bytess;
    if (!WriteFile(*port, data, 1, &bytess, NULL))
    {
        DWORD errorCode = GetLastError();
        LPSTR errorMessage;

        FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM |
                          FORMAT_MESSAGE_IGNORE_INSERTS,
                      NULL, errorCode,
                      MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (LPSTR)&errorMessage, 0, NULL);

        // printf("ОШИБКА transmit: %s", errorMessage);
        MessageBox(NULL, errorMessage, "Error", MB_OK);

        LocalFree(errorMessage);
    }
}

void WaitForData(HANDLE *port)
{
    DWORD eventMask;
    if (!SetCommMask(*port, EV_RXCHAR))
    {
        printf("ERROR: Failed to set comm mask\n");
        return;
    }

    if (!WaitCommEvent(*port, &eventMask, NULL))
    {
        DWORD errorCode = GetLastError();
        LPSTR errorMessage;

        FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM |
                          FORMAT_MESSAGE_IGNORE_INSERTS,
                      NULL, errorCode,
                      MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (LPSTR)&errorMessage, 0, NULL);

        MessageBox(NULL, errorMessage, "Error", MB_OK);

        LocalFree(errorMessage);
        return;
    }
}

void ReceiveDataCOM(HANDLE *port, unsigned char *buffer, int size) 
{ 
    DWORD totalBytesRead = 0; // Общее количество прочитанных байт
    DWORD bytesRead; // Количество байт, прочитанных за одну итерацию чтения

    while (totalBytesRead < size) {
        if (!ReadFile(*port, buffer + totalBytesRead, size - totalBytesRead, &bytesRead, NULL))
        {
            DWORD errorCode = GetLastError();
            LPSTR errorMessage;

            FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM |
                FORMAT_MESSAGE_IGNORE_INSERTS,
                NULL, errorCode,
                MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (LPSTR)&errorMessage, 0, NULL);

            MessageBox(NULL, errorMessage, "Error", MB_OK);

            LocalFree(errorMessage);
            break;
        }

        totalBytesRead += bytesRead;
    }
}


void delay(int microcseconds)
{
    
    
    LARGE_INTEGER frequency;
    LARGE_INTEGER start, current;

    QueryPerformanceFrequency(&frequency);

    ULONGLONG ticks = (ULONGLONG)(((double)microcseconds/ 1000000.0)*frequency.QuadPart);
    QueryPerformanceCounter(&start);
    do
    {
        QueryPerformanceCounter(&current);
    } while((current.QuadPart-start.QuadPart)<ticks);

}

#endif