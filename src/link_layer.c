#include <stdio.h>
#include <stdlib.h>
#include <signal.h>

// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int alarmEnabled = FALSE;
int alarmCount = 0;

void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++;

    printf("Alarm #%d received\n", alarmCount);
}

int llopen(LinkLayer connectionParameters)
{
    if (openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate) < 0)
    {
        perror("openSerialPort");
        exit(-1);
    }

    printf("Serial port %s opened\n", connectionParameters.serialPort);

    int bufSize = 5;
    unsigned char buf[bufSize];
    int STOP = FALSE;
    enum State state = FLAG_I;

    struct sigaction act = {0};
    act.sa_handler = &alarmHandler;
    if (sigaction(SIGALRM, &act, NULL) == -1)
    {
        perror("sigaction");
        exit(1);
    }

    if (connectionParameters.role == LlTx)
    {
        buf[0] = FLAG;
        buf[1] = A_T;
        buf[2] = SET;
        buf[3] = buf[1] ^ buf[2];
        buf[4] = FLAG;
        while (STOP == FALSE && alarmCount < 5)
        {
            if (alarmEnabled == FALSE)
            {
                int bytes = writeBytesSerialPort(buf, bufSize);
                printf("%d bytes written to serial port\n", bytes);

                alarm(3); // Set alarm to be triggered in 3s
                alarmEnabled = TRUE;
            }

            unsigned char byte;

            while (readByteSerialPort(&byte) == 1 && STOP == FALSE)
            {
                switch (state)
                {
                case FLAG_I:
                {
                    if (byte == FLAG)
                    {
                        printf("byte = 0x%02X\n", byte);
                        buf[state] = FLAG;
                        state++;
                    }
                    else
                    {
                        state = 0;
                    }
                    break;
                }
                case A_M:
                {
                    if (byte == 0x01)
                    {
                        printf("byte = 0x%02X\n", byte);
                        buf[state] = A_R;
                        state++;
                    }
                    else
                    {
                        state = 0;
                    }
                    break;
                }
                case C_M:
                {
                    if (byte == 0x07)
                    {
                        printf("byte = 0x%02X\n", byte);
                        buf[state] = UA;
                        state++;
                    }
                    else
                    {
                        state = 0;
                    }
                    break;
                }
                case BCC:
                {
                    if (byte == buf[1] ^ buf[2])
                    {
                        printf("byte = 0x%02X\n", byte);
                        buf[state] = buf[1] ^ buf[2];
                        state++;
                    }
                    else
                    {
                        state = 0;
                    }
                    break;
                }
                case FLAG_F:
                {
                    if (byte == FLAG)
                    {
                        printf("byte = 0x%02X\n", byte);
                        buf[state] = FLAG;
                        STOP = TRUE;
                        alarm(0);
                    }
                    else
                    {
                        state = 0;
                    }
                    break;
                }
                default:
                    break;
                }
            }
        }
    }
    else if (connectionParameters.role == LlRx)
    {
        int nBytesBuf = 0;

        while (STOP == FALSE)
        {
            
            unsigned char byte;
            int bytes = readByteSerialPort(&byte);
            nBytesBuf += bytes;
            if(bytes == 1){
            switch (state)
            {
            case FLAG_I:
            {
                if (byte == FLAG)
                {
                    printf("byte = 0x%02X\n", byte);
                    buf[state] = FLAG;
                    state++;
                }
                else
                {
                    state = 0;
                }

                break;
            }
            case A_M:
            {
                if (byte == A_T)
                {
                    printf("byte = 0x%02X\n", byte);
                    buf[state] = A_R;
                    state++;
                }
                else
                {
                    state = 0;
                }
                break;
            }
            case C_M:
            {
                if (byte == SET)
                {
                    printf("byte = 0x%02X\n", byte);
                    buf[state] = UA;
                    state++;
                }
                else
                {
                    state = 0;
                }
                break;
            }
            case BCC:
            {
                if (byte == A_T ^ SET)
                {
                    printf("byte = 0x%02X\n", byte);
                    buf[state] = A_R ^ UA;
                    state++;
                }
                else
                {
                    state = 0;
                }
                break;
            }
            case FLAG_F:
            {
                if (byte == FLAG)
                {
                    printf("byte = 0x%02X\n", byte);
                    buf[state] = FLAG;
                    writeBytesSerialPort(buf, bufSize);
                    STOP = TRUE;
                }
                else
                {
                    state = 0;
                }
                break;
            }
            default:
                break;
            }
        }
        }
    }

    return 0;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    int bytes = writeBytesSerialPort(buf, bufSize);

    printf("%d bytes written to serial port\n", bytes);

    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    while (readByteSerialPort(&packet) == 1)
    {
        printf("%c \n", packet);
    }

    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose()
{
    if (closeSerialPort() < 0)
    {
        perror("closeSerialPort");
        exit(-1);
    }

    printf("Serial port closed\n");

    return 0;
}
