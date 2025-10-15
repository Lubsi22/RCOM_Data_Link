#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>

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

        while (STOP == FALSE && alarmCount < connectionParameters.nRetransmissions)
        {
            if (alarmEnabled == FALSE)
            {
                writeBytesSerialPort(buf, bufSize);
                printf("Tx: Sent SET\n");

                alarm(connectionParameters.timeout); // Set alarm to be triggered in 3s
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
                        state = A;
                    }
                    else state = FLAG_I;
                    break;
                }
                case A:
                {
                    if (byte == A_R)
                    {
                        printf("byte = 0x%02X\n", byte);
                        state = C;
                    }
                    else state = FLAG_I;

                    break;
                }
                case C:
                {
                    if (byte == UA)
                    {
                        printf("byte = 0x%02X\n", byte);
                        state = BCC;
                    }
                    else state = FLAG_I;
                    break;
                }
                case BCC:
                {
                    if (byte == (A_R ^ UA))
                    {
                        printf("byte = 0x%02X\n", byte);
                        state = FLAG_F;
                    }
                    else state = FLAG_I;
                    break;
                }
                case FLAG_F:
                {
                    if (byte == FLAG)
                    {
                        printf("byte = 0x%02X\n", byte);
                        STOP = TRUE;
                        alarm(0);
                    }
                    else state = FLAG_I;
                    break;
                }
                default:
                    break;
                }
            }
        }

        if(STOP){
            printf("Tx: Got UA - link opened \n");
            return 0;
        }

    }
    else if (connectionParameters.role == LlRx)
    {

        printf("Rx: Waiting for SET...\n");
        while (STOP == FALSE)
        {
            
            unsigned char byte;
            if (readByteSerialPort(&byte) != 1) continue;

            switch (state)
            {
            case FLAG_I:
            {
                if (byte == FLAG)
                {
                    printf("byte = 0x%02X\n", byte);
                    state = A;
                }
                else state = FLAG_I;

                break;
            }
            case A:
            {
                if (byte == A_T)
                {
                    printf("byte = 0x%02X\n", byte);
                    state = C;
                }
                else state = FLAG_I;
                break;
            }
            case C:
            {
                if (byte == SET)
                {
                    printf("byte = 0x%02X\n", byte);
                    state = BCC;
                }
                else state = FLAG_I;
                break;
            }
            case BCC:
            {
                if (byte == (A_T ^ SET))
                {
                    printf("byte = 0x%02X\n", byte);
                    state = FLAG_F;
                }
                else state = FLAG_I;
                break;
            }
            case FLAG_F:
            {
                if (byte == FLAG)
                {
                    printf("byte = 0x%02X\n", byte);
                    STOP = TRUE;
                }
                else state = FLAG_I;
                break;
            }
            default:
                break;
            }
        }

        unsigned char ua[5] = {FLAG, A_R, UA, A_R ^ UA, FLAG};
        writeBytesSerialPort(ua, 5);
        printf("Rx: Sent UA\n");

        return 0;
    }

    return -1;
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
    while (readByteSerialPort(packet) == 1)
    {
        printf("%s \n", packet);
    }

    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(LinkLayer connectionParameters)
{   
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
        // Send Disc 

        buf[0] = FLAG;
        buf[1] = A_T;
        buf[2] = DISC;
        buf[3] = buf[1] ^ buf[2];
        buf[4] = FLAG;

        alarmCount = 0;
        alarmEnabled = FALSE;
        STOP = FALSE;
        state = FLAG_I;

        while (STOP == FALSE && alarmCount < connectionParameters.nRetransmissions)
        {
            if (alarmEnabled == FALSE)
            {
                writeBytesSerialPort(buf, 5);
                printf("Tx: Sent DISC\n");
                alarm(connectionParameters.timeout);
                alarmEnabled = TRUE;
            }

            unsigned char byte;
            while (readByteSerialPort(&byte) == 1 && STOP == FALSE)
            {
                switch (state)
                {
                case FLAG_I:
                    if (byte == FLAG) {
                        state = A;
                        printf("byte = 0x%02X\n", byte);
                    }
                    break;

                case A:
                    if (byte == A_R) {
                        state = C;
                        printf("byte = 0x%02X\n", byte);
                    }
                    else state = FLAG_I;
                    break;

                case C:
                    if (byte == DISC) {
                        state = BCC;
                        printf("byte = 0x%02X\n", byte);
                    }
                    else if (byte != FLAG) state = FLAG_I;
                    break;

                case BCC:
                    if (byte == (A_R ^ DISC)) {
                        state = FLAG_F;
                        printf("byte = 0x%02X\n", byte);
                    }
                    else state = FLAG_I;
                    break;

                case FLAG_F:
                    if (byte == FLAG)
                    {   
                        printf("byte = 0x%02X\n", byte);
                        printf("Tx: Received DISC from Rx\n");
                        STOP = TRUE;
                        alarm(0);
                    }
                    else state = FLAG_I;
                    break;
                }
            }
        }

        //Send UA
        if (!STOP)
        {
            return -1;
        }

        unsigned char ua[5] = {FLAG, A_T, UA, A_T ^ UA, FLAG};
        writeBytesSerialPort(ua, 5);
        printf("Tx: Sent UA, closing link.\n");

    }
    else if (connectionParameters.role == LlRx)
    {   
        printf("Rx: Waiting for DISC from Tx\n");

        STOP = FALSE;
        state = FLAG_I;

        while (STOP == FALSE)
        {
            unsigned char byte;
            if (readByteSerialPort(&byte) != 1) continue;

            switch (state)
            {
            case FLAG_I:
                if (byte == FLAG) {
                    state = A;
                    printf("byte = 0x%02X\n", byte);
                }
                break;

            case A:
                if (byte == A_T) {
                    state = C;
                    printf("byte = 0x%02X\n", byte);
                }
                else state = FLAG_I;
                break;

            case C:
                if (byte == DISC) {
                    state = BCC;
                    printf("byte = 0x%02X\n", byte);
                }
                else state = FLAG_I;
                break;

            case BCC:
                if (byte == (A_T ^ DISC)) {
                    state = FLAG_F;
                    printf("byte = 0x%02X\n", byte);
                }
                else state = FLAG_I;
                break;

            case FLAG_F:
                if (byte == FLAG)
                {
                    printf("Rx: Received DISC from Tx\n");
                    STOP = TRUE;
                }
                else state = FLAG_I;
                break;
            }
        }


        unsigned char disc[5] = {FLAG, A_R, DISC, A_R ^ DISC, FLAG};
        writeBytesSerialPort(disc, 5);
        printf("Rx: Sent DISC back\n");

        STOP = FALSE;
        state = FLAG_I;
        printf("Rx: Waiting for UA\n");

        while (STOP == FALSE)
        {
            unsigned char byte;
            if (readByteSerialPort(&byte) != 1) continue;

            switch (state)
            {
            case FLAG_I:
                if (byte == FLAG) {
                    state = A;
                    printf("byte = 0x%02X\n", byte);
                }
                break;

            case A:
                if (byte == A_T) {
                    state = C;
                    printf("byte = 0x%02X\n", byte);
                }
                else state = FLAG_I;
                break;

            case C:
                if (byte == UA) {
                    state = BCC;
                    printf("byte = 0x%02X\n", byte);
                }
                else state = FLAG_I;
                break;

            case BCC:
                if (byte == (A_T ^ UA)) {
                    state = FLAG_F;
                    printf("byte = 0x%02X\n", byte);
                }
                else state = FLAG_I;
                break;

            case FLAG_F:
                if (byte == FLAG)
                {   
                    printf("byte = 0x%02X\n", byte);
                    printf("Rx: Received UA, closing link.\n");
                    STOP = TRUE;
                }
                else state = FLAG_I;
                break;
            }
        }
    }

    


    if (closeSerialPort() < 0)
    {
        perror("closeSerialPort");
        exit(-1);
    }

    printf("Serial port closed successfully\n");

    return 0;
}
