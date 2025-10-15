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

                alarm(connectionParameters.timeout);
                alarmEnabled = TRUE;
            }

            unsigned char byte;

            while (readByteSerialPort(&byte) == 1 && STOP == FALSE)
            {
                switch (state)
                {
                case FLAG_I:
                    if (byte == FLAG)
                        state = A;
                    else
                        state = FLAG_I;
                    break;

                case A:
                    if (byte == A_R)
                        state = C;
                    else
                        state = FLAG_I;
                    break;

                case C:
                    if (byte == UA)
                        state = BCC;
                    else
                        state = FLAG_I;
                    break;

                case BCC:
                    if (byte == (A_R ^ UA))
                        state = FLAG_F;
                    else
                        state = FLAG_I;
                    break;

                case FLAG_F:
                    if (byte == FLAG)
                    {
                        STOP = TRUE;
                        alarm(0);
                    }
                    else
                        state = FLAG_I;
                    break;

                default:
                    break;
                }
            }
        }

        if (STOP)
        {
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
            if (readByteSerialPort(&byte) != 1)
                continue;

            switch (state)
            {
            case FLAG_I:
                if (byte == FLAG)
                    state = A;
                else
                    state = FLAG_I;
                break;

            case A:
                if (byte == A_T)
                    state = C;
                else
                    state = FLAG_I;
                break;

            case C:
                if (byte == SET)
                    state = BCC;
                else
                    state = FLAG_I;
                break;

            case BCC:
                if (byte == (A_T ^ SET))
                    state = FLAG_F;
                else
                    state = FLAG_I;
                break;

            case FLAG_F:
                if (byte == FLAG)
                    STOP = TRUE;
                else
                    state = FLAG_I;
                break;

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
int llwrite(const unsigned char *buf, int bufSize, LinkLayer connectionParameters)
{
    int size = 2 * bufSize + 4;
    unsigned char stuffed[size];

    unsigned char bcc2 = 0x00;
    int index = 0;

    stuffed[index++] = FLAG;
    stuffed[index++] = A_T;
    stuffed[index++] = C_0;
    stuffed[index++] = (A_T ^ C_0);

    for (int i = 0; i < bufSize; i++)
    {
        bcc2 ^= buf[i];
    }

    for (int i = 0; i < bufSize; i++)
    {
        if (buf[i] == FLAG)
        {
            stuffed[index++] = 0x7D;
            stuffed[index++] = 0x5E;
        }
        else if (buf[i] == 0x7D)
        {
            stuffed[index++] = 0x7D;
            stuffed[index++] = 0x5D;
        }
        else
        {
            stuffed[index++] = buf[i];
        }
    }

    if (bcc2 == FLAG)
    {
        stuffed[index++] = 0x7D;
        stuffed[index++] = 0x5E;
    }
    else if (bcc2 == 0x7D)
    {
        stuffed[index++] = 0x7D;
        stuffed[index++] = 0x5D;
    }
    else
    {
        stuffed[index++] = bcc2;
    }

    stuffed[index++] = FLAG;

    int bytes = 0;

    enum State state = FLAG_I;
    int STOP = FALSE;
    alarmCount = 0;
    alarmEnabled = FALSE;

    while (STOP == FALSE && alarmCount < connectionParameters.nRetransmissions)
    {
        if (alarmEnabled == FALSE)
        {
            bytes = writeBytesSerialPort(stuffed, index);
            printf("Tx: Sent I-frame\n");

            alarm(connectionParameters.timeout); 
            alarmEnabled = TRUE;
        }

        unsigned char byte;
        if(readByteSerialPort(&byte) != 1) continue;

        switch (state)
        {
        case FLAG_I:
            if (byte == FLAG)
                state = A;
            else
                state = FLAG_I;
            break;

        case A:
            if (byte == A_R)
                state = C;
            else
                state = FLAG_I;
            break;

        case C:
            if (byte == 0xAA)
                state = BCC;
            else
                state = FLAG_I;
            break;

        case BCC:
            if (byte == (A_R ^ 0xAA))
                state = FLAG_F;
            else
                state = FLAG_I;
            break;

        case FLAG_F:
            if (byte == FLAG)
            {
                printf("Tx: Received RR - message sent successfully\n");
                STOP = TRUE;
                alarm(0);
            }
            else
                state = FLAG_I;
            break;

        default:
            break;
        }
    }

    return bytes;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{   
    unsigned char byte;
    enum State state = FLAG_I;
    int idx = 0;
    int STOP = FALSE;
    unsigned char bcc2_calc = 0;

    while (STOP == FALSE)
    {
        unsigned char byte;
            if (readByteSerialPort(&byte) != 1)
                continue;

            switch (state)
            {
            case FLAG_I:
                if (byte == FLAG)
                {
                    state = A;
                }
                break;

            case A:
                if (byte == A_T)
                {
                    state = C;
                }
                else
                    state = FLAG_I;
                break;

            case C:
                if (byte == C_0)
                {
                    state = BCC;
                }
                else
                    state = FLAG_I;
                break;

            case BCC:
                if (byte == (A_T ^ C_0))
                {
                    state = DATA;
                    idx = 0;
                    bcc2_calc = 0;
                }
                else
                    state = FLAG_I;
                break;

            case DATA:
                if (byte == FLAG)
                {
                    if (idx > 0)
                    {
                        unsigned char bcc2_received = packet[idx - 1];
                        unsigned char bcc2_check = 0;
                        
                        for (int i = 0; i < idx - 1; i++)
                            bcc2_check ^= packet[i];
                        
                        if (bcc2_received == bcc2_check)
                        {
                            idx--;
                            STOP = TRUE;
                            
                            unsigned char rr[5] = {FLAG, A_R, 0xAA, A_R ^ 0xAA, FLAG};
                            writeBytesSerialPort(rr, 5);
                            printf("Rx: Sent RR\n");
                        }
                        else
                        {
                            unsigned char rej[5] = {FLAG, A_R, 0x54, A_R ^ 0x54, FLAG};
                            writeBytesSerialPort(rej, 5);
                            printf("Rx: BCC2 error - Sent REJ\n");
                            state = FLAG_I;
                        }
                    }
                    else
                    {
                        state = FLAG_I;
                    }
                }
                else
                {
                    if (byte == 0x7D)
                    {
                        while (readByteSerialPort(&byte) != 1);
                        
                        if (byte == 0x5E)
                            byte = FLAG;
                        else if (byte == 0x5D)
                            byte = 0x7D;
                    }
                    
                    packet[idx++] = byte;
                    
                    if (idx >= 4096)
                        return -1;
                }
                break;

            case FLAG_F:
                if (byte == FLAG)
                {
                    STOP = TRUE;
                }
                else
                    state = FLAG_I;
                break;
            }
    }

    return idx;
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
                    if (byte == FLAG)
                        state = A;
                    break;

                case A:
                    if (byte == A_R)
                        state = C;
                    else
                        state = FLAG_I;
                    break;

                case C:
                    if (byte == DISC)
                        state = BCC;
                    else if (byte != FLAG)
                        state = FLAG_I;
                    break;

                case BCC:
                    if (byte == (A_R ^ DISC))
                        state = FLAG_F;
                    else
                        state = FLAG_I;
                    break;

                case FLAG_F:
                    if (byte == FLAG)
                    {
                        printf("Tx: Received DISC from Rx\n");
                        STOP = TRUE;
                        alarm(0);
                    }
                    else
                        state = FLAG_I;
                    break;
                }
            }
        }

        if (!STOP)
            return -1;

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
            if (readByteSerialPort(&byte) != 1)
                continue;

            switch (state)
            {
            case FLAG_I:
                if (byte == FLAG)
                    state = A;
                break;

            case A:
                if (byte == A_T)
                    state = C;
                else
                    state = FLAG_I;
                break;

            case C:
                if (byte == DISC)
                    state = BCC;
                else
                    state = FLAG_I;
                break;

            case BCC:
                if (byte == (A_T ^ DISC))
                    state = FLAG_F;
                else
                    state = FLAG_I;
                break;

            case FLAG_F:
                if (byte == FLAG)
                {
                    printf("Rx: Received DISC from Tx\n");
                    STOP = TRUE;
                }
                else
                    state = FLAG_I;
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
            if (readByteSerialPort(&byte) != 1)
                continue;

            switch (state)
            {
            case FLAG_I:
                if (byte == FLAG)
                    state = A;
                break;

            case A:
                if (byte == A_T)
                    state = C;
                else
                    state = FLAG_I;
                break;

            case C:
                if (byte == UA)
                    state = BCC;
                else
                    state = FLAG_I;
                break;

            case BCC:
                if (byte == (A_T ^ UA))
                    state = FLAG_F;
                else
                    state = FLAG_I;
                break;

            case FLAG_F:
                if (byte == FLAG)
                {
                    printf("Rx: Received UA, closing link\n");
                    STOP = TRUE;
                }
                else
                    state = FLAG_I;
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
