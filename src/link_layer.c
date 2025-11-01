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

static int expectedNs = 0;

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
                    else if (byte == FLAG)
                        state = A;
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
        while (STOP == FALSE && alarmCount < connectionParameters.nRetransmissions)
        {

            if (alarmEnabled == FALSE)
            {
                alarm(connectionParameters.timeout);
                alarmEnabled = TRUE;
            }

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
                else if (byte == FLAG)
                    state = A;
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
                {
                    alarm(0);
                    STOP = TRUE;
                }
                else
                    state = FLAG_I;
                break;

            default:
                break;
            }
        }

        if (STOP != FALSE)
        {
            unsigned char ua[5] = {FLAG, A_R, UA, A_R ^ UA, FLAG};
            writeBytesSerialPort(ua, 5);
            printf("Rx: Sent UA\n");

            expectedNs = 0;

            return 0;
        }
    }

    return -1;
}

////////////////////////////////////////////////
// LLWRITE
// Function in Link layer used to receive the data to be transmited and create the frame with stuffing if needed
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize, LinkLayer connectionParameters)
{

    static int Number = 0;       // first interaction the Ns will be 0
    int size = 2 * bufSize + 7;  // Worst case scenario where all data is Flag and need to be stuffed
    unsigned char stuffed[size]; // the buffer used to hold the I-Frame

    // Variables used to do the bcc2 and to update the position we are writing to the buffer
    unsigned char bcc2 = 0x00;
    int index = 0;

    stuffed[index++] = FLAG;
    stuffed[index++] = A_T;

    // Selection of Control paramanter to send based on the Ns
    unsigned char control = (Number == 0 ? C_0 : C_1);
    stuffed[index++] = control;
    stuffed[index++] = (A_T ^ control);

    // Bcc2 before stuffing
    for (int i = 0; i < bufSize; i++)
    {
        bcc2 ^= buf[i];
    }

    // Stuffing if needed, if not needed add default value
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

    // Stuffing bcc2 if needed
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

    // End Flag of the I-Frame
    stuffed[index++] = FLAG;

    // Variable used to count bytes written in every transaction
    int bytes = 0;

    // Enum used for the State Machine
    enum State state = FLAG_I;
    int STOP = FALSE;

    // Variables reset of the alarm
    alarmCount = 0;
    alarmEnabled = FALSE;
    int rejRetries = 0;

    while (STOP == FALSE && (alarmCount + rejRetries) < connectionParameters.nRetransmissions)
    {

        // Alarm to send the first time the buffer or in case of a timeout or REJ to send agian the Data
        if (alarmEnabled == FALSE)
        {
            bytes = 0;
            bytes += writeBytesSerialPort(stuffed, index);
            printf("Tx: Sent I-frame\n");

            alarm(connectionParameters.timeout);
            alarmEnabled = TRUE;
        }

        // Read the serialPort
        unsigned char byte;
        if (readByteSerialPort(&byte) != 1)
            continue;

        // State machine of Supervision Frames
        //  If the Supervision Frame is correct it stops the alarm and return the number of Bytes written to the SerialPort
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
            else if (byte == FLAG)
                state = A;
            else
                state = FLAG_I;

            break;

        case C:
            if ((byte == RR0 && Number == 1) || (byte == RR1 && Number == 0))
                state = BCC;
            else if (byte == REJ0 || byte == REJ1)
            {
                printf("Tx: Resending Frame\n");
                alarm(0);
                rejRetries++;
                alarmEnabled = FALSE;
                state = FLAG_I;
            }
            else
                state = FLAG_I;

            break;

        case BCC:
            if (byte == (A_R ^ (Number == 0 ? RR1 : RR0)))
                state = FLAG_F;
            else
                state = FLAG_I;
            break;

        case FLAG_F:
            if (byte == FLAG)
            {
                Number ^= 1;
                printf("Tx: Received RR - message sent successfully\n");
                STOP = TRUE;
                alarm(0);
                printf("Tx: Bytes Written %d\n", bytes);
                return bytes;
            }
            else
                state = FLAG_I;
            break;

        default:
            break;
        }
    }

    return -1;
}

////////////////////////////////////////////////
// LLREAD
// Function in Link layer used to receive the I-Frame and to check it's correctness
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    enum State state = FLAG_I;
    int idx = 0;
    int STOP = FALSE;

    // Variable to Hold RR0 or RR1 depending on the Ns
    unsigned char frameNumber;

    // Variable to Hold REJ0 or REJ1 depending on the Ns
    unsigned char rejNumber;

    // Variables used for the check of duplicate frames
    int duplicate = 0;
    int ns = 0;

    unsigned char byteC;

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
            else if (byte == FLAG)
            {
                state = A;
            }
            else
                state = FLAG_I;
            break;

        case C:
            if (byte == C_0 || byte == C_1)
            {
                // From the control Byte we are to Take the correct ns, the RR to be used or the REJ in case of errors
                byteC = byte;
                ns = (byteC == C_1) ? 1 : 0;
                frameNumber = (byteC == C_0) ? RR1 : RR0;
                rejNumber = (byteC == C_0) ? REJ0 : REJ1;
                state = BCC;
            }
            else
                state = FLAG_I;
            break;

        case BCC:
            if (byte == (A_T ^ byteC))
            {
                state = DATA;
                idx = 0;
            }
            else
                state = FLAG_I;
            break;

        case DATA:
            // If we reached the last Byte(Flag) we are gona check if the data is Duplicate or check if bcc2 is Correct or Incorrect
            if (byte == FLAG)
            {
                duplicate = (ns != expectedNs);

                // if it is duplicate we send rr and restart the state machine and wait until a new frame arrives
                if (duplicate)
                {
                    unsigned char rr[5] = {FLAG, A_R, frameNumber, A_R ^ frameNumber, FLAG};
                    writeBytesSerialPort(rr, 5);
                    printf("Rx: Sent RR (Duplicate Frame)\n");

                    state = FLAG_I;
                    idx = 0;
                    continue;
                }
                else
                {
                    if (idx > 0)
                    {
                        // This is gonna be the Bcc2 because the Flag is the las byte
                        unsigned char bcc2_received = packet[idx - 1];
                        unsigned char bcc2_check = 0;

                        // Compute the Bcc
                        for (int i = 0; i < idx - 1; i++)
                        {
                            bcc2_check ^= packet[i];
                        }

                        // Check the bcc2 Received with the one computed if they are equal means we received everything right and we send RR else we send REJ
                        if (bcc2_received == bcc2_check)
                        {
                            idx--;

                            unsigned char rr[5] = {FLAG, A_R, frameNumber, A_R ^ frameNumber, FLAG};
                            writeBytesSerialPort(rr, 5);
                            printf("Rx: Sent RR\n");
                            expectedNs ^= 1;
                            return idx;
                        }
                        else
                        {
                            unsigned char rej[5] = {FLAG, A_R, rejNumber, A_R ^ rejNumber, FLAG};
                            writeBytesSerialPort(rej, 5);
                            printf("Rx: BCC2 error - Sent REJ\n");
                            state = FLAG_I;
                            idx = 0;
                        }
                    }
                    else
                    {
                        state = FLAG_I;
                    }
                }
            }
            else
            {
                // Destuffing the Data and putting it in the buffer packet
                if (byte == 0x7D)
                {
                    while (readByteSerialPort(&byte) != 1);

                    if (byte == 0x5E)
                        byte = FLAG;
                    else if (byte == 0x5D)
                        byte = 0x7D;
                }

                packet[idx++] = byte;
            }
            break;
        }
    }

    return -1;
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

        // Send UA
        unsigned char ua[5] = {FLAG, A_R, UA, A_R ^ UA, FLAG};
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

    return 0;
}

////////////////////////////////////////////////
// LLEND
// Fuction used if llopen failed to close connection without sending Disc
////////////////////////////////////////////////
int llend()
{
    if (closeSerialPort() < 0)
    {
        perror("closeSerialPort");
        exit(-1);
    }

    return 0;
}