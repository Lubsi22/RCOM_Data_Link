// Link layer header.
// DO NOT CHANGE THIS FILE

#ifndef _LINK_LAYER_H_
#define _LINK_LAYER_H_

typedef enum
{
    LlTx,
    LlRx,
} LinkLayerRole;

typedef struct
{
    char serialPort[50];
    LinkLayerRole role;
    int baudRate;
    int nRetransmissions;
    int timeout;
} LinkLayer;

// Size of maximum acceptable payload.
// Maximum number of bytes that application layer should send to link layer.
#define MAX_PAYLOAD_SIZE 1000

// MISC
#define FALSE 0
#define TRUE 1

#define FLAG 0x7E

#define A_T 0x03
#define A_R 0x01

#define SET 0x03
#define UA 0x07
#define DISC 0x0B
#define RR0 0xAA
#define RR1 0xAB
#define REJ0 0x54
#define REJ1 0x55

#define C_0 0x00
#define C_1 0x80

enum State
{
    FLAG_I,
    A,
    C,
    BCC,
    DATA,
    FLAG_F
};
// Open a connection using the "port" parameters defined in struct linkLayer.
// Return 0 on success or -1 on error.
int llopen(LinkLayer connectionParameters);

// Send data in buf with size bufSize.
// Return number of chars written, or -1 on error.
int llwrite(const unsigned char *buf, int bufSize, LinkLayer connectionParameters);

// Receive data in packet.
// Return number of chars read, or -1 on error.
int llread(unsigned char *packet);

// Close previously opened connection and print transmission statistics in the console.
// Return 0 on success or -1 on error.
int llclose(LinkLayer connectionParameters);

int llend();

#endif // _LINK_LAYER_H_
