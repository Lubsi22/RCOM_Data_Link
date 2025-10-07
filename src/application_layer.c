// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"

#include "string.h"
#include <stdio.h>

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{

    LinkLayerRole lRole;
    if (strcmp(role, "tx") == 0)
        lRole = LlTx;
    else if (strcmp(role, "rx") == 0)
        lRole = LlRx;
    

    LinkLayer serialport;
    strcpy(serialport.serialPort, serialPort);
    serialport.role = lRole;
    serialport.baudRate = baudRate;
    serialport.nRetransmissions = nTries;
    serialport.timeout = timeout;


    llopen(serialport);
    
    if(lRole == LlTx){
        llwrite("Hello World", 12);
    }
    else{
        unsigned char *packet;
        llread(packet);
    }

    llclose();
    
    }
