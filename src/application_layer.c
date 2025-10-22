// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"

#include "string.h"
#include <stdio.h>
#include <unistd.h>

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

        FILE* file = fopen(filename, "r");
        if(file == NULL){
            printf("The file is not opened.");
        }   

        fseek(file, 0, SEEK_END);
        int file_size = ftell(file);
        rewind(file);
        int filename_len = strlen(filename);
        int start_packet_size = 1 + 1 + 1 + sizeof(int) + 1 + 1 + filename_len; // Control + File size TLV + Filename TLV

        unsigned char start_packet[start_packet_size];
        int offset = 0;
        start_packet[offset++] = START;
        start_packet[offset++] = 0x00;
        start_packet[offset++] = sizeof(int);
        memcpy(start_packet+offset, &file_size, sizeof(int));
        offset += sizeof(int);
        start_packet[offset++] = 0x01; 
        start_packet[offset++] = strlen(filename);
        memcpy(start_packet+offset, filename, filename_len);
        offset += strlen(filename);

        llwrite(start_packet, offset, serialport);

        unsigned char data_packet[1 + 2 + BUFSIZE];
        size_t bytesRead;

        while((bytesRead = fread(data_packet + 3, 1, BUFSIZE, file))>0){
            data_packet[0] = DATA;
            data_packet[1] = (bytesRead >> 8) & 0xFF;
            data_packet[2] = bytesRead & 0xFF;

            llwrite(data_packet, bytesRead + 3, serialport);
        }

        unsigned char end_packet[start_packet_size];
        int offset = 0;
        end_packet[offset++] = END;
        end_packet[offset++] = 0x00;
        end_packet[offset++] = sizeof(int);
        memcpy(end_packet+offset, &file_size, sizeof(int));
        offset += sizeof(int);
        end_packet[offset++] = 0x01; 
        end_packet[offset++] = strlen(filename);
        memcpy(end_packet+offset, filename, filename_len);
        offset += strlen(filename);

        llwrite(end_packet, offset, serialport);

        fclose(file);
        
    }
    else{
        unsigned char packet[256];
        llread(packet);
        printf("Message: %s\n", packet);
    }
    

    llclose(serialport);
    
    }
