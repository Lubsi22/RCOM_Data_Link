// Application layer protocol implementation

#include "application_layer.h"

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

    LinkLayer serialport = CreateLinkLayer(serialPort, role, baudRate, nTries, timeout, lRole);


    llopen(serialport);
    
    if(lRole == LlTx){

        FILE* file = fopen(filename, "rb");
        if(file == NULL){
            printf("The file is not opened.");
        }   

        ControlPacket(filename, START_CONTROL, file, serialport);

        DataPacket(file, serialport);

        ControlPacket(filename, END_CONTROL, file, serialport);

        fclose(file);
        
    }
    else{
        unsigned char packet[BUFSIZE + 3];

        enum receiver r = START_TRANSACTION;
        FILE *file;

        while( r != END_TRANSACTION){
            llread(packet);
            switch (r)
            {
            case START_TRANSACTION:
                if(packet[0] == START_CONTROL){
                    int offset = 1;
                    unsigned char type = packet[offset++];
                    unsigned char length = packet[offset++];
                    int file_size = 0;
                    memcpy(&file_size, packet + offset, length);
                    offset += length;

                    unsigned char type2 = packet[offset++];
                    unsigned char length2 = packet[offset++];
                    char filename[256];
                    memcpy(filename, packet + offset, length2);
                    filename[length2] = '\0';
                    
                    char new_filename[300];
                    strcpy(new_filename, filename);
                    char *dot = strrchr(new_filename, '.');

                     if (dot) {
                        char temp[300];
                        strcpy(temp, dot);
                        *dot = '\0';
                        strcat(new_filename, "-received");
                        strcat(new_filename, temp);
                     }else{
                        strcat(new_filename, "-received");
                     }

                    file = fopen(new_filename, "wb");
                    if(file == NULL){
                        printf("Cannot open file for writing.\n");
                    }

                    r = DATA_TRANSACTION;
                }
                break;
            case DATA_TRANSACTION:
                if(packet[0] == DATA_CONTROL){
                    int data_length = (packet[1] << 8) | packet[2];
                    fwrite(packet+3, 1, data_length, file);
                }else if (packet[0] == END_CONTROL){
                    r = END_TRANSACTION;
                    fclose(file);
                }
                break;
            default:
                break;
            }
        }
    }
    

    llclose(serialport);
    
    }


void ControlPacket(const char *filename, int cValue, FILE* file, LinkLayer serialport){
    fseek(file, 0, SEEK_END);
    int file_size = ftell(file);
        
    rewind(file);
    int filename_len = strlen(filename);
    int control_packet_size = 1 + 1 + 1 + sizeof(int) + 1 + 1 + filename_len; // Control + File size TLV + Filename TLV

    unsigned char control_packet[control_packet_size];
    int offset = 0;
    control_packet[offset++] = cValue;
    control_packet[offset++] = 0x00;
    control_packet[offset++] = sizeof(int);
    memcpy(control_packet+offset, &file_size, sizeof(int));
    offset += sizeof(int);
    control_packet[offset++] = 0x01; 
    control_packet[offset++] = strlen(filename);
    memcpy(control_packet+offset, filename, filename_len);
    offset += strlen(filename);

    llwrite(control_packet, offset, serialport);

}

void DataPacket(FILE* file, LinkLayer serialport){
    unsigned char data_packet[3 + BUFSIZE];
        size_t bytesRead;

        while((bytesRead = fread(data_packet + 3, 1, BUFSIZE, file))>0){
            data_packet[0] = DATA_CONTROL;
            data_packet[1] = (bytesRead >> 8) & 0xFF;
            data_packet[2] = bytesRead & 0xFF;

            llwrite(data_packet, bytesRead + 3, serialport);
        }
}

LinkLayer CreateLinkLayer(const char *serialPort, const char *role, int baudRate, int nTries, int timeout, LinkLayerRole lRole){

    LinkLayer serialport;
    strcpy(serialport.serialPort, serialPort);
    serialport.role = lRole;
    serialport.baudRate = baudRate;
    serialport.nRetransmissions = nTries;
    serialport.timeout = timeout;

    return serialport;
}