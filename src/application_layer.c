// Application layer protocol implementation

#include "application_layer.h"

static struct timespec t_start, t_end;
static unsigned int bytes_app_rx = 0;

static double timeTransfer(struct timespec a, struct timespec b){
  return (b.tv_sec - a.tv_sec) + (b.tv_nsec - a.tv_nsec)/1e9;
}

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{

    LinkLayerRole lRole;
    if (strcmp(role, "tx") == 0)
        lRole = LlTx;
    else if (strcmp(role, "rx") == 0)
        lRole = LlRx;

    LinkLayer serialport = CreateLinkLayer(serialPort, role, baudRate, nTries, timeout, lRole);

    // Open Serial Port in Case of error it closes
    if (llopen(serialport) < 0)
    {
        printf("Application failed to open Serial Port \n");
        llend();
        return;
    }

    clock_gettime(CLOCK_MONOTONIC, &t_start);
    bytes_app_rx = 0;

    if (lRole == LlTx)
    {

        // Open File to Send
        FILE *file = fopen(filename, "rb");
        if (file == NULL)
        {
            printf("The file is not opened.");
        }

        // Send ControlPacket with Start value to signal to the receiver that we are gonna start sending Data
        if (ControlPacket(filename, START_CONTROL, file, serialport) < 0)
        {
            printf("Application failed to send START Packet \n");
            fclose(file);
            llclose(serialport);
            return;
        }

        // Send Content of the File
        if (DataPacket(file, serialport) < 0)
        {
            printf("Application failed to send DATA Packet \n");
            fclose(file);
            llclose(serialport);
            return;
        }

        // Send ControlPacket with End value to signal to the receiver that there is no more Data left to send
        if (ControlPacket(filename, END_CONTROL, file, serialport) < 0)
        {
            printf("Application failed to send START Packet \n");
            fclose(file);
            llclose(serialport);
            return;
        }

        fclose(file);
        printf("Application Sent Data succefully \n");
    }
    else
    {
        unsigned char packet[BUFSIZE + 3];

        enum receiver r = START_TRANSACTION;
        FILE *file;

        // State Machine to receive the File
        while (r != END_TRANSACTION)
        {
            llread(packet);
            switch (r)
            {
            case START_TRANSACTION:
                if (packet[0] == START_CONTROL)
                {
                    // Get the File Size
                    int offset = 1;
                    unsigned char type = packet[offset++];
                    unsigned char length = packet[offset++];
                    int file_size = 0;
                    memcpy(&file_size, packet + offset, length);
                    offset += length;

                    // Get the Filename
                    unsigned char type2 = packet[offset++];
                    unsigned char length2 = packet[offset++];
                    char filename[256];
                    memcpy(filename, packet + offset, length2);
                    filename[length2] = '\0';

                    // Create the Filename with "-received"
                    char new_filename[300];
                    strcpy(new_filename, filename);
                    char *dot = strrchr(new_filename, '.');

                    if (dot)
                    {
                        char temp[300];
                        strcpy(temp, dot);
                        *dot = '\0';
                        strcat(new_filename, "-received");
                        strcat(new_filename, temp);
                    }
                    else
                    {
                        strcat(new_filename, "-received");
                    }

                    // Open the new File with write mode
                    file = fopen(new_filename, "wb");
                    if (file == NULL)
                    {
                        printf("Cannot open file for writing.\n");
                    }

                    r = DATA_TRANSACTION;
                }
                break;
            case DATA_TRANSACTION:
                // while the first byte of the packet is equal to 2 we are receiving Data packets else it means that we received a Control packet and we should end the loop
                if (packet[0] == DATA_CONTROL)
                {
                    int data_length = (packet[1] << 8) | packet[2];
                    size_t wrote = fwrite(packet + 3, 1, data_length, file);
                    bytes_app_rx += wrote;
                }
                else if (packet[0] == END_CONTROL)
                {
                    r = END_TRANSACTION;
                    fclose(file);
                }
                break;
            default:
                break;
            }
        }
    }

    if (llclose(serialport) < 0)
    {
        printf("Serial Port closed with errors \n");
    }
    else
    {   
        printf("Serial Port closed successfully\n");
    }

    clock_gettime(CLOCK_MONOTONIC, &t_end);

    if(lRole == LlRx){
        double dt = timeTransfer(t_start, t_end);
        double R = dt > 0 ? (8.0*bytes_app_rx)/ dt : 0.0;
        double C = (double) baudRate;
        double S = R/C;
        printf("Bytes = %u, Time: %.3f, R = %.3f, C = %.3f, S = %.3f\n", bytes_app_rx, dt, R, C, S);
    }


}

// Auxiliar fuction to Create the Control Packets and send them
int ControlPacket(const char *filename, int cValue, FILE *file, LinkLayer serialport)
{
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
    memcpy(control_packet + offset, &file_size, sizeof(int));
    offset += sizeof(int);
    control_packet[offset++] = 0x01;
    control_packet[offset++] = strlen(filename);
    memcpy(control_packet + offset, filename, filename_len);
    offset += strlen(filename);

    if (llwrite(control_packet, offset, serialport) < 0)
    {
        printf("Application Transmission Failed after all Retransmissions\n");
        return -1;
    }

    return 0;
}

// Auxiliar Fuction to create the Data Packets and send them
int DataPacket(FILE *file, LinkLayer serialport)
{
    unsigned char data_packet[3 + BUFSIZE];
    size_t bytesRead;

    while ((bytesRead = fread(data_packet + 3, 1, BUFSIZE, file)) > 0)
    {
        data_packet[0] = DATA_CONTROL;
        data_packet[1] = (bytesRead >> 8) & 0xFF;
        data_packet[2] = bytesRead & 0xFF;

        if (llwrite(data_packet, bytesRead + 3, serialport) < 0)
        {
            printf("Application Transmission Failed after all Retransmissions\n");
            return -1;
        }
    }
    return 0;
}

// Create the struct of type LinkLayer
LinkLayer CreateLinkLayer(const char *serialPort, const char *role, int baudRate, int nTries, int timeout, LinkLayerRole lRole)
{

    LinkLayer serialport;
    strcpy(serialport.serialPort, serialPort);
    serialport.role = lRole;
    serialport.baudRate = baudRate;
    serialport.nRetransmissions = nTries;
    serialport.timeout = timeout;

    return serialport;
}