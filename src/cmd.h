#ifndef __CMD_H
#define __CMD_H

#define NETWORK_BASESTATION_ADDR  0x0100
#define NETWORK_BASESTATION_PAN_ID  0x0000
#define NETWORK_BASESTATION_CHANNEL 0x14

typedef union {
    float fval;
    unsigned long lval;
    short sval[2];
    unsigned char cval[4];
} uByte4;

typedef union {
    unsigned short sval;
    unsigned char cval[2];
} uByte2;

extern unsigned volatile int ADCBuffer[1] __attribute__((space(dma)));

void cmdSetup(void);
void cmdHandleRadioRxBuffer(void);
void send(unsigned char status, unsigned char length, unsigned char *frame, unsigned char type);

#endif // __CMD_H
