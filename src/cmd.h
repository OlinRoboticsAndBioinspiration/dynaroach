#ifndef __CMD_H
#define __CMD_H

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
unsigned int cmdHandleRadioRxBuffer(void);
void send(unsigned char status, unsigned char length, unsigned char *frame, unsigned char type,unsigned int dest);
unsigned int get_src_addr(void);
unsigned int get_basestation_addr(void);
unsigned int get_pan_id(void);
unsigned int get_channel(void);
extern int pid_input;

#endif // __CMD_H
