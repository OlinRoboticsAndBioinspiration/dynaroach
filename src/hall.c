#include "utils.h" 
#include "i2c.h"
#include "hall.h"
#include <string.h>

#define HALL_ADDR_RD             0x81    
#define HALL_ADDR_WR             0x80    // 0x40 <<1
#define HALL_ANG_WIDTH           2
#define HALL_MAG_WIDTH           2
#define HALL_SETUP_DELAY         100     //(Us)
#define HALL_DATA_WAIT           500
#define REG_MAG_H                (0xFC)
#define REG_MAG_L                (0xFD)
#define REG_ANGLE_H              (0xFE)
#define REG_ANGLE_L              (0xFF)
#define hallReadString(length,data,delay) MastergetsI2C2(length, data, delay)

static unsigned char hallAng[2];
static unsigned char hallMag[2];

static union halldata { 
        unsigned char chr_data[HALL_ANG_WIDTH+HALL_MAG_WIDTH];
        } Halldata;

static void hallWrite(unsigned char subaddr, unsigned char data);
static void hallSendByte(unsigned char byte);
static void hallStartTx(void);
static void hallEndTx(void);
static void hallSetupPeripheral(void);

/*Working on it
void hallSetupZeroPos(void) {

    hallSetupPeripheral();

    Angle= hallreadAngle();

    hallWrite(0x03, 0x01);
    delay_us(HALL_SETUP_DELAY)
    hallWrite(0x16, Angle);
    delay_us(HALL_SETUP_DELAY)

    hallReadString
    hallReadString(14,hallAngleData,);
}*/

unsigned char* hallReadAngle(void) {

    hallStartTx();
    hallSendByte(HALL_ADDR_WR);
    hallSendByte(0xFE);
    //hallSendByte(0xFF);
    hallEndTx();
    hallStartTx();
    hallSendByte(HALL_ADDR_RD);
    hallReadString(2, hallAng, HALL_DATA_WAIT); //Last Ack Byte
    hallEndTx(); 
    return hallAng; 
}

unsigned char* hallReadMag(void) {

    hallStartTx();
    hallSendByte(HALL_ADDR_WR);
    hallSendByte(0xFC);
    hallEndTx();
    hallStartTx();
    hallSendByte(HALL_ADDR_RD);
    hallReadString(2, hallMag, HALL_DATA_WAIT);
    hallEndTx();   
    return hallMag; 
}
unsigned char * hallReadAngleMag(void) {
    unsigned char hall_data[4];

    hallStartTx();
    hallSendByte(HALL_ADDR_WR);
    hallSendByte(0xFC);
    hallEndTx();
    hallStartTx();
    hallSendByte(HALL_ADDR_RD);
    hallReadString(4,hall_data, HALL_DATA_WAIT);
    hallEndTx();   

    Halldata.chr_data[0] = hall_data[0];
    Halldata.chr_data[1] = hall_data[1]<<2;
    Halldata.chr_data[2] = hall_data[2];
    Halldata.chr_data[3] = hall_data[3]<<2;
    return Halldata.chr_data; 
}

void hallDumpData(unsigned char* buffer) {

    memcpy(buffer, Halldata.chr_data + 2, 4);
}



static void hallWrite( unsigned char subaddr, unsigned char data){
    hallStartTx();
    hallSendByte(HALL_ADDR_WR);
    hallSendByte(subaddr);
    hallSendByte(data);
    hallEndTx();
}

static void hallSendByte( unsigned char byte ){
    MasterWriteI2C2(byte);
    while(I2C2STATbits.TRSTAT);
    while(I2C2STATbits.ACKSTAT);
}

static void hallStartTx(void){
    StartI2C2();
    while(I2C2CONbits.SEN);
}
static void hallEndTx(void){
    StopI2C2();
    while(I2C2CONbits.PEN);
}
static void hallSetupPeripheral(void) {
    unsigned int I2C2CONvalue, I2C2BRGvalue;
    I2C2CONvalue = I2C2_ON & I2C2_IDLE_CON & I2C2_CLK_HLD &
                   I2C2_IPMI_DIS & I2C2_7BIT_ADD & I2C2_SLW_DIS &
                   I2C2_SM_DIS & I2C2_GCALL_DIS & I2C2_STR_DIS &
                   I2C2_NACK & I2C2_ACK_DIS & I2C2_RCV_DIS &
                   I2C2_STOP_DIS & I2C2_RESTART_DIS & I2C2_START_DIS;
    // BRG = Fcy(1/Fscl - 1/10000000)-1, Fscl = 400KHz 	
    I2C2BRGvalue = 95; 
    OpenI2C2(I2C2CONvalue, I2C2BRGvalue);
    IdleI2C2();
}
