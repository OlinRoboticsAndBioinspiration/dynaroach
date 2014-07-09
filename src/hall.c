#include "utils.h"
#include "i2c.h"
#include "hall.h"

#define HALL_ADDR_RD             0x43    
#define HALL_ADDR_WR             0x42    // 0x15 << 1 
#define HALL_ANGLE_WIDTH         16
#define HALL_MAG_WIDTH           16
#define HALL_SETUP_DELAY         100     //(Us)

#define hallReadString(a,b,c) MastergetsI2C2(a,b,c)

static unsigned char hallAngleData[HALL_ANGLE_WIDTH+1];
static unsigned char hallAngleData[HALL_ANGLE_WIDTH+1];

static void hallWrite(unsigned char subaddr, unsigned char data);
static void hallSendByte(unsigned char byte );
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

unsigned char* hallReadAngle_high_8bits(void) {
    hallStartTx();
    hallSendByte(HALL_ADDR_WR);
    hallSendByte(0xFE);
    wiiEndTx();
    hallStartTx();
    hallSendByte(HALL_ADDR_RD);
    hallReadString(15, hallAngleData, HALL_DATA_WAIT);
    hallEndTx();   
    return hallAngle_high_8bits + 1; 
}

unsigned char* hallReadMagData(void) {
    hallStartTx();
    hallSendByte(HALL_ADDR_WR);
    hallSendByte(0xFC);
    wiiEndTx();
    hallStartTx();
    hallSendByte(HALL_ADDR_RD);
    hallReadString(15, hallMagData, HALL_DATA_WAIT);
    hallEndTx();   
    return hallAngleData + 1; 
}

static void hallWrite( unsigned char subaddr, unsigned char data ){
    wiiStartTx();
    wiiSendByte(WII_ADDR_WR);
    wiiSendByte(subaddr);
    wiiSendByte(data);
    wiiEndTx();
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
static void hallEndTx (void){
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
