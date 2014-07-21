/*********************************************************************************************************
* Name: main.c
* Desc: This is a program for controlling the dynaRoACH robot. Motor commands
* for the primary power motor as well as the SMA leg actuators can be issued
* from a laptop and handled here. Sensory data is written to flash memory for
* dumping over the wireless link after a run.
*
* The architecture is based on a function pointer queue scheduling model. The
* meat of the control logic resides in test.c. If the radio has received a
* command packet during the previous timer interval for Timer2, the appropriate
* function pointer is added to a queue in the interrupt service routine for
* Timer2 (interrupts.c). The main loop simply pops the function pointer off
* the top of the queue and executes it.
*
* Date: 2011-04-13
* Author: AMH
*********************************************************************************************************/
#include "p33Fxxxx.h"
#include "init_default.h"
#include "timer.h"
#include "adc.h"
#include "i2c.h"
#include "consts.h"
#include "utils.h"
#include "radio.h"
#include "gyro.h"
#include "xl.h"
#include "dfmem.h"
#include "cmd.h"
#include "motor_ctrl.h"
#include "attitude_q.h"
#include "sma.h"
#include "sclock.h"
#include "ppool.h"
#include "spi_controller.h"
#include "wii.h"
#include "pid.h"
#include "pid_hw.h"
#include <stdio.h>
#include "ams-enc.h"

//these need tuning!
#define PID_KP			.5
#define PID_KI          .5
#define PID_KD			.5

#define PID_KFF			0 //system dependent feedforward term. 
#define PID_KAW			.5 //antiwindup term
#define PID_DELAY		2//milliseconds

static unsigned char *pos_data;
static int current_pos;
static int time_counter;
static int last_pos;
static int delta;
static int freq;
static int i;
static int sample;
static float revs;

void initDma0(void)
{
	DMA0CONbits.AMODE = 0;                      //Configure DMA for register indirect with post increment
	DMA0CONbits.MODE = 0;                       //Configure DMA for continuous mode (not Ping Pong)
	DMA0PAD = (int)&ADC1BUF0;                   //Peripheral address register: the ADC1 buffer
	DMA0CNT = 0;                                //Transfer after ever 2 samples
	DMA0REQ = 13;                               //Select ADC1 as DMA request source
	DMA0STA = __builtin_dmaoffset(ADCBuffer);   //DMA RAM start address

	IFS0bits.DMA0IF = 0;                        //Clear DMA interrupt flag bit
	IEC0bits.DMA0IE = 1;                        //Enable DMA interrupt
	DMA0CONbits.CHEN = 1;                       //Enable DMA channel
}

static void timer1Setup(void);

static void timer1Setup(void)
{
	unsigned int conf_reg;
	unsigned long period;
	conf_reg = T1_ON & T1_SOURCE_INT & T1_PS_1_256 & T1_GATE_OFF & T1_SYNC_EXT_OFF;
	period = (unsigned int)0x271; //timer period 4ms = period/FCY * prescaler
	OpenTimer1(conf_reg, period);
	ConfigIntTimer1(T1_INT_PRIOR_4 & T1_INT_OFF);
}

static void timer2Setup(void)
{
	unsigned int conf_reg, period;

	conf_reg = T2_ON & T2_SOURCE_INT & T2_PS_1_256 & T2_GATE_OFF;
	//Period in us is 1/40*period.
	//period = (unsigned int)0x9c40; //timer period 1ms = period/FCY.
	//period = (unsigned int)0x9c40; //timer period 1ms = period/FCY.
	period = (unsigned int)0x138; //timer period 2ms = period/FCY * prescaler.
	OpenTimer2(conf_reg, period);
	ConfigIntTimer2(T2_INT_PRIOR_4 & T2_INT_OFF);
	_T2IE = 1;
}

static void timer6Setup(void)
{
	T6CONbits.TON = 0; // Disable Timer
	T6CONbits.TCS = 0; // Select internal instruction cycle clock 
	T6CONbits.TGATE = 0; // Disable Gated Timer mode
	T6CONbits.TCKPS = 0b11; // Select 1:256 Prescaler
	TMR1 = 0x00; // Clear timer register
	PR1 = 156000; // Load the period value (1125ms)
	IPC11bits.T6IP = 0x04; //priority
	IFS2bits.T6IF = 0; //Flag =0
	IEC2bits.T6IE = 1; //Enable interrupt
	T6CONbits.TON = 1; //Turn the timer on
}

static void timer5Setup(void)
{
	T5CONbits.TON = 0; // Disable Timer
	T5CONbits.TCS = 0; // Select internal instruction cycle clock 
	T5CONbits.TGATE = 0; // Disable Gated Timer mode
	T5CONbits.TCKPS = 0b10; // Select 1:256 Prescaler
	TMR1 = 0x00; // Clear timer register
	PR1 = 1250; // Load the period value (.002 s)
	IPC7bits.T5IP = 0x06; //priority
	IFS1bits.T5IF = 0; //Flag =0
	IEC1bits.T5IE = 1; //Enable interrupt
	T5CONbits.TON = 1; //Turn the timer on
}

static void timer7Setup(void)
{
	T7CONbits.TON = 0; // Disable Timer
	T7CONbits.TCS = 0; // Select internal instruction cycle clock 
	T7CONbits.TGATE = 0; // Disable Gated Timer mode
	T7CONbits.TCKPS = 0b10; // Select 1:64 Prescaler
	TMR1 = 0x00; // Clear timer register
	PR1 = 1250; // Load the period value (0.002s=500Hz)
	IPC12bits.T7IP = 0x04; //priority
	IFS3bits.T7IF = 0; //Flag =0
	IEC3bits.T7IE = 1; //Enable interrupt
	//T7CONbits.TON = 0; //Get called in Hall Function 
}

static void getFeedback(void)
{	//all of this is working with gear measurement, no conversion
    current_pos = (pos_data[0]<<6)+(pos_data[1]&0x3F);
    delta = abs(current_pos-last_pos);
    

    last_pos = current_pos;

    revs = delta/16384.0;

    freq = (revs/(PID_DELAY*500))*1000;//revs/s *1000;
}

void __attribute__((interrupt, no_auto_psv)) _T5Interrupt(void)//for Hall sampling
{	
	sample = 1;
    
    LED_1 = ~LED_1;
    _T5IF = 0;
}

int main ( void )
{

	unsigned int network_src_addr = get_src_addr();
	unsigned int network_basestation_channel = get_channel();
	unsigned int network_basestation_pan_id = get_pan_id();
	unsigned int network_basestation_addr = get_basestation_addr();

	pid_input = 0;//assume robot is stationary and should remain so until told to go//todo, make a speed controller. Something has to set this.
	int pid_feedback = 0;
	int dCycle;

	sample = 0;

	SetupClock();
	SwitchClocks();
	SetupPorts();

	spicSetupChannel1();
	spicSetupChannel2();
	ppoolInit();

	//BEGIN RADIO SETUP
	radioInit(50, 10); // tx_queue length: 50, rx_queue length: 10
	radioSetSrcAddr(network_src_addr);//defined by bootloader
	radioSetSrcPanID(network_basestation_pan_id);
	radioSetChannel(network_basestation_channel);
	//END RADIO SETUP

	//BEGIN I2C SETUP
	unsigned int I2C1CONvalue, I2C1BRGvalue;
	I2C1CONvalue = I2C1_ON & I2C1_IDLE_CON & I2C1_CLK_HLD &
				   I2C1_IPMI_DIS & I2C1_7BIT_ADD & I2C1_SLW_DIS &
				   I2C1_SM_DIS & I2C1_GCALL_DIS & I2C1_STR_DIS &
				   I2C1_NACK & I2C1_ACK_DIS & I2C1_RCV_DIS &
				   I2C1_STOP_DIS & I2C1_RESTART_DIS & I2C1_START_DIS;
	I2C1BRGvalue = 363; // Fcy(1/Fscl - 1/1111111)-1
	OpenI2C1(I2C1CONvalue, I2C1BRGvalue);
	IdleI2C1();
	//END I2C SETUP


	//BEGIN ADC SETUP
	AD1CON1bits.FORM = 0b00;    //integer (0000 00dd dddd dddd) format output
	AD1CON1bits.ADON = 0;       //disable
	AD1CON1bits.SSRC = 0b011;   //Sample clock source based on PWM
	AD1CON1bits.ASAM = 0;       //Auto sampling off
	AD1CON1bits.SIMSAM = 0;     //Do not sample channels simultaneously
	AD1CON1bits.ADSIDL = 0;     //continue in idle mode
	AD1CON1bits.AD12B = 0;      //10 bit mode

	AD1CON2bits.VCFG = 0b000;   //Vdd is pos. ref and Vss is neg. ref.
	AD1CON2bits.CSCNA = 0;      //Do not scan inputs
	AD1CON2bits.CHPS = 0b00;    //Convert channels 0 and 1
	AD1CON2bits.SMPI = 0b0000;  //Interrupt after 2 conversions (depends on CHPS and SIMSAM)

	AD1CON3bits.ADRC = 0;       //Derive conversion clock from system clock
	AD1CON3bits.ADCS = 0b00000010; // Each TAD is 3 Tcy

	AD1PCFGL = 0xFFF0;          //Enable AN0 - AN3 as analog inputs

	AD1CHS0bits.CH0SA = 0b00000;      //Select AN0 for CH0 +ve input
	AD1CHS0bits.CH0NA = 0b00000;      //Select Vref- for CH0 -ve input

	AD1CON1bits.ADON = 1;       //enable
	//END ADC SETUP

	mcSetup();
	gyroSetup();
	xlSetup();
	dfmemSetup();
	sclockSetup();
	timer1Setup();
	timer2Setup();
	timer5Setup();
	timer7Setup();

	cmdSetup();

	pidObj pctrl;

	pidObj *pctrl_ptr = &pctrl;

	pidInitPIDObj(pctrl_ptr, PID_KP, PID_KI, PID_KD, PID_KAW, PID_KFF);

	pidOnOff(pctrl_ptr,'1');

	attSetup(1.0/TIMER1_FCY);
	char j;

	for(j=0; j<6; j++){
		LED_1 = ~LED_1;
		delay_ms(100);
		LED_2 = ~LED_2;
		delay_ms(100);
		LED_3 = ~LED_3;
		delay_ms(100);
	}

	delay_ms(1000);
	LED_2 = 1;
	delay_ms(1000);

	encSetup();

	LED_2 = ~LED_2;
	//wiiSetupBasic();

	//set_zero();

	char frame[5];

	send(STATUS_UNUSED, 5, frame, '4', network_basestation_addr);
	send(STATUS_UNUSED, 5, frame, '4', network_basestation_addr);
	send(STATUS_UNUSED, 5, frame, '4', network_basestation_addr);
	//radioDeleteQueues();

	pidUpdate(pctrl_ptr,0);
	pidSetInput(pctrl_ptr,0);

	uByte2 out;
	char f[2];

	timer5Setup();
	while(1){
		cmdHandleRadioRxBuffer();

		if(sample == 1)
		{
			pos_data = encGetPos();
			getFeedback();
			out.sval = freq;
			f[0] = out.cval[0];
			f[1]= out.cval[1];
			sample = 0;
		}
	}
}

void set_zero()
{
    unsigned char *zero_pos = encGetPos();//get current position- this will be new zero

    //set programming enable
    i2cStartTx(2);
    i2cSendByte(2,0x81);
    i2cSendByte(2,0x03);
    i2cSendByte(2,0x01);
    i2cEndTx(2);

    //write zero to correct register
    i2cStartTx(2);
    i2cSendByte(2,0x81);
    i2cSendByte(2,0x16);
    i2cSendByte(2,zero_pos[0]);
    i2cSendByte(2,zero_pos[1]);
    i2cEndTx(2);

    //set burn enable
    i2cStartTx(2);
    i2cSendByte(2,0x81);
    i2cSendByte(2,0x03);
    i2cSendByte(2,0x08);
    i2cEndTx(2);

    zero_pos = encGetPos();

    int pos;
    pos = ((zero_pos[1]<<6)+(zero_pos[0] & 0x3F));

    if(pos == 0){
    	LED_2 = ~LED_2;
    }

    //set verify bit
    i2cStartTx(2);
    i2cSendByte(2,0x81);
    i2cSendByte(2,0x03);
    i2cSendByte(2,0x40);
    i2cEndTx(2);

    pos = ((zero_pos[1]<<6)+(zero_pos[0] & 0x3F));

    if(pos == 0){
    	LED_2 = ~LED_2;
    }
}
