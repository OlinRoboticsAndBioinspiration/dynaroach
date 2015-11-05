#include "p33Fxxxx.h"
#include "cmd.h"
#include "radio.h"
#include "mac_packet.h"
#include "dfmem.h"
#include "sma.h"
#include "motor_ctrl.h"
#include "attitude_q.h"
#include "payload.h"
#include "consts.h"
#include "utils.h"
#include "sclock.h"
#include "gyro.h"
#include "xl.h"
#include "statetransition.h"
#include "led.h"
#include "adc.h"
#include "wii.h"
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "ams-enc.h"
#include "carray.h"
#include "cmd.h"
#include "network.h"

#define EXC_FUNC_MAX 10

/*HALL ENCODER VARIABLES FOR ISR T7*/
#define HALLDATALENGTH 12
#define FULLROT 16384
#define GEARRATIO 5
#define REVTOOUTPUT 3277
#define CAMDATALENGTH 8
static unsigned char buf_idx = 1;
static uByte4 curr_halldata;
static uByte4 prev_halldata;
static uByte4 out_hallpos;
static int sethall= 0;
static uByte2 hall_total_cnt;
static uByte2 hall_mem_total_cnt;
static int numsamples= 0;
static int mem_pg_idx = 0;
static unsigned char rotation_cnt=0;
/**********************************/

/*PROCESS CAM WITH WII*/
static unsigned char * exc_wii_ptr;
static int sendCamData = 0;
static int readWiiData = 0;
static int Wiiinvalid= 0;
static unsigned int curr_blobsize =0;
static unsigned int prev_blobsize =0;
static unsigned int prev_blobxpos =0;
static unsigned int curr_blobxpos =0;
static float curr_speed =0.3;
static WiiBlob ExcBlobs[4]; 

#define FLASH_8MBIT_BYTES_PER_PAGE          264
#define FLASH_16MBIT_BYTES_PER_PAGE         528

static CircArray exc;
static unsigned char DataWrite[HALLDATALENGTH];//used to store data to be written to memory

//SETTING UP THE CIRCUILAR ARRAY WITH MAX FUNC =10//
void excSetup(){
    exc = carrayCreate(EXC_FUNC_MAX);
}

unsigned int get_src_addr(void){
    TBLPAG = 0x0;
    unsigned int addr = __builtin_tblrdl(SRC_ADDR_LOC);
    return addr;
}

unsigned int get_basestation_addr(void){
    TBLPAG=0x0;
    unsigned int addr = __builtin_tblrdl(SRC_ADDR_LOC+2);
    return addr;
}

unsigned int get_pan_id(void){
    TBLPAG=0x0;
    unsigned int addr = __builtin_tblrdl(SRC_ADDR_LOC+4);
    return addr;
}

unsigned int get_channel(void){
    TBLPAG=0x0;
    unsigned int addr = __builtin_tblrdl(SRC_ADDR_LOC+6);
    return addr;
}

//Function Declaration//
static void excGetHallEncPos(void);
static void excGetCamData(void);
static void excProcessCamMotor(void);
static void excRecordHallData(void);
static void excRecordCamData(void);

//Function Pointers//
void (*excGetHall)(void) = &excGetHallEncPos; 
void (*excGetCam)(void) = &excGetCamData;
void (*excProcessCam)(void) = &excProcessCamMotor;
void (*excRecordHall)(void) = &excRecordHallData;
void (*excRecordCam)(void) = &excRecordCamData;

static void excGetCamData(void)
{
    exc_wii_ptr = wiiReadData();
    delay_ms(100);
    send(STATUS_UNUSED, 12, exc_wii_ptr, CMD_WII_DUMP);
}

//Save data to pages 250 - n in memory
static void excRecordHallData(void)
{
        dfmemWriteBuffer(DataWrite, HALLDATALENGTH, numsamples*HALLDATALENGTH, buf_idx);
        numsamples++;

        if(numsamples*HALLDATALENGTH >= FLASH_8MBIT_BYTES_PER_PAGE) 
        {   
            dfmemWrite(DataWrite, HALLDATALENGTH, mem_pg_idx, numsamples, buf_idx);
            numsamples=0;
            mem_pg_idx++;
            hall_mem_total_cnt.sval++; //EveryTime one page worth of data was written increment one. 1pg= 44 samples
        }       
}

//Save data to pages 250 - n in memory
static void excRecordCamData(void)
{
        dfmemWriteBuffer(DataWrite,CAMDATALENGTH, numsamples*CAMDATALENGTH, buf_idx);
        numsamples++;

        if(numsamples*CAMDATALENGTH >= FLASH_8MBIT_BYTES_PER_PAGE) 
        {   
            dfmemWrite(DataWrite, CAMDATALENGTH, mem_pg_idx, numsamples, buf_idx);
            numsamples=0;
            mem_pg_idx++;
            //hall_mem_total_cnt.sval++; //EveryTime one page worth of data was written increment one. 1pg= 44 samples
        }       
}

static void excProcessCamMotor(void)
{
    
    exc_wii_ptr = wiiReadData();
    delay_ms(100);
    send(STATUS_UNUSED, 12, exc_wii_ptr, CMD_WII_DUMP);
    curr_blobxpos = ((exc_wii_ptr[2] & 0x30)<<8) +(exc_wii_ptr[0]);
    if(curr_blobxpos != 0x3FF && curr_blobxpos !=0)
    {
        if (curr_blobxpos>prev_blobxpos )
        {
            curr_speed -= 0.2;
            mcSetDutyCycle(1,curr_speed);
            MD_LED_1 =~LED_1;
        }
        if(curr_blobxpos<prev_blobxpos)
        {
            curr_speed += 0.2;
            mcSetDutyCycle(1,curr_speed);
            MD_LED_2 = ~LED_2;
        }

        prev_blobxpos  = curr_blobxpos;
    }
}

static void excGetHallEncPos(void)
{  
    uByte4 halltime;
    int i;

    halltime.lval = sclockGetTicks();
    curr_halldata.lval = encGetPos(); //encGetPos() returns short(int) for input gear

    if(curr_halldata.lval>prev_halldata.lval)
    {
        if(curr_halldata.lval-prev_halldata.lval>60)
        {
            rotation_cnt++;
        }   
    }
    if(rotation_cnt>4)
    {
        //out_hallpos.lval = curr_halldata.lval/GEARRATIO;
        out_hallpos.lval = (FULLROT-curr_halldata.lval)/GEARRATIO;
        //(16384-In_hallData.lval)/5;
        rotation_cnt = 0;
    }

    else
    {
        if(prev_halldata.lval-curr_halldata.lval<FULLROT)
        {
            out_hallpos.lval = FULLROT-(rotation_cnt*REVTOOUTPUT+(FULLROT-curr_halldata.lval)/GEARRATIO);
        }
    }

    prev_halldata.lval = curr_halldata.lval;

    for(i=0;i<4;i++)
    {
        DataWrite[i]=halltime.cval[i];
    }

    for(i=4;i<8;i++){
        DataWrite[i]=curr_halldata.cval[i-4];
    }

    for(i=8;i<12;i++){
        DataWrite[i]=out_hallpos.cval[i-8];
    }

}

//Handles the EXC carray (internal sampling sensors)
void cmdHandleExcBuffer(void){
    void(*item)();

    item = carrayPopHead(exc);
    if(item != NULL)
    {
        item();
    }
    return;
}

void __attribute__((interrupt, no_auto_psv)) _T7Interrupt(void)
{   
    // if(sethall)
    // {
    //     carrayAddTail(Exc,excGetHall);
    //     carrayAddTail(Exc, excRecord);
    //     hall_total_cnt.sval++;
    //     if(hall_total_cnt.sval % 100 == 0)
    //     {
    //         if(0)
    //         {
                carrayAddTail(exc,excGetCam);
                //carrayAddTail(Exc,excRecord);
            //}
    
            
            //else
            //{
                //carrayAddTail(Exc,excProcessCam);
                //carrayAddTail(Exc,excRecord);
            //}
        //} 
    //}
    _T7IF = 0;
}