#ifndef __NETWORK_H
#define __NETWORK_H



#if defined(__C1)
	#define NETWORK_BASESTATION_CHANNEL     0x14
	#define NETWORK_BASESTATION_PAN_ID      0x0000
	#define NETWORK_BASESTATION_ADDR        0x0101
	#define NETWORK_SRC_ADDR                0x0011

#elif defined(__C2)
	#define NETWORK_BASESTATION_CHANNEL     0x14
	#define NETWORK_BASESTATION_PAN_ID      0x0000
	#define NETWORK_BASESTATION_ADDR        0x0101
	#define NETWORK_SRC_ADDR                0x0012
	
#elif defined(__C3)
	#define NETWORK_BASESTATION_CHANNEL     0x14
	#define NETWORK_BASESTATION_PAN_ID      0x0000
	#define NETWORK_BASESTATION_ADDR        0x0101
	#define NETWORK_SRC_ADDR                0x0013
	
#elif defined(__C4)
	#define NETWORK_BASESTATION_CHANNEL     0x14
	#define NETWORK_BASESTATION_PAN_ID      0x0000
	#define NETWORK_BASESTATION_ADDR        0x0101
	#define NETWORK_SRC_ADDR                0x0014

#elif defined(__C5)
	#define NETWORK_BASESTATION_CHANNEL     0x14
	#define NETWORK_BASESTATION_PAN_ID      0x0000
	#define NETWORK_BASESTATION_ADDR        0x0100
	#define NETWORK_SRC_ADDR                0x0015
	
#elif defined(__C6)
	#define NETWORK_BASESTATION_CHANNEL     0x14
	#define NETWORK_BASESTATION_PAN_ID      0x0000
	#define NETWORK_BASESTATION_ADDR        0x0100
	#define NETWORK_SRC_ADDR                0x0016

#elif defined(__C7)
	#define NETWORK_BASESTATION_CHANNEL     0x14
	#define NETWORK_BASESTATION_PAN_ID      0x0000
	#define NETWORK_BASESTATION_ADDR        0x0100
	#define NETWORK_SRC_ADDR                0x0017

#elif defined(__C8)
	#define NETWORK_BASESTATION_CHANNEL     0x14
	#define NETWORK_BASESTATION_PAN_ID      0x0000
	#define NETWORK_BASESTATION_ADDR        0x0100
	#define NETWORK_SRC_ADDR                0x0018

#else
	#define NETWORK_BASESTATION_CHANNEL     0x14
	#define NETWORK_BASESTATION_PAN_ID      0x0000
	#define NETWORK_BASESTATION_ADDR        0x0100
	#define NETWORK_SRC_ADDR                0x0110
#endif
#endif
