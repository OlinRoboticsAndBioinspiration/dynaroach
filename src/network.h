#ifndef __NETWORK_H
#define __NETWORK_H

#if defined(__C2)
	#define NETWORK_BASESTATION_CHANNEL     0x14
	#define NETWORK_BASESTATION_PAN_ID      0x0000
	#define NETWORK_BASESTATION_ADDR        0x0101
	#define NETWORK_SRC_ADDR                0x0012

#elif defined(__C5)
	#define NETWORK_BASESTATION_CHANNEL     0x14
	#define NETWORK_BASESTATION_PAN_ID      0x0000
	#define NETWORK_BASESTATION_ADDR        0x0100
	#define NETWORK_SRC_ADDR                0x0015

#else
	#define NETWORK_BASESTATION_CHANNEL     0x14
	#define NETWORK_BASESTATION_PAN_ID      0x0000
	#define NETWORK_BASESTATION_ADDR        0x0100
	#define NETWORK_SRC_ADDR                0x0110
#endif
#endif
