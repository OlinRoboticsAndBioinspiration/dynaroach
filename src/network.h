#ifndef __NETWORK_H
#define __NETWORK_H

#define NETWORK_BASESTATION_CHANNEL     0x14
#define NETWORK_BASESTATION_PAN_ID      0x0000
#define NETWORK_SRC_ADDR                0x0110

void networkSetBaseStationAddr(int addr);
int networkGetBaseStationAddr(void);

#endif
