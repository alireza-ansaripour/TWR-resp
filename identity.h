/*
 * identity.h
 *
 *  Created on: Jun 22, 2019
 *      Author: milad
 */

#ifndef SRC_APPLICATION_IDENTITY_H_
#define SRC_APPLICATION_IDENTITY_H_

//#define ADDRESS_BROADCAST 0xFFFF
//#define ADDRESS_INVALID 0xFFFE

#include <stdint.h>


#define IDENTITY_ADDRESS_INVALID 0xFFFE
#define IDENTITY_ADDRESS_BROADCAST 0xFFFF

#define IDENTITY_OPERATIONS_NONE						0x0000
#define IDENTITY_OPERATIONS_TIMESYNC					0x0002
#define IDENTITY_OPERATIONS_DATA_TX						0x0040
#define IDENTITY_OPERATIONS_DATA_RX						0x0080
#define IDENTITY_OPERATIONS_ACK_EN						0x0100
#define IDENTITY_OPERATIONS_INIT_TWR        					0x0200
#define IDENTITY_OPERATIONS_RESP_TWR                    			0x0400
//#define IDENTITY_OPERATIONS_XYZ						0x0800
//#define IDENTITY_OPERATIONS_XYZ						0x1000
#define IDENTITY_OPERATIONS_UUID						0x2000
#define IDENTITY_OPERATIONS_LOG							0x4000
#define IDENTITY_OPERATIONS_MONITOR						0x8000
#define IDENTITY_OPERATIONS_ALL							0xFFFF

#define IDENTITY_OPERATIONS_ALL_UWB_RX					(IDENTITY_OPERATIONS_DATA_RX)
//#define IDENTITY_OPERATIONS_ALL_UWB_TX					(IDENTITY_OPERATIONS_TIMESYNC_MASTER|IDENTITY_OPERATIONS_RANGING_MASTER|IDENTITY_OPERATIONS_RANGING_SLAVE|IDENTITY_OPERATIONS_BLINK_TX)

struct identity_t{
	uint16_t address;
	uint16_t operations;
};



void identity_init(void);
//void identity_set_all(identity_t _id);
uint16_t identity_get_address(void);
void identity_set_address(uint16_t addr);
uint16_t identity_get_operations(void);
void identity_set_operations(uint16_t ops);
void identity_append_operations(uint16_t ops);
void identity_remove_operations(uint16_t ops);

#endif /* SRC_APPLICATION_IDENTITY_H_ */
