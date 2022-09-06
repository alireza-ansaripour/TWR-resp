/*
 * ring_buffer.h
 *
 *  Created on: Jun 18, 2019
 *      Author: milad
 */

#ifndef SRC_PLATFORM_RING_BUFFER_H_
#define SRC_PLATFORM_RING_BUFFER_H_

#include <stdint.h>
#include "port.h"

#define RING_BUFFER_SIZE 28000
typedef enum{
	BUFFER_READY,
	BUFFER_BUSY,
} buff_status;

typedef struct
{
	uint16_t read_index;
	uint16_t write_index;
	uint16_t len;
	uint16_t size;
	uint16_t chunk_len;
	uint8_t buffer[RING_BUFFER_SIZE];
	buff_status status;
} ring_buffer_t;

void ring_buffer_init(void);
uint16_t ring_buffer_get_chunk(uint8_t ** data_ptr); // returns chunk length
//uint8_t ring_buffer_free_chunk(uint8_t * data_ptr, uint16_t len); // returns result: 0 OK, 1 INVALID
void ring_buffer_free_chunk(void);
uint8_t ring_buffer_write(uint8_t * data, uint16_t len); // returns result: 0 OK, 1 BUFFER FULL

uint16_t ring_buffer_get_length(void);

#endif /* SRC_PLATFORM_RING_BUFFER_H_ */
