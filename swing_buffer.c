/*
 * ring_buffer.c
 *
 *  Created on: Jun 18, 2019
 *      Author: milad
 */

#include "ring_buffer.h"

//volatile ring_buffer_t __attribute__((section(".ram2"))) ring_buffer_s;
ring_buffer_t ring_buffer_s;
//buff_status status = BUFFER_READY;


void ring_buffer_init(void)
{
	ring_buffer_s.read_index = 0;
	ring_buffer_s.write_index = 0;
	ring_buffer_s.len = 0;
	ring_buffer_s.size = sizeof(ring_buffer_s.buffer) / sizeof(uint8_t);
	ring_buffer_s.chunk_len = 0;
	memset(ring_buffer_s.buffer, 0, sizeof(ring_buffer_s.buffer));
	ring_buffer_s.status = BUFFER_READY;
}

// returns chunk length
uint16_t ring_buffer_get_chunk(uint8_t ** data_ptr)
{
	uint16_t chunk_len;
	if (ring_buffer_s.status != BUFFER_BUSY && ring_buffer_s.len)
	{
		ring_buffer_s.status = BUFFER_BUSY;
		if (ring_buffer_s.read_index < ring_buffer_s.write_index)
		{
			chunk_len = ring_buffer_s.write_index - ring_buffer_s.read_index;
		}
		else
		{
			chunk_len = ring_buffer_s.size - ring_buffer_s.read_index;
		}
		*data_ptr = &ring_buffer_s.buffer[ring_buffer_s.read_index];
		ring_buffer_s.chunk_len = chunk_len;
	}
	else
	{
		chunk_len = 0;
	}
	return chunk_len;
}

void ring_buffer_free_chunk(void)
{
	ring_buffer_s.read_index = (ring_buffer_s.read_index + ring_buffer_s.chunk_len) % ring_buffer_s.size;
	ring_buffer_s.len -= ring_buffer_s.chunk_len;
	ring_buffer_s.chunk_len = 0;
	ring_buffer_s.status = BUFFER_READY;
}

// returns result: 0 OK, 1 BUFFER FULL
uint8_t ring_buffer_write(uint8_t * data, uint16_t len)
{
	uint8_t ret = 0;
	uint16_t chunk_len;
	if (len <= ring_buffer_s.size - ring_buffer_s.len)
	{
		if (ring_buffer_s.write_index + len >= ring_buffer_s.size)
		{
			chunk_len = ring_buffer_s.size - ring_buffer_s.write_index;
			memcpy(&ring_buffer_s.buffer[ring_buffer_s.write_index], data, chunk_len);
			ring_buffer_s.len += chunk_len;
			data += chunk_len;
			chunk_len = len - chunk_len;
			ring_buffer_s.write_index = 0;
		}
		else
		{
			chunk_len = len;
		}
		memcpy(&ring_buffer_s.buffer[ring_buffer_s.write_index], data, chunk_len);
		ring_buffer_s.write_index += chunk_len;
		ring_buffer_s.len += chunk_len;
//		ring_buffer_s.status = BUFFER_READY;
	}
	else
	{
		//buffer full
		ret = 1;
	}
	return ret;
}


uint16_t ring_buffer_get_length(void)
{
	return ring_buffer_s.len;
}

buff_status ring_buffer_get_status(){
	return ring_buffer_s.status;
}
