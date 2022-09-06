/*
 * identity.c
 *
 *  Created on: Jun 22, 2019
 *      Author: milad
 */

#include "identity.h"
#include "port.h"

struct identity_t identity_s;
void identity_init(void)
{
	identity_s.address = IDENTITY_ADDRESS_INVALID;
	identity_s.operations = IDENTITY_OPERATIONS_NONE;
}



uint16_t identity_get_address(void)
{
	return identity_s.address;
}

void identity_set_address(uint16_t addr)
{
	identity_s.address = addr;
}

uint16_t identity_get_operations(void)
{
	return identity_s.operations;
}

void identity_set_operations(uint16_t ops)
{
	identity_s.operations = ops;
}

void identity_append_operations(uint16_t ops)
{
	identity_s.operations |= ops;
}

void identity_remove_operations(uint16_t ops)
{
	identity_s.operations &= ~ops;
}
