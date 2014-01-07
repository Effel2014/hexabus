#include <stdlib.h>

#include "sequence_numbers.h"
#include "lib/crc16.h"
#include "lib/random.h"
#include "hexabus_types.h"

static uint16_t seqnum_table[SEQNUM_TABLE_LENGTH];

uint16_t next_sequence_number(const uip_ipaddr_t* toaddr) {

	uint16_t crc = crc16_data((unsigned char*)toaddr, 16, 0);
	uint8_t hash = (uint8_t)(((crc>>12)^(crc>>8)^(crc>>4)^crc)&0x000f);

	return seqnum_table[hash]++;

}

void sequence_number_init() {
	int i;
	for (i = 0; i < SEQNUM_TABLE_LENGTH; i++)
	{
		seqnum_table[i] = random_rand();
	}
}
