/*
 * Copyright (c) 2011, Fraunhofer ESK
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *
 * Author: 	Günter Hildebrandt <guenter.hildebrandt@esk.fraunhofer.de>
 *
 * @(#)$$
 */

#include <stdio.h>
#include <string.h>
#include "contiki.h"
#include <avr/io.h>
#include <util/delay.h>
#include "sys/clock.h"
#include "dev/leds.h"
#include "radio/rf230bb/rf230bb.h"
#include "dev/watchdog.h"
#include "net/netstack.h"
#include "packetbuf.h"
#include "eeprom_variables.h"
#include "provisioning.h"


#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif


extern uint16_t get_panid_from_eeprom(void);
extern void get_aes128key_from_eeprom(uint8_t*);
extern uint8_t encryption_enabled;

static volatile char in_provisioning = 0;
static volatile char provisioning_pkt_pending = 0;

#define PROVISIONING_HEADER "PROVISIONING"
#define PROV_TIMEOUT_USB 500
#define PROV_TIMEOUT_SOCKET 30

/** \internal The provisioning message structure. */
struct provisioning_m_t {
  char header[sizeof(PROVISIONING_HEADER)];
  uint16_t pan_id;
  uint8_t aes_key[16];
};

//indicates ongoing provisioning
void provisioning_leds(void)
{
	static clock_time_t blink_time;
	static bool red_on = true;
	watchdog_periodic();
	if(clock_time() - blink_time > CLOCK_SECOND / 2) {
		if (red_on) {
			leds_on(LEDS_RED);
		} else {
			leds_off(LEDS_RED);
		}
		red_on = !red_on;
		blink_time = clock_time();
	}
}

//indicates end of provisioning
void provisioning_done_leds(void)
{
	leds_off(LEDS_ALL);
	uint8_t i;
	for(i=0;i<5;i++){
		if (i & 1) {
			leds_on(LEDS_GREEN);
		} else {
			leds_off(LEDS_GREEN);
		}
		watchdog_periodic();
		_delay_ms(500);
	}
}

#if RAVEN_REVISION == HEXABUS_USB

#include "cdc_task.h"
/*
 * the master publishes its PAN ID on the bootloader channel
 */

int provisioning_master(void)
{
	process_start(&provisioning_process, NULL);
	return 0;
}

PROCESS(provisioning_process, "Provisioning process");

PROCESS_THREAD(provisioning_process, ev, data)
{
	PROCESS_EXITHANDLER(goto exit);

	static clock_time_t time;
	static uint16_t prov_pan_id;

	PROCESS_BEGIN();

	leds_off(LEDS_ALL);
	PRINTF("provisioning: started as master\n");
	in_provisioning = 1;

	extern uint16_t mac_dst_pan_id;
	extern uint16_t mac_src_pan_id;
	prov_pan_id = mac_src_pan_id;
	//set pan_id for frame creation to 0x0001
	mac_dst_pan_id = 0x0001;
	mac_src_pan_id = 0x0001;

	rf230_set_pan_addr(0x0001, 0, NULL);
	time = clock_time();
	//Wait max. PROV_TIMEOUT_USBs for provisioning message from Socket that we can start with the transfer
	do {
		PROCESS_PAUSE();

		if (clock_time() - time > (unsigned long long) CLOCK_SECOND * PROV_TIMEOUT_USB)
			break;
		while (!provisioning_pkt_pending) {
			PROCESS_PAUSE();
			provisioning_leds();
			if (clock_time() - time > (unsigned long long) CLOCK_SECOND * PROV_TIMEOUT_USB)
				break;
		}
		provisioning_pkt_pending = 0;
		NETSTACK_FRAMER.parse();
	} while(packetbuf_datalen() != sizeof(PROVISIONING_HEADER) || strncmp((char*)packetbuf_dataptr(), PROVISIONING_HEADER, sizeof(PROVISIONING_HEADER)));
	// timer expired
	if(clock_time() - time > (unsigned long long) CLOCK_SECOND * PROV_TIMEOUT_USB) {
		mac_dst_pan_id = prov_pan_id;
		mac_src_pan_id = prov_pan_id;
		rf230_set_pan_addr(prov_pan_id, 0, NULL);
		printf_P(PSTR(P_FAL_STR));
	} else {
		struct provisioning_m_t *packet;
		char buf[sizeof(struct provisioning_m_t)];

		packet = (struct provisioning_m_t *)buf;
		memset(packet, 0, sizeof(struct provisioning_m_t));

		strncpy(packet->header, PROVISIONING_HEADER, sizeof(PROVISIONING_HEADER));
		packet->pan_id = prov_pan_id;
		get_aes128key_from_eeprom(packet->aes_key);

		packetbuf_copyfrom(&buf, sizeof(struct provisioning_m_t));
		packetbuf_set_datalen(sizeof(struct provisioning_m_t));

		PRINTF("provisioning: sending provisioning message with pan_id: %x and AES_KEY: ", packet->pan_id);
		for (int i = 0; i < 16; i++) {
			PRINTF("%x ", packet->aes_key[i]);
		}
		PRINTF("\n");
		uint8_t tmp_enc = encryption_enabled;
		encryption_enabled = 0;
		NETSTACK_FRAMER.create();
		NETSTACK_RDC.send(NULL,NULL);
		packetbuf_clear();
		encryption_enabled = tmp_enc;
		//provisioning_done_leds();
		mac_dst_pan_id = prov_pan_id;
		mac_src_pan_id = prov_pan_id;
		rf230_set_pan_addr(prov_pan_id, 0, NULL);
		printf_P(PSTR(P_SUC_STR));
	}

exit:
	//indicate normal operation
	in_provisioning = 0;
	leds_off(LEDS_ALL);
	leds_on(LEDS_GREEN);
	PROCESS_END();
}


#elif RAVEN_REVISION == HEXABUS_SOCKET
#include "relay.h"
/*
 * the slave asks the master for its PAN ID
 */
int provisioning_slave(void)
{
	clock_time_t time;

	in_provisioning = 1;

	leds_off(LEDS_GREEN);
	PRINTF("provisioning: started as slave\n");

	extern uint16_t mac_dst_pan_id;
	extern uint16_t mac_src_pan_id;
	uint16_t old_pan_id = mac_src_pan_id;
	//set pan_id for frame creation to 0x0001
	mac_dst_pan_id = 0x0001;
	mac_src_pan_id = 0x0001;
	uint8_t tmp_enc = encryption_enabled;
	encryption_enabled = 0;

	rf230_set_pan_addr(0x0001, 0, NULL);
	time = clock_time();
	//Ask Master every 500ms and for max. PROV_TIMEOUT_SOCKETs to start provisioning
	uint16_t length;
	do {
		if (clock_time() - time > CLOCK_SECOND * PROV_TIMEOUT_SOCKET)
			break;
		provisioning_leds();
		packetbuf_clear();
		packetbuf_set_datalen(0);
		packetbuf_copyfrom(PROVISIONING_HEADER, sizeof(PROVISIONING_HEADER));
		packetbuf_set_datalen(sizeof(PROVISIONING_HEADER));
		NETSTACK_FRAMER.create();
		NETSTACK_RDC.send(NULL,NULL);
		for (int i = 0; !provisioning_pkt_pending && i < 100; i++) {
			process_run();
			_delay_ms(5);
		}
		provisioning_leds();
		//parse frame
		if (provisioning_pkt_pending) {
			NETSTACK_FRAMER.parse();
			provisioning_pkt_pending = 0;
		}
	} while(packetbuf_datalen() != sizeof(struct provisioning_m_t) || strncmp((char*)packetbuf_dataptr(), PROVISIONING_HEADER, sizeof(PROVISIONING_HEADER)));
	// timer expired
	if(clock_time() - time > CLOCK_SECOND * PROV_TIMEOUT_SOCKET) {
		mac_dst_pan_id = old_pan_id;
		mac_src_pan_id = old_pan_id;
		rf230_set_pan_addr(old_pan_id, 0, NULL);
		encryption_enabled = tmp_enc;
		//indicate normal operation
		relay_leds();
		in_provisioning = 0;
		return -1;

	} else {
		struct provisioning_m_t *packet;
		packet = (struct provisioning_m_t *)packetbuf_dataptr();
		PRINTF("provisioning: new pan_id is: %x\n", packet->pan_id);
		PRINTF("provisioning: new pan_id is: %x and new AES_KEY is: ", packet->pan_id);
		for (int i = 0; i < 16; i++) {
			PRINTF("%x ", packet->aes_key[i]);
		}
		PRINTF("\n");
		HAL_ENTER_CRITICAL_REGION();
		eeprom_write_word((uint16_t *)EE_PAN_ID, packet->pan_id);
		eeprom_write_block(packet->aes_key, (void *)EE_ENCRYPTION_KEY, EE_ENCRYPTION_KEY_SIZE);
		HAL_LEAVE_CRITICAL_REGION();
		provisioning_done_leds();
		mac_dst_pan_id = packet->pan_id;
		mac_src_pan_id = packet->pan_id;
		rf230_key_setup(packet->aes_key);
		rf230_set_pan_addr(packet->pan_id, 0, NULL);
		encryption_enabled = tmp_enc;
		//indicate normal operation
		relay_leds();
		in_provisioning = 0;
		return 0;
	}
}
#endif

extern const struct mac_driver PROVISIONING_NEXT_MAC;
/*---------------------------------------------------------------------------*/
static void
send_packet(mac_callback_t sent, void *ptr)
{
  PROVISIONING_NEXT_MAC.send(sent, ptr);
}
/*---------------------------------------------------------------------------*/
static void
packet_input(void)
{
	if (in_provisioning) {
		provisioning_pkt_pending = 1;
	} else {
		PROVISIONING_NEXT_MAC.input();
	}
}
/*---------------------------------------------------------------------------*/
static int
on(void)
{
  return PROVISIONING_NEXT_MAC.on();
}
/*---------------------------------------------------------------------------*/
static int
off(int keep_radio_on)
{
  return PROVISIONING_NEXT_MAC.off(keep_radio_on);
}
/*---------------------------------------------------------------------------*/
static unsigned short
channel_check_interval(void)
{
  return PROVISIONING_NEXT_MAC.channel_check_interval();
}
/*---------------------------------------------------------------------------*/
static void
init(void)
{
	PROVISIONING_NEXT_MAC.init();
}
/*---------------------------------------------------------------------------*/
#define STR2(x) #x
#define STR(x) STR2(x)
const struct mac_driver provisioning_mac_driver = {
  "provisioning_mac(" STR(PROVISIONING_NEXT_MAC) ")",
  init,
  send_packet,
  packet_input,
  on,
  off,
  channel_check_interval,
};
/*---------------------------------------------------------------------------*/
