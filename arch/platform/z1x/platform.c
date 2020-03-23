/*
 * Copyright (c) 2006, Swedish Institute of Computer Science
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
 */

#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "contiki.h"
#include "cc2420.h"
#include "dev/leds.h"
#include "dev/serial-line.h"
#include "dev/slip.h"
#include "dev/uart1.h"
#include "dev/watchdog.h"
#include "dev/xmem.h"
#include "lib/random.h"
#include "net/netstack.h"
#include "net/mac/framer/frame802154.h"
#include "dev/button-sensor.h"
#include "sys/clock.h"
#include "sys/stack-check.h"
#include "net/ipv6/uip-ds6.h"
#include "sys/energest.h"

#if SPHERE
#include "sphere.h"
#endif

#include "sys/node-id.h"
#include "cfs-coffee-arch.h"
#include "cfs/cfs-coffee.h"
#include "sys/autostart.h"

#include "bmp-280-sensor.h"
#include "opt-3001-sensor.h"
#include "hdc-1000-sensor.h"
#include "pir-sensor.h"

SENSORS(&button_sensor, &bmp_280_sensor, &hdc_1000_sensor,
    &opt_3001_sensor, &pir_sensor);

/*---------------------------------------------------------------------------*/
uint16_t node_id;
unsigned char node_mac[8];
/*---------------------------------------------------------------------------*/

#if DCOSYNCH_CONF_ENABLED
static struct timer mgt_timer;
#endif

#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/*---------------------------------------------------------------------------*/
void
init_platform(void)
{
  /* process_start(&sensors_process, NULL); */
}
/*---------------------------------------------------------------------------*/
void
platform_init_stage_one(void)
{
  /*
   * Initalize hardware.
   */
  msp430_cpu_init();

  clock_init();
  leds_init();
  leds_on(LEDS_RED);

  clock_wait(100);
}
/*---------------------------------------------------------------------------*/
void
node_id_init(void)
{
  unsigned char buf[12];
  xmem_pread(buf, 12, NODE_ID_XMEM_OFFSET);
  if(buf[0] == 0xad &&
     buf[1] == 0xde) {
    node_id = (buf[2] << 8) | buf[3];
    memcpy(node_mac, &buf[4], 8);
  } else {
    node_id = 0;
  }
}
/*---------------------------------------------------------------------------*/
void
node_id_burn(unsigned short id)
{
  unsigned char buf[12];
  memset(buf, 0, sizeof(buf));
  buf[0] = 0xad;
  buf[1] = 0xde;
  buf[2] = id >> 8;
  buf[3] = id & 0xff;
  xmem_erase(XMEM_ERASE_UNIT_SIZE, NODE_ID_XMEM_OFFSET);
  xmem_pwrite(buf, 12, NODE_ID_XMEM_OFFSET);
}
/*---------------------------------------------------------------------------*/
void
platform_init_stage_two(void)
{
  uart1_init(115200); /* Must come before first printf */

  xmem_init();

  /*
   * Hardware initialization done!
   */

  /* If no MAC address was burned, we use the node id or the Z1 product ID */
  if(!(node_mac[0] | node_mac[1] | node_mac[2] | node_mac[3] |
       node_mac[4] | node_mac[5] | node_mac[6] | node_mac[7])) {

#ifdef SERIALNUM
    if(!node_id) {
      PRINTF("Node id is not set, using Z1x product ID\n");
      node_id = SERIALNUM;
    }
#endif
    node_mac[0] = 0xc1;  /* Hardcoded for Z1 */
    node_mac[1] = 0x0c;  /* Hardcoded for Revision C */
    node_mac[2] = 0x00;  /* Hardcoded to arbitrary even number so that
                            the 802.15.4 MAC address is compatible with
                            an Ethernet MAC address - byte 0 (byte 2 in
                            the DS ID) */
    node_mac[3] = 0x00;  /* Hardcoded */
    node_mac[4] = 0x00;  /* Hardcoded */
    node_mac[5] = 0x00;  /* Hardcoded */
    node_mac[6] = node_id >> 8;
    node_mac[7] = node_id & 0xff;
  }

  /* Overwrite node MAC if desired at compile time */
#ifdef MACID
#warning "***** CHANGING DEFAULT MAC *****"
  node_mac[0] = 0xc1;  /* Hardcoded for Z1 */
  node_mac[1] = 0x0c;  /* Hardcoded for Revision C */
  node_mac[2] = 0x00;  /* Hardcoded to arbitrary even number so that
                          the 802.15.4 MAC address is compatible with
                          an Ethernet MAC address - byte 0 (byte 2 in
                          the DS ID) */
  node_mac[3] = 0x00;  /* Hardcoded */
  node_mac[4] = 0x00;  /* Hardcoded */
  node_mac[5] = 0x00;  /* Hardcoded */
  node_mac[6] = MACID >> 8;
  node_mac[7] = MACID & 0xff;
#endif

#ifdef IEEE_802154_MAC_ADDRESS
  /* for setting "hardcoded" IEEE 802.15.4 MAC addresses */
  {
    uint8_t ieee[] = IEEE_802154_MAC_ADDRESS;
    memcpy(node_mac, ieee, sizeof(uip_lladdr.addr));
    node_mac[7] = node_id & 0xff;
  }
#endif /* IEEE_802154_MAC_ADDRESS */

  /* Populate linkaddr_node_addr */
  memcpy(linkaddr_node_addr.u8, node_mac, LINKADDR_SIZE);

  /* Initialize the RNG */
  random_init(node_mac[6] + node_mac[7]);

  /*
   * main() will turn the radio on inside netstack_init(). The CC2420
   * must already be initialised by that time, so we do this here early.
   * Later on in stage three we set correct values for PANID and radio
   * short/long address.
   */
  cc2420_init();
}
/*---------------------------------------------------------------------------*/
void
platform_init_stage_three(void)
{
  uint16_t shortaddr;
      
  init_platform();

  shortaddr = (linkaddr_node_addr.u8[0] << 8) + linkaddr_node_addr.u8[1];
  cc2420_set_pan_addr(IEEE802154_PANID, shortaddr, linkaddr_node_addr.u8);

  leds_off(LEDS_ALL);

#if DCOSYNCH_CONF_ENABLED
  timer_set(&mgt_timer, DCOSYNCH_PERIOD * CLOCK_SECOND);
#endif
}
/*---------------------------------------------------------------------------*/
void
platform_idle(void)
{
  /*
   * Idle processing.
   */
  int s = splhigh();    /* Disable interrupts. */
  /* uart1_active is for avoiding LPM3 when still sending or receiving */
  if(process_nevents() != 0 || uart1_active()) {
    splx(s);      /* Re-enable interrupts. */
  } else {
#if DCOSYNCH_CONF_ENABLED
    /* before going down to sleep possibly do some management */
    if(timer_expired(&mgt_timer)) {
      watchdog_periodic();
      timer_reset(&mgt_timer);
      msp430_sync_dco();
#if CC2420_CONF_SFD_TIMESTAMPS
      cc2420_arch_sfd_init();
#endif /* CC2420_CONF_SFD_TIMESTAMPS */
    }
#endif

    /* Re-enable interrupts and go to sleep atomically. */
    ENERGEST_SWITCH(ENERGEST_TYPE_CPU, ENERGEST_TYPE_LPM);
    watchdog_stop();
    _BIS_SR(GIE | SCG0 | SCG1 | CPUOFF); /* LPM3 sleep. This
                                            statement will block
                                            until the CPU is
                                            woken up by an
                                            interrupt that sets
                                            the wake up flag. */

    watchdog_start();
    ENERGEST_SWITCH(ENERGEST_TYPE_LPM, ENERGEST_TYPE_CPU);
  }
}
/*---------------------------------------------------------------------------*/
