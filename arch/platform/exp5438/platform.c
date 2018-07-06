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

#include "contiki.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "dev/button-sensor.h"
#include "cc2420.h"
#include "dev/flash.h"
#include "dev/leds.h"
#include "dev/serial-line.h"
#include "dev/slip.h"
#include "dev/uart1.h"
#include "dev/watchdog.h"
#include "dev/xmem.h"
#include "lib/random.h"
#include "lib/sensors.h"
#include "net/mac/framer/frame802154.h"
#include "net/netstack.h"
#include "net/queuebuf.h"
#include "sys/autostart.h"
#include "os/sys/energest.h"

#include "sys/node-id.h"
#include "lcd.h"
#include "duty-cycle-scroller.h"

#if NETSTACK_CONF_WITH_IPV6
#include "net/ipv6/uip-ds6.h"
#endif /* NETSTACK_CONF_WITH_IPV6 */


#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

unsigned char node_mac[8];

//SENSORS(&button_sensor);
/*---------------------------------------------------------------------------*/
static void
set_rime_addr(void)
{
  linkaddr_t addr;

  memset(&addr, 0, sizeof(linkaddr_t));
#if NETSTACK_CONF_WITH_IPV6
  memcpy(addr.u8, node_mac, sizeof(addr.u8));
#else
  if(node_id == 0) {
    for(i = 0; i < sizeof(linkaddr_t); ++i) {
      addr.u8[i] = node_mac[7 - i];
    }
  } else {
    addr.u8[0] = node_id & 0xff;
    addr.u8[1] = node_id >> 8;
  }
#endif
  linkaddr_set_node_addr(&addr);
}

/*---------------------------------------------------------------------------*/
void
platform_idle(void)
{
  int s = splhigh();          /* Disable interrupts. */
  /* uart1_active is for avoiding LPM3 when still sending or receiving */
  if(process_nevents() != 0 || uart1_active()) {
    splx(s);                  /* Re-enable interrupts. */
  } else {
    /* Re-enable interrupts and go to sleep atomically. */
    ENERGEST_SWITCH(ENERGEST_TYPE_CPU, ENERGEST_TYPE_LPM);
    watchdog_stop();
    _BIS_SR(GIE | SCG0 | SCG1 | CPUOFF); /* LPM3 sleep. This
                                            statement will block
                                            until the CPU is
                                            woken up by an
                                            interrupt that sets
                                            the wake up flag. */

    /* We get the current processing time for interrupts that was
       done during the LPM and store it for next time around.  */
    dint();
    eint();
    watchdog_start();
    ENERGEST_SWITCH(ENERGEST_TYPE_LPM, ENERGEST_TYPE_CPU);
  }
}
/*--------------------------------------------------------------------------*/
void
platform_init_stage_one(void)
{
  /*
   * Initalize hardware.
   */
  msp430_cpu_init();

  leds_init();
  leds_on(LEDS_RED);
}
/*--------------------------------------------------------------------------*/
void
platform_init_stage_two(void)
{
  uart1_init(BAUD2UBR(115200)); /* Must come before first printf */
  leds_on(LEDS_GREEN);
  lcd_init();

#ifdef NODEID
  node_id = NODEID;

#ifdef BURN_NODEID
  flash_setup();
  flash_clear(0x1800);
  flash_write(0x1800, node_id);
  flash_done();
#endif /* BURN_NODEID */
#endif /* NODE_ID */

  if(node_id == 0) {
    node_id = *((unsigned short *)0x1800);
  }
  memset(node_mac, 0, sizeof(node_mac));
  node_mac[6] = node_id >> 8;
  node_mac[7] = node_id & 0xff;

  /* for setting "hardcoded" IEEE 802.15.4 MAC addresses */
#ifdef MAC_1
  {
    uint8_t ieee[] = { MAC_1, MAC_2, MAC_3, MAC_4, MAC_5, MAC_6, MAC_7, MAC_8 };
    memcpy(node_mac, ieee, sizeof(uip_lladdr.addr));
  }
#endif
  set_rime_addr();
  cc2420_init();
}
/*---------------------------------------------------------------------------*/
void
platform_init_stage_three(void)
{
  uint8_t longaddr[8];
  uint16_t shortaddr;

  shortaddr = (linkaddr_node_addr.u8[0] << 8) +
      linkaddr_node_addr.u8[1];
  memset(longaddr, 0, sizeof(longaddr));
  linkaddr_copy((linkaddr_t *)&longaddr, &linkaddr_node_addr);
  printf("MAC %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n",
      longaddr[0], longaddr[1], longaddr[2], longaddr[3],
      longaddr[4], longaddr[5], longaddr[6], longaddr[7]);

  cc2420_set_pan_addr(IEEE802154_PANID, shortaddr, longaddr);

  leds_off(LEDS_ALL);

  if(node_id > 0) {
    printf("Node id %u.\n", node_id);
  } else {
    printf("Node id not set.\n");
  }

#if !NETSTACK_CONF_WITH_IPV6
  uart1_set_input(serial_line_input_byte);
  serial_line_init();
#endif

#if TIMESYNCH_CONF_ENABLED
  timesynch_init();
  timesynch_set_authority_level(linkaddr_node_addr.u8[0]);
#endif /* TIMESYNCH_CONF_ENABLED */

  duty_cycle_scroller_start(CLOCK_SECOND * 2);
}
/*---------------------------------------------------------------------------*/
