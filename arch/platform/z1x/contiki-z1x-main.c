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


uint16_t node_id;
unsigned char node_mac[8];

SENSORS(&button_sensor, &bmp_280_sensor, &hdc_1000_sensor,
    &opt_3001_sensor, &pir_sensor);

#if DCOSYNCH_CONF_ENABLED
static struct timer mgt_timer;
#endif

#define UIP_OVER_MESH_CHANNEL 8

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
static void
set_rime_addr(void)
{
  linkaddr_t addr;
  int i;

  memset(&addr, 0, sizeof(linkaddr_t));
  memcpy(addr.u8, node_mac, sizeof(addr.u8));
  linkaddr_set_node_addr(&addr);
  printf("Node started with address ");
  for(i = 0; i < sizeof(addr.u8) - 1; i++) {
    printf("%d.", addr.u8[i]);
  }
  printf("%d\n", addr.u8[i]);
}
/*---------------------------------------------------------------------------*/
static void
print_processes(struct process *const processes[])
{
  /*  const struct process * const * p = processes;*/
  printf("Starting");
  while(*processes != NULL) {
    printf(" '%s'", (*processes)->name);
    processes++;
  }
  putchar('\n');
}
/*---------------------------------------------------------------------------*/
int
main(int argc, char **argv)
{
  /*
   * Initalize hardware.
   */
  msp430_cpu_init();
  clock_init();
  leds_init();
  leds_on(LEDS_RED);

  clock_wait(100);

  uart1_init(115200); /* Must come before first printf */

  xmem_init();

  rtimer_init();
  /*
   * Hardware initialization done!
   */

  stack_check_init();

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

  /*
   * Initialize Contiki and our processes.
   */
  random_init(node_mac[6] + node_mac[7]);
  process_init();
  process_start(&etimer_process, NULL);

  ctimer_init();

  init_platform();

  set_rime_addr();

  cc2420_init();

  {
    uint8_t longaddr[8];
    uint16_t shortaddr;

    shortaddr = (linkaddr_node_addr.u8[0] << 8) +
      linkaddr_node_addr.u8[1];
    memset(longaddr, 0, sizeof(longaddr));
    linkaddr_copy((linkaddr_t *)&longaddr, &linkaddr_node_addr);
    printf("MAC %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x ",
           longaddr[0], longaddr[1], longaddr[2], longaddr[3],
           longaddr[4], longaddr[5], longaddr[6], longaddr[7]);

    cc2420_set_pan_addr(IEEE802154_PANID, shortaddr, longaddr);
  }

  leds_off(LEDS_ALL);

#ifdef SERIALNUM
  PRINTF("Ref ID: %u\n", SERIALNUM);
#endif
  PRINTF(CONTIKI_VERSION_STRING " started. ");

  if(node_id) {
    PRINTF("Node id is set to %u.\n", node_id);
  } else {
    PRINTF("Node id not set\n");
  }

#if NETSTACK_CONF_WITH_IPV6
  memcpy(&uip_lladdr.addr, node_mac, sizeof(uip_lladdr.addr));
#endif

  netstack_init();

  printf("%s, radio channel %u\n",
         NETSTACK_MAC.name,
         CC2420_CONF_CHANNEL);

#if NETSTACK_CONF_WITH_IPV6 
  process_start(&tcpip_process, NULL);
  
  printf("Tentative link-local IPv6 address ");
  {
    uip_ds6_addr_t *lladdr;
    int i;
    uip_ipaddr_t ipaddr;
    lladdr = uip_ds6_get_link_local(-1);
    for(i = 0; i < sizeof(lladdr->ipaddr.u8) - 1; ++i) {
      printf("%02x%02x:", lladdr->ipaddr.u8[i * 2],
          lladdr->ipaddr.u8[(sizeof(lladdr->ipaddr.u8) - 1) * 2 + 1]);
    }
    printf("%02x%02x\n",
        lladdr->ipaddr.u8[sizeof(lladdr->ipaddr.u8) - 2],
        lladdr->ipaddr.u8[sizeof(lladdr->ipaddr.u8) - 1]);

    uip_ip6addr(&ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 0);
    uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
    uip_ds6_addr_add(&ipaddr, 0, ADDR_TENTATIVE);
    printf("Tentative global IPv6 address ");
    for(i = 0; i < sizeof(ipaddr.u8) - 1; ++i) {
        printf("%02x%02x:",
               ipaddr.u8[i * 2], ipaddr.u8[i * 2 + 1]);
    }
    printf("%02x%02x\n",
           ipaddr.u8[sizeof(ipaddr.u8) - 2], ipaddr.u8[sizeof(ipaddr.u8) - 1]);
  }
#endif

  leds_off(LEDS_GREEN);

#if TIMESYNCH_CONF_ENABLED
  timesynch_init();
  timesynch_set_authority_level(linkaddr_node_addr.u8[0]);
#endif /* TIMESYNCH_CONF_ENABLED */

  energest_init();
  ENERGEST_ON(ENERGEST_TYPE_CPU);

  process_start(&sensors_process, NULL);

  print_processes(autostart_processes);
  autostart_start(autostart_processes);

  /*
   * This is the scheduler loop.
   */
#if DCOSYNCH_CONF_ENABLED
  timer_set(&mgt_timer, DCOSYNCH_PERIOD * CLOCK_SECOND);
#endif
  watchdog_start();
  /*  watchdog_stop();*/
  while(1) {
    int r;
    do {
      /* Reset watchdog. */
      watchdog_periodic();
      r = process_run();
    } while(r > 0);

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
        timer_reset(&mgt_timer);
        msp430_sync_dco();
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

      /* We get the current processing time for interrupts that was
         done during the LPM and store it for next time around.  */
      dint();
      eint();
      watchdog_start();
      ENERGEST_SWITCH(ENERGEST_TYPE_LPM, ENERGEST_TYPE_CPU);
    }
  }

  return 0;
}
/*---------------------------------------------------------------------------*/
