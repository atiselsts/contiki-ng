/*
 * Copyright (c) 2018, University of Bristol
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
 */

/**
 * \file
 *         "Instant" scheduling mechanism for TSCH.
 * \author
 *         Atis Elsts <atis.elsts@bristol.ac.uk>
 */

/*
 * This file provides two modes of operation:
 * 1) The Instant scheduling mechanism.
 * 2) A mode that emulates Orchestra.

 * The second mode is used for comparative tests. The regular Orchestra
 * is not used as the existing scheduling API in files tsch-schedule.{h,c}
 * is completely removed to avoid confusion.
 *
 * The scheduling is done directly form the tsch_slot_operation protothread.
 * All operations except instant_init() are done in the interrupt context.
 * There is no real packets used in the current implentation: the function
 * create_data_packet() simply creates an empty packet with INSTANT_PACKET_SIZE.
 * However, it could be modified to actually take the packet from a queue.
 *
 * Switch between the two different modes by #defining INSTANT_EMULATE_ORCHESTRA to 0 or 1.
 */

#include "instant.h"
#include "net/netstack.h"
#include "net/ipv6/uip-ds6.h"
#include "net/ipv6/uip-ds6-route.h"
#include "sys/node-id.h"
#include "lib/random.h"
#include "tsch.h"
#include <stdio.h>

#if CONTIKI_TARGET_SRF06_CC26XX
#include "lpm.h"
#endif

/* ----------------------------------------------------------- */

volatile uint16_t instant_current_slotframe;

volatile uint8_t instant_selected_node = INSTANT_ID_NONE;

volatile uint8_t instant_num_slotframes_selected;

volatile instant_neighbor_t instant_neigbors[INSTANT_NUM_NEIGHBORS];

volatile instant_ack_info_t instant_acks[INSTANT_NUM_ACK_SUBSLOTS];

volatile instant_stats_t instant_stats;

volatile uint8_t instant_data_packets_per_slotframe;

uip_ipaddr_t instant_anycast_address;

static uint8_t sf_without_packets;

#ifndef RPL_ROUTE_INFINITE_LIFETIME
#define RPL_ROUTE_INFINITE_LIFETIME           0xFFFFFFFF
#endif

#if CONTIKI_TARGET_SRF06_CC26XX
#define LOCKABLE_DOMAINS ((uint32_t)(PRCM_DOMAIN_SERIAL | PRCM_DOMAIN_PERIPH))
extern lpm_registered_module_t *get_modules(void);
#endif

/*---------------------------------------------------------------------------*/
/* Orchestra emulation */

/* In Orchestra mode: slots for the wearables */
#if INSTANT_ORCHESTRA_RANDOMIZE
/* Make the slot depend on the slotframe as well as on the node ID */
#define ORCHESTRA_IS_LOCAL_WEARABLE_SLOT(slot, node_id) \
  ((slot) == 1 + (11 * ((node_id) + instant_current_slotframe)) % INSTANT_NUM_WEARABLES)
#else /* INSTANT_ORCHESTRA_RANDOMIZE */
/* Make the slot depend just on the node ID */
#define ORCHESTRA_IS_LOCAL_WEARABLE_SLOT(slot, node_id) \
  ((slot) == 1 + (node_id) - INSTANT_WEARABLE_OFFSET)
#endif /* INSTANT_ORCHESTRA_RANDOMIZE */

#define ORCHESTRA_IS_ANY_WEARABLE_SLOT(slot)                 \
  ((slot) > 0 && (slot) <= INSTANT_NUM_WEARABLES)

/* ----------------------------------------------------------- */
void
instant_init(void)
{
#if !INSTANT_EMULATE_ORCHESTRA
  const linkaddr_t *lla;
  uip_ds6_nbr_t *nbr;
  uip_ds6_route_t *route;
  int i;

  lla = address_from_node_id(INSTANT_ANYCAST_ID);

  uip_ip6addr(&instant_anycast_address, INSTANT_NETWORK_PREFIX, 0, 0, 0, 0, 0, 0, 0);
  uip_ds6_set_addr_iid(&instant_anycast_address, &uip_lladdr);
  instant_anycast_address.u8[15] = INSTANT_ANYCAST_ID;

  /* add a link-layer neighbor */
  nbr = uip_ds6_nbr_add(&instant_anycast_address, (uip_lladdr_t *)lla, 1 /* isrouter */,
                        NBR_REACHABLE, NBR_TABLE_REASON_MAC, NULL);

  /* set the neighbor timer to never expire */
  stimer_set(&nbr->reachable, (unsigned long)(0xffffffff / 2));

  /* add a route to the anycast address through itself */
  route = uip_ds6_route_add(&instant_anycast_address, 128, &instant_anycast_address);
  route->state.lifetime = RPL_ROUTE_INFINITE_LIFETIME;

  for(i = 0; i < INSTANT_NUM_WEARABLES; ++i) {
    instant_neigbors[i].first_slotframe_seen = 0xffff;
  }
#endif /* without RPL */

#if 0
  if(INSTANT_IS_WEARABLE(node_id)) {
    /* set up the "data" packets to send */
    instant_stats.num_data_packets = INSTANT_NUM_DATA_PACKETS;
  }
#endif

  printf("Instant started: connection_mode=%u inertia=%u emulate_orchestra=%u\n",
      INSTANT_IS_CONNECTION_MODE,
      INSTANT_INERTIA,
      INSTANT_EMULATE_ORCHESTRA);
}
/*---------------------------------------------------------------------------*/
void
instant_update_selected_node_from_routing(const linkaddr_t *new_addr)
{
  if(INSTANT_IS_WEARABLE(node_id)) {
    /* Update the selected node */
    if(new_addr == NULL) {
      instant_selected_node = INSTANT_ID_NONE;
    } else {
      instant_selected_node = node_id_from_address(new_addr);
      /* should not normally happen due to the LEAF mode, but just in case... */
      if(INSTANT_IS_WEARABLE(instant_selected_node)) {
        printf("BUG!!! selected another wearable as routing parent\n");
        instant_selected_node = INSTANT_ID_NONE;
      }
    }
    printf("set instant_selected_node to %u\n", instant_selected_node);
  }
}
/*---------------------------------------------------------------------------*/
void
instant_update_node(uint8_t sender_id, int8_t rssi, int is_probing)
{
  uint8_t index;

  if(!is_probing) {
    /* increase the data packet count */
    instant_data_packets_per_slotframe++;
  }

  /* mark the sender as seen */
  index = sender_id - INSTANT_WEARABLE_OFFSET;
  if(index < INSTANT_NUM_WEARABLES) {
    /* printf("instant: update node sf=%u\n", instant_current_slotframe); */
    instant_neigbors[index].last_rssi = rssi;
    instant_neigbors[index].last_slotframe = instant_current_slotframe;
    if(instant_neigbors[index].first_slotframe_seen == 0xffff) {
      instant_neigbors[index].first_slotframe_seen = instant_current_slotframe;
    }
  } else {
    printf("instant: bad sender id %u\n", sender_id);
  }
}
/*---------------------------------------------------------------------------*/
void
instant_select_gateway(void)
{
  int i;
  int best = -1;

  if(instant_stats.num_data_packets == 0) {
    /* no packets to send */
    instant_selected_node = INSTANT_ID_NONE;
    instant_num_slotframes_selected = 0;
    return;
  }

#if INSTANT_IS_CONNECTION_MODE
  if(instant_selected_node != INSTANT_ID_NONE) {
    /* keep max value */
    instant_num_slotframes_selected = 255;
    /* reset the state */
    for(i = 0; i < INSTANT_NUM_ACK_SUBSLOTS; ++i) {
      instant_acks[i].node_id = 0;
    }
    /* keep the old node */
    return;
  }
#endif /* INSTANT_IS_CONNECTION_MODE */

  for(i = 0; i < INSTANT_NUM_ACK_SUBSLOTS; ++i) {
    if(instant_acks[i].node_id != 0) {
      if(best == -1 || instant_acks[i].rssi > instant_acks[best].rssi) {
        best = i;
      }
    }
  }

  if(best != -1) {
    instant_selected_node = instant_acks[best].node_id;
    instant_num_slotframes_selected = instant_acks[best].num_active_slotframes;

    printf("%u sel gw %u for %u\n", (unsigned int)tsch_current_asn.ls4b,
        instant_selected_node, instant_num_slotframes_selected);
  } else {
    /* No valid gateway info; however, do nothing: may keep the previous one! */
  }

  /* reset the state */
  for(i = 0; i < INSTANT_NUM_ACK_SUBSLOTS; ++i) {
    instant_acks[i].node_id = 0;
  }

  if (instant_selected_node == INSTANT_ID_NONE) {
    printf("%u no gw\n", (unsigned int)tsch_current_asn.ls4b);
  }
}
/*---------------------------------------------------------------------------*/
static int8_t
estimate_num_active_slotframes(void)
{
#if INSTANT_IS_CONNECTION_MODE
  return 255; /* max value */
#else /* INSTANT_IS_CONNECTION_MODE */
  uint8_t num_active_wearables = 0;
  uint16_t num_slotframes_around = 0xffff;
  int i;
  uint16_t num_rounds;
  uint16_t per_wearable;

  for(i = 0; i < INSTANT_NUM_WEARABLES; ++i) {
    if(instant_neigbors[i].first_slotframe_seen == 0xffff) {
      continue;
    }

    if(instant_neigbors[i].last_slotframe + INSTANT_FRESH_ROUNDS >= instant_current_slotframe) {
      num_active_wearables++;
      num_slotframes_around = MIN(num_slotframes_around,
          (uint16_t)(1 + instant_current_slotframe - instant_neigbors[i].first_slotframe_seen));
    } else {
      instant_neigbors[i].first_slotframe_seen = 0xffff;
    }
  }

  if(0) {
    /* take into account number of wearables */
    per_wearable = (num_slotframes_around + num_active_wearables - 1) / num_active_wearables;
  } else {
    /* take into account just the number of slots */
    per_wearable = num_slotframes_around;
  }

  /* keep the result in range [INSTANT_INERTIA, 5 * INSTANT_INERTIA] */
  num_rounds = MAX(INSTANT_INERTIA, MIN(5 * INSTANT_INERTIA, per_wearable));

  return num_rounds;
#endif /* INSTANT_IS_CONNECTION_MODE */
}
/*---------------------------------------------------------------------------*/
uint8_t
instant_select_wearable(uint8_t sender_id)
{
  if(instant_selected_node == INSTANT_ID_NONE) {
    /* Not currently selected: do the random selection, i.e. select the first one
     * (this is random because the sending order is random!)
     */
    instant_selected_node = sender_id;
    instant_num_slotframes_selected = estimate_num_active_slotframes();

    /* printf("sel wearable %u for %u\n", sender_id, instant_num_slotframes_selected); */
  }

  return instant_selected_node == sender_id ? instant_num_slotframes_selected : 0;
}
/*---------------------------------------------------------------------------*/
void
instant_on_sloftrame_end(void)
{
  /* decrease the number of remaining active slotframes */
  if(instant_num_slotframes_selected > 0) {
    instant_num_slotframes_selected--;
  }

  if(instant_data_packets_per_slotframe == 0) {
    /* no packets in the last slotframe */
    instant_num_slotframes_selected = 0;

    if(!INSTANT_IS_WEARABLE(node_id)) {
      /* printf("no packets\n"); */
    }

    sf_without_packets++;
  }
  else {
    sf_without_packets = 0;
  }

#if INSTANT_EMULATE_ORCHESTRA
  /* print the messages used by the stats collection script */
  if(INSTANT_IS_WEARABLE(node_id) && instant_stats.num_data_packets && instant_stats.has_started_sending) {
    if(instant_data_packets_per_slotframe) {
      printf("%u sel gw %u for -1\n", (unsigned int)tsch_current_asn.ls4b,
          instant_selected_node);
    } else {
      printf("%u no gw\n", (unsigned int)tsch_current_asn.ls4b);
    }
  }
#endif

  if(sf_without_packets >= 4) {
    if(!INSTANT_IS_WEARABLE(node_id)) {
      /* probably there is not ongoing run; print statistics */
      /* printf("misses: ss=%u tx/rx=%u ack_u=%u ack_p=%u\n",
          instant_stats.num_missed_slot_start,
          instant_stats.num_missed_tx_offset,
          instant_stats.num_missed_probing_ack_offset,
          instant_stats.num_missed_unicast_ack_offset); */
    }
    sf_without_packets = 0;
  }
  instant_data_packets_per_slotframe = 0;

  if(INSTANT_IS_WEARABLE(node_id) && instant_stats.num_data_packets == 0) {
    /* no packets to send */
    instant_num_slotframes_selected = 0;
  }

  /* If no more active slotframes... */
  if(instant_num_slotframes_selected == 0) {
    /* ..deselect the node */
#if !INSTANT_EMULATE_ORCHESTRA
    instant_selected_node = INSTANT_ID_NONE;
#endif
  }
}

/*---------------------------------------------------------------------------*/
static void power_off(void) __attribute__((unused));
/*---------------------------------------------------------------------------*/
#if CONTIKI_TARGET_SRF06_CC26XX
/*---------------------------------------------------------------------------*/
static void
power_off(void)
{
  lpm_shutdown(IOID_UNUSED, 0, 0); 
}
/*---------------------------------------------------------------------------*/
#else /* CONTIKI_TARGET_SRF06_CC26XX */
/*---------------------------------------------------------------------------*/
static void
power_off(void)
{
  /* not supported */
}
/*---------------------------------------------------------------------------*/
#endif /* CONTIKI_TARGET_SRF06_CC26XX */
/*---------------------------------------------------------------------------*/
void
instant_finish(void)
{
#if 1
  printf("%u finished sending: %u d %u pr %u rxi %u rxuc %u rxbc %u txuc %u txbc %u >_8_tx\n",
      (unsigned int)tsch_current_asn.ls4b,
      instant_stats.num_data_packets_sent,
      instant_stats.num_probing_packets_sent,
      instant_stats.num_rx_idle,
      instant_stats.num_rx_uc,
      instant_stats.num_rx_bc,
      instant_stats.num_tx_uc,
      instant_stats.num_tx_bc,
      instant_stats.num_high_tx_attempts);
  printf("misses: ss=%u tx/rx=%u fin=%u ack_u=%u ack_p=%u\n",
      instant_stats.num_missed_slot_start,
      instant_stats.num_missed_tx_offset,
      instant_stats.num_missed_tx_fin,
      instant_stats.num_missed_probing_ack_offset,
      instant_stats.num_missed_unicast_ack_offset);

#endif

#if INSTANT_SEND_AND_POWER_OFF
  NETSTACK_RADIO.off();
  power_off();
#endif
}
/*---------------------------------------------------------------------------*/
int
node_id_from_address(const linkaddr_t *address)
{
  if(address == NULL) {
    return 0;
  }
  if(linkaddr_cmp(&tsch_eb_address, address)) {
    return 0x0;
  }
  if(linkaddr_cmp(&tsch_broadcast_address, address)) {
    return 0xff;
  }
  return address->u8[7];
}
/*---------------------------------------------------------------------------*/
const linkaddr_t *
address_from_node_id(int node_id)
{
  static linkaddr_t addr;
  if(node_id == 0xff) {
    return &tsch_broadcast_address;
  }

#if CONTIKI_TARGET_COOJA
  addr.u8[0] = node_id >> 8;
  addr.u8[1] = node_id & 0xff;
  addr.u8[2] = node_id >> 8;
  addr.u8[3] = node_id & 0xff;
  addr.u8[4] = node_id >> 8;
  addr.u8[5] = node_id & 0xff;
  addr.u8[6] = node_id >> 8;
  addr.u8[7] = node_id & 0xff;
#else
  memcpy(&addr, &linkaddr_node_addr, sizeof(addr));
  addr.u8[7] = node_id & 0xff;
#endif
  return &addr;
}
/*---------------------------------------------------------------------------*/
#if INSTANT_EMULATE_ORCHESTRA
/*---------------------------------------------------------------------------*/
static struct tsch_packet *
get_packet_and_neighbor_for_broadcast(struct tsch_neighbor **target_neighbor)
{
  struct tsch_packet *p = NULL;
  struct tsch_neighbor *n = NULL;

  /* fetch broadcast packets */

  n = n_broadcast;
  p = tsch_queue_get_packet_for_nbr(n, NULL);

  /* if it is a broadcast slot and there were no broadcast packets, pick any unicast packet */
  if(p == NULL) {
    /* do this only for the gateways */
    if(!INSTANT_IS_WEARABLE(node_id)) {
      p = tsch_queue_get_unicast_packet_for_any(&n, NULL);
    }
  }
  /* return nbr (by reference) */
  if(target_neighbor != NULL) {
    *target_neighbor = n;
  }

  return p;
}
/*---------------------------------------------------------------------------*/
static struct tsch_packet *
get_packet_and_neighbor_for_unicast(struct tsch_neighbor **target_neighbor)
{
  struct tsch_packet *p = NULL;
  struct tsch_neighbor *n = NULL;

  p = tsch_queue_get_unicast_packet_for_any(&n, NULL);

  /* return nbr (by reference) */
  if(target_neighbor != NULL) {
    *target_neighbor = n;
  }

  return p;
}
/*---------------------------------------------------------------------------*/
#endif /* INSTANT_EMULATE_ORCHESTRA */
/*---------------------------------------------------------------------------*/
void
instant_on_timeslot(int *do_tx, int *do_rx, int *channel_offset)
{
  uint16_t slot_index;
  static int has_reset_stats;

  *do_tx = 0;
  *do_rx = 0;
  *channel_offset = INSTANT_DEFAULT_CHANNEL_OFFSET;

  /* Get slot index */
  slot_index = INSTANT_GET_SLOT_INDEX(tsch_current_asn);
  instant_current_slotframe = INSTANT_GET_SLOTFRAME(tsch_current_asn);

  if(INSTANT_IS_BROADCAST_SLOT(slot_index)) {
    /* end the previous slotframe */
    instant_on_sloftrame_end();
  }

#if INSTANT_EMULATE_ORCHESTRA
  /*
   * This code block simulates the behavior of the sender-based Orchestra schedule
   * with slotframe size equal to the INSTANT_SLOTFRAME_SIZE both for broadcast and unicast.
   */

  /* By default, autogenerate packets */
  current_packet = NULL;
  current_neighbor = NULL;

  if(INSTANT_IS_BROADCAST_SLOT(slot_index)) {
    /* if have broadcast packets
       OR the local node is gateway and have any packet, send;
       otherwise receive
    */
    current_packet = get_packet_and_neighbor_for_broadcast(&current_neighbor);

    tsch_burst_link_scheduled = 0;
              
    if(current_packet != NULL) {
      *do_tx = 1;
    } else {
      if(INSTANT_IS_WEARABLE(node_id)) {
        /* Always receive */
        *do_rx = 1;

      } else {
        uint16_t r;

        r = random_rand();
        if(tsch_is_coordinator) {
          /* Occasionally generate EB */
          if((r >> 3) % 4 == 1) {
            *do_tx = 1;
          }
        } else {
          /* Occasionally generate EB */
          if((r >> 3) % 8 == 1) {
            *do_tx = 1;
          }
        }

        if(!*do_tx) {
          /* Receive if not sending own packets */
          *do_rx = 1;
        }
      }
    }

  } else {
    /* UNICAST */
   
    if(INSTANT_IS_WEARABLE(node_id)) {

#if INSTANT_ORCHESTRA_GREEDY
      if(!ORCHESTRA_IS_LOCAL_WEARABLE_SLOT(slot_index, node_id)
          && instant_stats.num_data_packets
          && instant_data_packets_per_slotframe) {
        /* Always assume a burst as long as the first packet went through */
        tsch_burst_link_scheduled = 1;
      }
#endif

      /* wearable */
      if(ORCHESTRA_IS_LOCAL_WEARABLE_SLOT(slot_index, node_id) || tsch_burst_link_scheduled) {
        /* send to the routing parent */
        if(instant_selected_node != INSTANT_ID_NONE) {
          /* Check if there are unicast packets to send */
          current_packet = get_packet_and_neighbor_for_unicast(&current_neighbor);

          if(current_packet != NULL || instant_stats.num_data_packets) {
            /* If there is a real packet to send, or if there are the fake data packets */
            *do_tx = 1;
          }
        }
      }
    } else {
      /* gateway */

#if INSTANT_ORCHESTRA_GREEDY
      if(instant_data_packets_per_slotframe) {
        /* Always assume a burst as long as the first packet went through */
        tsch_burst_link_scheduled = 1;
      }
#endif
      
      if(ORCHESTRA_IS_ANY_WEARABLE_SLOT(slot_index) || tsch_burst_link_scheduled) {
        /* receive from a wearable */
        *do_rx = 1;
      }
    }
  }

#else /* !INSTANT_EMULATE_ORCHESTRA */

  /* This code block is for the Instant schedule */
  if(INSTANT_IS_BROADCAST_SLOT(slot_index)) {
    /* BROADCAST */

    if(INSTANT_IS_WEARABLE(node_id)) {
      /* Always receive */
      *do_rx = 1;

    } else {
      uint16_t r;

      r = random_rand();
      if(tsch_is_coordinator) {
        /* Occasionally generate EB */
        if((r >> 3) % 4 == 1) {
          *do_tx = 1;
        }
      } else  {
        /* Occasionally generate EB */
        if((r >> 3) % 8 == 1) {
          *do_tx = 1;
        }
      }

      if(!*do_tx) {
        *do_rx = 1;
      }
    }
  } else if(INSTANT_IS_PROBING_SLOT(slot_index)) {
    /* PROBING */
    if(INSTANT_IS_WEARABLE(node_id)) {

      /* Do the probing only if no gateway currently selected */
      if(instant_selected_node == INSTANT_ID_NONE) {
        uint8_t round_robin_index;

        /* XXX: the sending order isn't really pseudorandom, but round-robin */
        round_robin_index = (node_id + instant_current_slotframe) % INSTANT_NUM_WEARABLES;

        if(round_robin_index == slot_index - 1
            && instant_stats.num_data_packets > 0) {
          *do_tx = 1;
        }
      }
    } else {
      /* not a wearable */
      *do_rx = 1;
    }
  } else {
    /* UNICAST */
    if(instant_selected_node) {
      if(INSTANT_IS_WEARABLE(node_id) && instant_stats.num_data_packets > 0) {
        *do_tx = 1;
        /* use own channel offset */
        *channel_offset = instant_get_channel_offset(node_id);
      } else {
        *do_rx = 1;
        /* use selected wearable's channel offset */
        *channel_offset = instant_get_channel_offset(instant_selected_node);
      }
    }
  }

#endif /* INSTANT_EMULATE_ORCHESTRA */      

  /* approx once per minute */
  if(INSTANT_IS_WEARABLE(node_id)) {

    if(instant_current_slotframe > INSTANT_GRACE_PERIOD_SLOTFRAMES
        && instant_current_slotframe % 128 == 0
        && instant_current_slotframe < INSTANT_EXPERIMENT_DURATION_SLOTFRAMES) {

#if INSTANT_EMULATE_ORCHESTRA
      /* Do not start sending if the routing is not ready yet */
      if(instant_selected_node != INSTANT_ID_NONE) {
#endif
        if(!has_reset_stats) {
          has_reset_stats = 1;

          /* has finished sending the previous batch? */
          if(instant_stats.num_data_packets == 0) {
            static volatile uint8_t has_started_once;
            
            memset((void *)&instant_stats, 0, sizeof(instant_stats));

#if INSTANT_RESTART
            /* this is the assumed length of the data packet queue */
            instant_stats.num_data_packets = INSTANT_NUM_DATA_PACKETS;
#else /* INSTANT_RESTART */
            if(!has_started_once) {
              /* do this just once */
              instant_stats.num_data_packets = INSTANT_NUM_DATA_PACKETS;
            }
#endif /* INSTANT_RESTART */
            has_started_once = 1;
            (void)has_started_once;

            *do_tx = 0;
            *do_rx = 0;
          } else {
            printf("instant: continue prev run\n");
          }
        }
#if INSTANT_EMULATE_ORCHESTRA
      }
#endif
    } else {
      /* Not in the right slotframe */
      has_reset_stats = 0;
    }
  }
}
