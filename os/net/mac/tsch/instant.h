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
 *         "Instant" scheduling mechanism for TSCH. See instant.c for more description.
 * \author
 *         Atis Elsts <atis.elsts@bristol.ac.uk>
 *
 */

#ifndef INSTANT_H
#define INSTANT_H

#include "contiki.h"
#include "net/ipv6/uip.h"

/*---------------------------------------------------------------------------*/
/* Application-tunable parameters */

/* Whether to use connection mode, i.e. always stick with the same gateway / wearable */
#ifdef INSTANT_CONF_IS_CONNECTION_MODE
#define INSTANT_IS_CONNECTION_MODE INSTANT_CONF_IS_CONNECTION_MODE
#else
#define INSTANT_IS_CONNECTION_MODE 0
#endif

/* If not in connection mode, how fast to switch between wearables? */
/* Example values: 1 - fast, 5 normal, 20 - slow. */
#ifdef INSTANT_CONF_INTERTIA
#define INSTANT_INERTIA INSTANT_CONF_INERTIA
#else
#define INSTANT_INERTIA 1
#endif

/* For evaluation only: emulate Orchestra mode? */
#ifdef INSTANT_CONF_EMULATE_ORCHESTRA
#define INSTANT_EMULATE_ORCHESTRA INSTANT_CONF_EMULATE_ORCHESTRA
#else
#define INSTANT_EMULATE_ORCHESTRA 0
#endif

/*
 * This is only required for the RPL+Orchestra mode;
 * "normal" Instant can start sending immediately after joining the network.
 */
#define INSTANT_GRACE_PERIOD_SLOTFRAMES (2 * 60 * 2) /* 2 minutes */

#if CONTIKI_TARGET_SRF06_CC26XX
/* real experiments: never stop */
#define INSTANT_EXPERIMENT_DURATION_SLOTFRAMES 0xffffffff
#else
/* simulation experiments: stop after 6 min */
#define INSTANT_EXPERIMENT_DURATION_SLOTFRAMES (6 * 60 * 2)
#endif

#ifdef INSTANT_CONF_ORCHESTRA_GREEDY
#define INSTANT_ORCHESTRA_GREEDY INSTANT_CONF_ORCHESTRA_GREEDY
#else
#define INSTANT_ORCHESTRA_GREEDY 1
#endif

#ifdef INSTANT_CONF_ORCHESTRA_RANDOMIZE
#define INSTANT_ORCHESTRA_RANDOMIZE INSTANT_CONF_ORCHESTRA_RANDOMIZE
#else
#define INSTANT_ORCHESTRA_RANDOMIZE 0
#endif

#ifdef INSTANT_CONF_RESTART
#define INSTANT_RESTART INSTANT_CONF_RESTART
#else
#define INSTANT_RESTART 1
#endif

/*---------------------------------------------------------------------------*/
/* Testing stuff: reduce the Tx power to make this more challenging */

/* Tx power for data and probing: keep this low */
#define INSTANT_TXPOWER    -10
/* Tx power for network join / sync: keep this normal */
#define INSTANT_EB_TXPOWER 0


/*---------------------------------------------------------------------------*/
/* Node IDs and channel offsets */

/* Invalid ID, used as NULL */
#define INSTANT_ID_NONE 0

/* Root GW ID */
#define INSTANT_GW_ID 1

/* The ID of the first wearable */
#define INSTANT_WEARABLE_OFFSET 192

/* Reserve this for anycast */
#define INSTANT_ANYCAST_ID 254

#define INSTANT_IS_WEARABLE(id) ((id) >= INSTANT_WEARABLE_OFFSET)

/* Channel offset for probing and other (e.g. broadcast) packets */
#define INSTANT_DEFAULT_CHANNEL_OFFSET 0
    
/* Get the channel offset to use for receibing data packets from a specific node */
static inline uint8_t
instant_get_channel_offset(uint8_t id)
{
  /* reuse the node's ID as its channel offset */
  return id;
}

/*---------------------------------------------------------------------------*/
/* Schedule dimensions */

/* Schedule size */
#define INSTANT_SLOTFRAME_SIZE 50

/* Support up to this many wearables */
#define INSTANT_NUM_GATEWAYS 4

/* Support up to this many wearables */
#ifdef INSTANT_CONF_NUM_WEARABLES
#define INSTANT_NUM_WEARABLES INSTANT_CONF_NUM_WEARABLES
#else
#define INSTANT_NUM_WEARABLES 4
#endif

#define INSTANT_NUM_NEIGHBORS MAX(INSTANT_NUM_GATEWAYS, INSTANT_NUM_WEARABLES)

/*---------------------------------------------------------------------------*/
/* Internal operation */

#define INSTANT_IS_BROADCAST_SLOT(slot_index) (slot_index == 0)
#define INSTANT_IS_PROBING_SLOT(slot_index) (slot_index >= 1 && slot_index < 1 + INSTANT_NUM_WEARABLES)
#define INSTANT_IS_UNICAST_SLOT(slot_index) (slot_index >= INSTANT_NUM_WEARABLES)

#define INSTANT_GET_SLOT_INDEX(asn) (uint16_t)((asn).ls4b % INSTANT_SLOTFRAME_SIZE)

#define INSTANT_GET_SLOTFRAME(asn) (uint16_t)((asn).ls4b / INSTANT_SLOTFRAME_SIZE)

/* Acks per slot ($N_a$) */
#define INSTANT_NUM_ACK_SUBSLOTS   3

/* $\tau_i$ */
#define INSTANT_IDLE_TIME_US 1000
/* $\tau_a$ */
#define INSTANT_ACK_TIME_US  600

/*---------------------------------------------------------------------------*/
/* Other config */

/* bytes, payload size */
#define INSTANT_PACKET_SIZE 104

/* XXX: for demo purposes, always constant 100 kB */
#define INSTANT_DATA_SIZE   100000
    
#define INSTANT_NUM_DATA_PACKETS (INSTANT_DATA_SIZE / INSTANT_PACKET_SIZE)

#define INSTANT_NETWORK_PREFIX 0xfd00

/* $T_{fresh}$ */
#define INSTANT_FRESH_ROUNDS 4

/* For energy consumption measurements */
#define INSTANT_SEND_AND_POWER_OFF 0


/*---------------------------------------------------------------------------*/
/* Types */

typedef struct instant_neighbor {
  /* Last rssi received */
  int8_t last_rssi;
  /* Last time seen */
  uint16_t last_slotframe;
  /* First time seen (reset when becomes inactive) */
  uint16_t first_slotframe_seen;
} instant_neighbor_t;


typedef struct instant_ack_info {
  /* Gateway ID */
  uint8_t node_id;
  /* The RSSI of the ACK */
  int8_t rssi;
  /* The offered number of slotframes reserved for this wearable */
  uint8_t num_active_slotframes;
} instant_ack_info_t;


typedef struct instant_stats {
  /* The assumed length of the data packet queue */
  uint16_t num_data_packets;
  /* The number of data packets sent; includes unacknowledged packets */
  uint16_t num_data_packets_sent;
  /* The number of probing packets sent */
  uint16_t num_probing_packets_sent;

  uint16_t num_rx_idle;
  uint16_t num_rx_uc;
  uint16_t num_rx_bc;
  uint16_t num_tx_uc;
  uint16_t num_tx_bc;

  uint8_t has_started_sending;

  /* How many transmissions in row have failed? */
  uint16_t num_missed_consecutive;
  /* Packets with more than 8 required transmissions */
  uint16_t num_high_tx_attempts;

  /* Slot timing misses: for debugging the TSCH timing. Should all be 0 normally. */
  uint16_t num_missed_slot_start;
  uint16_t num_missed_tx_offset;
  uint16_t num_missed_tx_fin;
  uint16_t num_missed_probing_ack_offset;
  uint16_t num_missed_unicast_ack_offset;
} instant_stats_t;


/*---------------------------------------------------------------------------*/
/* Variables */

/* Could be read from ASN, but for an optimization kept in a variable */
extern volatile uint16_t instant_current_slotframe;

/* Selected GW for wearable, selected wearable for a GW. INSTANT_ID_NONE if not selected */
extern volatile uint8_t instant_selected_node;

/* For how many slotframes still selected? ($a_{selected}$) */
extern volatile uint8_t instant_num_slotframes_selected;

/* Neighbor statistics (RSSI, last time seen), used to select the best one */
extern volatile instant_neighbor_t instant_neigbors[INSTANT_NUM_NEIGHBORS];

/* Probing ACK received in the current slot */
extern volatile instant_ack_info_t instant_acks[INSTANT_NUM_ACK_SUBSLOTS];

/* The stastics of a single run (i.e. 100 kB data transfer) */
extern volatile instant_stats_t instant_stats;

/* Data packets received sent during the current slotframe */
extern volatile uint8_t instant_data_packets_per_slotframe;

/* All gateways send ACK if receiving packet addressed to this anycast address*/
extern uip_ipaddr_t instant_anycast_address;


/*---------------------------------------------------------------------------*/
/* Functions */

/* Initialization: for example, setup the anycast address and add a route to it */
void instant_init(void);

/* Update node statistics upon receiving a packets / ACK */
void instant_update_node(uint8_t sender_id, int8_t rssi, int is_probing);

/* For Orchestra emulation mode */
void instant_update_selected_node_from_routing(const linkaddr_t *new_addr);

/* Sets selected node and num slotframes based on `instant_acks` */
void instant_select_gateway(void);

/* Returns number of active sloframes for sender_id if its selected, else 0 */
uint8_t instant_select_wearable(uint8_t sender_id);

/*
 * Choose the action on a timeslot.
 *
 * @param do_tx - set to nonzero if this slot should be used for Tx.
 * @param do_rx - set to nonzero if this slot should be used for Rx. 
 * @param channel_offset - sets to the channel offset to use.
 */
void instant_on_timeslot(int *do_tx, int *do_rx, int *channel_offset);

/* Bookkeeping on sloftrame ending */
void instant_on_sloftrame_end(void);

/* Bookkeeping on run ending (100 kB of data being sent) */
void instant_finish(void);


#endif /* INSTANT_H */
