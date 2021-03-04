/*
 * Copyright (c) 2018, Amber Agriculture
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
/**
 * \file
 *         Orchestra: a slotframe dedicated to unicast data transmission to the root.
 *         See the paper "TSCH for Long Range Low Data Rate Applications", IEEE Access
 *
 * \author Atis Elsts <atis.elsts@gmail.com>
 */

#include "contiki.h"
#include "orchestra.h"
//#include "net/ipv6/uip-ds6-route.h"
#include "net/packetbuf.h"
#include "net/routing/routing.h"

static struct tsch_slotframe *sf_tx;
static struct tsch_slotframe *sf_rx;
static uint16_t timeslot_tx;
static uint8_t is_root_rule_used = 0;

/*---------------------------------------------------------------------------*/
static inline uint8_t
self_is_root(void)
{
  /* In most situations, could simply look at the tsch_is_coordinator variable here.
   * However, the root rule code is generalizable to TSCH networks where more than one RPL
   * root node is present. As a result, the correct way is to look at the RPL status. */
  return NETSTACK_ROUTING.node_is_root();
}
/*---------------------------------------------------------------------------*/
static inline uint8_t
address_is_root(const linkaddr_t *addr)
{
  return tsch_roots_is_root(addr);
}
/*---------------------------------------------------------------------------*/
static uint16_t
get_node_timeslot(const linkaddr_t *addr)
{
  if(addr != NULL && ORCHESTRA_ROOT_PERIOD > 0) {
    return ORCHESTRA_LINKADDR_HASH(addr) % ORCHESTRA_ROOT_PERIOD;
  } else {
    return 0xffff;
  }
}
/*---------------------------------------------------------------------------*/
static uint16_t
get_node_channel_offset(const linkaddr_t *addr)
{
  if(addr != NULL && ORCHESTRA_UNICAST_MAX_CHANNEL_OFFSET >= ORCHESTRA_UNICAST_MIN_CHANNEL_OFFSET) {
    return ORCHESTRA_LINKADDR_HASH(addr) % (ORCHESTRA_UNICAST_MAX_CHANNEL_OFFSET - ORCHESTRA_UNICAST_MIN_CHANNEL_OFFSET + 1)
        + ORCHESTRA_UNICAST_MIN_CHANNEL_OFFSET;
  } else {
    return 0xffff;
  }
}
/*---------------------------------------------------------------------------*/
static int
select_packet(uint16_t *slotframe, uint16_t *timeslot, uint16_t *channel_offset)
{
  /* Select data packets to a root node, in case we are not the root ourselves */
  const linkaddr_t *dest = packetbuf_addr(PACKETBUF_ADDR_RECEIVER);
  if(!self_is_root()
      && packetbuf_attr(PACKETBUF_ATTR_FRAME_TYPE) == FRAME802154_DATAFRAME
      && dest != NULL
      && address_is_root(dest)) {
    if(slotframe != NULL) {
      *slotframe = sf_tx->handle;
    }
    if(timeslot != NULL) {
      *timeslot = timeslot_tx;
    }
    /* set per-packet channel offset */
    if(channel_offset != NULL) {
      *channel_offset = get_node_channel_offset(dest);
    }
    printf("packet to root %u\n", dest->u8[7]);
    return 1;
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
static void
new_time_source(const struct tsch_neighbor *old, const struct tsch_neighbor *new)
{
  /* nothing */
}
/*---------------------------------------------------------------------------*/
static void
init(uint16_t sf_handle)
{
  /* signal that the root rule is used */
  is_root_rule_used = 1;

  uint16_t slotframe_tx_handle = sf_handle;
  uint16_t slotframe_rx_handle = sf_handle | 0x8000;

  /* be careful: if the local node becomes a RPL root, it should add this slotframe! */
  if(self_is_root()) {
    /* Add a 1-slot slotframe for unicast reception */
    sf_rx = tsch_schedule_add_slotframe(slotframe_rx_handle, 1);
    /* Rx link */
    tsch_schedule_add_link(sf_rx,
        LINK_OPTION_SHARED | LINK_OPTION_RX,
        LINK_TYPE_NORMAL, &tsch_broadcast_address,
        0, get_node_channel_offset(&linkaddr_node_addr), 1);
  }

  /* Add a slotframe for unicast transmission to (other) root nodes, initially empty */
  sf_tx = tsch_schedule_add_slotframe(slotframe_tx_handle, ORCHESTRA_ROOT_PERIOD);
  timeslot_tx = get_node_timeslot(&linkaddr_node_addr);
}
/*---------------------------------------------------------------------------*/
struct tsch_link *
orchestra_add_link_to_root(const linkaddr_t *root)
{
  if(is_root_rule_used == 0) {
    /* because of this check, init() must be called before this function! */
    return NULL;
  }

  /* Note in case multiple roots are used:
   * if there are multiple roots in a direct reach, a TSCH cell is added to each of them,
   * at the same timeslot. In each slotframe, the root to use will be selected randomly.
   * This means that the cell's efficiency is reduced to 1/N of the full capacity,
   * where N is the number of directly reachable roots.
   */
  return tsch_schedule_add_link(sf_tx,
      LINK_OPTION_SHARED | LINK_OPTION_TX,
      LINK_TYPE_NORMAL, root,
      timeslot_tx, get_node_channel_offset(root), 0);
}
/*---------------------------------------------------------------------------*/
void
orchestra_remove_link_to_root(const linkaddr_t *root)
{
  if(is_root_rule_used == 0) {
    /* because of this check, init() must be called before this function! */
    return;
  }

  tsch_schedule_remove_link_by_timeslot(sf_tx, timeslot_tx, get_node_channel_offset(root));
  tsch_queue_free_packets_to(root);
}
/*---------------------------------------------------------------------------*/
struct orchestra_rule special_for_root = {
  init,
  new_time_source,
  select_packet,
  NULL,
  NULL,
};
