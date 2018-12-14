/*
 * Copyright (c) 2015, SICS Swedish ICT.
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
 * This file is part of the Contiki operating system.
 *
 */

/**
 * \file
 *         TSCH slot operation implementation, running from interrupt.
 * \author
 *         Simon Duquennoy <simonduq@sics.se>
 *         Beshr Al Nahas <beshr@sics.se>
 *         Atis Elsts <atis.elsts@bristol.ac.uk>
 *
 */

/**
 * \addtogroup tsch
 * @{
*/

#include "dev/radio.h"
#include "contiki.h"
#include "net/netstack.h"
#include "net/packetbuf.h"
#include "net/queuebuf.h"
#include "net/mac/framer/framer-802154.h"
#include "net/mac/tsch/tsch.h"
#include "sys/node-id.h"

#include "sys/log.h"
/* TSCH debug macros, i.e. to set LEDs or GPIOs on various TSCH
 * timeslot events */
#ifndef TSCH_DEBUG_INIT
#define TSCH_DEBUG_INIT()
#endif
#ifndef TSCH_DEBUG_INTERRUPT
#define TSCH_DEBUG_INTERRUPT()
#endif
#ifndef TSCH_DEBUG_RX_EVENT
#define TSCH_DEBUG_RX_EVENT()
#endif
#ifndef TSCH_DEBUG_TX_EVENT
#define TSCH_DEBUG_TX_EVENT()
#endif
#ifndef TSCH_DEBUG_SLOT_START
#define TSCH_DEBUG_SLOT_START()
#endif
#ifndef TSCH_DEBUG_SLOT_END
#define TSCH_DEBUG_SLOT_END()
#endif

/* Check if TSCH_MAX_INCOMING_PACKETS is power of two */
#if (TSCH_MAX_INCOMING_PACKETS & (TSCH_MAX_INCOMING_PACKETS - 1)) != 0
#error TSCH_MAX_INCOMING_PACKETS must be power of two
#endif

/* Check if TSCH_DEQUEUED_ARRAY_SIZE is power of two and greater or equal to QUEUEBUF_NUM */
#if TSCH_DEQUEUED_ARRAY_SIZE < QUEUEBUF_NUM
#error TSCH_DEQUEUED_ARRAY_SIZE must be greater or equal to QUEUEBUF_NUM
#endif
#if (TSCH_DEQUEUED_ARRAY_SIZE & (TSCH_DEQUEUED_ARRAY_SIZE - 1)) != 0
#error TSCH_DEQUEUED_ARRAY_SIZE must be power of two
#endif

/* Truncate received drift correction information to maximum half
 * of the guard time (one fourth of TSCH_DEFAULT_TS_RX_WAIT) */
#define SYNC_IE_BOUND ((int32_t)US_TO_RTIMERTICKS(tsch_timing_us[tsch_ts_rx_wait] / 4))

/* By default: check that rtimer runs at >=32kHz and use a guard time of 10us */
#if RTIMER_SECOND < (32 * 1024)
#error "TSCH: RTIMER_SECOND < (32 * 1024)"
#endif
#if CONTIKI_TARGET_COOJA
/* Use 0 usec guard time for Cooja Mote with a 1 MHz Rtimer*/
#define RTIMER_GUARD 0u
#elif RTIMER_SECOND >= 200000
#define RTIMER_GUARD (RTIMER_SECOND / 100000)
#else
#define RTIMER_GUARD 2u
#endif

/* A ringbuf storing outgoing packets after they were dequeued.
 * Will be processed layer by tsch_tx_process_pending */
struct ringbufindex dequeued_ringbuf;
struct tsch_packet *dequeued_array[TSCH_DEQUEUED_ARRAY_SIZE];
/* A ringbuf storing incoming packets.
 * Will be processed layer by tsch_rx_process_pending */
struct ringbufindex input_ringbuf;
struct input_packet input_array[TSCH_MAX_INCOMING_PACKETS];

/* Last time we received Sync-IE (ACK or data packet from a time source) */
static struct tsch_asn_t last_sync_asn;
clock_time_t last_sync_time; /* Same info, in clock_time_t units */

/* A global lock for manipulating data structures safely from outside of interrupt */
static volatile int tsch_locked = 0;
/* As long as this is set, skip all slot operation */
static volatile int tsch_lock_requested = 0;

/* Last estimated drift in RTIMER ticks
 * (Sky: 1 tick = 30.517578125 usec exactly) */
static int32_t drift_correction = 0;
/* Is drift correction used? (Can be true even if drift_correction == 0) */
static uint8_t is_drift_correction_used;

/* Used from tsch_slot_operation and sub-protothreads */
static rtimer_clock_t volatile current_slot_start;

/* Are we currently inside a slot? */
static volatile int tsch_in_slot_operation = 0;

/* Info about the link, packet and neighbor of
 * the current (or next) slot */
struct tsch_link *current_link = NULL;

/* Indicates whether an extra link is needed to handle the current burst */
int tsch_burst_link_scheduled = 0;

struct tsch_packet *current_packet = NULL;
struct tsch_neighbor *current_neighbor = NULL;

/* Protothread for association */
PT_THREAD(tsch_scan(struct pt *pt));
/* Protothread for slot operation, called from rtimer interrupt
 * and scheduled from tsch_schedule_slot_operation */
static PT_THREAD(tsch_slot_operation(struct rtimer *t, void *ptr));
static struct pt slot_operation_pt;
/* Sub-protothreads of tsch_slot_operation */
static PT_THREAD(tsch_tx_slot(struct pt *pt, struct rtimer *t));
static PT_THREAD(tsch_rx_slot(struct pt *pt, struct rtimer *t));

/*---------------------------------------------------------------------------*/
/* TSCH locking system. TSCH is locked during slot operations */

/* Is TSCH locked? */
int
tsch_is_locked(void)
{
  return tsch_locked;
}

/* Lock TSCH (no slot operation) */
int
tsch_get_lock(void)
{
  if(!tsch_locked) {
    rtimer_clock_t busy_wait_time;
    int busy_wait = 0; /* Flag used for logging purposes */
    /* Make sure no new slot operation will start */
    tsch_lock_requested = 1;
    /* Wait for the end of current slot operation. */
    if(tsch_in_slot_operation) {
      busy_wait = 1;
      busy_wait_time = RTIMER_NOW();
      while(tsch_in_slot_operation) {
        watchdog_periodic();
      }
      busy_wait_time = RTIMER_NOW() - busy_wait_time;
    }
    if(!tsch_locked) {
      /* Take the lock if it is free */
      tsch_locked = 1;
      tsch_lock_requested = 0;
      if(busy_wait) {
        /* Issue a log whenever we had to busy wait until getting the lock */
        TSCH_LOG_ADD(tsch_log_message,
            snprintf(log->message, sizeof(log->message),
                "!get lock delay %u", (unsigned)busy_wait_time);
        );
      }
      return 1;
    }
  }
  TSCH_LOG_ADD(tsch_log_message,
      snprintf(log->message, sizeof(log->message),
                      "!failed to lock");
          );
  return 0;
}

/* Release TSCH lock */
void
tsch_release_lock(void)
{
  tsch_locked = 0;
}

/*---------------------------------------------------------------------------*/
static struct queuebuf *
create_eb(void)
{
  uint8_t hdr_len = 0;
  uint8_t tsch_sync_ie_offset;
  if(tsch_packet_create_eb(&hdr_len, &tsch_sync_ie_offset) <= 0) {
    printf("failed to create EB!\n");
    return NULL;
  }

  return queuebuf_new_from_packetbuf();
}

/*---------------------------------------------------------------------------*/
static struct queuebuf *
create_queuebuf(void)
{
  packetbuf_set_addr(PACKETBUF_ADDR_SENDER, &linkaddr_node_addr);
  packetbuf_set_attr(PACKETBUF_ATTR_FRAME_TYPE, FRAME802154_DATAFRAME);
  packetbuf_set_attr(PACKETBUF_ATTR_MAC_ACK, 1); /* need an ACK */
  packetbuf_set_attr(PACKETBUF_ATTR_MAC_SEQNO, 1); /* always seqno one */
  /* packetbuf_get_attr(PACKETBUF_ATTR_MAC_METADATA, 1); */

  if(NETSTACK_FRAMER.create() < 0) {
    printf("framer create failed\n");
    return NULL;
  }
  return queuebuf_new_from_packetbuf(); 
}

static struct queuebuf *
create_probing_packet(void)
{
  /* Send an almost empty packet, include just the selected node ID and num frames selected */
  uint8_t declared_gw_info[2] = {
    instant_selected_node, instant_num_slotframes_selected
  };
  packetbuf_clear();
  packetbuf_copyfrom(declared_gw_info, sizeof(declared_gw_info));
  packetbuf_set_addr(PACKETBUF_ADDR_RECEIVER, address_from_node_id(INSTANT_ANYCAST_ID));
  return create_queuebuf();
}

/*---------------------------------------------------------------------------*/

static struct queuebuf *
create_data_packet(void)
{
  /* Simply send an dummy packet */
  packetbuf_clear();
  packetbuf_set_datalen(INSTANT_PACKET_SIZE);
  packetbuf_set_addr(PACKETBUF_ADDR_RECEIVER, address_from_node_id(instant_selected_node));
  return create_queuebuf();
}

/*---------------------------------------------------------------------------*/
/* Channel hopping utility functions */

/* Return channel from ASN and channel offset */
uint8_t
tsch_calculate_channel(struct tsch_asn_t *asn, uint8_t channel_offset)
{
  uint16_t index_of_0 = TSCH_ASN_MOD(*asn, tsch_hopping_sequence_length);
  uint16_t index_of_offset = (index_of_0 + channel_offset) % tsch_hopping_sequence_length.val;
  return tsch_hopping_sequence[index_of_offset];
}

/*---------------------------------------------------------------------------*/
/* Timing utility functions */

/* Checks if the current time has passed a ref time + offset. Assumes
 * a single overflow and ref time prior to now. */
static uint8_t
check_timer_miss(rtimer_clock_t ref_time, rtimer_clock_t offset, rtimer_clock_t now)
{
  rtimer_clock_t target = ref_time + offset;
  int now_has_overflowed = now < ref_time;
  int target_has_overflowed = target < ref_time;

  if(now_has_overflowed == target_has_overflowed) {
    /* Both or none have overflowed, just compare now to the target */
    return target <= now;
  } else {
    /* Either now or target of overflowed.
     * If it is now, then it has passed the target.
     * If it is target, then we haven't reached it yet.
     *  */
    return now_has_overflowed;
  }
}
/*---------------------------------------------------------------------------*/
/* Schedule a wakeup at a specified offset from a reference time.
 * Provides basic protection against missed deadlines and timer overflows
 * A return value of zero signals a missed deadline: no rtimer was scheduled. */
static uint8_t
tsch_schedule_slot_operation(struct rtimer *tm, rtimer_clock_t ref_time, rtimer_clock_t offset, const char *str)
{
  rtimer_clock_t now = RTIMER_NOW();
  int r;
  /* Subtract RTIMER_GUARD before checking for deadline miss
   * because we can not schedule rtimer less than RTIMER_GUARD in the future */
  int missed = check_timer_miss(ref_time, offset - RTIMER_GUARD, now);

  if(missed) {
    TSCH_LOG_ADD(tsch_log_message,
                snprintf(log->message, sizeof(log->message),
                    "!dl-miss %s %d %d",
                        str, (int)(now-ref_time), (int)offset);
    );
  } else {
    r = rtimer_set(tm, ref_time + offset, 1, (void (*)(struct rtimer *, void *))tsch_slot_operation, NULL);
    if(r == RTIMER_OK) {
      return 1;
    }
  }

  /* block until the time to schedule comes */
  RTIMER_BUSYWAIT_UNTIL_ABS(0, ref_time, offset);
  return 0;
}
/*---------------------------------------------------------------------------*/
/* Schedule slot operation conditionally, and YIELD if success only.
 * Always attempt to schedule RTIMER_GUARD before the target to make sure to wake up
 * ahead of time and then busy wait to exactly hit the target. */
#define TSCH_SCHEDULE_AND_YIELD(pt, tm, ref_time, offset, str, stats) \
  do { \
    if(tsch_schedule_slot_operation(tm, ref_time, offset - RTIMER_GUARD, str)) { \
      PT_YIELD(pt); \
      RTIMER_BUSYWAIT_UNTIL_ABS(0, ref_time, offset); \
    } else {                                    \
      stats++;                                  \
    }                                           \
  } while(0);
/*---------------------------------------------------------------------------*/
static int
rx_ack(int ack_slot_nr, rtimer_clock_t ref_time, rtimer_clock_t offset, uint8_t seqno)
{
  uint8_t ack_buf[TSCH_PACKET_MAX_LEN];
  int ack_len;
  rtimer_clock_t ack_start_time;
  frame802154_t frame;
  linkaddr_t source;
  uint8_t source_id;
  int status = MAC_TX_NOACK;
  struct ieee802154_ies ack_ies;
  uint8_t ack_hdrlen;

  /* XXX: need to add at least +1 for rx to always work on Z1 */ 
#define EXTRA_WAIT_TIME_TICKS 2

  offset += offset + RADIO_DELAY_BEFORE_DETECT + EXTRA_WAIT_TIME_TICKS;

  if(!RTIMER_CLOCK_LT(RTIMER_NOW(), ref_time + offset)) {
    instant_stats.num_missed_probing_ack_offset++;
  }

  RTIMER_BUSYWAIT_UNTIL_ABS(NETSTACK_RADIO.receiving_packet(), ref_time, offset);

  ack_start_time = RTIMER_NOW() - RADIO_DELAY_BEFORE_DETECT;

  /* Wait for ACK to finish */
  RTIMER_BUSYWAIT_UNTIL_ABS(!NETSTACK_RADIO.receiving_packet(),
                     ack_start_time, tsch_timing[tsch_ts_max_ack]);

  /* Read ack frame */
  ack_len = NETSTACK_RADIO.read((void *)ack_buf, sizeof(ack_buf));

  /* The radio driver should return 0 if no valid packets are in the rx buffer */
  status = MAC_TX_NOACK;

  if(ack_len > 0) {
    /* HACK: extract the source ID from body of the ACK */
    source_id = ack_buf[ack_len - 1];
    ack_len--;

    if(tsch_packet_parse_eack(ack_buf, ack_len, &source, seqno,
            &frame, &ack_ies, &ack_hdrlen) == 0) {
      ack_len = 0;
    }
  }

  if(ack_len > 0) {
    radio_value_t radio_last_rssi;

    status = MAC_TX_OK;

    /* Store the info only if the gateway is willing to accept this node */
    if(ack_ies.num_active_slotframes) {
      instant_acks[ack_slot_nr].node_id = source_id;
      NETSTACK_RADIO.get_value(RADIO_PARAM_LAST_RSSI, &radio_last_rssi);
      instant_acks[ack_slot_nr].rssi = radio_last_rssi;
      instant_acks[ack_slot_nr].num_active_slotframes = ack_ies.num_active_slotframes;
    }
  }

  return status;
}
/*---------------------------------------------------------------------------*/
static
PT_THREAD(tsch_tx_slot(struct pt *pt, struct rtimer *t))
{
  /**
   * TX slot
   **/

  /* tx status */
  static uint8_t mac_tx_status;

  /* is the packet in its neighbor's queue? */
  uint8_t in_queue;
  static int dequeued_index;

  static struct queuebuf *to_send;
  static int is_probing;
  static int is_eb = 0;

  PT_BEGIN(pt);

  TSCH_DEBUG_TX_EVENT();

  NETSTACK_RADIO.on();

  /* Flush any existing packets */
  /* NETSTACK_RADIO.read(NULL, 0); */

#if INSTANT_EMULATE_ORCHESTRA
  is_probing = 0;
#else
  is_probing = INSTANT_IS_PROBING_SLOT(INSTANT_GET_SLOT_INDEX(tsch_current_asn));
#endif

  /* create the packet to send */
  to_send = NULL;

  if(current_packet != NULL) {

    /* First check if we have space to store a newly dequeued packet (in case of
     * successful Tx or Drop) */
    dequeued_index = ringbufindex_peek_put(&dequeued_ringbuf);
    if(dequeued_index != -1) {
      if(current_packet->qb == NULL) {
        mac_tx_status = MAC_TX_ERR_FATAL;
      } else {
        to_send = current_packet->qb;
      }
    }

  } else { /* no packets to send */

    if(INSTANT_IS_WEARABLE(node_id)) {
      /* wearable */
      if(is_probing) {
        /* probing operation */
        to_send = create_probing_packet();
      } else {
        to_send = create_data_packet();
      }
    } else {
      /* a gateway */
      to_send = create_eb();
      is_eb = 1;
    }
  }

  if(to_send != NULL) {
      /* packet payload */
      static void *packet;
      /* packet payload length */
      static uint8_t packet_len;
      /* packet seqno */
      static uint8_t seqno = 1;
      /* is this a broadcast packet? (wait for ack?) */
      static uint8_t is_broadcast;
      static rtimer_clock_t tx_start_time;
      /* Did we set the frame pending bit to request an extra burst link? */
      static int burst_link_requested;

      if(is_eb) {
        NETSTACK_RADIO.set_value(RADIO_PARAM_TXPOWER, INSTANT_EB_TXPOWER);
      }

      /* get payload */
      packet = queuebuf_dataptr(to_send);
      packet_len = queuebuf_datalen(to_send);
      /* is this a broadcast packet? (wait for ack?) */
      if(current_neighbor != NULL) {
        is_broadcast = current_neighbor->is_broadcast;
      } else {
        is_broadcast = !INSTANT_IS_WEARABLE(node_id);
      }

      if(current_packet != NULL) {
        /* read seqno from payload */
        seqno = ((uint8_t *)(packet))[2];

        if(is_broadcast) {
          instant_stats.num_tx_bc++;
        } else {
          instant_stats.num_tx_uc++;
        }
      }

#if INSTANT_EMULATE_ORCHESTRA
      /* Unicast. More packets in queue for the neighbor? */
      burst_link_requested = 0;
      if(INSTANT_IS_WEARABLE(node_id) && instant_stats.num_data_packets) {
        burst_link_requested = 1;
        tsch_packet_set_frame_pending(packet, packet_len);
      }
#endif /* INSTANT_EMULATE_ORCHESTRA */

      /* prepare packet to send: copy to radio buffer */
      if(NETSTACK_RADIO.prepare(packet, packet_len) == 0) { /* 0 means success */
          static rtimer_clock_t tx_duration;

          /* delay before TX */
          TSCH_SCHEDULE_AND_YIELD(pt, t, current_slot_start, tsch_timing[tsch_ts_tx_offset] - RADIO_DELAY_BEFORE_TX,
              "TxBeforeTx", instant_stats.num_missed_tx_offset);
          TSCH_DEBUG_TX_EVENT();
          /* send packet already in radio tx buffer */
          mac_tx_status = NETSTACK_RADIO.transmit(packet_len);
          tx_count++;
          /* Save tx timestamp */
          tx_start_time = current_slot_start + tsch_timing[tsch_ts_tx_offset];
          /* calculate TX duration based on sent packet len */
          tx_duration = TSCH_PACKET_DURATION(packet_len);
          /* limit tx_time to its max value */
          tx_duration = MIN(tx_duration, tsch_timing[tsch_ts_max_tx]);

          if(current_packet == NULL) {
            if(INSTANT_IS_WEARABLE(node_id)) {
              if(is_probing) {
                instant_stats.num_probing_packets_sent++;
              } else {
                instant_stats.num_data_packets_sent++;
              }
            }
          }

          if(mac_tx_status == RADIO_TX_OK) {
            if(!is_broadcast) {
              uint8_t ackbuf[TSCH_PACKET_MAX_LEN];
              int ack_len;
              rtimer_clock_t ack_start_time;
              int is_time_source;
              struct ieee802154_ies ack_ies;
              uint8_t ack_hdrlen;
              frame802154_t frame;

              if(is_probing) {
                rtimer_clock_t offset;
                int i;

                offset = tsch_timing[tsch_ts_tx_offset] + tx_duration;
                offset += US_TO_RTIMERTICKS(INSTANT_IDLE_TIME_US);

                /* Loop for all ACK subslots */
                for(i = 0; i < INSTANT_NUM_ACK_SUBSLOTS; ++i) {
                  if(rx_ack(i, tx_start_time, offset, seqno) == MAC_TX_OK) {
                    mac_tx_status = MAC_TX_OK;
                  }
                  offset += US_TO_RTIMERTICKS(INSTANT_ACK_TIME_US);
                }

                /* do the selection based on RSSI from the ACK */
                instant_select_gateway();

              } else { /* Unicast: wait for ack after tx: sleep until ack time */

                TSCH_SCHEDULE_AND_YIELD(pt, t, current_slot_start,
                    tsch_timing[tsch_ts_tx_offset] + tx_duration + tsch_timing[tsch_ts_rx_ack_delay] - RADIO_DELAY_BEFORE_RX,
                    "TxBeforeAck", instant_stats.num_missed_unicast_ack_offset);
                TSCH_DEBUG_TX_EVENT();
                /* Wait for ACK to come */
                RTIMER_BUSYWAIT_UNTIL_ABS(NETSTACK_RADIO.receiving_packet(),
                    tx_start_time, tx_duration + tsch_timing[tsch_ts_rx_ack_delay] + tsch_timing[tsch_ts_ack_wait] + RADIO_DELAY_BEFORE_DETECT);
                TSCH_DEBUG_TX_EVENT();

                ack_start_time = RTIMER_NOW() - RADIO_DELAY_BEFORE_DETECT;

                /* Wait for ACK to finish */
                RTIMER_BUSYWAIT_UNTIL_ABS(!NETSTACK_RADIO.receiving_packet(),
                    ack_start_time, tsch_timing[tsch_ts_max_ack]);
                TSCH_DEBUG_TX_EVENT();

                /* Read ack frame */
                ack_len = NETSTACK_RADIO.read((void *)ackbuf, sizeof(ackbuf));

                is_time_source = 0;
                /* The radio driver should return 0 if no valid packets are in the rx buffer */
                if(ack_len > 0) {
                  if(current_neighbor != NULL) {
                    is_time_source = current_neighbor->is_time_source;
                  } else {
                    if(INSTANT_IS_WEARABLE(node_id)) {
                      is_time_source = 1;
                    }
                  }

                  if(tsch_packet_parse_eack(ackbuf, ack_len, NULL, seqno,
                          &frame, &ack_ies, &ack_hdrlen) == 0) {
                    ack_len = 0;
                  }
                }

                if(ack_len != 0) {
#if INSTANT_EMULATE_ORCHESTRA
                  /* sync on ACK if coming from the time source */
                  if(is_time_source) {
#else
                  /* disable sync on ACK */
                  if(0 && is_time_source) {
#endif
                    int32_t eack_time_correction = US_TO_RTIMERTICKS(ack_ies.ie_time_correction);
                    drift_correction = eack_time_correction;
                    is_drift_correction_used = 1;
                    /* Keep track of sync time */
                    last_sync_asn = tsch_current_asn;
                    last_sync_time = clock_time();
                  }
                  mac_tx_status = MAC_TX_OK;

                  /* We requested an extra slot and got an ack. This means
                     the extra slot will be scheduled at the received */
                  if(burst_link_requested) {
                    tsch_burst_link_scheduled = 1;
                  }
                } else {
                  mac_tx_status = MAC_TX_NOACK;
                }
              }
            } else {
              mac_tx_status = MAC_TX_OK;
            }
          } else {
            mac_tx_status = MAC_TX_ERR;
          }

          if(current_packet == NULL) {
            if(!is_probing && !is_broadcast && INSTANT_IS_WEARABLE(node_id)) {
              /* this means that we sent an Instant packet */

              if(mac_tx_status == MAC_TX_OK) {
                instant_data_packets_per_slotframe++;

#if INSTANT_EMULATE_ORCHESTRA
                link_stats_packet_sent(address_from_node_id(instant_selected_node), MAC_TX_OK,
                    1 + instant_stats.num_missed_consecutive);
#endif
                if(instant_stats.num_missed_consecutive >= 8) {
                  instant_stats.num_high_tx_attempts++;
                }

                /* reset the counter */
                instant_stats.num_missed_consecutive = 0;

                if(instant_stats.num_data_packets > 0) {
                  instant_stats.num_data_packets--;

                  /* All finished? */
                  if(instant_stats.num_data_packets == 0) {
                    instant_finish();
                  }
                }
              } else { /* mac_tx_status != MAC_TX_OK */
                instant_stats.num_missed_consecutive++;

#if INSTANT_EMULATE_ORCHESTRA
                /* On every 8 missed packets, increase the ETX value by 10  */
                if(instant_stats.num_missed_consecutive % 8 == 0) {
                  link_stats_packet_sent(address_from_node_id(instant_selected_node), MAC_TX_NOACK, 10);
                }
#endif
              }
            }
          } /* current_packet == NULL */
      }
      
      queuebuf_free(to_send);
  }

  NETSTACK_RADIO.off();

  if(is_eb) {
    /* restore the normal Tx power */
    NETSTACK_RADIO.set_value(RADIO_PARAM_TXPOWER, INSTANT_TXPOWER);
  }

  if(current_packet) {
    current_packet->transmissions++;
    current_packet->ret = mac_tx_status;

    /* printf("send st=%u tx=%u\n", mac_tx_status, current_packet->transmissions); */

    /* Post TX: Update neighbor queue state */
    in_queue = tsch_queue_packet_sent(current_neighbor, current_packet, NULL, mac_tx_status);

    /* The packet was dequeued, add it to dequeued_ringbuf for later processing */
    if(in_queue == 0) {
      dequeued_array[dequeued_index] = current_packet;
      ringbufindex_put(&dequeued_ringbuf);
    }

    /* Poll process for later processing of packet sent events and logs */
    process_poll(&tsch_pending_events_process);
  }
  
  TSCH_DEBUG_TX_EVENT();

  PT_END(pt);
}
/*---------------------------------------------------------------------------*/
static
PT_THREAD(tsch_rx_slot(struct pt *pt, struct rtimer *t))
{
  /**
   * RX slot
   **/

  static linkaddr_t source_address;
  static linkaddr_t destination_address;
  static int16_t input_index;
  static int input_queue_drop = 0;

  static int is_probing;

  PT_BEGIN(pt);

  TSCH_DEBUG_RX_EVENT();

  NETSTACK_RADIO.on();

  /* Flush any existing packets */
  /* NETSTACK_RADIO.read(NULL, 0); */

#if INSTANT_EMULATE_ORCHESTRA
  is_probing = 0;
#else
  is_probing = INSTANT_IS_PROBING_SLOT(INSTANT_GET_SLOT_INDEX(tsch_current_asn));
#endif

  input_index = ringbufindex_peek_put(&input_ringbuf);
  if(input_index == -1) {
    input_queue_drop++;
  } else {
    static struct input_packet *current_input;
    /* Estimated drift based on RX time */
    static int32_t estimated_drift;
    /* Rx timestamps */
    static rtimer_clock_t rx_start_time;
    static rtimer_clock_t expected_rx_time;
    static rtimer_clock_t packet_duration;
    uint8_t packet_seen;

    expected_rx_time = current_slot_start + tsch_timing[tsch_ts_tx_offset];
    /* Default start time: expected Rx time */
    rx_start_time = expected_rx_time;

    current_input = &input_array[input_index];

    /* Wait before starting to listen */
    TSCH_SCHEDULE_AND_YIELD(pt, t, current_slot_start, tsch_timing[tsch_ts_rx_offset] - RADIO_DELAY_BEFORE_RX,
        "RxBeforeListen", instant_stats.num_missed_tx_offset);
    TSCH_DEBUG_RX_EVENT();

    /* Start radio for at least guard time */
    packet_seen = NETSTACK_RADIO.receiving_packet() || NETSTACK_RADIO.pending_packet();
    if(!packet_seen) {
      /* Check if receiving within guard time */
      RTIMER_BUSYWAIT_UNTIL_ABS((packet_seen = NETSTACK_RADIO.receiving_packet()),
          current_slot_start, tsch_timing[tsch_ts_rx_offset] + tsch_timing[tsch_ts_rx_wait] + RADIO_DELAY_BEFORE_DETECT);
    }
    if(!packet_seen) {
      /* no packets on air */
      instant_stats.num_rx_idle++;
    } else { /* packet_seen */
      static uint8_t sent_ack;

      TSCH_DEBUG_RX_EVENT();
      /* Save packet timestamp */
      rx_start_time = RTIMER_NOW() - RADIO_DELAY_BEFORE_DETECT;

      sent_ack = 0;

      /* Wait until packet is received, turn radio off */
      RTIMER_BUSYWAIT_UNTIL_ABS(!NETSTACK_RADIO.receiving_packet(),
          current_slot_start, tsch_timing[tsch_ts_rx_offset] + tsch_timing[tsch_ts_rx_wait] + tsch_timing[tsch_ts_max_tx]);
      TSCH_DEBUG_RX_EVENT();

      if(NETSTACK_RADIO.pending_packet()) {
        static int frame_valid;
        static int header_len;
        static frame802154_t frame;
        radio_value_t radio_last_rssi;

        /* Read packet */
        current_input->len = NETSTACK_RADIO.read((void *)current_input->payload, TSCH_PACKET_MAX_LEN);
        NETSTACK_RADIO.get_value(RADIO_PARAM_LAST_RSSI, &radio_last_rssi);

        header_len = frame802154_parse(current_input->payload, current_input->len, &frame);
        frame_valid = header_len > 0 &&
          frame802154_check_dest_panid(&frame) &&
          frame802154_extract_linkaddr(&frame, &source_address, &destination_address);

#if TSCH_RESYNC_WITH_SFD_TIMESTAMPS
        /* At the end of the reception, get an more accurate estimate of SFD arrival time */
        NETSTACK_RADIO.get_object(RADIO_PARAM_LAST_PACKET_TIMESTAMP, &rx_start_time, sizeof(rtimer_clock_t));
#endif

        packet_duration = TSCH_PACKET_DURATION(current_input->len);

        if(!frame_valid) {
          TSCH_LOG_ADD(tsch_log_message,
              snprintf(log->message, sizeof(log->message),
              "!failed to parse frame %u %u", header_len, current_input->len));
        }

        if(frame_valid) {
          if(frame.fcf.frame_type != FRAME802154_DATAFRAME
            && frame.fcf.frame_type != FRAME802154_BEACONFRAME) {
              TSCH_LOG_ADD(tsch_log_message,
                  snprintf(log->message, sizeof(log->message),
                  "!discarding frame with type %u, len %u", frame.fcf.frame_type, current_input->len));
              frame_valid = 0;
          }
        }

        if(frame_valid) {
          uint8_t address_match = 0;
          static uint16_t source_id;
          uint16_t destination_id = node_id_from_address(&destination_address);

          source_id = node_id_from_address(&source_address);

          /* ACK packets addressed to self */
          if(linkaddr_cmp(&destination_address, &linkaddr_node_addr)
             || linkaddr_cmp(&destination_address, &linkaddr_null)) {
            address_match = 1;
          } else if(destination_id == INSTANT_ANYCAST_ID) {
            /* all nodes ack to anycast! */
            address_match = 1;
          }

          if(address_match) {
            int do_nack = 0;

            rx_count++;
            estimated_drift = RTIMER_CLOCK_DIFF(expected_rx_time, rx_start_time);

            if(frame.fcf.ack_required) {
              static uint8_t ack_buf[TSCH_PACKET_MAX_LEN];
              static int ack_len;
              int8_t num_active_slotframes;

              if(INSTANT_IS_WEARABLE(source_id)) {
                /* always update the node status */
                instant_update_node(source_id, radio_last_rssi, is_probing);
              }

              if(is_probing) {
                /* This must be an Instant probing packet */
                uint8_t declared_gw_id;
                uint8_t declared_num_active_slotframes;

                declared_gw_id = current_input->payload[header_len];
                declared_num_active_slotframes = current_input->payload[header_len + 1];
                if(declared_gw_id == node_id) {
                  /* Ignore the declared value: will reselect */
                  declared_num_active_slotframes = 0;
                }

                if(declared_num_active_slotframes != 0) {
                  /* This wearable already belongs to someone else */
                  num_active_slotframes = 0;

                  if(instant_selected_node == source_id) {
                    /* The local gateway thinks it was mine; reset the local state! */
                    instant_num_slotframes_selected = 0;
                    instant_selected_node = INSTANT_ID_NONE;
                  }
                } else {
                  /* Decide how many active slotframes to reserve for this wearable */
                  num_active_slotframes = instant_select_wearable(source_id);
                }

              } else { /* not probing */
                num_active_slotframes = 0; /* leave this unset in the ACK */
              }

              /* Build ACK frame */
              ack_len = tsch_packet_create_eack(ack_buf, sizeof(ack_buf),
                  &source_address, frame.seq,
                  (int16_t)RTIMERTICKS_TO_US(estimated_drift), do_nack, 0,
                  num_active_slotframes, is_probing);

              if(ack_len > 0) {
                static rtimer_clock_t offset;

                if(is_probing) {
                  /* HACK: include own ID in the ACK (instead of the source address: takes much less space!) */
                  ack_buf[ack_len] = (uint8_t)node_id;
                  ack_len++;
                }

                /* Copy to radio buffer */
                NETSTACK_RADIO.prepare((const void *)ack_buf, ack_len);

                offset = packet_duration - RADIO_DELAY_BEFORE_TX;
                if(is_probing) {
                  /* Select the slot to use for the ACK */
                  uint8_t ack_slot = (tsch_current_asn.ls4b + node_id) % INSTANT_NUM_ACK_SUBSLOTS;
                  offset += US_TO_RTIMERTICKS(INSTANT_IDLE_TIME_US) + ack_slot * US_TO_RTIMERTICKS(INSTANT_ACK_TIME_US);

                } else { /* not probing */
                  offset += tsch_timing[tsch_ts_rx_ack_delay];
                }

                /* Wait for time to ACK and transmit ACK */
                TSCH_SCHEDULE_AND_YIELD(pt, t, rx_start_time, offset,
                    "RxBeforeAck", instant_stats.num_missed_unicast_ack_offset);

                TSCH_DEBUG_RX_EVENT();
                NETSTACK_RADIO.transmit(ack_len);
                sent_ack = 1;

                /* Schedule a burst link iff the frame pending bit was set */
                tsch_burst_link_scheduled = tsch_packet_get_frame_pending(current_input->payload, current_input->len);
              }
            }

            /* If the sender is a time source, proceed to clock drift compensation */
            struct tsch_neighbor *n;
            n = tsch_queue_get_nbr(&source_address);
            uint8_t do_sync = 0;
            if(source_id == 1) {
              do_sync = 1;
            } else if(INSTANT_IS_WEARABLE(node_id)) {
              do_sync = 1;
            } else if(n != NULL && n->is_time_source) {
              do_sync = 1;
            }
            if(do_sync) {
              /* Keep track of last sync time */
              last_sync_asn = tsch_current_asn;
              last_sync_time = clock_time();
              /* Save estimated drift */
              drift_correction = -estimated_drift;
              is_drift_correction_used = 1;
              sync_count++;

              // if(!INSTANT_IS_WEARABLE(node_id)) {
              //   printf("sync\n");
              // }
            }

            if(!is_probing) {
              /* Add current input to ringbuf */
              ringbufindex_put(&input_ringbuf);
            }
          }

          if(!is_probing) {
            /* Poll process for processing of pending input and logs */
            process_poll(&tsch_pending_events_process);
          }
        }
      }

      if(sent_ack) {
        instant_stats.num_rx_uc++;
      } else {
        instant_stats.num_rx_bc++;
      }
    }
  }
  NETSTACK_RADIO.off();

  TSCH_DEBUG_RX_EVENT();

  PT_END(pt);
}
/*---------------------------------------------------------------------------*/
/* Protothread for slot operation, called from rtimer interrupt
 * and scheduled from tsch_schedule_slot_operation */
static
PT_THREAD(tsch_slot_operation(struct rtimer *t, void *ptr))
{
  TSCH_DEBUG_INTERRUPT();
  PT_BEGIN(&slot_operation_pt);

  /* Loop over all active slots */
  while(tsch_is_associated) {

    int do_tx, do_rx, channel_offset;

    if(tsch_lock_requested) { /* Skip slot operation if there is a pending request for getting the lock */
      /* Issue a log whenever skipping a slot */
      TSCH_LOG_ADD(tsch_log_message,
                      snprintf(log->message, sizeof(log->message),
                          "!skipped slot %u %u %u",
                            tsch_locked,
                            tsch_lock_requested,
                            NULL == NULL);
      );
    } else {
      TSCH_DEBUG_SLOT_START();
      tsch_in_slot_operation = 1;
      /* Reset drift correction */
      drift_correction = 0;
      is_drift_correction_used = 0;

      instant_on_timeslot(&do_tx, &do_rx, &channel_offset);

      if(do_tx || do_rx) {
        if(tsch_burst_link_scheduled) {
          /* reset the state for the next slot; will set again within the slot */
          tsch_burst_link_scheduled = 0;
        } else {
          /* Hop channel */
          uint8_t tsch_current_channel = tsch_calculate_channel(&tsch_current_asn, channel_offset);
          NETSTACK_RADIO.set_value(RADIO_PARAM_CHANNEL, tsch_current_channel);
        }

        if(do_tx) {
          static struct pt slot_tx_pt;
          PT_SPAWN(&slot_operation_pt, &slot_tx_pt, tsch_tx_slot(&slot_tx_pt, t));

          if(INSTANT_IS_WEARABLE(node_id)
              && instant_stats.num_data_packets
              && !instant_stats.has_started_sending) {
            instant_stats.has_started_sending = 1;
            printf("%u start send\n", (unsigned int)tsch_current_asn.ls4b);
          }         
        } else { /* do_rx */
          static struct pt slot_rx_pt;
          PT_SPAWN(&slot_operation_pt, &slot_rx_pt, tsch_rx_slot(&slot_rx_pt, t));
        }
      } else {
        /* Make sure to end the burst in cast, for some reason, we were
         * in a burst but now without any more packet to send. */
        tsch_burst_link_scheduled = 0;
      }
      TSCH_DEBUG_SLOT_END();
    }

    /* End of slot operation, schedule next slot or resynchronize */

    /* Do we need to resynchronize? i.e., wait for EB again */
    if(!tsch_is_coordinator && (TSCH_ASN_DIFF(tsch_current_asn, last_sync_asn) >
        (100 * TSCH_CLOCK_TO_SLOTS(TSCH_DESYNC_THRESHOLD / 100, tsch_timing[tsch_ts_timeslot_length])))) {
      TSCH_LOG_ADD(tsch_log_message,
            snprintf(log->message, sizeof(log->message),
                "! leaving the network, last sync %u",
                          (unsigned)TSCH_ASN_DIFF(tsch_current_asn, last_sync_asn));
      );
      tsch_disassociate();
    } else {
      rtimer_clock_t prev_slot_start;
      /* Time to next wake up */
      rtimer_clock_t time_to_next_active_slot;
      int schedule_ok = 1;
      /* Schedule next wakeup skipping slots if missed deadline */
      do {
        if(!schedule_ok) {
          instant_stats.num_missed_slot_start++;
        }
        /* Update ASN */
        TSCH_ASN_INC(tsch_current_asn, 1);
        /* Time to next wake up */
        time_to_next_active_slot = tsch_timing[tsch_ts_timeslot_length] + drift_correction;
        drift_correction = 0;
        is_drift_correction_used = 0;
        /* Update current slot start */
        prev_slot_start = current_slot_start;
        current_slot_start += time_to_next_active_slot;
        schedule_ok = tsch_schedule_slot_operation(t, prev_slot_start, time_to_next_active_slot, "main");
        if(!schedule_ok) {
          instant_on_timeslot(&do_tx, &do_rx, &channel_offset);
        }
      } while(!schedule_ok);
    }

    tsch_in_slot_operation = 0;
    PT_YIELD(&slot_operation_pt);
  }

  PT_END(&slot_operation_pt);
}
/*---------------------------------------------------------------------------*/
/* Set global time before starting slot operation,
 * with a rtimer time and an ASN */
void
tsch_slot_operation_start(void)
{
  static struct rtimer slot_operation_timer;
  rtimer_clock_t time_to_next_active_slot;
  rtimer_clock_t prev_slot_start;
  TSCH_DEBUG_INIT();
  NETSTACK_RADIO.set_value(RADIO_PARAM_TXPOWER, INSTANT_TXPOWER);
  NETSTACK_RADIO.on();
  do {
    /* Update ASN */
    TSCH_ASN_INC(tsch_current_asn, 1);
    /* Time to next wake up */
    time_to_next_active_slot = tsch_timing[tsch_ts_timeslot_length];
    /* Update current slot start */
    prev_slot_start = current_slot_start;
    current_slot_start += time_to_next_active_slot;
  } while(!tsch_schedule_slot_operation(&slot_operation_timer, prev_slot_start, time_to_next_active_slot, "assoc"));
}
/*---------------------------------------------------------------------------*/
/* Start actual slot operation */
void
tsch_slot_operation_sync(rtimer_clock_t next_slot_start,
    struct tsch_asn_t *next_slot_asn)
{
  current_slot_start = next_slot_start;
  tsch_current_asn = *next_slot_asn;
  last_sync_asn = tsch_current_asn;
  last_sync_time = clock_time();
}
/*---------------------------------------------------------------------------*/
/** @} */
