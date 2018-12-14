#include <stdint.h>

#define SPHERE 1
#define PROPER_MAC_ADDRS 1

struct key_addr_info_s {
  uint8_t security_key[16];
  uint8_t ieee_address[8];
  uint8_t ble_address[6];
} __attribute__((packed));

extern struct key_addr_info_s key_addr_info;

/*---------------------------------------------------------------------------*/

/* Keep the number of routes and neighbors sensible: use the Contiki-NG defaults */
/* Actually, the RPL grid simulations work *worse* if the number of routes is set to 100 */
#define NBR_TABLE_CONF_MAX_NEIGHBORS 16
#define NETSTACK_MAX_ROUTE_ENTRIES   16

#define SICSLOWPAN_CONF_FRAG 0 /* No fragmentation */
#define UIP_CONF_BUFFER_SIZE 200

#define STARTUP_CONF_VERBOSE 1

/*******************************************************/
/******************* Configure TSCH ********************/
/*******************************************************/

/* IEEE802.15.4 PANID */
#undef IEEE802154_CONF_PANID
#define IEEE802154_CONF_PANID 0xdada

/* Do not start TSCH at init, wait for NETSTACK_MAC.on() */
#define TSCH_CONF_AUTOSTART 0

#define TSCH_CONF_INIT_SCHEDULE_FROM_EB 0

#define TSCH_CONF_ADAPTIVE_TIMESYNC 0

/* Custom schedule, collision free for unicast */
#define TSCH_SCHEDULE_CONF_WITH_6TISCH_MINIMAL 0

/* TSCH slotframe size */
#define TSCH_SCHEDULE_CONF_DEFAULT_LENGTH INSTANT_SLOTFRAME_SIZE
/* just one slotframe max */
#define TSCH_SCHEDULE_CONF_MAX_SLOTFRAMES   1

/* TCSH hopping sequence */
#define TSCH_HOPPING_SEQUENCE_INSTANT (uint8_t[]){ 25, 20, 26, 15, 11, 25, 20, 26, 15}

#define TSCH_CONF_DEFAULT_HOPPING_SEQUENCE TSCH_HOPPING_SEQUENCE_INSTANT

/* Do not include dst address in ACKs */
#define TSCH_PACKET_CONF_EACK_WITH_DEST_ADDR 0

/* keep the XOSC on even when radio is off */
#undef CC2650_FAST_RADIO_STARTUP
#define CC2650_FAST_RADIO_STARTUP   1

/* IEEE802.15.4 frame version */
#undef FRAME802154_CONF_VERSION
#define FRAME802154_CONF_VERSION FRAME802154_IEEE802154_2015

#define TSCH_LOG_CONF_ID_FROM_LINKADDR(addr) node_id_from_address(addr)

/* Disable HW frame filtering */
#define TSCH_CONF_HW_FRAME_FILTERING 0

#define IEEE_ADDR_CONF_LOCATION_SECONDARY  key_addr_info.ieee_address

/* TSCH and RPL callbacks */
#define RPL_CALLBACK_PARENT_SWITCH tsch_rpl_callback_parent_switch
#define RPL_CALLBACK_NEW_DIO_INTERVAL tsch_rpl_callback_new_dio_interval

#define RPL_CONF_DIO_INTERVAL_MIN 11 /* 2.048 s */

/* do not change the RPL interval too much (default is 8, giving 1024 seconds) */
#ifndef RPL_CONF_DIO_INTERVAL_DOUBLINGS
#define RPL_CONF_DIO_INTERVAL_DOUBLINGS 2   /* Corresponds to 8 seconds */
#endif

/* Do not turn off TSCH within a timeslot: not enough time */
#define TSCH_CONF_RADIO_ON_DURING_TIMESLOT 1

/* Reduce guard times */
#define TSCH_CONF_RX_WAIT         1800
#define TSCH_CONF_RX_WAIT_STARTUP 1800

/* leave the default tx offset value */
#define TSCH_CONF_TS_TX_OFFSET   2120

#ifdef INSTANT_CONF_EMULATE_ORCHESTRA
/* Retry up to 7 times */
#define TSCH_CONF_MAC_MAX_FRAME_RETRIES 7
#else
/* Never retry */
#define TSCH_CONF_MAC_MAX_FRAME_RETRIES 0
#endif

/* Note that EB sending logic is moved to tsch-slot-operation.c and keepalive sending is disabled */

/* Packet timeout after which to leave the network  */
#if CONTIKI_TARGET_COOJA
/* Never desync in the simulator */
#define TSCH_CONF_DESYNC_THRESHOLD (3600 * CLOCK_SECOND)
#else
#define TSCH_CONF_DESYNC_THRESHOLD   (CLOCK_SECOND * 50)
#endif

/* Disable security */
#define USE_TSCH_SECURITY 0

/* 2 times per second */
#define TSCH_CONF_CHANNEL_SCAN_DURATION (CLOCK_SECOND / 2)

#ifdef CONF_NUM_WEARABLES
/* default to at least 4 */
#define INSTANT_CONF_NUM_WEARABLES    MAX(4, CONF_NUM_WEARABLES)
#endif

/*******************************************************/
/************* Other system configuration **************/
/*******************************************************/

/* Logging */
#define LOG_CONF_LEVEL_MAC          LOG_LEVEL_ERR

/* Disable sleeping */
#define LPM_CONF_MODE_MAX_SUPPORTED LPM_MODE_AWAKE

/* define to 1 for connection mode */
/* #define INSTANT_CONF_IS_CONNECTION_MODE 1 */

/* define to 1 for orchestra (greedy) mode: RPL also must be enabled in the Makefile! */
#ifndef INSTANT_CONF_EMULATE_ORCHESTRA
#if ROUTING_CONF_NULLROUTING
#define INSTANT_CONF_EMULATE_ORCHESTRA 0
#else /* ROUTING_CONF_NULLROUTING */
#define INSTANT_CONF_EMULATE_ORCHESTRA 1
#endif /* ROUTING_CONF_NULLROUTING */
#endif /* INSTANT_CONF_EMULATE_ORCHESTRA not defined */
