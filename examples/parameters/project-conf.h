#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_


/*******************************************************/
/******************* Configure network stack ***********/
/*******************************************************/

#define UDP_PORT	8765

/* Enable printing of packet counters */
#define LINK_STATS_CONF_PACKET_COUNTERS          1

/*******************************************************/
/******************* Configure TSCH ********************/
/*******************************************************/

/* IEEE802.15.4 PANID */
#define IEEE802154_CONF_PANID 0x81a5

/* Do not start TSCH at init, wait for NETSTACK_MAC.on() */
#define TSCH_CONF_AUTOSTART 0

/* 6TiSCH minimal schedule length.
 * Larger values result in less frequent active slots: reduces capacity and saves energy. */
#ifndef TSCH_SCHEDULE_CONF_DEFAULT_LENGTH
#define TSCH_SCHEDULE_CONF_DEFAULT_LENGTH 7
#endif

/* Orchestra unicast slotframe size (when Orchestra is enabled) */
#ifndef ORCHESTRA_CONF_UNICAST_PERIOD
#define ORCHESTRA_CONF_UNICAST_PERIOD             19
#endif

#define ORCHESTRA_CONF_RULES { &eb_per_time_source, &unicast_per_neighbor_rpl_storing, &default_common }

/*******************************************************/
/************* Other system configuration **************/
/*******************************************************/

/* Logging */
#define LOG_CONF_LEVEL_RPL                         LOG_LEVEL_ERR
#define LOG_CONF_LEVEL_TCPIP                       LOG_LEVEL_ERR
#define LOG_CONF_LEVEL_IPV6                        LOG_LEVEL_ERR
#define LOG_CONF_LEVEL_6LOWPAN                     LOG_LEVEL_ERR
#define LOG_CONF_LEVEL_MAC                         LOG_LEVEL_WARN
#define LOG_CONF_LEVEL_FRAMER                      LOG_LEVEL_ERR
#define TSCH_LOG_CONF_PER_SLOT                     0

/*******************************************************/
/****************** Firmware Type options **************/
/*******************************************************/

/* 6tisch minimal */
#define FIRMWARE_TYPE_6TISCH_MIN 1
/* receiver based Orchestra + RPL */
#define FIRMWARE_TYPE_ORCHESTRA_RB 2

/*******************************************************/
/*************** Configure other settings **************/
/*******************************************************/

#define ROOT_ID 1

#ifndef SEND_INTERVAL_SEC
#define SEND_INTERVAL_SEC 10
#endif

#define WARM_UP_PERIOD_SEC 70

#endif /* PROJECT_CONF_H_ */
