#define ROM_BOOTLOADER_ENABLE                 1

#define UIP_CONF_BUFFER_SIZE 60
#define UIP_CONF_ND6_SEND_RA 0
#define UIP_CONF_ND6_SEND_NA 0
#define UIP_CONF_ND6_SEND_NS 0
#define UIP_CONF_UDP 0
#define UIP_CONF_TCP 0
#define UIP_CONF_MAX_ROUTES 0
#define NETSTACK_MAX_ROUTE_ENTRIES 0
#define NBR_TABLE_CONF_MAX_NEIGHBORS 0
#define QUEUEBUF_CONF_NUM 0


#if CONTIKI_TARGET_SRF06_CC26XX
extern void hw_watchdog_periodic(void);
#else
#define hw_watchdog_periodic()
#endif
