/* Enable security */
#define LLSEC802154_CONF_ENABLED 1

/* Disable fragmentation */
#define SICSLOWPAN_CONF_FRAG     0

/* Enable logging */
#define LOG_CONF_LEVEL_6LOWPAN   LOG_LEVEL_DBG
#define LOG_CONF_LEVEL_TCPIP     LOG_LEVEL_DBG

/* Packet size (excluding all the headers) */
/*#define UDP_PAYLOAD_SIZE 59 */   /* works */
#define UDP_PAYLOAD_SIZE 60        /* works first hop only, fails in forwarding! */
/*#define UDP_PAYLOAD_SIZE 66 */   /* works first hop only, fails in forwarding (no attempt to send) */
/*#define UDP_PAYLOAD_SIZE 77 */   /* does not work */
