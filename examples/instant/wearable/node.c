#include "contiki.h"
#include "net/netstack.h"
#include "net/ipv6/uip-ds6.h"
#include "net/ipv6/uip-ds6-route.h"
#include "instant.h"

#include "sys/log.h"
#define LOG_MODULE "Wearable"
#define LOG_LEVEL LOG_LEVEL_INFO

#if INSTANT_CONF_EMULATE_ORCHESTRA
#include "net/routing/rpl-classic/rpl.h"
#endif

void
print_network_status(void)
{
  int i;
  uint8_t state;
  uip_ds6_defrt_t *default_route;
  uip_ds6_route_t *route;

  LOG_INFO("--- Network status ---\n");

  /* Our IPv6 addresses */
  LOG_INFO("-- Server IPv6 addresses:\n");
  for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
    state = uip_ds6_if.addr_list[i].state;
    if(uip_ds6_if.addr_list[i].isused &&
       (state == ADDR_TENTATIVE || state == ADDR_PREFERRED)) {
      LOG_INFO("-- ");
      LOG_INFO_6ADDR(&uip_ds6_if.addr_list[i].ipaddr);
      LOG_INFO_("\n");
    }
  }

  /* Our default route */
  LOG_INFO("-- Default route:\n");
  default_route = uip_ds6_defrt_lookup(uip_ds6_defrt_choose());
  if(default_route != NULL) {
    LOG_INFO("-- ");
    LOG_INFO_6ADDR(&default_route->ipaddr);
    LOG_INFO(" (lifetime: %lu seconds)\n", (unsigned long)default_route->lifetime.interval);
  } else {
    LOG_INFO("-- None\n");
  }

  /* Our routing entries */
  LOG_INFO("-- Routing entries (%u in total):\n", uip_ds6_route_num_routes());
  route = uip_ds6_route_head();
  while(route != NULL) {
    LOG_INFO("-- ");
    LOG_INFO_6ADDR(&route->ipaddr);
    LOG_INFO_(" via ");
    LOG_INFO_6ADDR(uip_ds6_route_nexthop(route));
    LOG_INFO_(" (lifetime: %lu seconds)\n", (unsigned long)route->state.lifetime);
    route = uip_ds6_route_next(route);
  }

  LOG_INFO("----------------------\n");
}
/*---------------------------------------------------------------------------*/
PROCESS(node_process, "Node");
AUTOSTART_PROCESSES(&node_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(node_process, ev, data)
{
  PROCESS_BEGIN();
  
#if INSTANT_CONF_EMULATE_ORCHESTRA
#if ROUTING_CONF_RPL_CLASSIC
  /* For RPL+Orchestra experiments */
  rpl_set_mode(RPL_MODE_LEAF);
#endif
#else
  uip_ipaddr_t ipaddr;
  uip_ip6addr(&ipaddr, INSTANT_NETWORK_PREFIX, 0, 0, 0, 0, 0, 0, 0);
  uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
  uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);
#endif

  instant_init();

  /* Turn on TSCH */
  NETSTACK_MAC.on();

  LOG_INFO("Wearable node started\n");

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
